#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
农业机器人后端 API
============================================================
主要功能：
1. 提供机器人状态查询接口。
2. 通过 ROS 2 获取里程计、速度和路径信息。
3. 将 ROS 2 数据提供给 Vue 前端。
4. 接收网页指令，并发布到 ROS 2。


运行方式：
    uvicorn main:app --host 0.0.0.0 --port 8000


"""

# ============================================================
# 1. 标准库导入
# ============================================================

from contextlib import asynccontextmanager
from datetime import datetime
from math import atan2, degrees, hypot, isfinite
from threading import Lock, Thread
from time import monotonic
import json
import hashlib
from pathlib import Path as FilePath

try:
    from .devices import device_cache, install_ros_devices, install_routes, service_call
except ImportError:
    import sys
    sys.path.insert(0, str(FilePath(__file__).resolve().parent.parent))
    from agri_backend.devices import device_cache, install_ros_devices, install_routes, service_call


# ============================================================
# 2. FastAPI 导入
# ============================================================

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field


# ============================================================
# 3. ROS 2 导入
# ============================================================

# 如果 ROS 2 环境不可用，后端仍然可以启动，
# 但机器人状态和急停控制接口将无法正常连接 ROS 2。

ROS_AVAILABLE = True
ROS_IMPORT_ERROR = None

try:
    import rclpy

    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool

except Exception as exc:
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = str(exc)


# ============================================================
# 4. 全局配置
# ============================================================

ROBOT_ID = "agri_robot_01"

# 里程计超过该时间未更新，则认为机器人离线。
ODOM_TIMEOUT_SECONDS = 3.0

# 允许的 Vue 前端跨域来源。
ALLOWED_ORIGINS = [
    "http://localhost:5173",
    "http://127.0.0.1:5173",
]


# ============================================================
# 5. 机器人状态缓存
# ============================================================

class RobotState:
    """
    保存从 ROS 2 接收到的机器人状态。

    使用 Lock 保护共享数据，避免 ROS 2 回调线程
    与 FastAPI 请求线程同时访问时出现数据竞争。
    """

    def __init__(self):
        self.lock = Lock()

        self.data = {
            # 机器人位置
            "position_x": None,
            "position_y": None,

            # 机器人航向角，单位：度
            "heading": None,

            # 里程计反馈的实际速度
            "speed": None,
            "angular_speed": None,

            # ROS 2 速度指令
            "commanded_speed": None,
            "commanded_angular_speed": None,

            # 路径信息
            "reference_path_points": 0,
            "tracked_path_points": 0,
            "reference_path": [],
            "reference_frame": "",
            "reference_sequence": 0,
            "reference_fingerprint": "",
            "tracked_path": [],
            "tracked_frame": "",
            "waypoint_path": [],
            "tracked_source": "tracking",

            # 最近一次收到里程计的时间
            "last_odom_time": None,

            # 使用单调时钟计算里程计数据的新鲜度
            "last_odom_monotonic": None,
        }

    def update(self, **kwargs):
        """更新机器人状态。"""

        with self.lock:
            self.data.update(kwargs)

    def snapshot(self):
        """获取状态快照，避免直接暴露内部字典。"""

        with self.lock:
            return self.data.copy()


# 创建全局机器人状态对象。
robot_state = RobotState()


# ============================================================
# 6. 急停状态缓存
# ============================================================

class EmergencyStopState:
    """
    记录后端最近发送的急停指令状态。

    active=True：
        最近发送的指令为触发急停。

    active=False：
        最近发送的指令为解除急停。

    """

    def __init__(self):
        self.lock = Lock()

        # 默认按急停状态处理。
        self.active = False

    def set(self, active: bool):
        """更新最近发送的急停指令状态。"""

        with self.lock:
            self.active = active

    def get(self) -> bool:
        """读取最近发送的急停指令状态。"""

        with self.lock:
            return self.active


emergency_stop_state = EmergencyStopState()


# ============================================================
# 7. ROS 2 状态与控制节点
# ============================================================

if ROS_AVAILABLE:

    class RobotStatusNode(Node):
        """
        ROS 2 与 FastAPI 之间的数据桥接节点。

        订阅：
            /odom
            /cmd_vel
            /reference_path
            /tracked_path

        发布：
            /emergency_stop
        """

        def __init__(self):
            super().__init__("agri_backend_bridge")

            # ------------------------------------------------
            # 7.1 订阅机器人里程计
            # ------------------------------------------------

            self.create_subscription(
                Odometry,
                "/odom",
                self.odom_callback,
                10,
            )

            # ------------------------------------------------
            # 7.2 订阅速度指令
            # ------------------------------------------------

            self.create_subscription(
                Twist,
                "/cmd_vel",
                self.cmd_vel_callback,
                10,
            )

            # ------------------------------------------------
            # 7.3 发布急停状态
            # ------------------------------------------------

            self.emergency_stop_pub = self.create_publisher(
                Bool,
                "/emergency_stop",
                10,
            )

            # ------------------------------------------------
            # 7.4 订阅参考路径
            # ------------------------------------------------

            self.reference_subscription = self.create_subscription(
                Path,
                "/reference_path",
                self.reference_path_callback,
                10,
            )
            self.reference_transient = False
            self.create_timer(1.0, self.refresh_reference_qos)

            # ------------------------------------------------
            # 7.5 订阅实际跟踪路径
            # ------------------------------------------------

            self.create_subscription(
                Path,
                "/tracked_path",
                self.tracked_path_callback,
                10,
            )
            self.create_subscription(Path, '/waypoint/trajectory',
                lambda msg: self.tracked_path_callback(msg, 'waypoint'), 1)
            self.create_subscription(Path, '/waypoint/targets', self.waypoint_targets_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

            install_ros_devices(self)
            try:
                from .ros_workspace import RosWorkspace
            except ImportError:
                from agri_backend.ros_workspace import RosWorkspace
            self.workspace_ros = RosWorkspace(self)

            self.get_logger().info(
                "农业机器人 ROS 2 桥接节点已启动"
            )

        def refresh_reference_qos(self):
            """Receive a latched plan even when backend starts after planning.

            The legacy CSV publisher is volatile, so adapt to the offered profile.
            This only changes a read-only subscription, never a control publisher.
            """
            publishers = self.get_publishers_info_by_topic('/reference_path')
            if not publishers:
                return
            transient = all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in publishers)
            if transient == self.reference_transient:
                return
            self.destroy_subscription(self.reference_subscription)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
            self.reference_subscription = self.create_subscription(Path, '/reference_path', self.reference_path_callback, qos)
            self.reference_transient = transient

        # ====================================================
        # 7.6 里程计回调
        # ====================================================

        def odom_callback(self, msg: Odometry):
            """读取位置、航向角和实际运动速度。"""

            # 机器人位置
            position = msg.pose.pose.position

            x = position.x
            y = position.y

            # 四元数转换为平面航向角。
            orientation = msg.pose.pose.orientation

            yaw = atan2(
                2.0 * (
                    orientation.w * orientation.z
                    + orientation.x * orientation.y
                ),
                1.0 - 2.0 * (
                    orientation.y * orientation.y
                    + orientation.z * orientation.z
                ),
            )

            heading = degrees(yaw) % 360.0

            # 里程计反馈的实际运动速度。
            velocity = msg.twist.twist

            speed = hypot(
                velocity.linear.x,
                velocity.linear.y,
            )

            angular_speed = velocity.angular.z

            if not all(isfinite(v) for v in (x, y, heading, speed, angular_speed)):
                return

            # 当前系统时间，仅用于显示。
            timestamp = datetime.now().astimezone().isoformat(
                timespec="seconds"
            )

            # 单调时钟用于计算数据是否过期。
            receive_time = monotonic()

            robot_state.update(
                position_x=round(x, 3),
                position_y=round(y, 3),
                heading=round(heading, 2),

                speed=round(speed, 3),
                angular_speed=round(angular_speed, 3),

                last_odom_time=timestamp,
                last_odom_monotonic=receive_time,
            )

        # ====================================================
        # 7.7 速度指令回调
        # ====================================================

        def cmd_vel_callback(self, msg: Twist):
            """
            记录 /cmd_vel 中的速度指令。

            注意：
                指令速度不等于车辆实际运动速度。
            """

            robot_state.update(
                commanded_speed=round(
                    msg.linear.x,
                    3,
                ),
                commanded_angular_speed=round(
                    msg.angular.z,
                    3,
                ),
            )

        # ====================================================
        # 7.8 参考路径回调
        # ====================================================

        def reference_path_callback(self, msg: Path):
            """保存参考路径点，供前端展示。"""

            points = [
                {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                }
                for pose in msg.poses
            ]

            robot_state.update(
                reference_path_points=len(points),
                reference_path=points if all(isfinite(p['x']) and isfinite(p['y']) for p in points) else [],
                reference_frame=msg.header.frame_id,
                reference_sequence=robot_state.snapshot()['reference_sequence'] + 1,
                reference_fingerprint=hashlib.sha256(json.dumps(
                    [msg.header.frame_id, points], allow_nan=True, sort_keys=True).encode()).hexdigest(),
            )

        # ====================================================
        # 7.9 实际跟踪路径回调
        # ====================================================

        def tracked_path_callback(self, msg: Path, source='tracking'):
            """记录实际跟踪路径，供实时页面与参考路径叠加。"""

            points = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses[-10000:]]

            robot_state.update(
                tracked_path_points=len(msg.poses),
                tracked_path=points if all(isfinite(p['x']) and isfinite(p['y']) for p in points) else [],
                tracked_frame=msg.header.frame_id,
                tracked_source=source,
            )

        def waypoint_targets_callback(self, msg: Path):
            points = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
            robot_state.update(waypoint_path=points if msg.header.frame_id == 'map' and all(
                isfinite(p['x']) and isfinite(p['y']) for p in points) else [])


# ============================================================
# 8. ROS 2 后台桥接管理
# ============================================================

class RosBridge:
    """
    管理 ROS 2 节点、执行器和后台线程。

    FastAPI 通过该对象访问 ROS 2 节点，
    无需在每个 HTTP 请求中重新创建 ROS 2 节点。
    """

    def __init__(self):
        self.node = None
        self.executor = None
        self.thread = None

        self.started = False
        self.error = ROS_IMPORT_ERROR

    def start(self):
        """初始化并启动 ROS 2 后台线程。"""

        if not ROS_AVAILABLE:
            self.error = ROS_IMPORT_ERROR
            return

        try:
            # 初始化 ROS 2。
            if not rclpy.ok():
                rclpy.init(args=None)

            # 创建桥接节点。
            self.node = RobotStatusNode()

            # 创建执行器并添加节点。
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)

            # 在后台线程运行 ROS 2 回调。
            self.thread = Thread(
                target=self.executor.spin,
                daemon=True,
                name="ros2_bridge_thread",
            )

            self.thread.start()

            self.started = True
            self.error = None

        except Exception as exc:
            self.error = str(exc)
            self.started = False

            if self.node is not None:
                try:
                    self.node.destroy_node()
                except Exception:
                    pass

                self.node = None

            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

    def stop(self):
        """停止 ROS 2 执行器和后台线程。"""

        if not ROS_AVAILABLE:
            return

        if self.executor is not None:
            try:
                self.executor.shutdown()
            except Exception:
                pass

        if self.thread is not None:
            self.thread.join(timeout=2.0)

        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass

        self.node = None
        self.executor = None
        self.thread = None
        self.started = False

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


# ============================================================
# 9. FastAPI 生命周期
# ============================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    后端启动时：
        创建 ROS 2 桥接并启动后台线程。

    后端关闭时：
        停止 ROS 2 桥接并释放资源。
    """

    bridge = RosBridge()

    # 保存桥接对象，供各个 API 接口使用。
    app.state.ros_bridge = bridge

    bridge.start()

    if bridge.started:
        print("[ROS 2] 实时状态桥接已启动")
    else:
        print(
            f"[ROS 2] 桥接未启动：{bridge.error}"
        )

    try:
        yield

    finally:
        bridge.stop()
        print("[ROS 2] 桥接已关闭")


# ============================================================
# 10. 创建 FastAPI 应用
# ============================================================

app = FastAPI(
    title="Agri Robot API",
    description="农业机器人管理系统后端接口",
    version="0.3.0",
    lifespan=lifespan,
)


@app.exception_handler(RequestValidationError)
async def invalid_request(request, exc):
    # Do not echo NaN/Inf inputs into a JSON error response or hide useful field errors.
    messages = ['.'.join(map(str, item['loc'][1:])) + ': ' + item['msg'] for item in exc.errors()[:5]]
    return JSONResponse(status_code=422, content={'detail': '参数校验失败：' + '；'.join(messages)})


# ============================================================
# 11. 跨域配置
# ============================================================

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ============================================================
# 12. 通用辅助函数
# ============================================================

def get_ros_bridge() -> RosBridge:
    """获取当前 ROS 2 桥接对象。"""

    return getattr(
        app.state,
        "ros_bridge",
        None,
    )


def require_ros_bridge() -> RosBridge:
    """
    获取可用的 ROS 2 桥接对象。

    如果 ROS 2 桥接没有启动，则返回 HTTP 503。
    """

    bridge = get_ros_bridge()

    if (
        bridge is None
        or not bridge.started
        or bridge.node is None
    ):
        raise HTTPException(
            status_code=503,
            detail="ROS 2 桥接未启动，无法执行此操作",
        )

    return bridge


def publish_emergency_stop(active: bool) -> dict:
    """
    向 ROS 2 发布急停指令。

    参数：
        active=True：触发急停。
        active=False：发送解除急停指令。

    返回：
        安全服务的确认响应；实际状态由 safety Topic 提供。

    注意：
        发布成功不等于底盘已经停止或解除锁定。
    """

    bridge = require_ros_bridge()

    # True also uses the immediate stop topic; reset requires a positive service response.
    if active:
        msg = Bool()
        msg.data = True
        bridge.node.emergency_stop_pub.publish(msg)
    return service_call(bridge.node, "/safety/emergency_stop", active=active)


# ============================================================
# 13. 基础 API
# ============================================================

@app.get("/")
def root():
    """检查后端 API 是否正常运行。"""

    return {
        "message": "Agri Robot API is running",
        "status": "ok",
    }


# ============================================================
# 14. 机器人状态 API
# ============================================================

@app.get("/api/robot/status")
def get_robot_status():
    """
    查询机器人状态。

    包括：
        在线状态、位置、航向角、实际速度、
        控制指令、路径点数量和 ROS 2 连接状态。
    """

    data = robot_state.snapshot()
    devices = device_cache.snapshot()

    # --------------------------------------------------------
    # 14.1 判断里程计数据是否过期
    # --------------------------------------------------------

    last_odom = data["last_odom_monotonic"]

    if last_odom is not None:
        data_age = monotonic() - last_odom
    else:
        data_age = None

    online = (
        data_age is not None
        and data_age < ODOM_TIMEOUT_SECONDS
    )

    # --------------------------------------------------------
    # 14.2 判断机器人运行模式
    # --------------------------------------------------------

    if not online:
        mode = "offline"

    elif (
        data["speed"] is not None
        and data["speed"] > 0.05
    ):
        mode = "running"

    else:
        mode = "standby"

    # --------------------------------------------------------
    # 14.3 查询 ROS 2 桥接状态
    # --------------------------------------------------------

    bridge = get_ros_bridge()

    ros_connected = bool(
        bridge is not None
        and bridge.started
        and bridge.node is not None
    )

    ros_error = (
        bridge.error
        if bridge is not None and bridge.error
        else None
    )

    # --------------------------------------------------------
    # 14.4 返回机器人状态
    # --------------------------------------------------------

    return {
        # 原有前端字段
        "robot_id": ROBOT_ID,
        "latitude": devices["gnss"].get("latitude") if devices["gnss"]["fresh"] else None,
        "longitude": devices["gnss"].get("longitude") if devices["gnss"]["fresh"] else None,
        "control_mode": devices["safety"].get("owner"),
        "task_state": devices.get(devices["safety"].get("owner"), {}).get("state"),
        "safety": devices["safety"],
        "online": online,
        "mode": mode,

        "speed": data["speed"],
        "heading": data["heading"],

        # 尚未接入电池数据，不伪造电量
        "battery": None,
        "battery_voltage": devices['voltage'].get('volts') if devices['voltage']['fresh'] else None,

        "timestamp": datetime.now().astimezone().isoformat(
            timespec="seconds"
        ),
        "last_odom_time": data['last_odom_time'],

        # ROS 2 连接状态
        "ros_connected": ros_connected,
        "ros_error": ros_error,

        # 机器人位置
        "position_x": data["position_x"],
        "position_y": data["position_y"],

        # 实际运动状态
        "angular_speed": data["angular_speed"],

        # 控制器速度指令
        "commanded_speed": data["commanded_speed"],
        "commanded_angular_speed": (
            data["commanded_angular_speed"]
        ),

        # 路径状态
        "reference_path_points": (
            data["reference_path_points"]
        ),
        "tracked_path_points": (
            data["tracked_path_points"]
        ),

        # 里程计数据年龄，单位：秒
        "odom_age_seconds": (
            round(data_age, 2)
            if data_age is not None
            else None
        ),
    }


# ============================================================
# 15. 参考路径 API
# ============================================================

@app.get("/api/robot/path/reference")
def get_reference_path():
    """获取当前参考路径的全部路径点。"""

    data = robot_state.snapshot()

    points = data["reference_path"]

    return {
        "count": len(points),
        "points": points,
        "frame_id": data['reference_frame'],
    }


@app.get('/api/robot/path/tracked')
def get_tracked_path():
    data = robot_state.snapshot()
    return {'count': len(data['tracked_path']), 'points': data['tracked_path'],
            'frame_id': data['tracked_frame']}


@app.get('/api/robot/waypoints')
def get_waypoints():
    points = robot_state.snapshot()['waypoint_path']
    return {'count': len(points), 'frame_id': 'map', 'points': points}


# ============================================================
# 16. 急停控制 API
# ============================================================

@app.post("/api/robot/emergency-stop")
def trigger_emergency_stop():
    """
    触发软件急停。

    HTTP 方法：
        POST

    接口地址：
        /api/robot/emergency-stop

    ROS 2 消息：
        /emergency_stop = True
    """

    return publish_emergency_stop(True)


@app.post("/api/robot/emergency-stop/reset")
def reset_emergency_stop():
    """
    发送解除软件急停指令。

    HTTP 方法：
        POST

    接口地址：
        /api/robot/emergency-stop/reset

    ROS 2 服务：
        /safety/emergency_stop，SetBool(data=False)

    注意：
        此接口会发送解除指令，不代表车辆已经
        具备安全启动条件。
    """

    return publish_emergency_stop(False)


@app.get("/api/robot/emergency-stop/status")
def get_emergency_stop_status():
    """
    查询安全模块反馈的急停状态；失联时为未知。

    HTTP 方法：
        GET

    接口地址：
        /api/robot/emergency-stop/status
    """

    bridge = get_ros_bridge()

    ros_connected = bool(
        bridge is not None
        and bridge.started
        and bridge.node is not None
    )

    safety = device_cache.snapshot()["safety"]
    return {
        "emergency_stop": safety.get("emergency_stop"),
        "fresh": safety["fresh"],
        "ros_connected": ros_connected,
    }

# ============================================================
# 17. 农田边界坐标 API
# ============================================================

class CoordinatePoint(BaseModel):
    """单个农田边界点"""

    longitude: float = Field(ge=-180, le=180)
    latitude: float = Field(ge=-90, le=90)


class CoordinateRequest(BaseModel):
    """农田边界坐标请求"""

    points: list[CoordinatePoint] = Field(min_length=3)


@app.post("/api/field/coordinate")
def receive_coordinate(data: CoordinateRequest):
    """
    接收前端提交的农田边界经纬度，并保存到 JSON 文件。
    """

    points = data.points

    # 将坐标转换成普通 Python 字典
    points_data = [
        {
            "longitude": point.longitude,
            "latitude": point.latitude,
        }
        for point in points
    ]

    # 项目根目录：~/agri_robot
    project_root = FilePath(__file__).resolve().parent.parent

    # 数据保存路径
    data_dir = project_root / "data"
    data_dir.mkdir(parents=True, exist_ok=True)

    file_path = data_dir / "field_boundary.json"

    # 组织保存内容
    boundary_data = {
        "point_count": len(points_data),
        "points": points_data,
    }

    # 写入 JSON 文件
    try:
        with file_path.open("w", encoding="utf-8") as f:
            json.dump(
                boundary_data,
                f,
                ensure_ascii=False,
                indent=2,
            )

    except OSError as e:
        print(f"地块坐标保存失败：{e}")

        return {
            "success": False,
            "message": "地块坐标接收成功，但保存到文件失败",
            "error": str(e),
        }

    print("\n收到前端提交的农田边界坐标")
    print(f"边界点数量：{len(points_data)}")
    print(f"保存位置：{file_path}")

    for index, point in enumerate(points_data, start=1):
        print(
            f"第 {index} 个点："
            f"经度={point['longitude']}, "
            f"纬度={point['latitude']}"
        )

    return {
        "success": True,
        "message": "农田边界坐标接收成功并已保存",
        "data": boundary_data,
        "file_path": str(file_path),
    }


install_routes(app, require_ros_bridge)

try:
    from .workspace import install_routes as install_workspace_routes
except ImportError:
    # Supports the documented `cd agri_backend; uvicorn main:app` invocation.
    import sys
    sys.path.insert(0, str(FilePath(__file__).resolve().parent.parent))
    from agri_backend.workspace import install_routes as install_workspace_routes
install_workspace_routes(app, require_ros_bridge, robot_state.snapshot)
