#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from pathlib import Path

import rclpy
import yaml

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String


def wrap_angle(angle):
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def valid_lat_lon(latitude, longitude):
    """检查经纬度是否为有效数值。"""
    return (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    )


def calculate_distance_and_bearing(
    lat1_deg,
    lon1_deg,
    lat2_deg,
    lon2_deg
):
    """
    计算当前位置到目标点的球面距离和初始方位角。

    返回：
        distance：距离，单位 m
        bearing：方位角，单位 rad
                 正北为零，顺时针为正
    """

    earth_radius = 6371000.0

    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(dlon / 2.0) ** 2
    )

    a = max(0.0, min(1.0, a))

    distance = (
        2.0
        * earth_radius
        * math.asin(math.sqrt(a))
    )

    y = math.sin(dlon) * math.cos(lat2)

    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1)
        * math.cos(lat2)
        * math.cos(dlon)
    )

    bearing = math.atan2(y, x)

    return distance, wrap_angle(bearing)


class MultiPointNav(Node):

    def __init__(self):
        super().__init__('multi_point_nav')

        # ==================================================
        # 1. 参数声明
        # ==================================================

        default_waypoints_file = (
            Path.home()
            / 'agri_robot/robot_sim/src/agri_multi_point_nav/config/waypoints.yaml'
        )

        self.declare_parameter(
            'waypoints_file',
            str(default_waypoints_file)
        )

        self.declare_parameter(
            'position_topic',
            '/mgx23/position'
        )

        self.declare_parameter(
            'attitude_topic',
            '/mgx23/attitude'
        )

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel'
        )

        # 目标点到达距离，单位 m
        self.declare_parameter(
            'arrival_tolerance',
            0.2
        )

        # 直线行驶速度，单位 m/s
        self.declare_parameter(
            'linear_speed',
            0.20
        )

        # 最大角速度，单位 rad/s
        self.declare_parameter(
            'max_angular_speed',
            0.35
        )

        # 航向误差比例系数
        self.declare_parameter(
            'heading_kp',
            1.0
        )

        # 原地转向阈值：60° = pi/3
        self.declare_parameter(
            'rotate_in_place_angle',
            math.pi / 3.0
        )

        # 到达目标点后停留时间，单位 s
        self.declare_parameter(
            'dwell_time',
            5.0
        )

        # 定位及航向数据超时时间，单位 s
        self.declare_parameter(
            'sensor_timeout',
            1.0
        )

        # north_cw：
        # 正北为零，顺时针为正，单位 rad
        self.declare_parameter(
            'heading_mode',
            'north_cw'
        )

        # 控制循环频率，单位 Hz
        self.declare_parameter(
            'control_rate',
            10.0
        )

        # ==================================================
        # 2. 读取参数
        # ==================================================

        self.waypoints_file = str(
            self.get_parameter('waypoints_file').value
        )

        self.position_topic = str(
            self.get_parameter('position_topic').value
        )

        self.attitude_topic = str(
            self.get_parameter('attitude_topic').value
        )

        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )

        self.arrival_tolerance = float(
            self.get_parameter('arrival_tolerance').value
        )

        self.linear_speed = float(
            self.get_parameter('linear_speed').value
        )

        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )

        self.heading_kp = float(
            self.get_parameter('heading_kp').value
        )

        self.rotate_in_place_angle = float(
            self.get_parameter('rotate_in_place_angle').value
        )

        self.dwell_time = float(
            self.get_parameter('dwell_time').value
        )

        self.sensor_timeout = float(
            self.get_parameter('sensor_timeout').value
        )

        self.heading_mode = str(
            self.get_parameter('heading_mode').value
        )

        control_rate = float(
            self.get_parameter('control_rate').value
        )

        # ==================================================
        # 3. 参数检查
        # ==================================================

        if self.arrival_tolerance <= 0.0:
            raise ValueError(
                'arrival_tolerance 必须大于 0'
            )

        if self.linear_speed <= 0.0:
            raise ValueError(
                'linear_speed 必须大于 0'
            )

        if self.max_angular_speed <= 0.0:
            raise ValueError(
                'max_angular_speed 必须大于 0'
            )

        if self.heading_kp < 0.0:
            raise ValueError(
                'heading_kp 不能小于 0'
            )

        if not 0.0 < self.rotate_in_place_angle <= math.pi:
            raise ValueError(
                'rotate_in_place_angle 必须在 (0, pi] 内'
            )

        if self.dwell_time < 0.0:
            raise ValueError(
                'dwell_time 不能小于 0'
            )

        if self.sensor_timeout <= 0.0:
            raise ValueError(
                'sensor_timeout 必须大于 0'
            )

        if control_rate <= 0.0:
            raise ValueError(
                'control_rate 必须大于 0'
            )

        if self.heading_mode != 'north_cw':
            raise ValueError(
                "本程序要求 heading_mode='north_cw'，"
                '即正北为零、顺时针为正、单位 rad'
            )

        # ==================================================
        # 4. 加载导航点
        # ==================================================

        self.waypoints = self.load_waypoints(
            self.waypoints_file
        )

        if not self.waypoints:
            raise ValueError(
                '导航点列表为空，请检查 waypoints.yaml'
            )

        self.current_index = 0

        # 任务是否全部完成
        self.task_finished = False

        # ==================================================
        # 5. 任务状态
        # ==================================================

        self.state = 'WAITING_FOR_SENSORS'

        # 进入目标点停留状态的时间
        self.dwell_start_time = None

        # 用于避免重复打印状态日志
        self.last_logged_state = None

        # ==================================================
        # 6. 传感器数据
        # ==================================================

        self.latitude = None
        self.longitude = None
        self.heading = None

        self.last_position_time = None
        self.last_attitude_time = None

        # ==================================================
        # 7. ROS 订阅
        # ==================================================

        self.position_sub = self.create_subscription(
            NavSatFix,
            self.position_topic,
            self.position_callback,
            10
        )

        self.attitude_sub = self.create_subscription(
            Vector3,
            self.attitude_topic,
            self.attitude_callback,
            10
        )

        # ==================================================
        # 8. ROS 发布
        # ==================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/multi_point_nav/status',
            10
        )

        self.target_pub = self.create_publisher(
            String,
            '/multi_point_nav/current_target',
            10
        )

        # ==================================================
        # 9. 定时器
        # ==================================================

        self.timer = self.create_timer(
            1.0 / control_rate,
            self.control_callback
        )

        # ==================================================
        # 10. 启动日志
        # ==================================================

        self.get_logger().info(
            f'多点导航启动，共 {len(self.waypoints)} 个目标点'
        )

        self.get_logger().info(
            f'导航点文件：{self.waypoints_file}'
        )

        self.get_logger().info(
            f'位置输入：{self.position_topic}'
        )

        self.get_logger().info(
            f'航向输入：{self.attitude_topic}'
        )

        self.get_logger().info(
            f'速度输出：{self.cmd_vel_topic}'
        )

        self.get_logger().info(
            f'到达距离：{self.arrival_tolerance:.2f} m'
        )

        self.get_logger().info(
            f'到达后停留：{self.dwell_time:.1f} s'
        )

        self.get_logger().info(
            '原地转向阈值：'
            f'{math.degrees(self.rotate_in_place_angle):.1f}°'
        )

        self.get_logger().info(
            '航向定义：正北为零，顺时针为正'
        )

        # 发布第一个目标点
        self.publish_current_target()

        self.publish_status(
            'WAITING_FOR_SENSORS'
        )

    # ==================================================
    # 读取 YAML 导航点
    # ==================================================

    def load_waypoints(self, filename):

        path = Path(filename).expanduser()

        if not path.exists():
            raise FileNotFoundError(
                f'找不到导航点配置文件：{path}'
            )

        with path.open(
            'r',
            encoding='utf-8'
        ) as f:
            config = yaml.safe_load(f) or {}

        points = config.get(
            'waypoints',
            []
        )

        if not isinstance(points, list):
            raise ValueError(
                'waypoints 必须是列表'
            )

        result = []

        for i, point in enumerate(points):

            if not isinstance(point, dict):
                raise ValueError(
                    f'第 {i + 1} 个导航点格式错误'
                )

            if (
                'latitude' not in point
                or 'longitude' not in point
            ):
                raise ValueError(
                    f'第 {i + 1} 个导航点必须包含 '
                    'latitude 和 longitude'
                )

            try:
                latitude = float(
                    point['latitude']
                )

                longitude = float(
                    point['longitude']
                )

            except (TypeError, ValueError):
                raise ValueError(
                    f'第 {i + 1} 个导航点经纬度必须为数值'
                )

            if not valid_lat_lon(
                latitude,
                longitude
            ):
                raise ValueError(
                    f'第 {i + 1} 个导航点经纬度无效'
                )

            # 排除常见的未初始化坐标
            if (
                abs(latitude) < 1e-12
                and abs(longitude) < 1e-12
            ):
                raise ValueError(
                    f'第 {i + 1} 个导航点经纬度为 (0, 0)'
                )

            result.append({
                'name': str(
                    point.get(
                        'name',
                        f'target_{i + 1}'
                    )
                ),
                'latitude': latitude,
                'longitude': longitude,
            })

        return result

    # ==================================================
    # 接收 MGX23 经纬度
    # ==================================================

    def position_callback(self, msg):

        lat = float(msg.latitude)
        lon = float(msg.longitude)

        if not valid_lat_lon(lat, lon):
            return

        # 排除未初始化的 (0, 0)
        if (
            abs(lat) < 1e-12
            and abs(lon) < 1e-12
        ):
            return

        self.latitude = lat
        self.longitude = lon

        self.last_position_time = (
            self.get_clock().now()
        )

    # ==================================================
    # 接收 MGX23 航向角
    # ==================================================

    def attitude_callback(self, msg):

        raw_heading = float(msg.z)

        if not math.isfinite(raw_heading):
            return

        # 正北为零，顺时针为正
        self.heading = wrap_angle(
            raw_heading
        )

        self.last_attitude_time = (
            self.get_clock().now()
        )

    # ==================================================
    # 判断传感器数据是否有效且未超时
    # ==================================================

    def sensors_ready(self):

        if (
            self.latitude is None
            or self.longitude is None
            or self.heading is None
            or self.last_position_time is None
            or self.last_attitude_time is None
        ):
            return False

        now = self.get_clock().now()

        position_age = (
            now - self.last_position_time
        ).nanoseconds / 1e9

        attitude_age = (
            now - self.last_attitude_time
        ).nanoseconds / 1e9

        return (
            position_age <= self.sensor_timeout
            and attitude_age <= self.sensor_timeout
        )

    # ==================================================
    # 发布状态
    # ==================================================

    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_pub.publish(msg)

        if status != self.last_logged_state:

            self.get_logger().info(
                f'导航状态：{status}'
            )

            self.last_logged_state = status

    # ==================================================
    # 发布当前目标点
    # ==================================================

    def publish_current_target(self):

        if self.task_finished:
            return

        if not (
            0 <= self.current_index < len(self.waypoints)
        ):
            return

        point = self.waypoints[
            self.current_index
        ]

        msg = String()

        msg.data = (
            f"name={point['name']},"
            f"latitude={point['latitude']:.8f},"
            f"longitude={point['longitude']:.8f},"
            f"index={self.current_index + 1},"
            f"total={len(self.waypoints)}"
        )

        self.target_pub.publish(msg)

    # ==================================================
    # 发布速度
    # ==================================================

    def publish_velocity(
        self,
        linear=0.0,
        angular=0.0
    ):

        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    # ==================================================
    # 切换下一个目标点
    # ==================================================

    def advance_waypoint(self):

        self.current_index += 1

        self.dwell_start_time = None

        # 所有目标点完成
        if self.current_index >= len(self.waypoints):

            self.task_finished = True
            self.state = 'COMPLETED'

            self.publish_velocity()

            self.publish_status(
                'COMPLETED'
            )

            self.get_logger().info(
                '所有导航点已完成，机器人保持停车'
            )

            return

        # 切换到下一个目标点
        target = self.waypoints[
            self.current_index
        ]

        self.get_logger().info(
            f'切换至目标点 '
            f'[{self.current_index + 1}/'
            f'{len(self.waypoints)}] '
            f'{target["name"]}'
        )

        self.publish_current_target()

        self.state = 'RUNNING'

        self.publish_status(
            'RUNNING'
        )

    # ==================================================
    # 主控制循环
    # ==================================================

    def control_callback(self):

        # --------------------------------------------------
        # 1. 任务全部完成
        # --------------------------------------------------

        if self.task_finished:

            self.publish_velocity()

            self.publish_status(
                'COMPLETED'
            )

            return

        # --------------------------------------------------
        # 2. 传感器数据无效或超时
        # --------------------------------------------------

        if not self.sensors_ready():

            # 无论此前处于什么状态，都立即停车
            self.publish_velocity()

            self.publish_status(
                'WAITING_FOR_SENSORS'
            )

            # 不清除 dwell_start_time：
            # 恢复传感器后继续检查停留时间。
            return

        # --------------------------------------------------
        # 3. 当前目标点
        # --------------------------------------------------

        target = self.waypoints[
            self.current_index
        ]

        # --------------------------------------------------
        # 4. 到达目标点后的停留状态
        # --------------------------------------------------

        if self.state == 'DWELLING':

            # 停留期间持续发布零速度
            self.publish_velocity()

            now = self.get_clock().now()

            elapsed = (
                now - self.dwell_start_time
            ).nanoseconds / 1e9

            if elapsed >= self.dwell_time:

                self.get_logger().info(
                    f'{target["name"]} 停留结束，'
                    '准备切换目标点'
                )

                self.advance_waypoint()

            else:

                self.publish_status(
                    'DWELLING'
                )

            return

        # --------------------------------------------------
        # 5. 计算当前位置到目标点的距离与方位角
        # --------------------------------------------------

        distance, target_bearing = (
            calculate_distance_and_bearing(
                self.latitude,
                self.longitude,
                target['latitude'],
                target['longitude']
            )
        )

        # --------------------------------------------------
        # 6. 判断是否到达目标点
        # --------------------------------------------------

        if distance <= self.arrival_tolerance:

            self.get_logger().info(
                f'已到达目标点 {target["name"]}，'
                f'距离={distance:.2f} m，'
                f'开始停留 {self.dwell_time:.1f} 秒'
            )

            # 立即停车
            self.publish_velocity()

            # 记录停留开始时间
            self.dwell_start_time = (
                self.get_clock().now()
            )

            self.state = 'DWELLING'

            self.publish_status(
                'DWELLING'
            )

            return

        # --------------------------------------------------
        # 7. 计算航向误差
        # --------------------------------------------------

        current_bearing = self.heading

        heading_error = wrap_angle(
            target_bearing - current_bearing
        )

        # --------------------------------------------------
        # 8. 计算角速度
        # --------------------------------------------------

        # 方位角误差顺时针为正，
        # ROS angular.z 逆时针为正，因此取反。
        angular = (
            -self.heading_kp * heading_error
        )

        angular = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular
            )
        )

        # --------------------------------------------------
        # 9. 根据航向误差选择运动方式
        # --------------------------------------------------

        # 航向误差超过 ±60° 时原地转向
        if (
            abs(heading_error)
            > self.rotate_in_place_angle
        ):

            self.publish_velocity(
                linear=0.0,
                angular=angular
            )

        else:

            # 航向误差在 ±60° 范围内，允许前进
            # 同时保留航向修正
            heading_factor = max(
                0.25,
                math.cos(heading_error)
            )

            linear = (
                self.linear_speed
                * heading_factor
            )

            self.publish_velocity(
                linear=linear,
                angular=angular
            )

        # --------------------------------------------------
        # 10. 发布导航状态
        # --------------------------------------------------

        self.publish_status(
            'RUNNING'
        )

    # ==================================================
    # 销毁节点时停车
    # ==================================================

    def destroy_node(self):

        try:
            self.publish_velocity()

        except Exception:
            pass

        super().destroy_node()


# ======================================================
# 程序入口
# ======================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = MultiPointNav()

        rclpy.spin(node)

    except (
        FileNotFoundError,
        ValueError,
        yaml.YAMLError
    ) as exc:

        print(
            f'多点导航启动失败：{exc}'
        )

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()