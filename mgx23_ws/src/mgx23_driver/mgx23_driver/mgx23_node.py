import socket
import time
import math
import base64
import json
import uuid
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Vector3Stamped
from std_msgs.msg import UInt32, Float32, String
from sensor_msgs.msg import NavSatFix, NavSatStatus

from mgx23_driver.mgx23_parser import (
    HEADER,
    FRAME_LEN,
    TAIL,
    parse_frame,
)


class MGX23Node(Node):

    def __init__(self):
        super().__init__("mgx23_node")

        # ==============================
        # MGX23 TCP 配置
        # ==============================
        for name, default in {
            "transport": "tcp", "device_ip": "172.16.1.250",
            "port": 9000, "bind_address": "0.0.0.0",
            "max_accuracy": 0.5,
            "raw_log_directory": "",
        }.items():
            self.declare_parameter(name, default)
        self.ip = self.get_parameter("device_ip").value
        self.port = self.get_parameter("port").value
        self.transport = self.get_parameter("transport").value
        if self.transport not in ("tcp", "udp"):
            raise ValueError("transport must be tcp or udp")
        self.max_accuracy = float(self.get_parameter("max_accuracy").value)
        if not math.isfinite(self.max_accuracy) or self.max_accuracy <= 0:
            raise ValueError("max_accuracy must be positive")
        self.last_device_time = None
        self.last_status_time = None
        self.raw_session = uuid.uuid4().hex
        self.raw_sequence = 0
        self.raw_pub = self.create_publisher(String, "/mgx23/raw", 100)
        self.raw_log = None
        raw_folder = self.get_parameter("raw_log_directory").value
        if raw_folder:
            folder = Path(raw_folder).expanduser()
            folder.mkdir(parents=True, exist_ok=True)
            name = datetime.now().strftime("mgx23_%Y%m%d_%H%M%S_%f.bin")
            self.raw_log = (folder / name).open("xb")
        self.attitude_stamped_pub = self.create_publisher(Vector3Stamped, "/mgx23/attitude_stamped", 10)
        self.velocity_stamped_pub = self.create_publisher(Vector3Stamped, "/mgx23/velocity_stamped", 10)
        self.status_pub = self.create_publisher(UInt32, "/mgx23/status", 10)
        self.accuracy_pub = self.create_publisher(Float32, "/mgx23/accuracy", 10)

        # ==============================
        # 创建三个 ROS 2 发布器
        # ==============================
        self.attitude_pub = self.create_publisher(
            Vector3,
            "/mgx23/attitude",
            10,
        )

        self.position_pub = self.create_publisher(
            NavSatFix,
            "/mgx23/position",
            10,
        )

        self.velocity_pub = self.create_publisher(
            Vector3,
            "/mgx23/velocity",
            10,
        )

        # TCP 接收缓存
        self.buffer = bytearray()

        # 非阻塞 TCP 套接字
        self.sock = None

        # 连接重试时间
        self.last_connect_attempt = 0.0

        self.get_logger().info(
            "MGX23 ROS 2 driver started."
        )

        self.get_logger().info(
            f"Connecting to {self.ip}:{self.port}"
        )

        # 定时处理 TCP 数据，避免阻塞 ROS 2 事件循环
        self.timer = self.create_timer(
            0.005,
            self.receive_data,
        )

    def connect_device(self):
        """尝试建立 MGX23 TCP 连接。"""

        now = time.monotonic()

        # 每隔 2 秒尝试连接一次
        if now - self.last_connect_attempt < 2.0:
            return

        self.last_connect_attempt = now

        try:
            sock = socket.socket(socket.AF_INET,
                                 socket.SOCK_STREAM if self.transport == "tcp" else socket.SOCK_DGRAM)
            if self.transport == "tcp":
                sock.settimeout(0.5)
                sock.connect((self.ip, self.port))
            else:
                sock.bind((self.get_parameter("bind_address").value, self.port))

            # 连接成功后切换为非阻塞模式
            sock.setblocking(False)

            self.sock = sock
            self.buffer.clear()
            self.last_device_time = None

            self.get_logger().info(
                f"Connected to MGX23: {self.ip}:{self.port}"
            )

        except OSError as exc:
            if "sock" in locals():
                sock.close()

            self.get_logger().warning(
                f"MGX23 connection failed: {exc}"
            )

    def receive_data(self):
        """接收 TCP 数据，提取完整协议帧并发布话题。"""

        if self.sock is None:
            self.connect_device()
            return

        try:
            if self.transport == "udp":
                chunk, sender = self.sock.recvfrom(65535)
                if self.ip and sender[0] != self.ip:
                    return
                # Datagram boundaries are not TCP stream boundaries.
                self.buffer.clear()
            else:
                chunk = self.sock.recv(4096)

            # 对端关闭连接
            if not chunk:
                self.get_logger().warning(
                    "MGX23 TCP connection closed."
                )
                self.close_socket()
                return

            if self.raw_log is not None:
                try:
                    self.raw_log.write(chunk)
                except OSError as exc:
                    self.get_logger().warning(f"Raw logging disabled: {exc}")
                    try:
                        self.raw_log.close()
                    except OSError:
                        pass
                    self.raw_log = None

            # Exact received bytes, before framing/checksum/accuracy filtering.
            # The task recorder owns task-scoped storage; this does not change
            # the existing continuous raw log or any navigation validity test.
            self.raw_sequence += 1
            if self.raw_pub.get_subscription_count():
                self.raw_pub.publish(String(data=json.dumps({
                    'schema_version': 1, 'session': self.raw_session,
                    'sequence': self.raw_sequence, 'transport': self.transport,
                    'stamp_ns': self.get_clock().now().nanoseconds,
                    'unix_ns': time.time_ns(),
                    'data_base64': base64.b64encode(chunk).decode('ascii'),
                })))
            self.buffer.extend(chunk)

        except BlockingIOError:
            # 当前没有新数据
            return

        except OSError as exc:
            self.get_logger().warning(
                f"MGX23 TCP receive error: {exc}"
            )
            self.close_socket()
            return

        # 从缓存中提取完整数据帧
        while True:

            header_index = self.buffer.find(HEADER)

            if header_index < 0:
                # 没有完整帧头，保留可能属于帧头的最后 3 字节
                if len(self.buffer) > len(HEADER) - 1:
                    del self.buffer[
                        :len(self.buffer) - (len(HEADER) - 1)
                    ]
                return

            # 丢弃帧头之前的无效数据
            if header_index > 0:
                del self.buffer[:header_index]

            # 数据还不足一个完整帧，等待下一次接收
            if len(self.buffer) < FRAME_LEN:
                return

            # 帧尾不匹配，向后移动一个字节继续寻找帧头
            if self.buffer[98:100] != TAIL:
                del self.buffer[0]
                continue

            frame = bytes(self.buffer[:FRAME_LEN])
            del self.buffer[:FRAME_LEN]

            # 解析并检查校验和
            data = parse_frame(frame)

            if data is None or not data["checksum_ok"]:
                self.get_logger().warning(
                    "Invalid MGX23 frame; discarded."
                )
                continue

            self.publish_data(data)

    def publish_data(self, data):
        """将解析后的导航数据发布到三个 ROS 2 话题。"""

        # Initialization diagnostics must remain available before position/INS
        # accuracy qualifies for navigation. Bad checksums never reach here.
        # A frozen device sample must not refresh the status watchdog either.
        if math.isfinite(data['time']) and data['time'] != self.last_status_time:
            self.status_pub.publish(UInt32(data=data['status']))
            self.last_status_time = data['time']

        fields = ("time", "latitude", "longitude", "altitude", "roll", "pitch",
                  "yaw", "vel_north", "vel_east", "vel_down", "accuracy")
        if (not all(math.isfinite(data[key]) for key in fields)
                or not -90 <= data["latitude"] <= 90
                or not -180 <= data["longitude"] <= 180
                or not -math.pi <= data["yaw"] <= math.pi
                or not 0 <= data["accuracy"] <= self.max_accuracy):
            return
        # Repeated device samples must not refresh a navigation watchdog.
        if data["time"] == self.last_device_time:
            return
        self.last_device_time = data["time"]
        stamp = self.get_clock().now().to_msg()

        # ==============================
        # 1. 姿态角
        # ==============================
        attitude_msg = Vector3()

        attitude_msg.x = float(data["roll"])
        attitude_msg.y = float(data["pitch"])
        attitude_msg.z = float(data["yaw"])

        self.attitude_pub.publish(attitude_msg)
        stamped = Vector3Stamped()
        stamped.header.stamp = stamp
        stamped.header.frame_id = "gnss"
        stamped.vector = attitude_msg
        self.attitude_stamped_pub.publish(stamped)

        # ==============================
        # 2. 经纬度和高程
        # ==============================
        position_msg = NavSatFix()

        position_msg.header.stamp = stamp
        position_msg.header.frame_id = "gnss"

        position_msg.latitude = float(data["latitude"])
        position_msg.longitude = float(data["longitude"])
        position_msg.altitude = float(data["altitude"])

        # Initialization flags do not establish a NavSatFix/RTK solution grade.
        position_msg.status.status = NavSatStatus.STATUS_NO_FIX
        position_msg.status.service = NavSatStatus.SERVICE_GPS

        # ==============================
        # 3. NED 速度
        # ==============================
        velocity_msg = Vector3()

        velocity_msg.x = float(data["vel_north"])
        velocity_msg.y = float(data["vel_east"])
        velocity_msg.z = float(data["vel_down"])

        self.velocity_pub.publish(velocity_msg)
        velocity_stamped = Vector3Stamped()
        velocity_stamped.header = stamped.header
        velocity_stamped.vector = velocity_msg
        self.velocity_stamped_pub.publish(velocity_stamped)
        self.accuracy_pub.publish(Float32(data=float(data["accuracy"])))
        self.position_pub.publish(position_msg)

    def close_socket(self):
        """关闭 TCP 连接并清理缓存。"""

        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

            self.sock = None

        self.buffer.clear()

    def destroy_node(self):
        """退出节点时释放 TCP 连接。"""

        self.close_socket()
        if self.raw_log is not None:
            self.raw_log.close()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = MGX23Node()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
