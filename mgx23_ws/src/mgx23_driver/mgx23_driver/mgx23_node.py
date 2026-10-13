import socket
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
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
        self.ip = "172.16.1.250"
        self.port = 9000

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
            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM,
            )

            sock.settimeout(0.5)
            sock.connect((self.ip, self.port))

            # 连接成功后切换为非阻塞模式
            sock.setblocking(False)

            self.sock = sock
            self.buffer.clear()

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
            chunk = self.sock.recv(4096)

            # 对端关闭连接
            if not chunk:
                self.get_logger().warning(
                    "MGX23 TCP connection closed."
                )
                self.close_socket()
                return

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

            if data is None:
                self.get_logger().warning(
                    "Invalid MGX23 frame; discarded."
                )
                continue

            self.publish_data(data)

    def publish_data(self, data):
        """将解析后的导航数据发布到三个 ROS 2 话题。"""

        # ==============================
        # 1. 姿态角
        # ==============================
        attitude_msg = Vector3()

        attitude_msg.x = float(data["roll"])
        attitude_msg.y = float(data["pitch"])
        attitude_msg.z = float(data["yaw"])

        self.attitude_pub.publish(attitude_msg)

        # ==============================
        # 2. 经纬度和高程
        # ==============================
        position_msg = NavSatFix()

        position_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )
        position_msg.header.frame_id = "gnss"

        position_msg.latitude = float(data["latitude"])
        position_msg.longitude = float(data["longitude"])
        position_msg.altitude = float(data["altitude"])

        # 当前只传递位置，不在这里推断定位状态
        position_msg.status.status = NavSatStatus.STATUS_NO_FIX
        position_msg.status.service = NavSatStatus.SERVICE_GPS

        self.position_pub.publish(position_msg)

        # ==============================
        # 3. NED 速度
        # ==============================
        velocity_msg = Vector3()

        velocity_msg.x = float(data["vel_north"])
        velocity_msg.y = float(data["vel_east"])
        velocity_msg.z = float(data["vel_down"])

        self.velocity_pub.publish(velocity_msg)

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