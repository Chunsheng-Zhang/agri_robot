#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
农业机器人软件急停节点

订阅：
    /cmd_vel_raw       路径跟踪节点输出的原始速度指令
    /emergency_stop   急停控制信号，std_msgs/Bool

发布：
    /cmd_vel           经过安全控制后的最终速度指令

安全策略：
    1. 节点启动时默认急停锁定。
    2. 急停触发后持续发布零速度。
    3. 原始速度指令超时后发布零速度。
    4. 解除急停后不会自动启动运动，只有收到新的有效速度指令才会转发。
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class EmergencyStop(Node):

    def __init__(self):
        super().__init__('emergency_stop')

        # 参数
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )
        self.cmd_timeout = float(
            self.get_parameter('cmd_timeout').value
        )

        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate 必须大于 0')

        if self.cmd_timeout <= 0.0:
            raise ValueError('cmd_timeout 必须大于 0')

        # 默认锁定，防止节点启动后意外运动
        self.estop_active = False

        # 最新原始速度指令
        self.latest_cmd = Twist()
        self.last_cmd_time = None

        # 订阅原始速度指令
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_vel_callback,
            10
        )

        # 订阅急停信号
        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            10
        )

        # 发布最终速度指令
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 定时输出速度，确保急停时持续发送零速度
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.control_loop
        )

        self.get_logger().warn(
            '急停节点已启动：当前处于急停锁定状态，请确认安全后再复位。'
        )

    def cmd_vel_callback(self, msg: Twist):
        """记录最新的原始速度指令。"""
        self.latest_cmd = msg
        self.last_cmd_time = time.monotonic()

    def estop_callback(self, msg: Bool):
        """接收急停信号。True 为急停，False 为解除急停。"""
        if msg.data:
            if not self.estop_active:
                self.get_logger().error('收到急停信号，立即停止运动。')

            self.estop_active = True

        else:
            if self.estop_active:
                self.get_logger().warn(
                    '收到急停解除信号。请确认周围安全；节点不会主动发出启动指令。'
                )

            self.estop_active = False

            # 清除旧指令，避免复位后继续执行急停前的旧速度
            self.latest_cmd = Twist()
            self.last_cmd_time = None

    def control_loop(self):
        """周期性发布最终速度指令。"""
        output = Twist()

        if self.estop_active:
            # 急停锁定：始终输出零速度
            self.cmd_pub.publish(output)
            return

        # 检查是否收到过速度指令
        if self.last_cmd_time is None:
            self.cmd_pub.publish(output)
            return

        # 检查原始速度指令是否超时
        elapsed = time.monotonic() - self.last_cmd_time

        if elapsed > self.cmd_timeout:
            self.cmd_pub.publish(output)
            return

        # 正常运行：转发最新速度指令
        self.cmd_pub.publish(self.latest_cmd)


def main(args=None):
    rclpy.init(args=args)

    node = EmergencyStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前尽力发布零速度
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()