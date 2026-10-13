#!/usr/bin/env python3
import sys
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.2
        self.angular_speed = 0.3

        self.get_logger().info('键盘速度控制节点已启动')
        self.get_logger().info('W:前进  S:后退  A:左转  D:右转')
        self.get_logger().info('空格:停止  Q:退出')

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                linear = 0.0
                angular = 0.0

                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'w':
                        linear = self.linear_speed
                    elif key == 's':
                        linear = -self.linear_speed
                    elif key == 'a':
                        angular = self.angular_speed
                    elif key == 'd':
                        angular = -self.angular_speed
                    elif key == ' ':
                        pass
                    elif key == 'q':
                        break

                self.publish_velocity(linear, angular)

        finally:
            self.publish_velocity(0.0, 0.0)
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_velocity(0.0, 0.0)
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
