#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        self.declare_parameter(
            'csv_file',
            str(Path.home() / 'agri_robot/robot_sim/output/boundary_path.csv')
        )

        self.declare_parameter('frame_id', 'local_map')

        csv_file = self.get_parameter('csv_file').value
        self.frame_id = self.get_parameter('frame_id').value

        self.path_msg = self.load_path(csv_file)

        self.publisher = self.create_publisher(
            PathMsg,
            '/reference_path',
            10
        )

        # 定期发布，确保后来启动的订阅节点也能收到路径
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(
            f'参考路径加载完成，共 {len(self.path_msg.poses)} 个点'
        )

        self.get_logger().info(
            f'路径文件：{csv_file}'
        )

    def load_path(self, csv_file):

        csv_path = Path(csv_file).expanduser()

        if not csv_path.exists():
            raise FileNotFoundError(
                f'找不到路径文件：{csv_path}'
            )

        msg = PathMsg()
        msg.header.frame_id = self.frame_id

        with csv_path.open('r', newline='', encoding='utf-8-sig') as file:
            reader = csv.DictReader(file)

            required_columns = {'x_m', 'y_m', 'yaw_rad'}
            if not reader.fieldnames or not required_columns.issubset(
                reader.fieldnames
            ):
                raise ValueError(
                    f'CSV 必须包含列：{required_columns}；'
                    f'实际列名：{reader.fieldnames}'
                )

            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id

                pose.pose.position.x = float(row['x_m'])
                pose.pose.position.y = float(row['y_m'])
                pose.pose.position.z = 0.0

                yaw = float(row['yaw_rad'])

                # 二维航向角转换为四元数
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                msg.poses.append(pose)

        if not msg.poses:
            raise ValueError('CSV 中没有有效路径点')

        return msg

    def publish_path(self):

        now = self.get_clock().now().to_msg()

        self.path_msg.header.stamp = now

        for pose in self.path_msg.poses:
            pose.header.stamp = now

        self.publisher.publish(self.path_msg)


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = PathPublisher()
        rclpy.spin(node)

    except (FileNotFoundError, ValueError) as error:
        print(f'路径加载失败：{error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
