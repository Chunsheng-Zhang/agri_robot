#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus


class MGX23CSVReplay(Node):

    def __init__(self):
        super().__init__('mgx23_csv_replay')

        self.declare_parameter(
            'csv_file',
            str(
                Path.home()
                / 'agri_robot'
                / 'mgx23_ws'
                / 'data'
                / 'mgx23_logs'
                / 'mgx23_20261014_102345.csv'
            )
        )

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('require_checksum_ok', True)

        self.csv_file = self.get_parameter(
            'csv_file'
        ).value

        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        self.loop = bool(
            self.get_parameter('loop').value
        )

        self.require_checksum_ok = bool(
            self.get_parameter('require_checksum_ok').value
        )

        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate 必须大于 0')

        self.rows = self.load_csv(self.csv_file)

        if not self.rows:
            raise RuntimeError('CSV 中没有可回放的有效记录')

        self.index = 0
        self.loop_count = 0

        self.position_pub = self.create_publisher(
            NavSatFix,
            '/mgx23/position',
            10
        )

        self.attitude_pub = self.create_publisher(
            Vector3,
            '/mgx23/attitude',
            10
        )

        self.velocity_pub = self.create_publisher(
            Vector3,
            '/mgx23/velocity',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_next
        )

        self.get_logger().info(
            f'CSV 回放启动：{self.csv_file}'
        )

        self.get_logger().info(
            f'有效记录数：{len(self.rows)}，'
            f'发布频率：{self.publish_rate} Hz，'
            f'循环播放：{self.loop}'
        )

        self.get_logger().warn(
            '回放数据只用于测试。若 CSV 只有一条记录，'
            '车辆位置不会随时间变化。'
        )

    def load_csv(self, filename):
        path = Path(filename).expanduser()

        if not path.is_file():
            raise FileNotFoundError(
                f'找不到 CSV 文件：{path}'
            )

        rows = []

        with path.open(
            'r',
            newline='',
            encoding='utf-8-sig'
        ) as file:
            reader = csv.DictReader(file)

            required = {
                'latitude',
                'longitude',
                'altitude',
                'vel_north',
                'vel_east',
                'vel_down',
                'roll',
                'pitch',
                'yaw',
                'checksum_ok',
            }

            missing = required - set(reader.fieldnames or [])

            if missing:
                raise ValueError(
                    f'CSV 缺少字段：{sorted(missing)}'
                )

            for line_number, row in enumerate(reader, start=2):
                try:
                    values = {
                        key: float(row[key])
                        for key in (
                            'latitude',
                            'longitude',
                            'altitude',
                            'vel_north',
                            'vel_east',
                            'vel_down',
                            'roll',
                            'pitch',
                            'yaw',
                        )
                    }

                    if not all(
                        math.isfinite(value)
                        for value in values.values()
                    ):
                        raise ValueError('存在非有限数值')

                    if not (
                        -90.0 <= values['latitude'] <= 90.0
                        and -180.0 <= values['longitude'] <= 180.0
                    ):
                        raise ValueError('经纬度超出有效范围')

                    checksum_ok = (
                        str(row['checksum_ok']).strip().lower()
                        == 'true'
                    )

                    if self.require_checksum_ok and not checksum_ok:
                        continue

                    rows.append(values)

                except (ValueError, TypeError, KeyError) as exc:
                    self.get_logger().warning(
                        f'跳过 CSV 第 {line_number} 行：{exc}'
                    )

        return rows

    def publish_next(self):
        if self.index >= len(self.rows):
            if not self.loop:
                self.get_logger().info('CSV 回放完成，停止发布')
                self.timer.cancel()
                return

            self.index = 0
            self.loop_count += 1

            self.get_logger().info(
                f'开始第 {self.loop_count + 1} 轮回放'
            )

        row = self.rows[self.index]
        self.index += 1

        position = NavSatFix()
        position.header.stamp = self.get_clock().now().to_msg()
        position.header.frame_id = 'gnss'
        position.status.status = NavSatStatus.STATUS_FIX
        position.status.service = NavSatStatus.SERVICE_GPS
        position.latitude = row['latitude']
        position.longitude = row['longitude']
        position.altitude = row['altitude']

        attitude = Vector3()
        attitude.x = row['roll']
        attitude.y = row['pitch']
        attitude.z = row['yaw']

        velocity = Vector3()
        velocity.x = row['vel_north']
        velocity.y = row['vel_east']
        velocity.z = row['vel_down']

        self.position_pub.publish(position)
        self.attitude_pub.publish(attitude)
        self.velocity_pub.publish(velocity)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = MGX23CSVReplay()
        rclpy.spin(node)

    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f'[ERROR] {exc}')

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()