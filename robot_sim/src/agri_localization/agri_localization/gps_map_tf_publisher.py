#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
根据 GPS 经纬度实时发布 map → odom 的 TF 变换。

功能：
1. 订阅 /mgx23/position (NavSatFix)
2. 将 GPS 坐标转换为以 target_1 为原点的局部坐标
3. 发布 map → odom 的 TF 变换

这样机器人就能知道自己在 map 坐标系（地块坐标系）中的位置。

用法：
    ros2 run agri_localization gps_map_tf_publisher \
      --ros-args \
      -p origin_latitude:=36.6468527633281 \
      -p origin_longitude:=117.021433066391
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix

from pyproj import CRS, Transformer

import tf2_ros


class GPSMapTFPublisher(Node):
    """根据 GPS 发布 map → odom TF 变换的节点。"""

    def __init__(self):
        super().__init__('gps_map_tf_publisher')

        # ============================================================
        # 1. 参数：固定原点（target_1）
        # ============================================================

        self.declare_parameter('origin_latitude', 36.6468527633281)
        self.declare_parameter('origin_longitude', 117.021433066391)

        self.origin_lat = (
            self.get_parameter('origin_latitude')
            .get_parameter_value().double_value
        )
        self.origin_lon = (
            self.get_parameter('origin_longitude')
            .get_parameter_value().double_value
        )

        self.get_logger().info(
            f'固定原点：lat={self.origin_lat:.8f}, lon={self.origin_lon:.8f}'
        )

        # ============================================================
        # 2. 初始化 UTM 转换器
        # ============================================================

        self.transformer = None
        self.origin_easting = None
        self.origin_northing = None

        self.init_utm_transformer()

        # ============================================================
        # 3. 订阅 GPS 数据
        # ============================================================

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mgx23/position',
            self.gps_callback,
            10
        )

        # ============================================================
        # 4. TF 广播器
        # ============================================================

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('GPS Map TF Publisher 已启动')

    def init_utm_transformer(self):
        """初始化 UTM 投影转换器。"""

        if not self.is_valid_gps(self.origin_lat, self.origin_lon):
            raise ValueError('固定原点经纬度无效')

        # 计算 UTM 带号
        zone = int((self.origin_lon + 180.0) // 6.0) + 1
        zone = max(1, min(zone, 60))

        # 北半球：EPSG = 32600 + zone
        # 南半球：EPSG = 32700 + zone
        if self.origin_lat >= 0.0:
            epsg = 32600 + zone
        else:
            epsg = 32700 + zone

        self.transformer = Transformer.from_crs(
            CRS.from_epsg(4326),
            CRS.from_epsg(epsg),
            always_xy=True
        )

        # 计算原点的 UTM 坐标
        self.origin_easting, self.origin_northing = self.transformer.transform(
            self.origin_lon, self.origin_lat
        )

        self.get_logger().info(f'UTM 分区：{zone}, EPSG: {epsg}')
        self.get_logger().info(
            f'原点 UTM: easting={self.origin_easting:.3f}m, '
            f'northing={self.origin_northing:.3f}m'
        )

    @staticmethod
    def is_valid_gps(lat, lon):
        """检查 GPS 坐标是否有效。"""

        return (
            math.isfinite(lat)
            and math.isfinite(lon)
            and -90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
        )

    def gps_callback(self, msg: NavSatFix):
        """
        GPS 回调函数。

        收到 GPS 数据后：
        1. 转换为 UTM 坐标
        2. 计算机器人在 map 坐标系的位置（相对于 target_1）
        3. 发布 map → odom 的 TF 变换
        """

        lat = msg.latitude
        lon = msg.longitude

        if not self.is_valid_gps(lat, lon):
            self.get_logger().warning('收到无效 GPS 坐标，跳过')
            return

        # 转换为 UTM 坐标
        easting, northing = self.transformer.transform(lon, lat)

        # 计算机器人在 map 坐标系的位置（相对于 target_1）
        map_x = easting - self.origin_easting
        map_y = northing - self.origin_northing

        # ============================================================
        # 发布 TF: map → odom
        # ============================================================

        t = TransformStamped()

        # 时间戳
        t.header.stamp = self.get_clock().now().to_msg()

        # 父坐标系：map（地块坐标系，原点在 target_1）
        t.header.frame_id = 'map'

        # 子坐标系：odom（里程计坐标系）
        t.child_frame_id = 'odom'

        # 平移：机器人在 map 坐标系的位置
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 旋转：这里假设 odom 和 map 的朝向一致
        # 如果有 IMU 航向数据，可以在这里计算 yaw
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # 发布 TF 变换
        self.tf_broadcaster.sendTransform(t)

        # 每 5 秒打印一次日志
        if self.get_clock().now().nanoseconds % 5000000000 < 100000000:
            self.get_logger().info(
                f'发布 TF: map→odom, x={map_x:.2f}m, y={map_y:.2f}m'
            )


def main(args=None):
    """主函数。"""

    rclpy.init(args=args)

    node = GPSMapTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

