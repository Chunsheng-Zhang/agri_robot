#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped

from pyproj import CRS, Transformer


class CoordinateConverter(Node):

    def __init__(self):
        super().__init__('coordinate_converter')

        # 是否使用第一条有效定位作为局部原点
        self.declare_parameter('use_first_fix_as_origin', True)

        # 固定原点的经纬度
        self.declare_parameter('origin_latitude', 0.0)
        self.declare_parameter('origin_longitude', 0.0)

        self.use_first_fix_as_origin = (
            self.get_parameter('use_first_fix_as_origin')
            .get_parameter_value().bool_value
        )

        self.origin_latitude = (
            self.get_parameter('origin_latitude')
            .get_parameter_value().double_value
        )

        self.origin_longitude = (
            self.get_parameter('origin_longitude')
            .get_parameter_value().double_value
        )

        # UTM 投影转换器
        self.transformer = None
        self.utm_epsg = None

        # 局部坐标原点的 UTM 坐标
        self.origin_easting = None
        self.origin_northing = None

        # 订阅 MGX23 经纬度
        self.position_sub = self.create_subscription(
            NavSatFix,
            '/mgx23/position',
            self.position_callback,
            10
        )

        # 发布局部坐标
        self.position_pub = self.create_publisher(
            PointStamped,
            '/agri_localization/position',
            10
        )

        # 如果使用固定原点，启动时直接初始化
        if not self.use_first_fix_as_origin:
            self.initialize_fixed_origin()

        self.get_logger().info('坐标转换节点已启动')
        self.get_logger().info(
            f'使用第一条定位作为原点：'
            f'{self.use_first_fix_as_origin}'
        )

    @staticmethod
    def get_utm_epsg(lat, lon):
        """根据原点经纬度确定 UTM 投影分区。"""
        zone = int((lon + 180.0) // 6.0) + 1
        zone = max(1, min(zone, 60))

        if lat >= 0.0:
            epsg = 32600 + zone
        else:
            epsg = 32700 + zone

        return zone, epsg

    @staticmethod
    def valid_position(lat, lon):
        """检查经纬度是否为有效数值。"""
        return (
            math.isfinite(lat)
            and math.isfinite(lon)
            and -90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
        )

    def initialize_projection(self, lat, lon):
        """初始化 UTM 投影转换器。"""
        zone, self.utm_epsg = self.get_utm_epsg(lat, lon)

        self.transformer = Transformer.from_crs(
            CRS.from_epsg(4326),
            CRS.from_epsg(self.utm_epsg),
            always_xy=True
        )

        self.get_logger().info(
            f'UTM 分区：{zone}，EPSG：{self.utm_epsg}'
        )

    def initialize_fixed_origin(self):
        """使用配置的固定原点初始化局部坐标系。"""
        lat = self.origin_latitude
        lon = self.origin_longitude

        if not self.valid_position(lat, lon):
            raise ValueError('固定原点经纬度无效')

        self.initialize_projection(lat, lon)

        self.origin_easting, self.origin_northing = (
            self.transformer.transform(lon, lat)
        )

        self.get_logger().info(
            f'固定原点：lat={lat:.8f}, lon={lon:.8f}'
        )

    def position_callback(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude

        if not self.valid_position(lat, lon):
            self.get_logger().warning(
                '收到无效经纬度，跳过本次数据'
            )
            return

        # 第一次有效定位时初始化投影与原点
        if self.transformer is None:
            self.initialize_projection(lat, lon)

        easting, northing = self.transformer.transform(lon, lat)

        # 临时模式：将第一条有效定位作为原点
        if self.use_first_fix_as_origin:
            if self.origin_easting is None:
                self.origin_easting = easting
                self.origin_northing = northing

                self.get_logger().info(
                    '已建立局部坐标原点'
                )

        x = easting - self.origin_easting
        y = northing - self.origin_northing

        # 发布局部位置
        output = PointStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'local_map'

        output.point.x = float(x)
        output.point.y = float(y)
        output.point.z = 0.0

        self.position_pub.publish(output)

        self.get_logger().info(
            f'局部位置：x={x:.3f} m, y={y:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)

    node = CoordinateConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
