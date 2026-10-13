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
    lat1_deg, lon1_deg, lat2_deg, lon2_deg
):
    """
    计算当前位置到目标点的球面距离和初始方位角。

    返回：
        distance: 米
        bearing: 弧度，正北为 0，顺时针为正
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

    distance = 2.0 * earth_radius * math.asin(math.sqrt(a))

    y = math.sin(dlon) * math.cos(lat2)

    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    )

    bearing = math.atan2(y, x)

    return distance, bearing


class MultiPointNav(Node):

    def __init__(self):
        super().__init__('multi_point_nav')

        # ------------------------------
        # 参数
        # ------------------------------

        self.declare_parameter(
            'waypoints_file',
            str(
                Path.home()
                / 'agri_robot/robot_sim/src/agri_multi_point_nav/config/waypoints.yaml'
            )
        )

        self.declare_parameter('position_topic', '/mgx23/position')
        self.declare_parameter('attitude_topic', '/mgx23/attitude')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # 目标点到达距离
        self.declare_parameter('arrival_tolerance', 1.0)

        # 直线行驶速度，m/s
        self.declare_parameter('linear_speed', 0.20)

        # 原地转向角速度，rad/s
        self.declare_parameter('max_angular_speed', 0.35)

        # 航向误差比例系数
        self.declare_parameter('heading_kp', 1.0)

        # 当航向误差大于此值时，先原地调整方向
        self.declare_parameter('rotate_in_place_angle', 0.45)

        # 允许直线行驶的最大航向误差
        self.declare_parameter('drive_heading_tolerance', 0.25)

        # 定位及航向数据超时
        self.declare_parameter('sensor_timeout', 1.0)

        # MGX23 航向角的解释方式：
        # north_cw：正北为零，顺时针为正，单位 rad
        # ros：ROS yaw，X 轴正方向为零，逆时针为正，单位 rad
        self.declare_parameter('heading_mode', 'north_cw')

        # 控制循环频率
        self.declare_parameter('control_rate', 10.0)

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

        self.drive_heading_tolerance = float(
            self.get_parameter('drive_heading_tolerance').value
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

        if self.arrival_tolerance <= 0.0:
            raise ValueError('arrival_tolerance 必须大于 0')

        if self.linear_speed <= 0.0:
            raise ValueError('linear_speed 必须大于 0')

        if self.max_angular_speed <= 0.0:
            raise ValueError('max_angular_speed 必须大于 0')

        if self.sensor_timeout <= 0.0 or control_rate <= 0.0:
            raise ValueError('sensor_timeout 和 control_rate 必须大于 0')

        if self.heading_mode not in ('north_cw', 'ros'):
            raise ValueError(
                "heading_mode 必须是 'north_cw' 或 'ros'"
            )

        # ------------------------------
        # 读取目标点
        # ------------------------------

        self.waypoints = self.load_waypoints(
            self.waypoints_file
        )

        if not self.waypoints:
            raise ValueError('导航点列表为空')

        self.current_index = 0
        self.task_finished = False

        # ------------------------------
        # 传感器数据
        # ------------------------------

        self.latitude = None
        self.longitude = None
        self.heading = None

        self.last_position_time = None
        self.last_attitude_time = None

        # ------------------------------
        # ROS 订阅
        # ------------------------------

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

        # ------------------------------
        # ROS 发布
        # ------------------------------

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

        self.timer = self.create_timer(
            1.0 / control_rate,
            self.control_callback
        )

        self.get_logger().info(
            f'经纬度多点导航已加载，共 {len(self.waypoints)} 个目标点'
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

        self.publish_status('WAITING_FOR_SENSORS')

    # ==================================================
    # 读取经纬度导航点
    # ==================================================

    def load_waypoints(self, filename):

        path = Path(filename).expanduser()

        if not path.exists():
            raise FileNotFoundError(
                f'找不到导航点配置文件：{path}'
            )

        with path.open('r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}

        points = config.get('waypoints', [])

        if not isinstance(points, list):
            raise ValueError('waypoints 必须是列表')

        result = []

        for i, point in enumerate(points):

            if not isinstance(point, dict):
                raise ValueError(
                    f'第 {i + 1} 个导航点格式错误'
                )

            if 'latitude' not in point or 'longitude' not in point:
                raise ValueError(
                    f'第 {i + 1} 个导航点必须包含 latitude 和 longitude'
                )

            latitude = float(point['latitude'])
            longitude = float(point['longitude'])

            if not valid_lat_lon(latitude, longitude):
                raise ValueError(
                    f'第 {i + 1} 个导航点的经纬度无效'
                )

            result.append({
                'name': str(point.get('name', f'target_{i + 1}')),
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

        # 排除常见的未定位占位值。
        # 不依赖 NavSatFix.status，因为驱动状态字可能尚未映射。
        if abs(lat) < 1e-12 and abs(lon) < 1e-12:
            return

        self.latitude = lat
        self.longitude = lon
        self.last_position_time = self.get_clock().now()

    # ==================================================
    # 接收 MGX23 航向角
    # ==================================================

    def attitude_callback(self, msg):

        raw_heading = float(msg.z)

        if not math.isfinite(raw_heading):
            return

        self.heading = wrap_angle(raw_heading)
        self.last_attitude_time = self.get_clock().now()

    # ==================================================
    # 判断传感器数据是否超时
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
    # 航向转换为统一方位角
    # ==================================================

    def get_current_bearing(self):
        """
        返回正北为 0、顺时针为正的方位角。
        """

        if self.heading_mode == 'north_cw':
            return self.heading

        # ROS yaw：X 轴正方向为 0，逆时针为正
        # 转换为正北为 0、顺时针为正
        return wrap_angle(math.pi / 2.0 - self.heading)

    # ==================================================
    # 发布状态
    # ==================================================

    def publish_status(self, status):

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    # ==================================================
    # 发布当前目标点
    # ==================================================

    def publish_current_target(self):

        if self.task_finished:
            return

        point = self.waypoints[self.current_index]

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

    def publish_velocity(self, linear=0.0, angular=0.0):

        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    # ==================================================
    # 主控制循环
    # ==================================================

    def control_callback(self):

        # 任务完成后保持零速度
        if self.task_finished:
            self.publish_velocity()
            self.publish_status('COMPLETED')
            return

        # 传感器数据未准备好或超时，停车
        if not self.sensors_ready():
            self.publish_velocity()
            self.publish_status('WAITING_FOR_SENSORS')
            return

        target = self.waypoints[self.current_index]

        distance, target_bearing = calculate_distance_and_bearing(
            self.latitude,
            self.longitude,
            target['latitude'],
            target['longitude']
        )

        # 到达当前目标点
        if distance <= self.arrival_tolerance:

            self.get_logger().info(
                f'已到达 {target["name"]}，'
                f'距离目标点 {distance:.2f} m'
            )

            self.current_index += 1

            # 全部目标点完成
            if self.current_index >= len(self.waypoints):

                self.task_finished = True

                self.publish_velocity()
                self.publish_status('COMPLETED')

                self.get_logger().info(
                    '全部经纬度导航点已完成，已发布零速度'
                )

                return

            # 切换到下一个目标点
            self.publish_current_target()

            target = self.waypoints[self.current_index]

            self.get_logger().info(
                f'切换至目标点 [{self.current_index + 1}/'
                f'{len(self.waypoints)}] {target["name"]}'
            )

            # 本周期停车，下一周期重新计算方向
            self.publish_velocity()
            self.publish_status('RUNNING')
            return

        current_bearing = self.get_current_bearing()

        # 目标方位角与当前方位角之间的误差
        heading_error = wrap_angle(
            target_bearing - current_bearing
        )

        # north_cw 方位误差为顺时针正方向；
        # ROS angular.z 以逆时针为正，因此控制符号取反。
        angular = -self.heading_kp * heading_error

        angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular)
        )

        # 航向误差较大时，先原地调整方向
        if abs(heading_error) > self.rotate_in_place_angle:

            self.publish_velocity(
                linear=0.0,
                angular=angular
            )

        else:
            # 航向误差较小时允许向目标行驶。
            # 误差越大，速度越低。
            heading_factor = max(
                0.25,
                math.cos(heading_error)
            )

            linear = self.linear_speed * heading_factor

            self.publish_velocity(
                linear=linear,
                angular=angular
            )

        self.publish_status('RUNNING')

    # ==================================================
    # 销毁节点时发布零速度
    # ==================================================

    def destroy_node(self):

        try:
            self.publish_velocity()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = MultiPointNav()
        rclpy.spin(node)

    except (FileNotFoundError, ValueError) as exc:
        print(f'多点导航启动失败：{exc}')

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()