#!/usr/bin/env python3
# Copyright (c) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""先沿地头开到场头，再一口气走完覆盖率路径。

和 demo_coverage.py 的区别：demo_coverage.py 走
NavigateCompleteCoverage -> BT -> ComputeCoveragePath + FollowPath，
这里绕开 BT，自己调 ComputeCoveragePath 拿路径，再把它和一段「开到场头」的
transit 拼成一条路径，一次性发给 controller_server 的 /follow_path。

为什么必须拼 transit：
  RPP 的 max_robot_pose_search_dist（demo_params.yaml 里是 10.0）限制的是
  「从路径起点算起的累计弧长」，不是到车的欧氏距离 —— 见
  nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp
  的 transformGlobalPlan():
      closest_pose_upper_bound = first_after_integrated_distance(
          begin, end, max_robot_pose_search_dist_);
      transformation_begin = min_by(begin, closest_pose_upper_bound, 距车距离);
  覆盖率路径的起点离车 150 m，而且沿路径绕 19.3 km 才经过车，所以 RPP 在前 10 m
  里根本找不到车，转换后的局部路径是 0 个点，每个控制周期都抛
  "Resulting plan has 0 poses in it."，车一步都不动。
  把 transit 拼在最前面，车就落在路径 index 0 上，这个参数不用改。

这个 demo 没有全局规划器（bt_navigator 只注册了 navigate_complete_coverage，
也没有 planner_server / global_costmap），所以用不了 NavigateToPose，
「开到场头」这段路只能自己造。

带洞地块（地块里的障碍/不作业区）：
  load_boundary_rings() 把 GeoJSON 里所有环都读出来，goal.polygons[0] 放外边界、
  polygons[1:] 每个内环一个 Coordinates；服务端 getFieldFromGoal() 对第 1 个之后的
  每个环调 field.addRing()，F2C 的作业段生成会绕开这些洞。
  之前这版只取 coordinates[0]，洞是在下发前就被丢掉的，服务端根本没机会避开。
  注意 transit 仍然是沿外边界（self.ring）走的——地头本来就在外圈，
  洞不参与 transit 选路（洞若贴着外边界，这段沿边路径可能切过洞）。

用法:
    # 常规：coverage_demo_launch.py 会自动把它拉起来，不用另开终端
    ros2 launch opennav_coverage_demo coverage_demo_launch.py
    # 想临时关掉自动跑，用 launch 参数 use_transit:=false（改回跑 demo_coverage）
    ros2 launch opennav_coverage_demo coverage_demo_launch.py use_transit:=false
    # 换路由模式（TSP 会走 OR-Tools，连接段沿地头绕）
    ros2 launch opennav_coverage_demo coverage_demo_launch.py route_mode:=TSP
    # TSP 求解时间上限太短的话可以调长
    ros2 launch opennav_coverage_demo coverage_demo_launch.py route_mode:=TSP tsp_time_limit:=5

    # 也可以单独跑
    ros2 run opennav_coverage_demo demo_coverage_via_headland
    python3 demo_coverage_via_headland.py --json <地块JSON> --no-transit
    python3 demo_coverage_via_headland.py --no-transit --route-mode TSP

被 launch 拉起时会比 gazebo / lifecycle 节点先启动，所以这里对 TF、action server、
goal 接受全部做了等待/重试，启动顺序不用关心。
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from opennav_coverage_msgs.action import ComputeCoveragePath
from shapely.geometry import LineString, Point
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULT_JSON = os.path.join(
    os.path.expanduser('~'),
    'open_navigation_ws/src/navigation_ws/results/ee_field_6_utm_boundary.json')

# transit 采样间隔，和覆盖率路径的 0.1 m 保持一致
TRANSIT_STEP = 0.1


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def ring_of(coords):
    """一组坐标 -> 闭合的 (x, y) 列表。"""
    ring = [(float(x), float(y)) for x, y in coords]
    if ring and ring[0] != ring[-1]:
        ring.append(ring[0])
    return ring


def load_boundary_rings(json_path):
    """读 GeoJSON 地块边界，返回 [外边界, 内环1, ...]，每个环都是闭合的。

    GeoJSON 的 Polygon.coordinates 是「外边界 + 洞」的列表，
    wkt_to_utm.py 生成文件时按这个顺序写，这里原样取出：
    polygons[0] 是外边界，polygons[1:] 是挖空。服务端
    getFieldFromGoal()（opennav_coverage/utils.hpp）就是这么解析的，
    每个环都要求首尾点相同，否则直接报 "polygon malformed"。

    老格式（只有 'boundary' 键、没有洞）仍然支持，返回单环列表。
    """
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    if data.get('type') == 'Feature':
        rings = [ring_of(ring) for ring in data['geometry']['coordinates']]
    elif 'boundary' in data:
        rings = [ring_of([[p['x'], p['y']] for p in data['boundary']])]
    else:
        raise ValueError(f'不支持的 JSON 格式: {json_path}')
    if not rings or not rings[0]:
        raise ValueError(f'地块边界是空的: {json_path}')
    return rings


class HeadlandTransitDemo(Node):
    def __init__(self, json_path, frame_id, route_mode='', tsp_time_limit=0):
        super().__init__('demo_coverage_via_headland')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.frame_id = frame_id
        # 空字符串 = 不设置，用服务端 default_route_type（BOUSTROPHEDON）
        self.route_mode = route_mode
        self.tsp_time_limit = tsp_time_limit

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.coverage_client = ActionClient(
            self, ComputeCoveragePath, 'compute_coverage_path')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')

        # rings[0] 是外边界，rings[1:] 是洞；transit 沿外边界走
        self.rings = load_boundary_rings(json_path)
        self.ring = LineString(self.rings[0])
        if len(self.rings) > 1:
            self.get_logger().info(
                f'地块带 {len(self.rings) - 1} 个内环（洞），随 goal.polygons 一起下发')

    # ---------- 1. 取车当前位姿 ----------

    def robot_pose(self):
        """查 map -> base_link，返回 (x, y, yaw)。

        被 launch 拉起时会比 gazebo / robot_state_publisher 先起来，所以一直等。
        """
        self.get_logger().info(f'等待 {self.frame_id} -> base_link 的 TF ...')
        waited = 0.0
        while rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame_id, 'base_link', rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return t.x, t.y, yaw
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)
                waited += 0.2
                if waited % 5.0 < 0.2:
                    self.get_logger().info(f'  ... 还没拿到 TF ({waited:.0f}s)')
        raise RuntimeError(f'拿不到 {self.frame_id} -> base_link 的 TF')

    def send_until_accepted(self, client, goal, feedback_cb=None, what=''):
        """发 goal 直到被接受。

        lifecycle 节点在 configure 完但还没 activate 时，action server 已经出现在图上
        （wait_for_server 会返回），但接下来发 goal 会被拒。所以这里重试。
        """
        waited = 0.0
        while rclpy.ok():
            send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
            handle = send_future.result()
            if handle is not None and handle.accepted:
                return handle
            time.sleep(1.0)
            waited += 1.0
            if waited % 5.0 < 1.0:
                self.get_logger().info(f'  {what} goal 还没被接受 ({waited:.0f}s)')
        raise RuntimeError(f'{what} goal 始终没被接受')

    # ---------- 2. 规划覆盖率路径 ----------

    def toCoordinates(self, field):
        from opennav_coverage_msgs.msg import Coordinate, Coordinates
        poly = Coordinates()
        for coord in field:
            pt = Coordinate()
            pt.axis1 = coord[0]
            pt.axis2 = coord[1]
            poly.coordinates.append(pt)
        return poly

    def compute_coverage(self):
        self.get_logger().info('等待 compute_coverage_path server ...')
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('  ... 还没起来')

        goal = ComputeCoveragePath.Goal()
        goal.frame_id = self.frame_id
        # 第一个是外边界，后面每个内环单独一个 Coordinates = 挖空一块
        for ring in self.rings:
            goal.polygons.append(self.toCoordinates(ring))
        goal.generate_headland = True
        goal.generate_route = True
        goal.generate_path = True
        if self.route_mode:
            goal.route_mode.mode = self.route_mode
        if self.tsp_time_limit > 0:
            goal.route_mode.tsp_time_limit = self.tsp_time_limit
        if self.route_mode:
            self.get_logger().info(
                f'路由模式: {self.route_mode}'
                + (f', TSP 时间上限 {self.tsp_time_limit}s' if self.tsp_time_limit > 0 else ''))

        self.get_logger().info('开始规划覆盖率路径（可能需要几秒）...')
        handle = self.send_until_accepted(
            self.coverage_client, goal, what='compute_coverage_path')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != ComputeCoveragePath.Result.NONE:
            raise RuntimeError(f'规划失败, error_code={result.error_code}')
        return result.nav_path

    # ---------- 3. 沿地头造 transit ----------

    def build_transit(self, robot_xyz, goal_pose):
        """从车当前位置沿地块边界走到 goal_pose，返回 PoseStamped 列表（不含 goal 本身）。

        沿边界两个方向取短的那条。地头宽度 0.25 m，路径起点离边界也只有 0.25 m，
        所以贴着边界走就等价于沿地头走。
        """
        rx, ry, _ = robot_xyz
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        ring = self.ring
        total = ring.length
        s_robot = ring.project(Point(rx, ry))
        s_goal = ring.project(Point(gx, gy))

        fwd = (s_goal - s_robot) % total
        bwd = total - fwd
        if fwd <= bwd:
            dist, sign = fwd, 1.0
        else:
            dist, sign = bwd, -1.0

        samples = []
        d = 0.0
        while d < dist:
            samples.append(ring.interpolate((s_robot + sign * d) % total))
            d += TRANSIT_STEP
        samples.append(ring.interpolate(s_goal))

        pts = [(p.x, p.y) for p in samples]
        # 起点就是车自己的位置，保证 RPP 在路径 index 0 就能找到车
        pts.insert(0, (rx, ry))
        # 收尾：从边界上的投影点切到真正的路径起点
        pts.append((gx, gy))

        return pts, dist

    @staticmethod
    def build_path(pts, frame_id, stamp):
        """按行进方向给每个点算朝向。"""
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp
        for i, (x, y) in enumerate(pts):
            if i + 1 < len(pts):
                nx, ny = pts[i + 1]
            else:
                nx, ny = pts[i - 1]
                x, y, nx, ny = nx, ny, x, y   # 末点朝向来路反方向
            yaw = math.atan2(ny - y, nx - x)
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(ps)
        return path

    # ---------- 4. 下发合并后的路径 ----------

    def follow(self, path):
        self.get_logger().info('等待 follow_path server ...')
        while not self.follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('  ... 还没起来')

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'

        handle = self.send_until_accepted(
            self.follow_client, goal, feedback_cb=self.on_feedback, what='follow_path')

        self.goal_handle = handle
        self.get_logger().info('goal 已接受，开始跟路径...')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status

    def on_feedback(self, msg):
        self.fb = msg.feedback
        now = time.time()
        if now - getattr(self, '_last_fb', 0.0) >= 5.0:
            self._last_fb = now
            self.get_logger().info(
                f'  剩余 {self.fb.distance_to_goal:7.1f} m, 速度 {self.fb.speed:.2f} m/s')

    def cancel(self):
        h = getattr(self, 'goal_handle', None)
        if h is not None:
            self.get_logger().warn('取消 follow_path goal ...')
            h.cancel_goal_async()
            for _ in range(30):
                rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(description='先沿地头开到覆盖率路径起点，再走完覆盖')
    parser.add_argument('--json', default=DEFAULT_JSON, help='地块边界 JSON')
    parser.add_argument('--frame', default='map', help='坐标系 (默认 map)')
    parser.add_argument('--no-transit', action='store_true',
                        help='不拼 transit，直接发覆盖率路径（对照用）')
    parser.add_argument('--route-mode', default='',
                        help='路由模式: BOUSTROPHEDON / SNAKE / SPIRAL / TSP；'
                             '留空用服务端 default_route_type')
    parser.add_argument('--tsp-time-limit', type=int, default=0,
                        help='TSP 求解时间上限（秒），0 = 用 msgs 里的默认值 1')
    # 用 parse_known_args：被 launch 或 ros2 run 拉起时会带上 --ros-args 等参数，
    # 直接 parse_args 会因为不认识而报错退出
    args, _ = parser.parse_known_args()

    if not os.path.exists(args.json):
        print(f'错误：找不到地块 JSON {args.json}')
        sys.exit(1)

    rclpy.init()
    node = HeadlandTransitDemo(
        args.json, args.frame, args.route_mode, args.tsp_time_limit)
    try:
        rx, ry, ryaw = node.robot_pose()
        node.get_logger().info(
            f'车当前位置: ({rx:.3f}, {ry:.3f}), yaw={math.degrees(ryaw):.1f}°')

        nav_path = node.compute_coverage()
        if not nav_path.poses:
            raise RuntimeError('覆盖率路径是空的')
        start = nav_path.poses[0].pose.position
        node.get_logger().info(
            f'覆盖率路径: {len(nav_path.poses)} 个点, '
            f'起点 ({start.x:.2f}, {start.y:.2f})')

        stamp = node.get_clock().now().to_msg()
        if args.no_transit:
            merged = nav_path
            merged.header.frame_id = args.frame
            merged.header.stamp = stamp
        else:
            pts, ring_dist = node.build_transit((rx, ry, ryaw), nav_path.poses[0])
            transit = node.build_path(pts, args.frame, stamp)
            node.get_logger().info(
                f'transit: {len(transit.poses)} 个点, 沿地头 {ring_dist:.1f} m')

            merged = Path()
            merged.header.frame_id = args.frame
            merged.header.stamp = stamp
            merged.poses = transit.poses + list(nav_path.poses)
            node.get_logger().info(
                f'合并后路径: {len(merged.poses)} 个点 '
                f'(约 {(len(merged.poses) - 1) * 0.1 / 1000:.1f} km)')

        status = node.follow(merged)
        if status == 4:      # STATUS_SUCCEEDED
            node.get_logger().info('✅ 覆盖任务完成')
        elif status == 5:    # STATUS_CANCELED
            node.get_logger().warn('任务被取消')
        else:
            node.get_logger().error(f'❌ 任务失败, status={status}')
    except KeyboardInterrupt:
        node.cancel()
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f'出错: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
