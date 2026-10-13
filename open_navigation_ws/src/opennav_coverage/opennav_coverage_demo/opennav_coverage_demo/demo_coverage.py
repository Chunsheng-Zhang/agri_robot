#! /usr/bin/env python3
# Copyright 2023 Open Navigation LLC
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

from enum import Enum
import json
import os
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageNavigatorTester(Node):

    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = coord[0]
            pt.y = coord[1]
            poly.points.append(pt)
        return poly

    def loadFieldFromJSON(self, json_path):
        """从 JSON 文件读取地块边界环（GeoJSON 格式）。

        返回 [外边界, 内环1, ...]：GeoJSON 的 Polygon.coordinates 就是
        「外边界 + 洞」的列表。之前只取 coordinates[0]，带洞地块的洞在
        下发前就丢了。老格式（只有 'boundary' 键）当作单环处理。
        """
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 支持 GeoJSON Polygon 格式
        if data.get('type') == 'Feature':
            rings = data['geometry']['coordinates']
        # 支持简化格式
        elif 'boundary' in data:
            rings = [[[p['x'], p['y']] for p in data['boundary']]]
        else:
            raise ValueError("不支持的 JSON 格式")

        out = []
        for ring in rings:
            pts = [[float(x), float(y)] for x, y in ring]
            if pts and pts[0] != pts[-1]:
                pts.append(pts[0])   # 服务端要求每个环首尾点相同
            out.append(pts)
        return out

    def navigateCoverage(self, field):
        """Send a `NavToPose` action request."""
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        # 第一个是外边界，后面每个内环单独一个 Polygon = 挖空一块
        # （navigator 原样放进黑板，BT 节点逐个转成 ComputeCoveragePath 的
        #  polygons[i]，服务端对 i>=1 的每个环 field.addRing()）
        for ring in field:
            goal_msg.polygons.append(self.toPolygon(ring))

        print('Navigating to with field of size: ' + str(len(field)) + '...')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Navigate Coverage request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        print('Task succeeded!')
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='bt_navigator'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return


def main():
    rclpy.init()

    navigator = CoverageNavigatorTester()
    navigator.startup()

    # 从 JSON 文件读取地块边界点
    json_path = os.path.join(
        os.path.expanduser('~'),
        'open_navigation_ws/src/navigation_ws/results/ee_field_6_utm_boundary.json'
    )
    if not os.path.exists(json_path):
        print(f'错误：找不到 JSON 文件 {json_path}')
        return

    field = navigator.loadFieldFromJSON(json_path)
    print(f'已从 JSON 加载 {len(field)} 个边界点')
    navigator.navigateCoverage(field)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        time.sleep(1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


if __name__ == '__main__':
    main()
