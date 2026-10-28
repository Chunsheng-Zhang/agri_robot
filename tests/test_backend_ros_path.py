"""Opt-in DDS test, isolated from the robot's ROS domain; no hardware commands."""
import os
import sys
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    os.getenv('AGRI_RUN_ROS_BRIDGE_TEST') != '1', reason='opt-in isolated DDS test')


def test_bridge_receives_plan_published_before_it_started(monkeypatch):
    assert 200 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 230, 'Use an isolated test domain'
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from agri_backend import main
    monkeypatch.setattr(main, 'robot_state', main.RobotState())
    rclpy.init()
    node = rclpy.create_node('isolated_reference_fixture')
    executor = SingleThreadedExecutor()
    bridge = None
    try:
        publisher = node.create_publisher(RosPath, '/reference_path', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        msg = RosPath()
        msg.header.frame_id = 'map'
        point = PoseStamped()
        point.pose.position.x = 12.3
        msg.poses = [point]
        publisher.publish(msg)  # Deliberately before the backend exists. No republish.
        bridge = main.RobotStatusNode()
        executor.add_node(node)
        executor.add_node(bridge)
        deadline = time.monotonic() + 8
        while not main.robot_state.snapshot()['reference_path'] and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert main.robot_state.snapshot()['reference_path'] == [{'x': 12.3, 'y': 0.0}]
        # Then replace with the legacy volatile publisher: backend must stay compatible.
        node.destroy_publisher(publisher)
        publisher = node.create_publisher(RosPath, '/reference_path', 10)
        msg.poses[0].pose.position.x = 99.0
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            publisher.publish(msg)
            executor.spin_once(timeout_sec=0.1)
            if main.robot_state.snapshot()['reference_path'][0]['x'] == 99.0:
                break
        assert main.robot_state.snapshot()['reference_path'][0]['x'] == 99.0
    finally:
        executor.shutdown()
        if bridge is not None:
            bridge.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
