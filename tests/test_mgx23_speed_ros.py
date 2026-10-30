"""Opt-in isolated DDS test of MGX23 -> localization/backend -> status API."""
import math
import os
from time import monotonic

import pytest

pytestmark = pytest.mark.skipif(os.getenv('AGRI_ROS_TESTS') != '1',
                                reason='Opt-in isolated ROS integration')


def test_mgx_speed_and_timeout_across_ros_callbacks(monkeypatch):
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 230
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import NavSatFix
    from agri_base.localization import Localization
    from agri_backend import main

    monkeypatch.setattr(main, 'robot_state', main.RobotState())
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    nodes, odoms = [], []
    try:
        loc, backend, bench = Localization(), main.RobotStatusNode(), Node('mgx_speed_testbench')
        nodes = [loc, backend, bench]
        for node in nodes:
            executor.add_node(node)
        fix_pub = bench.create_publisher(NavSatFix, '/mgx23/position', qos_profile_sensor_data)
        att_pub = bench.create_publisher(Vector3Stamped, '/mgx23/attitude_stamped', qos_profile_sensor_data)
        vel_pub = bench.create_publisher(Vector3Stamped, '/mgx23/velocity_stamped', qos_profile_sensor_data)
        bench.create_subscription(Odometry, '/odom', odoms.append, 10)
        velocity_enabled = [True]

        def sensors():
            fix, att, vel = NavSatFix(), Vector3Stamped(), Vector3Stamped()
            fix.header.stamp = bench.get_clock().now().to_msg()
            att.header = vel.header = fix.header
            fix.latitude, fix.longitude = loc.frame.latitude, loc.frame.longitude
            vel.vector.x, vel.vector.y, vel.vector.z = 0.3, -0.4, 2.0
            fix_pub.publish(fix)
            att_pub.publish(att)
            if velocity_enabled[0]:
                vel_pub.publish(vel)

        bench.create_timer(0.1, sensors)

        def until(predicate, timeout=5):
            deadline = monotonic() + timeout
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                if predicate():
                    return
            raise AssertionError('Timed out waiting for MGX23 speed/status')

        until(lambda: odoms and main.get_robot_status()['speed'] is not None)
        until(lambda: any(math.hypot(o.twist.twist.linear.x, o.twist.twist.linear.y)
                          == pytest.approx(0.5) for o in odoms))
        assert main.get_robot_status()['speed'] == pytest.approx(0.5)
        velocity_enabled[0] = False
        until(lambda: main.get_robot_status()['speed'] is None)
        assert main.get_robot_status()['online']
        assert main.get_robot_status()['speed_source'] is None
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
