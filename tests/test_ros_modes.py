"""Isolated mode/lease races and legacy CSV path QoS. No hardware nodes."""
import os
from time import monotonic

import pytest

pytestmark = pytest.mark.skipif(os.getenv('AGRI_ROS_TESTS') != '1', reason='Opt-in isolated DDS tests')


def spin_until(executor, predicate, timeout=4):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return
    raise AssertionError('Isolated ROS condition timed out')


def test_switch_rejects_stale_acquisition_release_and_velocity():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist, TwistStamped
    from std_srvs.srv import Trigger, SetBool
    from agri_interfaces.srv import AcquireControl
    from agri_safety.node import Safety
    rclpy.init(args=['--ros-args', '-p', 'safety:require_lidar:=false', '-p', 'safety:command_timeout:=2.0'])
    safety, bench = Safety(), Node('mode_race_testbench')
    executor = SingleThreadedExecutor()
    executor.add_node(safety); executor.add_node(bench)
    outputs = []
    bench.create_subscription(Twist, '/cmd_vel', lambda msg: outputs.append(msg.linear.x), 1)
    publisher = bench.create_publisher(TwistStamped, '/control/web/cmd_vel', 1)

    def call(kind, name, **values):
        client = bench.create_client(kind, name)
        spin_until(executor, client.service_is_ready)
        future = client.call_async(kind.Request(**values))
        spin_until(executor, future.done)
        result = future.result()
        bench.destroy_client(client)
        return result

    def lease(enable, revision, generation):
        return call(AcquireControl, '/safety/acquire_control', source='web', enable=enable,
                    mode_revision=revision, task_generation=generation)
    try:
        spin_until(executor, lambda: safety.count_publishers('/cmd_vel') == 1)
        assert call(SetBool, '/safety/emergency_stop', data=False).success
        assert lease(True, 0, 1).success
        old_stamp = bench.get_clock().now().to_msg()
        assert call(Trigger, '/system/mode/waypoint').success
        assert not lease(True, 0, 1).success
        assert call(Trigger, '/system/mode/base').success
        assert lease(True, 2, 2).success
        assert not lease(False, 0, 1).success
        assert not lease(False, 2, 1).success
        assert safety.gate.owner == 'web'
        stale = TwistStamped()
        stale.header.frame_id, stale.header.stamp = 'base_link', old_stamp
        stale.twist.linear.x = 0.2
        publisher.publish(stale)
        for _ in range(4):
            executor.spin_once(timeout_sec=0.01)
        assert not safety.gate.samples
        fresh = TwistStamped()
        fresh.header.frame_id, fresh.header.stamp = 'base_link', bench.get_clock().now().to_msg()
        fresh.twist.linear.x = 0.1
        publisher.publish(fresh)
        spin_until(executor, lambda: outputs and outputs[-1] > 0)
        assert call(Trigger, '/system/mode/coverage').success
        spin_until(executor, lambda: outputs[-1] == 0)
        assert call(Trigger, '/system/mode/base').success
        assert not safety.gate.owner and outputs[-1] == 0
    finally:
        executor.shutdown()
        bench.destroy_node(); safety.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('transient', [False, True])
def test_tracker_accepts_legacy_volatile_and_latched_paths_but_rejects_conflicts(transient):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from agri_path_tracking.node import PathTracker
    rclpy.init()
    bench = Node('path_qos_testbench')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
    publisher = bench.create_publisher(Path, '/reference_path', qos)
    path = Path()
    path.header.frame_id = 'map'
    for x in (0.0, 1.0):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.orientation.w = x, 1.0
        path.poses.append(pose)
    publisher.publish(path)  # Latched case is deliberately never republished.
    tracker = PathTracker()
    executor = SingleThreadedExecutor()
    executor.add_node(bench); executor.add_node(tracker)
    if not transient:
        bench.create_timer(0.05, lambda: publisher.publish(path))
    try:
        spin_until(executor, lambda: tracker.pending_path is not None)
        assert len(tracker.pending_path.poses) == 2
        other = bench.create_publisher(Path, '/reference_path', 1)
        spin_until(executor, lambda: tracker.count_publishers('/reference_path') == 2)
        tracker.refresh_path_qos()
        assert tracker.pending_path is None
        with pytest.raises(ValueError, match='exactly one'):
            tracker.prepare()
        bench.destroy_publisher(other)
    finally:
        executor.shutdown()
        tracker.destroy_node(); bench.destroy_node(); rclpy.shutdown()
