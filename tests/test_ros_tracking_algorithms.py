"""Opt-in DDS tests with synthetic pose only, no drivers, serial ports or real domain."""
import json
import math
import os
from time import monotonic

import pytest

pytestmark = pytest.mark.skipif(os.getenv('AGRI_ROS_TESTS') != '1', reason='Explicit isolated DDS test')


def test_four_algorithms_ros_parameters_safety_pause_resume_and_recording(tmp_path, monkeypatch):
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 230
    assert os.environ['ROS_DOMAIN_ID'] != '225' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger
    from agri_base.runtime import LATCHED
    from agri_path_tracking.node import PathTracker
    from agri_safety.node import Safety

    rclpy.init(args=['--ros-args', '-p', 'require_lidar:=false', '-p', 'require_joy:=false',
                    '-p', 'max_linear_speed:=1.0'])
    executor = SingleThreadedExecutor()
    nodes = []
    state = dict(x=0., y=0., yaw=0., v=0., w=0.)
    try:
        safety, tracker, follower, bench = Safety(), PathTracker(), PathTracker(following=True), Node('algorithm_testbench')
        nodes = [safety, tracker, follower, bench]
        for node in nodes:
            executor.add_node(node)
        pose_pub = bench.create_publisher(Odometry, '/odom', 1)
        valid_pub = bench.create_publisher(Bool, '/localization/valid', 1)
        origin_pub = bench.create_publisher(String, '/localization/origin', LATCHED)
        path_pubs = {source: bench.create_publisher(Path, topic, LATCHED)
                     for source, topic in (('tracking', '/reference_path'), ('following', '/following/reference_path'))}
        bench.create_subscription(Twist, '/cmd_vel', lambda m: state.update(v=m.linear.x, w=m.angular.z), 1)

        def sensors():
            state['x'] += state['v']*math.cos(state['yaw'])*.02
            state['y'] += state['v']*math.sin(state['yaw'])*.02
            state['yaw'] += state['w']*.02
            msg = Odometry()
            msg.header.stamp = bench.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
            msg.pose.pose.position.x, msg.pose.pose.position.y = state['x'], state['y']
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(state['yaw']/2), math.cos(state['yaw']/2)
            msg.twist.twist.linear.x = state['v']
            pose_pub.publish(msg)
            valid_pub.publish(Bool(data=True))
            origin_pub.publish(String(data=json.dumps(tracker.frame.identity)))
        bench.create_timer(.02, sensors)

        def until(predicate, timeout=8.):
            end = monotonic()+timeout
            while monotonic() < end:
                executor.spin_once(timeout_sec=.005)
                if predicate():
                    return
            raise AssertionError(f'Timeout: {state}; tracker={tracker.state}/{tracker.reason}; safety={safety.gate.reason}')

        def service(kind, name, request):
            client = bench.create_client(kind, name)
            try:
                until(client.service_is_ready)
                future = client.call_async(request)
                until(future.done)
                return future.result()
            finally:
                bench.destroy_client(client)

        def configure(node, algorithm, directory):
            values = dict(controller_algorithm=algorithm, speed=.5, completion_tolerance=.1,
                          trajectory_directory=str(directory))
            request = SetParametersAtomically.Request(parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in values.items()])
            return service(SetParametersAtomically, f'/{node.get_name()}/set_parameters_atomically', request).result

        until(lambda: tracker.frame_match and follower.frame_match and safety.localization_ok())
        for source, node in (('tracking', tracker), ('following', follower)):
            mode = 'coverage' if source == 'tracking' else 'following'
            assert service(Trigger, f'/system/mode/{mode}', Trigger.Request()).success
            until(node.healthy)
            assert service(SetBool, '/safety/emergency_stop', SetBool.Request(data=False)).success
            for algorithm in ('pp', 'stanley', 'lqr', 'mpc'):
                until(lambda: not safety.gate.owner and state['v'] == state['w'] == 0.)
                state.update(x=0., y=0., yaw=0.)
                until(lambda: abs(node.odom.pose.pose.position.x) < .01)
                directory = tmp_path/source/algorithm
                assert configure(node, algorithm, directory).successful
                path = Path()
                path.header.frame_id = 'map'
                path.header.stamp = bench.get_clock().now().to_msg()
                for x in (0., .2, .4, .6):
                    p = PoseStamped()
                    p.header = path.header
                    p.pose.position.x = x
                    p.pose.orientation.w = 1.
                    path.poses.append(p)
                path_pubs[source].publish(path)
                until(lambda: node.pending_path is not None)
                assert service(Trigger, f'/{source}/start', Trigger.Request()).success
                until(lambda: node.state == 'RUNNING' and state['v'] > 0)
                assert node.controller.tracker.algorithm_id == algorithm
                assert not configure(node, 'stanley' if algorithm == 'pp' else 'pp', directory).successful
                assert service(Trigger, f'/{source}/pause', Trigger.Request()).success
                until(lambda: state['v'] == state['w'] == 0.)
                assert not configure(node, 'stanley' if algorithm == 'pp' else 'pp', directory).successful
                assert service(Trigger, f'/{source}/resume', Trigger.Request()).success
                until(lambda: node.state == 'COMPLETED', 10.)
                until(lambda: not safety.gate.owner and state['v'] == state['w'] == 0.)
                documents = [json.loads(p.read_text()) for p in directory.glob('track_*.json')]
                assert len(documents) == 1 and documents[0]['tracking_controller']['algorithm'] == algorithm
        # Invalid or removed choices are rejected while idle too.
        assert not configure(tracker, 'legacy_pp', tmp_path/'invalid').successful
        # Inject numerical faults after the common alignment. Every case must
        # release the lease and publish zero, without using a fallback algorithm.
        import time
        import numpy as np
        from agri_path_tracking import algorithms
        for fault in ('nonconverged', 'nonfinite', 'singular', 'overrun'):
            state.update(x=0., y=0., yaw=0.)
            until(lambda: abs(follower.odom.pose.pose.position.x) < .01)
            directory = tmp_path/'faults'/fault
            assert configure(follower, 'mpc', directory).successful
            with monkeypatch.context() as patch:
                if fault == 'nonconverged':
                    patch.setattr(algorithms, 'box_qp', lambda *a, **kw: (np.zeros(20), False, 200))
                elif fault == 'nonfinite':
                    patch.setattr(algorithms.RobotController, 'command', lambda *a, **kw: (float('nan'),0.,0.,0.,0.,False))
                elif fault == 'singular':
                    def singular(*args, **kwargs):
                        raise np.linalg.LinAlgError('injected singular matrix')
                    patch.setattr(algorithms, 'box_qp', singular)
                else:
                    def slow(*args, **kwargs):
                        time.sleep(.21)
                        return .5,0.,0.,0.,0.,False
                    patch.setattr(algorithms.RobotController, 'command', slow)
                assert service(Trigger, '/following/start', Trigger.Request()).success
                until(lambda: follower.state == 'FAULT')
                until(lambda: not safety.gate.owner and state['v'] == state['w'] == 0.)
                assert follower.controller.tracker.algorithm_id == 'mpc'
                assert abs(state['x']) < .01
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
