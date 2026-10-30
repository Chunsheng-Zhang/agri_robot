"""Isolated HTTP -> ROS obstacle policy -> chassis stop integration."""
import json
import math
import os
from threading import Thread
from time import monotonic, sleep
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.skipif(os.getenv('AGRI_ROS_TESTS') != '1',
                                reason='Opt-in isolated ROS integration')


def test_runtime_lidar_policy_and_no_automatic_motion(monkeypatch):
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 230
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from geometry_msgs.msg import Twist, TwistStamped
    from sensor_msgs.msg import Joy, LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from agri_interfaces.srv import AcquireControl
    from agri_safety.node import Safety
    from agri_base.chassis_adapter import ChassisAdapter
    from agri_backend import devices, main, ros_workspace

    cache = devices.DeviceCache()
    for module in (devices, main, ros_workspace):
        monkeypatch.setattr(module, 'device_cache', cache)
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    nodes, thread, client = [], None, None
    try:
        safety, chassis, backend, bench = Safety(), ChassisAdapter(), main.RobotStatusNode(), Node('lidar_policy_bench')
        nodes = [safety, chassis, backend, bench]
        for node in nodes:
            executor.add_node(node)
        state = {'scan': True, 'front': 0.4, 'v': 0.0}
        scan_pub = bench.create_publisher(LaserScan, '/scan', 1)
        joy_pub = bench.create_publisher(Joy, '/joy', 1)
        manual_pub = bench.create_publisher(String, '/manual/status', 1)
        command_pub = bench.create_publisher(TwistStamped, '/control/manual/cmd_vel', 1)
        bench.create_subscription(Twist, '/base/driver_cmd_vel', lambda m: state.update(v=m.linear.x), 1)

        def inputs():
            stamp = bench.get_clock().now().to_msg()
            joy = Joy()
            joy.header.stamp, joy.axes, joy.buttons = stamp, [0.0] * 3, [0] * 15
            joy_pub.publish(joy)
            manual_pub.publish(String(data=json.dumps({'stamp': stamp.sec + stamp.nanosec / 1e9, 'connected': True})))
            if state['scan']:
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, 'laser'
                scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
                scan.angle_max = scan.angle_min + 719 * scan.angle_increment
                scan.range_min, scan.range_max = 0.05, 10.0
                ranges = [5.0] * 720
                ranges[360] = state['front']
                scan.ranges = ranges
                scan_pub.publish(scan)
            # Keep emitting old commands: changing policy must still release control.
            cmd = TwistStamped()
            cmd.header.stamp, cmd.header.frame_id = stamp, 'base_link'
            cmd.twist.linear.x = 0.2
            command_pub.publish(cmd)

        bench.create_timer(.05, inputs)
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()
        app = FastAPI()
        devices.install_routes(app, lambda: SimpleNamespace(node=backend))
        client = TestClient(app)

        def until(predicate, timeout=5):
            deadline = monotonic() + timeout
            while monotonic() < deadline:
                if predicate():
                    return
                sleep(.02)
            raise AssertionError(f'Timeout: {safety.gate.owner}/{safety.gate.reason}; {state}')

        def service(kind, name, **kwargs):
            rpc = bench.create_client(kind, name)
            until(rpc.service_is_ready)
            future = rpc.call_async(kind.Request(**kwargs))
            until(future.done)
            result = future.result()
            bench.destroy_client(rpc)
            return result

        def toggle(enabled):
            result = client.post('/api/robot/lidar-protection', json={'enabled': enabled})
            assert result.status_code == 200, result.text
            until(lambda: client.get('/api/devices').json()['safety'].get('lidar_required') is enabled)
            return result.json()

        def arm():
            assert service(SetBool, '/safety/manual/enable', data=True).success
            until(lambda: state['v'] > 0)

        until(lambda: safety.joystick_ok() and safety.lidar_ok())
        assert safety.get('require_lidar') is False and not safety.lidar_required
        assert service(SetBool, '/safety/emergency_stop', data=False).success
        arm()
        assert not safety.obstacle.active  # Close obstacle ignored while disabled.
        old_revision = safety.mode_revision
        toggle(True)
        until(lambda: state['v'] == 0 and not safety.gate.owner)
        assert safety.mode_revision == old_revision + 1
        stale = service(AcquireControl, '/safety/acquire_control', source='manual', enable=True,
                        mode_revision=old_revision, task_generation=1)
        assert not stale.success  # Queued requests cannot restore previous motion.
        assert service(SetBool, '/safety/manual/enable', data=True).success
        until(lambda: safety.obstacle.active and state['v'] == 0)
        toggle(False)
        assert not safety.obstacle.active and not safety.gate.owner and not safety.gate.emergency_stop
        sleep(.25)
        assert state['v'] == 0
        arm()
        # Repeated identical requests are idempotent and do not stop a new task.
        revision = safety.mode_revision
        toggle(False)
        assert safety.gate.owner == 'manual' and safety.mode_revision == revision
        state['front'] = 5.0
        toggle(True)
        until(lambda: state['v'] == 0 and not safety.gate.owner)
        until(lambda: safety.geometry.global_distance is not None and safety.geometry.global_distance > 4)
        arm()
        state['scan'] = False
        until(lambda: safety.obstacle.active and state['v'] == 0)
        toggle(False)
        arm()  # Disabled policy permits motion without scan data.
        toggle(True)
        until(lambda: state['v'] == 0 and not safety.gate.owner)
        assert not service(SetBool, '/safety/manual/enable', data=True).success
        assert client.get('/api/devices').json()['safety']['lidar_required']
        # Changing protection must never reset a separate software emergency stop.
        assert service(SetBool, '/safety/emergency_stop', data=True).success
        toggle(False)
        assert safety.gate.emergency_stop and state['v'] == 0
    finally:
        if client:
            client.close()
        executor.shutdown()
        if thread:
            thread.join(timeout=3)
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
