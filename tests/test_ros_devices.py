"""Opt-in synthetic devices; calls original vendor joystick logic without hardware."""

import math
import os
from pathlib import Path
from time import monotonic

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('AGRI_ROS_TESTS') != '1', reason='Opt-in isolated DDS test')


def test_joystick_scan_camera_and_stop_chain(tmp_path):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TwistStamped
    from sensor_msgs.msg import Joy, LaserScan, Image
    from std_msgs.msg import Int32, Bool
    from std_srvs.srv import SetBool, Trigger
    from agri_base.joy_adapter import make_adapter
    from agri_base.device_status import CameraStatus
    from agri_base.chassis_adapter import ChassisAdapter
    from agri_safety.node import Safety

    fixture = Path(__file__).parent / 'fixtures/devices.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(fixture),
                    '-p', f'event_directory:={tmp_path}'])
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        safety, adapter, chassis, camera, bench = Safety(), make_adapter(), ChassisAdapter(), CameraStatus(), Node('devices_bench')
        assert safety.get('joy_estop_button') == adapter.estop_index == 6
        assert adapter.reset_index == 8
        assert safety.gate.max_linear == safety.gate.max_lateral == 1.0
        assert safety.gate.max_angular == 5.0
        assert (adapter.xspeed_limit, adapter.yspeed_limit, adapter.angular_speed_limit) == (1.0, 1.0, 5.0)
        nodes = [safety, adapter, chassis, camera, bench]
        for node in nodes:
            executor.add_node(node)
        state = {'axes': [0.0] * 6, 'buttons': [0] * 15,
                 'joy': True, 'scan': True, 'image': True, 'front': 5.0, 'rear': 5.0,
                 'v': 0.0, 'vy': 0.0, 'w': 0.0}
        joy_pub = bench.create_publisher(Joy, '/joy', 1)
        scan_pub = bench.create_publisher(LaserScan, '/scan', 1)
        image_pub = bench.create_publisher(Image, '/camera/color/image_raw', 1)
        lights, buzzers = [], []
        bench.create_subscription(Int32, '/RGBLight', lambda m: lights.append(m.data), 1)
        bench.create_subscription(Bool, '/Buzzer', lambda m: buzzers.append(m.data), 1)
        def command(msg):
            state['v'], state['vy'], state['w'] = msg.linear.x, msg.linear.y, msg.angular.z
        bench.create_subscription(Twist, '/base/driver_cmd_vel', command, 1)

        def sensors():
            stamp = bench.get_clock().now().to_msg()
            if state['joy']:
                joy = Joy()
                joy.header.stamp, joy.axes, joy.buttons = stamp, state['axes'], state['buttons']
                joy_pub.publish(joy)
            if state['scan']:
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, 'laser'
                scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
                scan.angle_max = scan.angle_min + 719 * scan.angle_increment
                scan.range_min, scan.range_max = 0.05, 10.0
                values = [5.0] * 720
                values[360], values[0] = state['front'], state['rear']
                scan.ranges = values
                scan_pub.publish(scan)
            if state['image']:
                image = Image()
                image.header.stamp = stamp
                image.width, image.height, image.encoding, image.step = 8, 8, 'bgr8', 24
                image.data = bytes(192)
                image_pub.publish(image)
        bench.create_timer(0.05, sensors)

        def until(predicate, timeout=4):
            deadline = monotonic() + timeout
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                if predicate():
                    return
            raise AssertionError(f'{safety.gate.owner}/{safety.gate.reason}, obstacle={safety.obstacle.active}, '
                                 f'joy={safety.joystick_ok()}, state={state}')
        def spin(seconds):
            deadline = monotonic() + seconds
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
        def service(kind, name, **kwargs):
            client = bench.create_client(kind, name)
            until(client.service_is_ready)
            future = client.call_async(kind.Request(**kwargs))
            until(future.done)
            result = future.result()
            bench.destroy_client(client)
            return result
        def zero():
            return state['v'] == state['vy'] == state['w'] == 0
        def arm():
            state['axes'] = [0.0] * 6
            spin(0.15)
            adapter.cancel_time = 0  # Bypass only the original 1 s UI debounce in this test.
            state['buttons'][9] = 1
            until(lambda: safety.gate.owner == 'manual' and adapter.Joy_active)
            state['buttons'][9] = 0
            spin(0.1)

        until(lambda: safety.joystick_ok() and safety.lidar_ok() and camera.frames > 0)
        assert not safety.localization_ok()  # Manual control must not depend on GNSS.
        assert service(SetBool, '/safety/emergency_stop', data=False).success
        arm()
        state['axes'][1], state['axes'][0] = 0.5, 0.3
        until(lambda: state['v'] > 0 and state['vy'] > 0)
        # The old stop index is again an unused vendor button, not a stop alias.
        state['buttons'][0] = 1
        spin(.2)
        assert not safety.gate.emergency_stop and safety.gate.owner == 'manual'
        state['buttons'][0] = 0
        # Camera loss reports offline without interrupting motion.
        state['image'] = False
        spin(1.1)
        assert monotonic() - camera.received > camera.timeout
        assert state['v'] > 0 and state['vy'] > 0
        state['image'] = True
        # Original auxiliary buttons and gear toggles are still active.
        state['buttons'][7], state['buttons'][11], state['buttons'][13], state['buttons'][14] = 1, 1, 1, 1
        until(lambda: bool(lights) and bool(buzzers) and adapter.linear_Gear != 1 and adapter.angular_Gear != 1)
        gear, angular_gear = adapter.linear_Gear, adapter.angular_Gear
        spin(0.3)
        assert adapter.linear_Gear == gear  # Autorepeat must not repeatedly change gear.
        assert adapter.angular_Gear == angular_gear
        state['buttons'][7], state['buttons'][11], state['buttons'][13], state['buttons'][14] = 0, 0, 0, 0

        # Estop release does not unlock. Reset with deflected stick must be rejected.
        state['buttons'][6] = 1
        until(lambda: safety.gate.emergency_stop and zero())
        state['buttons'][6] = 0
        spin(0.2)
        assert safety.gate.emergency_stop
        assert not service(SetBool, '/safety/emergency_stop', data=False).success
        state['buttons'][8] = 1
        spin(1.1)
        assert safety.gate.emergency_stop and zero()  # Deflected-stick reset is rejected.
        state['buttons'][8] = 0
        state['axes'] = [0.0] * 6
        spin(0.15)
        # The old reset index no longer unlocks, even when held with neutral sticks.
        state['buttons'][3] = 1
        spin(1.1)
        assert safety.gate.emergency_stop and zero()
        state['buttons'][3] = 0
        # Simultaneous stop/reset always keeps stop latched.
        state['buttons'][6], state['buttons'][8] = 1, 1
        spin(1.1)
        assert safety.gate.emergency_stop and zero()
        state['buttons'][6] = 0
        # The new hold-to-reset button is an explicit request, never a motion enable.
        until(lambda: not safety.gate.emergency_stop, 3)
        state['buttons'][8] = 0
        assert not safety.gate.owner and zero()
        arm()
        state['axes'][1] = 0.5
        until(lambda: state['v'] > 0)
        # A close rear return does not stop forward travel; front return does.
        state['rear'] = 0.5
        spin(0.2)
        assert state['v'] > 0 and not safety.obstacle.active
        state['front'] = 0.6
        until(lambda: safety.obstacle.active and zero())
        state['front'], state['rear'] = 5.0, 5.0
        spin(0.5)
        assert safety.obstacle.active and zero()
        assert service(Trigger, '/safety/obstacle/reset').success
        spin(0.1)
        assert not safety.gate.owner and zero()
        arm()
        state['axes'][1] = -0.5
        until(lambda: state['v'] < 0)
        state['rear'] = 0.6
        until(lambda: safety.obstacle.active and zero())
        state['rear'] = 5.0
        spin(0.5)
        assert service(Trigger, '/safety/obstacle/reset').success
        arm()
        state['axes'][1] = 0.5
        until(lambda: state['v'] > 0)
        state['scan'] = False
        until(lambda: safety.obstacle.active and zero())
        state['scan'] = True
        spin(0.5)
        assert service(Trigger, '/safety/obstacle/reset').success
        arm()
        state['axes'][1] = 0.5
        until(lambda: state['v'] > 0)
        state['joy'] = False
        until(lambda: safety.gate.emergency_stop and zero())
        state['joy'], state['axes'] = True, [0.0] * 6
        spin(0.3)
        assert safety.gate.emergency_stop and not safety.gate.owner
        assert list(tmp_path.glob('safety_*.jsonl'))
        assert 'OBSTACLE_STOP' in next(tmp_path.glob('safety_*.jsonl')).read_text()
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
