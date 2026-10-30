"""Opt-in DDS closed-loop checks; synthetic GNSS/INS only, no hardware driver."""

import json
import math
import os
from pathlib import Path as FilePath
from time import monotonic

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('AGRI_ROS_TESTS') != '1',
                                reason='Set AGRI_ROS_TESTS=1 in an isolated ROS domain')


@pytest.mark.parametrize('approach_offset', [0.0, 3.0])
def test_services_safety_and_closed_loop(tmp_path, approach_offset):
    import csv
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
    from nav_msgs.msg import Path
    from sensor_msgs.msg import NavSatFix, LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger
    from pyproj.enums import TransformDirection
    from agri_base.localization import Localization
    from agri_base.chassis_adapter import ChassisAdapter
    from agri_base.coordinates import LocalFrame, wrap_angle
    from agri_base.runtime import LATCHED
    from agri_safety.node import Safety
    from agri_waypoint.node import WaypointNode
    from agri_path_tracking.node import PathTracker

    fixture = FilePath(__file__).parent / 'fixtures'
    rclpy.init(args=['--ros-args', '--params-file', str(fixture / 'core.yaml'),
                    '-p', f'waypoints_file:={fixture / "waypoints.yaml"}',
                    '-p', f'trajectory_directory:={tmp_path}'])
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        localization, safety, chassis = Localization(), Safety(), ChassisAdapter()
        waypoint, tracker, bench = WaypointNode(), PathTracker(), Node('testbench')
        nodes = [localization, safety, chassis, waypoint, tracker, bench]
        for node in nodes:
            executor.add_node(node)
        frame = LocalFrame(36.0, 117.0)
        for node in (localization, waypoint, tracker):
            assert node.frame.identity == frame.identity
            result = node.set_parameters([Parameter('origin_latitude', value=36.1)])[0]
            assert not result.successful
            assert node.get_parameter('origin_latitude').value == 36.0
        state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 0.0, 'w': 0.0, 'sensors': True, 'front': 5.0}
        fix_pub = bench.create_publisher(NavSatFix, '/mgx23/position', 1)
        att_pub = bench.create_publisher(Vector3Stamped, '/mgx23/attitude_stamped', 1)
        path_pub = bench.create_publisher(Path, '/reference_path', LATCHED)
        scan_pub = bench.create_publisher(LaserScan, '/scan', 1)

        def command(msg):
            state['v'], state['w'] = msg.linear.x, msg.angular.z

        bench.create_subscription(Twist, '/base/driver_cmd_vel', command, 1)

        def sensors():
            state['x'] += state['v'] * math.cos(state['yaw']) * 0.05
            state['y'] += state['v'] * math.sin(state['yaw']) * 0.05
            state['yaw'] = wrap_angle(state['yaw'] + state['w'] * 0.05)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = bench.get_clock().now().to_msg(), 'laser'
            scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
            scan.angle_max = scan.angle_min + 719 * scan.angle_increment
            scan.range_min, scan.range_max = 0.05, 10.0
            ranges = [5.0] * 720
            ranges[345:376] = [state['front']] * 31
            scan.ranges = ranges
            scan_pub.publish(scan)
            if not state['sensors']:
                return
            lon, lat = frame.transformer.transform(frame.e0 + state['x'], frame.n0 + state['y'],
                                                    direction=TransformDirection.INVERSE)
            fix = NavSatFix()
            fix.header.stamp, fix.header.frame_id = bench.get_clock().now().to_msg(), 'gnss'
            fix.latitude, fix.longitude = lat, lon
            att = Vector3Stamped()
            att.header = fix.header
            att.vector.z = wrap_angle(frame.yaw(lat, lon, 0) - state['yaw'])
            att_pub.publish(att)
            fix_pub.publish(fix)

        bench.create_timer(0.05, sensors)

        def until(predicate, timeout=4):
            deadline = monotonic() + timeout
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                if predicate():
                    return
            raise AssertionError(f'Timeout: waypoint={waypoint.state}, tracker={tracker.state}, '
                                 f'gate={safety.gate.owner}/{safety.gate.reason}, pose={state}')

        def spin_for(seconds):
            deadline = monotonic() + seconds
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)

        def service(service_type, name, **kwargs):
            client = bench.create_client(service_type, name)
            until(client.service_is_ready)
            future = client.call_async(service_type.Request(**kwargs))
            until(future.done)
            result = future.result()
            bench.destroy_client(client)
            return result

        assert service(Trigger, '/system/mode/waypoint').success
        until(lambda: waypoint.healthy() and safety.healthy() and safety.lidar_ok())
        assert safety.gate.emergency_stop and state['v'] == 0
        assert service(SetBool, '/safety/emergency_stop', data=False).success
        assert service(Trigger, '/waypoint/start').success
        until(lambda: waypoint.state == 'RUNNING' and state['v'] > 0)
        assert service(Trigger, '/waypoint/pause').success
        until(lambda: state['v'] == state['w'] == 0)
        index = waypoint.index
        spin_for(0.5)
        assert waypoint.index == index and safety.gate.owner == 'waypoint'
        assert service(Trigger, '/waypoint/resume').success
        state['front'] = 0.6
        until(lambda: safety.obstacle.active and waypoint.state == 'PAUSED' and state['v'] == 0)
        assert not service(Trigger, '/waypoint/resume').success
        assert not service(Trigger, '/safety/obstacle/reset').success
        index = waypoint.index
        state['front'] = 5.0
        spin_for(1.2)
        assert waypoint.index == index and waypoint.state == 'PAUSED'
        assert service(Trigger, '/safety/obstacle/reset').success
        spin_for(0.1)
        assert state['v'] == 0 and waypoint.state == 'PAUSED'
        assert service(Trigger, '/waypoint/resume').success
        until(lambda: waypoint.state == 'COMPLETED', 15)
        until(lambda: not safety.gate.owner and state['v'] == 0)

        # Cover both a nearby path and one whose start is farther than the safety
        # path-error limit. Approach must report corridor error, not goal range.
        assert service(Trigger, '/system/mode/coverage').success
        until(tracker.healthy)
        state['yaw'] = 0.0
        path = Path()
        path.header.frame_id = 'map'
        for dx in (0, 0.3, 0.6):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = state['x'] + approach_offset + dx, state['y']
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        path_pub.publish(path)
        until(lambda: tracker.pending_path is not None)
        assert service(Trigger, '/tracking/start').success
        until(lambda: tracker.state == 'RUNNING')
        assert not service(Trigger, '/waypoint/start').success  # Different work mode cannot acquire control.
        assert safety.gate.owner == 'tracking'
        if approach_offset:
            assert tracker.controller.phase == 'APPROACHING'
            assert service(Trigger, '/tracking/pause').success
            until(lambda: state['v'] == state['w'] == 0)
            spin_for(.35)  # Paused approach still supplies fresh deviation feedback.
            assert safety.gate.owner == 'tracking' and tracker.controller.progress == 0
            assert service(Trigger, '/tracking/resume').success
            until(lambda: state['v'] > 0)
            # Localization recovery cannot resume a failed approach by itself.
            state['sensors'] = False
            until(lambda: tracker.state == 'FAULT' and not safety.gate.owner and state['v'] == 0)
            state['sensors'] = True
            until(tracker.healthy)
            spin_for(.3)
            assert not safety.gate.owner and tracker.state == 'FAULT'
            assert service(Trigger, '/tracking/start').success
            until(lambda: tracker.state == 'RUNNING')
            assert service(Trigger, '/tracking/cancel').success
            until(lambda: not safety.gate.owner and state['v'] == 0)
            spin_for(.3)
            assert tracker.state == 'CANCELED'
            assert service(Trigger, '/tracking/start').success
            until(lambda: tracker.state == 'RUNNING')
        until(lambda: tracker.state == 'COMPLETED', 25 if approach_offset else 10)
        until(lambda: state['v'] == 0 and not safety.gate.owner)
        assert len(tracker.trajectory.poses) > 3
        files = list(tmp_path.glob('track_*.csv'))
        files.sort()
        assert len(files) == (3 if approach_offset else 1)
        metadata = json.loads(files[-1].with_suffix('.json').read_text())
        assert metadata['projection'] == frame.identity
        assert metadata['frame_id'] == metadata['reference_path']['frame_id'] == 'map'
        assert metadata['position_reference'] == 'base_link'
        expected = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        assert [(p['x'], p['y']) for p in metadata['reference_path']['points']] == expected
        with files[-1].open() as stream:
            rows = list(csv.DictReader(stream))
        if approach_offset:
            approach_file = tmp_path / metadata['approach_csv']
            approach_meta = json.loads(approach_file.with_suffix('.json').read_text())
            assert approach_meta['task_type'] == 'coverage_approach'
            assert approach_meta['reference_path']['points'][0]['x'] == expected[0][0]
            with approach_file.open() as stream:
                approach_rows = list(csv.DictReader(stream))
            assert approach_rows and rows
            assert float(approach_rows[-1]['distance']) <= .15
            assert float(rows[0]['x']) >= expected[0][0] - .15
            assert len(rows) + len(approach_rows) == len(tracker.trajectory.poses)
            assert all(float(row['progress']) >= 0 for row in rows)
        else:
            assert len(rows) == len(tracker.trajectory.poses)
            for row, recorded in zip(rows, tracker.trajectory.poses):
                assert float(row['x']) == recorded.pose.position.x
                assert float(row['y']) == recorded.pose.position.y
                assert recorded.header.frame_id == 'map'

        # Localization watchdog revokes ownership and does not restart on recovery.
        assert service(Trigger, '/system/mode/waypoint').success
        until(waypoint.healthy)
        assert service(Trigger, '/waypoint/start').success
        until(lambda: waypoint.state == 'RUNNING')
        state['sensors'] = False
        until(lambda: waypoint.state == 'FAULT' and state['v'] == 0)
        state['sensors'] = True
        until(waypoint.healthy)
        spin_for(0.4)
        assert not safety.gate.owner and state['v'] == 0

        assert service(Trigger, '/waypoint/start').success
        until(lambda: waypoint.state == 'RUNNING')
        assert service(SetBool, '/safety/emergency_stop', data=True).success
        until(lambda: state['v'] == 0 and waypoint.state == 'FAULT')
        assert safety.gate.emergency_stop
        assert service(SetBool, '/safety/emergency_stop', data=False).success
        spin_for(0.3)
        assert not safety.gate.owner

        assert service(Trigger, '/waypoint/start').success
        until(lambda: waypoint.state == 'RUNNING')
        rogue = bench.create_publisher(Twist, '/cmd_vel', 1)
        rogue.publish(Twist())
        until(lambda: waypoint.state == 'FAULT' and state['v'] == 0)
        bench.destroy_publisher(rogue)
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
