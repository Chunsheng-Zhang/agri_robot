"""Explicitly isolated synthetic ROS + real OpenNav/FastAPI runtime for Web checks.

Never imports a hardware driver. Requires ROS_DOMAIN_ID in 200..230 and localhost DDS.
Run from the project root after sourcing scripts/setup_environment.bash.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
from pathlib import Path
from threading import Thread
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class SyntheticSystem:
    def __init__(self):
        assert 200 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 230, 'Use an isolated test domain'
        assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost-only DDS'
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from ament_index_python.packages import get_package_prefix
        from geometry_msgs.msg import Twist, Vector3Stamped
        from sensor_msgs.msg import NavSatFix, LaserScan
        from std_msgs.msg import Float32
        from pyproj.enums import TransformDirection
        from agri_base.localization import Localization
        from agri_base.chassis_adapter import ChassisAdapter
        from agri_base.runtime import yaw_of
        from agri_safety.node import Safety
        from agri_waypoint.node import WaypointNode
        from agri_path_tracking.node import PathTracker
        from agri_path_planning.node import CoverageAdapter
        from agri_backend import main
        from agri_backend.missions import MissionStore
        from agri_backend.devices import device_cache

        self.temporary = tempfile.TemporaryDirectory(prefix='agri-web-synthetic-')
        self.directory = Path(self.temporary.name)
        self.logs, self.processes = [], []
        self.nodes = []
        config = ROOT / 'agri_robot_ws/src/agri_bringup/config/system.yaml'
        for package, executable, extra in (
            ('opennav_coverage', 'opennav_coverage', ['--params-file', str(config)]),
            ('nav2_lifecycle_manager', 'lifecycle_manager', ['-r', '__node:=coverage_lifecycle_manager',
                '-p', 'autostart:=true', '-p', 'node_names:=[coverage_server]']),
        ):
            log = (self.directory / f'{executable}.log').open('w+')
            self.logs.append(log)
            binary = Path(get_package_prefix(package)) / 'lib' / package / executable
            self.processes.append(subprocess.Popen([str(binary), '--ros-args', *extra], stdout=log, stderr=subprocess.STDOUT))
        rclpy.init(args=['--ros-args', '--params-file', str(config), '-p', 'safety:require_lidar:=false'])
        device_cache.values.clear()
        device_cache.events.clear()
        self.localization = Localization()
        self.safety, self.chassis = Safety(), ChassisAdapter()
        self.waypoint, self.tracker, self.planner = WaypointNode(), PathTracker(), CoverageAdapter()
        self.backend, self.bench = main.RobotStatusNode(), Node('synthetic_web_testbench')
        self.nodes = [self.localization, self.safety, self.chassis, self.waypoint, self.tracker, self.planner, self.backend, self.bench]
        self.executor = SingleThreadedExecutor()
        for node in self.nodes:
            self.executor.add_node(node)
        self.state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 0.0, 'w': 0.0}
        self.bench.create_subscription(Twist, '/base/driver_cmd_vel', self.command, 1)
        fix = self.bench.create_publisher(NavSatFix, '/mgx23/position', 1)
        attitude = self.bench.create_publisher(Vector3Stamped, '/mgx23/attitude_stamped', 1)
        scan = self.bench.create_publisher(LaserScan, '/scan', 1)
        voltage = self.bench.create_publisher(Float32, '/voltage', 1)

        def sensors():
            s = self.state
            s['x'] += s['v'] * math.cos(s['yaw']) * 0.05
            s['y'] += s['v'] * math.sin(s['yaw']) * 0.05
            s['yaw'] = math.atan2(math.sin(s['yaw'] + s['w'] * 0.05), math.cos(s['yaw'] + s['w'] * 0.05))
            frame = self.localization.frame
            lon, lat = frame.transformer.transform(frame.e0 + s['x'], frame.n0 + s['y'], direction=TransformDirection.INVERSE)
            position = NavSatFix()
            position.header.stamp = self.bench.get_clock().now().to_msg()
            position.latitude, position.longitude = lat, lon
            att = Vector3Stamped()
            att.header = position.header
            heading = frame.yaw(lat, lon, 0) - s['yaw']
            att.vector.z = math.atan2(math.sin(heading), math.cos(heading))
            attitude.publish(att)
            fix.publish(position)
            laser = LaserScan()
            laser.header.stamp, laser.header.frame_id = position.header.stamp, 'laser'
            laser.angle_min, laser.angle_increment = -math.pi, math.pi / 360
            laser.angle_max = laser.angle_min + 719 * laser.angle_increment
            laser.range_min, laser.range_max = 0.05, 10.0
            laser.ranges = [5.0] * 720
            scan.publish(laser)
            voltage.publish(Float32(data=12.4))
        self.bench.create_timer(0.05, sensors)
        main.app.state.ros_bridge = SimpleNamespace(node=self.backend, started=True, error=None)
        self.workspace = main.app.state.workspace
        self.workspace.store = MissionStore(self.directory)
        self.workspace.store.config_path = config
        self.workspace.active = None
        self.workspace.synthetic = True
        self.workspace.robot_snapshot = main.robot_state.snapshot
        self.app = main.app

        @self.app.get('/api/test-environment')
        def test_environment():
            return {'synthetic': True, 'domain': int(os.environ['ROS_DOMAIN_ID']), 'hardware': False}

        @self.app.post('/api/test/align-path-start')
        def align_path_start():
            assert not self.safety.gate.owner
            assert self.tracker.pending_path is not None
            p = self.tracker.pending_path.poses[0].pose
            self.state.update(x=p.position.x, y=p.position.y, yaw=yaw_of(p.orientation), v=0.0, w=0.0)
            return {'success': True}
        self.thread = Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def command(self, msg):
        self.state['v'], self.state['w'] = msg.linear.x, msg.angular.z

    def close(self):
        import rclpy
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        for node in reversed(self.nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for log in self.logs:
            log.close()
        self.temporary.cleanup()


if __name__ == '__main__':
    import uvicorn
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=18000)
    args = parser.parse_args()
    system = SyntheticSystem()
    try:
        print(json.dumps({'synthetic': True, 'data': str(system.directory), 'port': args.port}), flush=True)
        uvicorn.run(system.app, host='127.0.0.1', port=args.port, lifespan='off', access_log=False)
    finally:
        system.close()
