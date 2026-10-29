import math
from pathlib import Path
from time import monotonic

import yaml

from agri_base.coordinates import finite_positive, wrap_angle
from agri_base.runtime import TaskNode, run, yaw_of
from agri_base.runtime import LATCHED
from agri_base.recording import open_recording
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from .controller import WaypointController


class WaypointNode(TaskNode):
    def __init__(self):
        super().__init__('waypoint', 'waypoint')
        defaults = {'waypoints_file': '', 'max_linear': 0.2, 'min_linear': 0.03,
                    'max_angular': 0.5, 'arrival_tolerance': 0.2, 'heading_kp': 1.0,
                    'distance_kp': 0.5, 'rotate_threshold': 1.0, 'align_threshold': 0.2,
                    'dwell_time': 2.0, 'control_rate': 20.0, 'trajectory_directory': ''}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.controller = WaypointController(**{name: self.get_parameter(name).value for name in (
            'max_linear', 'min_linear', 'max_angular', 'arrival_tolerance', 'heading_kp',
            'distance_kp', 'rotate_threshold', 'align_threshold')})
        self.dwell_default = finite_positive(self.get_parameter('dwell_time').value, 'dwell_time', True)
        self.points = []
        self.index, self.dwell = 0, None
        self.last_tick = monotonic()
        self.record_file = self.record_writer = None
        self.record_count, self.last_plot = 0, -math.inf
        self.trajectory = RosPath()
        self.trajectory.header.frame_id = 'map'
        self.target_pub = self.create_publisher(RosPath, '/waypoint/targets', LATCHED)
        self.trajectory_pub = self.create_publisher(RosPath, '/waypoint/trajectory', 1)
        rate = finite_positive(self.get_parameter('control_rate').value, 'control_rate')
        self.create_timer(1 / rate, self.tick)

    def prepare(self):
        # Task parameters may be configured while idle through the standard ROS service.
        self.controller = WaypointController(**{name: self.get_parameter(name).value for name in (
            'max_linear', 'min_linear', 'max_angular', 'arrival_tolerance', 'heading_kp',
            'distance_kp', 'rotate_threshold', 'align_threshold')})
        self.dwell_default = finite_positive(self.get_parameter('dwell_time').value, 'dwell_time', True)
        path = Path(self.get_parameter('waypoints_file').value).expanduser()
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list) or not data['waypoints']:
            raise ValueError('YAML must contain a nonempty waypoints list')
        points = []
        for p in data['waypoints']:
            x, y = self.frame.xy(float(p['latitude']), float(p['longitude']))
            dwell = finite_positive(p.get('dwell_time', self.dwell_default), 'dwell_time', True)
            points.append((x, y, dwell))
        self.points, self.index, self.dwell = points, 0, None
        self.controller.rotating = False
        self.trajectory.poses = []
        self.trajectory_pub.publish(self.trajectory)
        targets = RosPath()
        targets.header.frame_id = 'map'
        targets.header.stamp = self.get_clock().now().to_msg()
        for x, y, dwell in points:
            p = PoseStamped()
            p.header = targets.header
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
            targets.poses.append(p)
        self.target_pub.publish(targets)
        folder = self.get_parameter('trajectory_directory').value
        if folder:
            self.record_file, self.record_writer = open_recording(folder, 'waypoint', self.frame,
                [{'x': x, 'y': y, 'dwell_time': dwell} for x, y, dwell in points],
                ['stamp', 'x', 'y', 'yaw', 'target_index', 'distance', 'heading_error'],
                task_type='waypoint', target_index_base=0)
            self.record_count = 0

    def close_recording(self):
        if self.record_file:
            try:
                self.record_file.close()
            except OSError as exc:
                self.get_logger().error(f'Trajectory close failed: {exc}')
            finally:
                self.record_file = self.record_writer = None

    def finish(self, state, reason=''):
        super().finish(state, reason)
        self.close_recording()
        self.trajectory_pub.publish(self.trajectory)

    def destroy_node(self):
        self.close_recording()
        return super().destroy_node()

    def record_pose(self, distance, heading):
        if self.trajectory.poses and self.trajectory.poses[-1].header.stamp == self.odom.header.stamp:
            return
        p = PoseStamped()
        p.header.frame_id, p.header.stamp = 'map', self.odom.header.stamp
        p.pose = self.odom.pose.pose
        self.trajectory.poses.append(p)
        self.trajectory.poses = self.trajectory.poses[-10000:]
        self.trajectory.header.stamp = p.header.stamp
        if self.record_writer:
            self.record_writer.writerow([p.header.stamp.sec + p.header.stamp.nanosec * 1e-9,
                p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation), self.index, distance, heading])
            self.record_count += 1
            if self.record_count % 20 == 0:
                self.record_file.flush()
        if monotonic() - self.last_plot > 0.5:
            self.trajectory_pub.publish(self.trajectory)
            self.last_plot = monotonic()

    def tick(self):
        now = monotonic()
        dt, self.last_tick = min(0.2, now - self.last_tick), now
        distance, error = None, None
        if self.ready_to_step():
            if self.index < len(self.points):
                pose = self.odom.pose.pose
                dx, dy = self.points[self.index][0] - pose.position.x, self.points[self.index][1] - pose.position.y
                distance = math.hypot(dx, dy)
                error = wrap_angle(math.atan2(dy, dx) - yaw_of(pose.orientation))
                try:
                    self.record_pose(distance, error)
                except OSError as exc:
                    self.finish('FAULT', f'Trajectory write failed: {exc}')
                    return
            if self.dwell is not None:
                self.send()
                self.dwell -= dt
                if self.dwell <= 0:
                    self.index += 1
                    self.dwell = None
                    if self.index >= len(self.points):
                        self.finish('COMPLETED')
            else:
                p = self.odom.pose.pose
                v, w, distance, error, arrived = self.controller.command(
                    p.position.x, p.position.y, yaw_of(p.orientation), self.points[self.index])
                self.send(v, w)
                if arrived:
                    self.dwell = self.points[self.index][2]
        self.publish_status(index=self.index, count=len(self.points), distance=distance,
                            heading_error=error, dwelling=self.dwell is not None)


def main(args=None):
    run(WaypointNode, args)
