import csv
import math
from datetime import datetime
from pathlib import Path as FilePath

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path

from agri_base.coordinates import finite_positive
from agri_base.runtime import LATCHED, TaskNode, run, yaw_of
from .controller import Polyline, PurePursuit


class PathTracker(TaskNode):
    def __init__(self):
        super().__init__('path_tracker', 'tracking')
        defaults = {'lookahead': 0.8, 'speed': 0.2, 'max_angular': 0.5,
                    'completion_tolerance': 0.2, 'search_window': 1.0,
                    'start_tolerance': 0.8, 'max_path_error': 2.0,
                    'control_rate': 20.0, 'trajectory_directory': ''}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            if isinstance(value, float):
                finite_positive(self.get_parameter(name).value, name)
        self.pending_path = None
        self.controller = None
        self.record_file, self.record_writer = None, None
        self.record_count = 0
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'
        self.error_pub = self.create_publisher(Vector3Stamped, '/tracking/error', 1)
        self.track_pub = self.create_publisher(Path, '/tracked_path', 1)
        self.create_subscription(Path, '/reference_path', self.on_path, LATCHED)
        self.create_timer(1 / self.get_parameter('control_rate').value, self.tick)

    def on_path(self, msg):
        if self.state in ('RUNNING', 'PAUSED', 'ACQUIRING'):
            return  # Explicit cancel/start is required to replace an executing path.
        try:
            if msg.header.frame_id != 'map' or any(p.header.frame_id != 'map' for p in msg.poses):
                raise ValueError('Reference path must be in map')
            Polyline([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
            # Reject reverse sections even for externally supplied nav_msgs/Path.
            for a, b in zip(msg.poses, msg.poses[1:]):
                dx, dy = b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y
                yaw = yaw_of(a.pose.orientation)
                if math.hypot(dx, dy) > 1e-5 and dx * math.cos(yaw) + dy * math.sin(yaw) < -1e-4:
                    raise ValueError('Reverse path section unsupported')
            self.pending_path = msg
        except ValueError as exc:
            self.pending_path = None
            self.get_logger().warning(str(exc))

    def prepare(self):
        if self.pending_path is None:
            raise ValueError('No validated forward path received')
        line = Polyline([(p.pose.position.x, p.pose.position.y) for p in self.pending_path.poses])
        p = self.odom.pose.pose.position
        if math.dist((p.x, p.y), line.points[0]) > self.get_parameter('start_tolerance').value:
            raise ValueError('Robot too far from path start; approach start before tracking')
        self.controller = PurePursuit(line, **{name: self.get_parameter(name).value for name in (
            'lookahead', 'speed', 'max_angular', 'completion_tolerance', 'search_window')})
        self.trajectory.poses = []
        folder = self.get_parameter('trajectory_directory').value
        if folder:
            target = FilePath(folder).expanduser()
            target.mkdir(parents=True, exist_ok=True)
            name = datetime.now().strftime('track_%Y%m%d_%H%M%S_%f.csv')
            self.record_file = (target / name).open('x', newline='', encoding='utf-8')
            self.record_writer = csv.writer(self.record_file)
            self.record_writer.writerow(['stamp', 'x', 'y', 'yaw', 'cross_track', 'heading_error', 'progress'])
            self.record_count = 0

    def finish(self, state, reason=''):
        super().finish(state, reason)
        self.close_recording()

    def close_recording(self):
        if self.record_file is not None:
            try:
                self.record_file.close()
            except OSError as exc:
                self.get_logger().error(f'Trajectory save failed: {exc}')
            self.record_file, self.record_writer = None, None

    def destroy_node(self):
        self.close_recording()
        return super().destroy_node()

    def tick(self):
        if self.ready_to_step():
            pose = self.odom.pose.pose
            yaw = yaw_of(pose.orientation)
            v, w, distance, lateral, heading, done = self.controller.command(pose.position.x, pose.position.y, yaw)
            error = Vector3Stamped()
            error.header.frame_id = 'map'
            error.header.stamp = self.get_clock().now().to_msg()
            # x is distance to the current segment, including endpoint overrun.
            error.vector.x, error.vector.y, error.vector.z = distance, heading, self.controller.progress
            self.error_pub.publish(error)
            if not self.trajectory.poses or self.trajectory.poses[-1].header.stamp != self.odom.header.stamp:
                stamped = PoseStamped()
                stamped.header.stamp, stamped.header.frame_id = self.odom.header.stamp, 'map'
                stamped.pose = pose
                self.trajectory.poses.append(stamped)
                self.trajectory.poses = self.trajectory.poses[-10000:]
                if self.record_writer is not None:
                    try:
                        self.record_writer.writerow([
                            self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9,
                            pose.position.x, pose.position.y, yaw, lateral, heading, self.controller.progress])
                        self.record_count += 1
                        if self.record_count % 20 == 0:
                            self.record_file.flush()
                    except OSError as exc:
                        self.finish('FAULT', f'Trajectory write failed: {exc}')
                        return
                self.trajectory.header.stamp = error.header.stamp
                self.track_pub.publish(self.trajectory)
            if distance > self.get_parameter('max_path_error').value:
                self.finish('FAULT', 'Path deviation exceeded')
            elif done:
                self.finish('COMPLETED')
            else:
                self.send(v, w)
        elif self.state == 'PAUSED' and self.controller is not None:
            # Safety still requires live deviation feedback while holding ownership.
            pose = self.odom.pose.pose
            distance, _, _, _ = self.controller.path.project(
                pose.position.x, pose.position.y, self.controller.progress, self.controller.window)
            error = Vector3Stamped()
            error.header.frame_id, error.header.stamp = 'map', self.get_clock().now().to_msg()
            error.vector.x = distance
            self.error_pub.publish(error)
        self.publish_status(progress=self.controller.progress if self.controller else 0.0,
                            length=self.controller.path.total if self.controller else 0.0)


def main(args=None):
    run(PathTracker, args)
