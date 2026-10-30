import math
from time import monotonic

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy

from agri_base.coordinates import finite_positive
from agri_base.runtime import TaskNode, run, yaw_of
from agri_base.recording import open_recording
from .controller import Polyline
from .coverage import CoverageController


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
        self.approach_file, self.approach_writer = None, None
        self.record_count = 0
        self.last_plot = -math.inf
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'
        self.error_pub = self.create_publisher(Vector3Stamped, '/tracking/error', 1)
        self.track_pub = self.create_publisher(Path, '/tracked_path', 1)
        self.path_subscription = self.create_subscription(Path, '/reference_path', self.on_path, 10)
        self.path_transient = False
        self.create_timer(1.0, self.refresh_path_qos)
        self.create_timer(1 / self.get_parameter('control_rate').value, self.tick)

    def refresh_path_qos(self):
        publishers = self.get_publishers_info_by_topic('/reference_path')
        if len(publishers) > 1:
            self.pending_path = None
            if self.state in ('RUNNING', 'PAUSED', 'ACQUIRING'):
                self.generation += 1
                self.finish('FAULT', 'Multiple reference path publishers')
            return
        if not publishers:
            return
        transient = publishers[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        if transient != self.path_transient:
            self.destroy_subscription(self.path_subscription)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
            self.path_subscription = self.create_subscription(Path, '/reference_path', self.on_path, qos)
            self.path_transient = transient

    def on_path(self, msg):
        if self.count_publishers('/reference_path') > 1:
            self.pending_path = None
            if self.state in ('RUNNING', 'PAUSED', 'ACQUIRING'):
                self.generation += 1
                self.finish('FAULT', 'Multiple reference path publishers')
            return
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
        if self.count_publishers('/reference_path') != 1:
            raise ValueError('Require exactly one reference path publisher')
        if self.pending_path is None:
            raise ValueError('No validated forward path received')
        line = Polyline([(p.pose.position.x, p.pose.position.y) for p in self.pending_path.poses])
        p = self.odom.pose.pose.position
        if math.hypot(p.x, p.y) > self.frame.max_radius:
            raise ValueError('Robot outside configured local operating radius')
        self.controller = CoverageController(line, (p.x, p.y),
            start_tolerance=self.get_parameter('start_tolerance').value,
            **{name: self.get_parameter(name).value for name in (
            'lookahead', 'speed', 'max_angular', 'completion_tolerance', 'search_window')})
        self.trajectory.poses = []
        self.last_plot = -math.inf
        self.track_pub.publish(self.trajectory)
        folder = self.get_parameter('trajectory_directory').value
        self.close_recording()
        if folder:
            points = [{'x': p.pose.position.x, 'y': p.pose.position.y, 'yaw': yaw_of(p.pose.orientation)}
                      for p in self.pending_path.poses]
            try:
                if self.controller.phase == 'APPROACHING':
                    self.approach_file, self.approach_writer = open_recording(
                        folder, 'approach', self.frame,
                        [{'x': line.points[0][0], 'y': line.points[0][1]}],
                        ['stamp', 'x', 'y', 'yaw', 'target_index', 'distance', 'heading_error'],
                        task_type='coverage_approach', target_index_base=0,
                        approach_start={'x': p.x, 'y': p.y})
                self.record_file, self.record_writer = open_recording(folder, 'track', self.frame, points,
                    ['stamp', 'x', 'y', 'yaw', 'cross_track', 'heading_error', 'progress'],
                    task_type='coverage_tracking',
                    approach_csv=self.approach_file.name.split('/')[-1] if self.approach_file else None)
            except (ValueError, OSError):
                self.close_recording()
                raise
            self.record_count = 0

    def finish(self, state, reason=''):
        super().finish(state, reason)
        self.close_recording()
        self.track_pub.publish(self.trajectory)

    def close_recording(self):
        for stream in (self.record_file, self.approach_file):
            if stream is not None:
                try:
                    stream.close()
                except OSError as exc:
                    self.get_logger().error(f'Trajectory save failed: {exc}')
        self.record_file, self.record_writer = None, None
        self.approach_file, self.approach_writer = None, None

    def destroy_node(self):
        self.close_recording()
        return super().destroy_node()

    def tick(self):
        if self.ready_to_step():
            pose = self.odom.pose.pose
            yaw = yaw_of(pose.orientation)
            recording_approach = self.controller.phase == 'APPROACHING'
            v, w, distance, lateral, heading, done = self.controller.command(pose.position.x, pose.position.y, yaw)
            error = Vector3Stamped()
            error.header.frame_id = 'map'
            error.header.stamp = self.get_clock().now().to_msg()
            # x is deviation from the approach leg or active coverage segment,
            # never the remaining distance to a distant coverage start.
            error.vector.x, error.vector.y, error.vector.z = distance, heading, self.controller.progress
            self.error_pub.publish(error)
            if not self.trajectory.poses or self.trajectory.poses[-1].header.stamp != self.odom.header.stamp:
                stamped = PoseStamped()
                stamped.header.stamp, stamped.header.frame_id = self.odom.header.stamp, 'map'
                stamped.pose = pose
                self.trajectory.poses.append(stamped)
                self.trajectory.poses = self.trajectory.poses[-10000:]
                writer = self.approach_writer if recording_approach else self.record_writer
                if writer is not None:
                    try:
                        values = [self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9,
                                  pose.position.x, pose.position.y, yaw]
                        writer.writerow(values + ([0, self.controller.distance_to_start, heading]
                            if recording_approach else [lateral, heading, self.controller.progress]))
                        self.record_count += 1
                        if self.record_count % 20 == 0:
                            (self.approach_file if recording_approach else self.record_file).flush()
                    except OSError as exc:
                        self.finish('FAULT', f'Trajectory write failed: {exc}')
                        return
                self.trajectory.header.stamp = error.header.stamp
                if monotonic() - self.last_plot >= 0.5:
                    self.track_pub.publish(self.trajectory)
                    self.last_plot = monotonic()
            if distance > self.get_parameter('max_path_error').value:
                self.finish('FAULT', 'Path deviation exceeded')
            elif done:
                self.finish('COMPLETED')
            else:
                self.send(v, w)
        elif self.state == 'PAUSED' and self.controller is not None:
            # Safety still requires live deviation feedback while holding ownership.
            pose = self.odom.pose.pose
            distance, _, _ = self.controller.error(pose.position.x, pose.position.y)
            error = Vector3Stamped()
            error.header.frame_id, error.header.stamp = 'map', self.get_clock().now().to_msg()
            error.vector.x = distance
            self.error_pub.publish(error)
        self.publish_status()

    def publish_status(self, **extra):
        # Also include phase/progress in the immediate start/pause/cancel reply.
        # A state-only first message must not masquerade as completed approach.
        controller = self.controller
        super().publish_status(
            phase=controller.phase if controller else 'IDLE',
            approach_distance=controller.distance_to_start if controller and controller.phase == 'APPROACHING' else None,
            approach_target={'x': controller.target[0], 'y': controller.target[1]} if controller else None,
            progress=controller.progress if controller else 0.0,
            length=controller.path.total if controller else 0.0, **extra)


def main(args=None):
    run(PathTracker, args)
