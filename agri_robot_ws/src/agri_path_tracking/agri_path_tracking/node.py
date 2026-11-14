import math
from time import monotonic

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from agri_base.coordinates import finite_positive
from agri_base.runtime import TaskNode, run, yaw_of
from agri_base.recording import open_recording
from .controller import Polyline, validate_velocity_limit
from .coverage import CoverageController, AlignmentTimeout
from agri_base.path_geometry import directed_poses, point_turn_indices
from agri_base.path_geometry import motion_signature


class PathTracker(TaskNode):
    def __init__(self, following=False):
        self.following = following
        self.reference_topic = '/following/reference_path' if following else '/reference_path'
        super().__init__('path_follower' if following else 'path_tracker', 'following' if following else 'tracking')
        defaults = {'lookahead': 0.8, 'speed': 0.2, 'max_angular': 1.0,
                    'completion_tolerance': 0.2, 'search_window': 1.0,
                    'start_tolerance': 0.8, 'max_path_error': 2.0,
                    'alignment_tolerance': math.radians(10), 'alignment_hold_time': 0.5,
                    'alignment_max_angular': 0.3, 'alignment_timeout': 30.0,
                    'control_rate': 20.0, 'trajectory_directory': '', 'allow_reverse': False}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            if isinstance(value, float):
                finite_positive(self.get_parameter(name).value, name)
        for name in ('speed', 'max_angular'):
            validate_velocity_limit(name, self.get_parameter(name).value)
        if self.get_parameter('alignment_tolerance').value >= math.pi / 2:
            raise ValueError('alignment_tolerance must be less than pi/2')
        self.pending_path = None
        self.reference_fingerprint = None
        self.controller = None
        self.record_file, self.record_writer = None, None
        self.approach_file, self.approach_writer = None, None
        self.alignment_file, self.alignment_writer = None, None
        self.record_count = 0
        self.lateral_error, self.heading_error = None, None
        self.last_plot = -math.inf
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'
        self.error_pub = self.create_publisher(Vector3Stamped, '/following/error' if following else '/tracking/error', 1)
        self.track_pub = self.create_publisher(Path, '/following/tracked_path' if following else '/tracked_path', 1)
        self.path_subscription = self.create_subscription(Path, self.reference_topic, self.on_path, 10)
        self.path_transient = False
        self.create_timer(1.0, self.refresh_path_qos)
        self.create_timer(1 / self.get_parameter('control_rate').value, self.tick)

    def configure_idle(self, parameters):
        result = super().configure_idle(parameters)
        if not result.successful:
            return result
        try:
            for parameter in parameters:
                if parameter.name in ('speed', 'max_angular'):
                    validate_velocity_limit(parameter.name, parameter.value)
                if parameter.name == 'alignment_tolerance' and parameter.value >= math.pi / 2:
                    raise ValueError('alignment_tolerance must be less than pi/2')
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return result

    def handle_request(self, action, response):
        # Arm alignment before the inherited resume publishes RUNNING. The base
        # class still checks localization and motion inhibition and owns leases.
        if (action == 'resume' and self.state == 'PAUSED' and self.healthy()
                and not self.motion_inhibited and self.controller is not None):
            self.controller.align_for_resume()
        return super().handle_request(action, response)

    def refresh_path_qos(self):
        publishers = self.get_publishers_info_by_topic(self.reference_topic)
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
            self.path_subscription = self.create_subscription(Path, self.reference_topic, self.on_path, qos)
            self.path_transient = transient

    def on_path(self, msg):
        if self.count_publishers(self.reference_topic) > 1:
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
            directed_poses([(p.pose.position.x,p.pose.position.y,yaw_of(p.pose.orientation)) for p in msg.poses])
            point_turn_indices([(p.pose.position.x,p.pose.position.y,yaw_of(p.pose.orientation)) for p in msg.poses])
            self.pending_path = msg
            self.reference_fingerprint = motion_signature([(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in msg.poses])
        except ValueError as exc:
            self.pending_path = None
            self.get_logger().warning(str(exc))

    def prepare(self):
        if self.count_publishers(self.reference_topic) != 1:
            raise ValueError('Require exactly one reference path publisher')
        if self.pending_path is None:
            raise ValueError('No validated path received')
        poses,directions=directed_poses([(p.pose.position.x,p.pose.position.y,yaw_of(p.pose.orientation))
                                         for p in self.pending_path.poses])
        if any(d<0 for d in directions) and not self.get_parameter('allow_reverse').value:
            raise ValueError('Reverse path requires explicit allow_reverse')
        stops=point_turn_indices([(p.pose.position.x,p.pose.position.y,yaw_of(p.pose.orientation))
                                  for p in self.pending_path.poses])
        line = Polyline([p[:2] for p in poses],directions,stops)
        p = self.odom.pose.pose.position
        if self.following and math.dist((p.x, p.y), line.points[0]) > self.get_parameter('start_tolerance').value:
            raise ValueError('请先用手柄将车辆移至示教起点附近，再释放手柄并开始路径跟随')
        if math.hypot(p.x, p.y) > self.frame.max_radius:
            raise ValueError('Robot outside configured local operating radius')
        self.controller = CoverageController(line, (p.x, p.y),
            start_tolerance=self.get_parameter('start_tolerance').value,
            **{name: self.get_parameter(name).value for name in (
            'lookahead', 'speed', 'max_angular', 'completion_tolerance', 'search_window',
            'alignment_tolerance', 'alignment_hold_time', 'alignment_max_angular', 'alignment_timeout')})
        self.trajectory.poses = []
        self.lateral_error, self.heading_error = None, None
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
                    ['stamp', 'x', 'y', 'yaw', 'cross_track', 'heading_error', 'progress', 'direction'],
                    task_type='taught_path_tracking' if self.following else 'coverage_tracking',
                    allow_reverse=self.get_parameter('allow_reverse').value,
                    segment_directions=directions,
                    point_turn_indices=stops,
                    approach_csv=self.approach_file.name.split('/')[-1] if self.approach_file else None)
                self.alignment_file, self.alignment_writer = open_recording(
                    folder, 'alignment', self.frame, points,
                    ['stamp', 'x', 'y', 'yaw', 'target_yaw', 'heading_error', 'progress'],
                    task_type='taught_path_alignment' if self.following else 'coverage_alignment',
                    alignment_parameters={name: self.get_parameter(name).value for name in (
                        'alignment_tolerance', 'alignment_hold_time', 'alignment_max_angular', 'alignment_timeout')})
            except (ValueError, OSError):
                self.close_recording()
                raise
            self.record_count = 0

    def finish(self, state, reason=''):
        super().finish(state, reason)
        self.close_recording()
        self.track_pub.publish(self.trajectory)

    def close_recording(self):
        for stream in (self.record_file, self.approach_file, self.alignment_file):
            if stream is not None:
                try:
                    stream.close()
                except OSError as exc:
                    self.get_logger().error(f'Trajectory save failed: {exc}')
        self.record_file, self.record_writer = None, None
        self.approach_file, self.approach_writer = None, None
        self.alignment_file, self.alignment_writer = None, None

    def destroy_node(self):
        self.close_recording()
        return super().destroy_node()

    def tick(self):
        if self.ready_to_step():
            pose = self.odom.pose.pose
            yaw = yaw_of(pose.orientation)
            recording_phase = self.controller.phase
            try:
                stamp = self.odom.header.stamp
                v, w, distance, lateral, heading, done = self.controller.command(
                    pose.position.x, pose.position.y, yaw,
                    sample_stamp=stamp.sec + stamp.nanosec * 1e-9)
            except AlignmentTimeout as exc:
                self.finish('FAULT', str(exc))
                self.publish_status()
                return
            error = Vector3Stamped()
            error.header.frame_id = 'map'
            error.header.stamp = self.get_clock().now().to_msg()
            # x is deviation from the approach leg or active coverage segment,
            # never the remaining distance to a distant coverage start.
            error.vector.x, error.vector.y, error.vector.z = distance, heading, self.controller.progress
            self.error_pub.publish(error)
            self.lateral_error, self.heading_error = lateral, heading
            if not self.trajectory.poses or self.trajectory.poses[-1].header.stamp != self.odom.header.stamp:
                stamped = PoseStamped()
                stamped.header.stamp, stamped.header.frame_id = self.odom.header.stamp, 'map'
                stamped.pose = pose
                self.trajectory.poses.append(stamped)
                self.trajectory.poses = self.trajectory.poses[-10000:]
                writer, stream = {'APPROACHING': (self.approach_writer, self.approach_file),
                                  'ALIGNING': (self.alignment_writer, self.alignment_file),
                                  'TRACKING': (self.record_writer, self.record_file)}[recording_phase]
                if writer is not None:
                    try:
                        values = [self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9,
                                  pose.position.x, pose.position.y, yaw]
                        if recording_phase == 'ALIGNING':
                            values += [self.controller.alignment_yaw, heading, self.controller.progress]
                        elif recording_phase == 'APPROACHING':
                            values += [0, self.controller.distance_to_start, heading]
                        else:
                            values += [lateral, heading, self.controller.progress,self.controller.tracker.leg[2]]
                        writer.writerow(values)
                        self.record_count += 1
                        if self.record_count % 20 == 0:
                            stream.flush()
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
            distance, self.lateral_error, tangent = self.controller.error(pose.position.x, pose.position.y)
            self.heading_error = math.atan2(math.sin(tangent-yaw_of(pose.orientation)),
                                           math.cos(tangent-yaw_of(pose.orientation)))
            error = Vector3Stamped()
            error.header.frame_id, error.header.stamp = 'map', self.get_clock().now().to_msg()
            error.vector.x = distance
            error.vector.y = self.heading_error
            error.vector.z = self.controller.progress
            self.error_pub.publish(error)
        self.publish_status()

    def publish_status(self, **extra):
        # Also include phase/progress in the immediate start/pause/cancel reply.
        # A state-only first message must not masquerade as completed approach.
        controller = self.controller
        super().publish_status(
            lateral_error=self.lateral_error, heading_error=self.heading_error,
            reference_fingerprint=self.reference_fingerprint if self.pending_path else None,
            phase=controller.phase if controller else 'IDLE',
            alignment_error=controller.alignment_error if controller and controller.phase == 'ALIGNING' else None,
            alignment_next_phase=controller.alignment_next_phase if controller and controller.phase == 'ALIGNING' else None,
            approach_distance=controller.distance_to_start if controller and controller.phase == 'APPROACHING' else None,
            approach_target={'x': controller.target[0], 'y': controller.target[1]} if controller else None,
            progress=controller.progress if controller else 0.0,
            direction=controller.tracker.leg[2] if controller else 1,
            direction_leg=controller.tracker.leg_index if controller else 0,
            length=controller.path.total if controller else 0.0, **extra)


def main(args=None):
    run(PathTracker, args)


def following_main(args=None):
    run(lambda: PathTracker(following=True), args)
