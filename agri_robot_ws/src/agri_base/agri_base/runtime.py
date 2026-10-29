"""Small common ROS helpers and the task control protocol."""

import json
import math
import secrets
from time import monotonic

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .coordinates import frame_from_node, finite_positive
from agri_interfaces.srv import AcquireControl


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fresh_stamp(node, stamp, timeout):
    age = node.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(stamp)
    return -0.05 <= age <= timeout and stamp_seconds(stamp) > 0


def yaw_of(q):
    values = (q.x, q.y, q.z, q.w)
    norm = sum(v * v for v in values)
    if not all(math.isfinite(v) for v in values) or abs(norm - 1.0) > 0.01:
        raise ValueError('Invalid quaternion')
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def run(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class TaskNode(Node):
    """Independent task execution, sharing only state/lease/stop plumbing.

    Start is asynchronous: service acknowledges acquisition, status confirms RUNNING.
    Safety owns the lease. Pausing retains it; faults require an explicit new start.
    """

    def __init__(self, name, source):
        super().__init__(name)
        self.source = source
        self.frame = frame_from_node(self)
        self.declare_parameter('sensor_timeout', 0.5, ParameterDescriptor(read_only=True))
        self.timeout = finite_positive(self.get_parameter('sensor_timeout').value, 'sensor_timeout')
        self.state, self.reason = 'IDLE', ''
        self.odom, self.pose_time = None, -math.inf
        self.valid, self.valid_time = False, -math.inf
        self.frame_match = False
        self.generation = secrets.randbits(52)
        self.mode_revision, self.lease_revision, self.lease_generation = 0, 0, 0
        self.motion_inhibited = True
        self.mode_matches = False
        self.started_monotonic, self.elapsed_seconds = None, 0.0
        self.cmd = self.create_publisher(TwistStamped, f'/control/{source}/cmd_vel', 1)
        self.status = self.create_publisher(String, f'/{source}/status', LATCHED)
        self.lease = self.create_client(AcquireControl, '/safety/acquire_control')
        self.create_subscription(Odometry, '/odom', self.on_odom, 1)
        self.create_subscription(Bool, '/localization/valid', self.on_valid, 1)
        self.create_subscription(String, '/localization/origin', self.on_origin, LATCHED)
        self.create_subscription(String, '/safety/status', self.on_safety, LATCHED)
        for action in ('start', 'pause', 'resume', 'cancel'):
            self.create_service(Trigger, f'/{source}/{action}',
                                lambda req, res, a=action: self.handle_request(a, res))
        self.add_on_set_parameters_callback(self.configure_idle)

    def configure_idle(self, parameters):
        if self.state in ('RUNNING', 'PAUSED', 'ACQUIRING'):
            return SetParametersResult(successful=False, reason='Cancel task before changing parameters')
        if any(p.name == 'control_rate' and self.has_parameter(p.name)
               and p.value != self.get_parameter(p.name).value for p in parameters):
            return SetParametersResult(successful=False, reason='Control rate requires core restart')
        for p in parameters:
            if isinstance(p.value, (int, float)) and not isinstance(p.value, bool):
                if not math.isfinite(p.value) or p.value < 0 or (p.value == 0 and p.name != 'dwell_time'):
                    return SetParametersResult(successful=False, reason='Task parameters must be finite and positive')
        return SetParametersResult(successful=True)

    def on_origin(self, msg):
        self.frame_match = json.loads(msg.data) == self.frame.identity

    def on_valid(self, msg):
        self.valid, self.valid_time = msg.data, monotonic()

    def on_odom(self, msg):
        try:
            p = msg.pose.pose.position
            yaw_of(msg.pose.pose.orientation)
            if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link'
                    or not all(math.isfinite(v) for v in (p.x, p.y))
                    or not fresh_stamp(self, msg.header.stamp, self.timeout)):
                raise ValueError('Invalid pose')
            self.odom, self.pose_time = msg, monotonic()
        except ValueError:
            self.odom = None

    def healthy(self):
        now = monotonic()
        return (self.frame_match and self.mode_matches and self.valid and self.odom is not None
                and now - self.valid_time < self.timeout
                and now - self.pose_time < self.timeout
                and fresh_stamp(self, self.odom.header.stamp, self.timeout))

    def on_safety(self, msg):
        data = json.loads(msg.data)
        self.mode_revision = data.get('mode_revision', 0)
        self.mode_matches = data.get('mode') == {'waypoint': 'waypoint', 'tracking': 'coverage'}[self.source]
        self.motion_inhibited = data.get('motion_inhibited', data.get('emergency_stop', True))
        if self.state in ('RUNNING', 'PAUSED', 'ACQUIRING') and not self.mode_matches:
            self.generation += 1
            self.finish('CANCELED', 'Work mode changed')
        elif self.state in ('RUNNING', 'PAUSED') and data['owner'] != self.source:
            self.finish('FAULT', data['reason'])
        elif self.state == 'RUNNING' and self.motion_inhibited:
            self.state, self.reason = 'PAUSED', data['reason']
            self.send()

    def send(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(linear), float(angular)
        self.cmd.publish(msg)

    def publish_status(self, **extra):
        directory = self.get_parameter('trajectory_directory').value if self.has_parameter('trajectory_directory') else ''
        data = {'state': self.state, 'reason': self.reason, 'run_directory': directory,
                'generation': self.generation, 'mode_revision': self.mode_revision,
                'elapsed_seconds': monotonic() - self.started_monotonic if self.started_monotonic is not None else self.elapsed_seconds,
                **extra}
        self.status.publish(String(data=json.dumps(data, allow_nan=False)))

    def prepare(self):
        raise NotImplementedError

    def handle_request(self, action, response):
        response.success = False
        if action == 'start':
            if self.state in ('RUNNING', 'PAUSED', 'ACQUIRING') or not self.healthy():
                response.message = 'Task busy or localization/origin unavailable'
                return response
            if not self.lease.service_is_ready():
                response.message = 'Safety service unavailable'
                return response
            try:
                self.prepare()
            except (ValueError, OSError, KeyError, TypeError) as exc:
                response.message = str(exc)
                return response
            self.state, self.reason = 'ACQUIRING', ''
            self.started_monotonic, self.elapsed_seconds = None, 0.0
            self.generation += 1
            token = self.generation
            self.lease_revision, self.lease_generation = self.mode_revision, token
            future = self.lease.call_async(AcquireControl.Request(
                source=self.source, enable=True, mode_revision=self.lease_revision, task_generation=token))
            future.add_done_callback(lambda f: self.acquired(f, token))
        elif action == 'pause' and self.state == 'RUNNING':
            self.state = 'PAUSED'
            self.send()
        elif (action == 'resume' and self.state == 'PAUSED' and self.healthy()
              and not self.motion_inhibited):
            self.state = 'RUNNING'
            self.reason = ''
        elif action == 'cancel':
            if self.state == 'ACQUIRING':
                response.message = 'Acquisition pending; retry cancel after status changes'
                return response
            self.finish('CANCELED')
        else:
            response.message = f'{action} unavailable in {self.state}'
            return response
        response.success, response.message = True, self.state
        self.publish_status()
        return response

    def acquired(self, future, token):
        if token != self.generation:
            return
        try:
            result = future.result()
            if token == self.generation and result.success and self.healthy():
                self.state = 'RUNNING'
                self.started_monotonic = monotonic()
                self.send()
            else:
                self.finish('FAULT', result.message)
        except Exception as exc:
            self.finish('FAULT', str(exc))
        self.publish_status()

    def finish(self, state, reason=''):
        self.send()
        if self.started_monotonic is not None:
            self.elapsed_seconds = monotonic() - self.started_monotonic
            self.started_monotonic = None
        self.state, self.reason = state, reason
        if self.lease.service_is_ready():
            self.lease.call_async(AcquireControl.Request(source=self.source, enable=False,
                mode_revision=self.lease_revision, task_generation=self.lease_generation))

    def ready_to_step(self):
        if self.state in ('RUNNING', 'PAUSED'):
            if not self.healthy():
                self.finish('FAULT', 'Localization expired or invalid')
            else:
                if self.state == 'PAUSED':
                    self.send()
                return self.state == 'RUNNING'
        return False
