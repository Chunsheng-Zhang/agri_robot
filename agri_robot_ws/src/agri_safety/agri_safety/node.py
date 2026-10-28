"""Single velocity authority: manual/autonomous leases and latched safety stops."""

import json
import math
from datetime import datetime
from pathlib import Path
from time import monotonic

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger

from agri_base.coordinates import finite_positive
from agri_base.runtime import LATCHED, fresh_stamp, run, stamp_seconds, yaw_of
from .gate import CommandGate
from .obstacles import ScanGeometry, ObstacleLatch


class Safety(Node):
    def __init__(self):
        super().__init__('safety')
        defaults = {'max_linear_speed': 0.3, 'max_lateral_speed': 0.3,
                    'max_angular_speed': 0.6, 'command_timeout': 0.25,
                    'sensor_timeout': 0.5, 'max_path_error': 2.0, 'publish_rate': 20.0,
                    'require_lidar': True, 'require_joy': False, 'allow_lateral': False,
                    'joy_timeout': 0.5, 'joy_estop_button': 0, 'scan_timeout': 0.5,
                    'stop_distance': 0.8, 'resume_distance': 1.2, 'clear_duration': 1.0,
                    'laser_frame': 'laser', 'laser_x': 0.0435, 'laser_y': 0.00005258,
                    'laser_yaw': 0.0, 'body_length': 0.45, 'body_width': 0.35,
                    'footprint_margin': 0.05, 'front_half_angle': math.pi / 3,
                    'rear_half_angle': math.pi / 3, 'min_valid_fraction': 0.5,
                    'min_coverage_fraction': 0.85, 'event_directory': ''}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.get = lambda name: self.get_parameter(name).value
        for name in ('sensor_timeout', 'joy_timeout', 'scan_timeout', 'publish_rate'):
            finite_positive(self.get(name), name)
        if not isinstance(self.get('joy_estop_button'), int) or self.get('joy_estop_button') < 0:
            raise ValueError('Invalid joystick estop button')
        self.timeout = self.get('sensor_timeout')
        self.gate = CommandGate(self.get('max_linear_speed'), self.get('max_angular_speed'),
                                self.get('command_timeout'), self.get('max_path_error'),
                                self.get('max_lateral_speed'))
        self.geometry = ScanGeometry(**{name: self.get(name) for name in (
            'laser_x', 'laser_y', 'laser_yaw', 'body_length', 'body_width',
            'front_half_angle', 'rear_half_angle', 'min_valid_fraction', 'min_coverage_fraction')},
            margin=self.get('footprint_margin'))
        self.obstacle = ObstacleLatch(self.get('stop_distance'), self.get('resume_distance'), self.get('clear_duration'))
        self.valid, self.odom = False, None
        self.valid_time, self.pose_time = -math.inf, -math.inf
        self.scan_time, self.scan_stamp, self.scan_valid = -math.inf, None, False
        self.scan_reason = 'Waiting for LaserScan'
        self.joy_time, self.manual_time = -math.inf, -math.inf
        self.joy_stamp = None
        self.joy_pressed, self.joy_neutral, self.manual_connected = False, False, False
        self.joy_seen = False
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/safety/status', LATCHED)
        self.estop_pub = self.create_publisher(Bool, '/safety/emergency_stop', LATCHED)
        self.obstacle_pub = self.create_publisher(Bool, '/safety/obstacle_stop', LATCHED)
        self.distance_pub = self.create_publisher(String, '/safety/obstacles', 1)
        self.nearest_pub = self.create_publisher(Float32, '/safety/nearest_distance', 1)
        self.event_pub = self.create_publisher(String, '/safety/events', LATCHED)
        self.event_file = None
        if self.get('event_directory'):
            directory = Path(self.get('event_directory')).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            filename = datetime.now().strftime('safety_%Y%m%d_%H%M%S_%f.jsonl')
            self.event_file = (directory / filename).open('x', encoding='utf-8')
        self.create_subscription(Bool, '/localization/valid', self.on_valid, 1)
        self.create_subscription(Odometry, '/odom', self.on_pose, 1)
        self.create_subscription(Bool, '/emergency_stop', self.on_stop, 1)
        self.create_subscription(Joy, '/joy', self.on_joy, qos_profile_sensor_data)
        self.create_subscription(String, '/manual/status', self.on_manual, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, '/tracking/error', self.on_error, 1)
        self.create_service(SetBool, '/safety/emergency_stop', self.stop_service)
        self.create_service(Trigger, '/safety/obstacle/reset', self.reset_obstacle)
        for source in ('waypoint', 'tracking', 'manual', 'web'):
            self.create_subscription(TwistStamped, f'/control/{source}/cmd_vel',
                                     lambda msg, s=source: self.on_command(s, msg), 1)
            self.create_service(SetBool, f'/safety/{source}/enable',
                                lambda req, res, s=source: self.enable(s, req, res))
        self.create_timer(1 / self.get('publish_rate'), self.tick)

    def event(self, kind, reason):
        data = {'stamp': self.get_clock().now().nanoseconds * 1e-9,
                'event': kind, 'reason': reason, 'owner': self.gate.owner}
        line = json.dumps(data, allow_nan=False)
        self.event_pub.publish(String(data=line))
        self.get_logger().warning(line)
        if self.event_file is not None:
            try:
                self.event_file.write(line + '\n')
                self.event_file.flush()
            except OSError as exc:
                self.get_logger().error(f'Safety event log unavailable: {exc}')
                try:
                    self.event_file.close()
                except OSError:
                    pass
                self.event_file = None

    def on_valid(self, msg):
        self.valid, self.valid_time = msg.data, monotonic()

    def on_pose(self, msg):
        try:
            yaw_of(msg.pose.pose.orientation)
            p = msg.pose.pose.position
            if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link'
                    or not all(math.isfinite(v) for v in (p.x, p.y))):
                raise ValueError('Invalid pose')
            self.odom, self.pose_time = msg, monotonic()
        except ValueError:
            self.odom = None

    def localization_ok(self):
        now = monotonic()
        return (self.valid and now - self.valid_time < self.timeout and self.odom is not None
                and now - self.pose_time < self.timeout
                and fresh_stamp(self, self.odom.header.stamp, self.timeout)
                and self.count_publishers('/odom') == 1)

    def joystick_ok(self):
        return (self.manual_connected and monotonic() - self.joy_time < self.get('joy_timeout')
                and monotonic() - self.manual_time < self.get('joy_timeout')
                and self.joy_stamp is not None and fresh_stamp(self, self.joy_stamp, self.get('joy_timeout'))
                and self.count_publishers('/joy') == 1 and self.count_publishers('/manual/status') == 1)

    def lidar_ok(self):
        return (self.scan_valid and monotonic() - self.scan_time < self.get('scan_timeout')
                and self.scan_stamp is not None and fresh_stamp(self, self.scan_stamp, self.get('scan_timeout'))
                and self.count_publishers('/scan') == 1)

    def healthy(self, source=None):
        """Runtime health, excluding the separately latched laser inhibit."""
        source = source or self.gate.owner or 'waypoint'
        common = (self.count_publishers('/cmd_vel') == 1
                  and (not self.get('require_joy') or self.joystick_ok()))
        return common and (self.joystick_ok() if source == 'manual' else self.localization_ok())

    def on_joy(self, msg):
        i = self.get('joy_estop_button')
        if i < len(msg.buttons) and msg.buttons[i]:
            self.joy_pressed = True
            if not self.gate.emergency_stop:
                self.event('EMERGENCY_STOP', 'Joystick button pressed')
            self.gate.stop(True, True)
            self.tick()
        if (len(msg.axes) < 3 or i >= len(msg.buttons)
                or not fresh_stamp(self, msg.header.stamp, self.get('joy_timeout'))
                or not all(math.isfinite(v) and abs(v) <= 1.01 for v in msg.axes)):
            return
        self.joy_pressed = bool(msg.buttons[i])
        self.joy_neutral = all(abs(v) < 0.2 for v in msg.axes[:3])
        self.joy_time, self.joy_stamp, self.joy_seen = monotonic(), msg.header.stamp, True

    def on_manual(self, msg):
        try:
            data = json.loads(msg.data)
            if abs(self.get_clock().now().nanoseconds * 1e-9 - data['stamp']) > self.get('joy_timeout'):
                return
            self.manual_connected, self.manual_time = bool(data['connected']), monotonic()
        except (ValueError, KeyError, TypeError):
            self.manual_connected = False

    def on_scan(self, msg):
        self.scan_time, self.scan_stamp = monotonic(), msg.header.stamp
        try:
            if msg.header.frame_id != self.get('laser_frame') or not fresh_stamp(self, msg.header.stamp, self.get('scan_timeout')):
                raise ValueError('Wrong laser frame or stale LaserScan')
            self.geometry.update(msg.ranges, msg.angle_min, msg.angle_max,
                                 msg.angle_increment, msg.range_min, msg.range_max)
            self.scan_valid, self.scan_reason = True, ''
        except ValueError as exc:
            self.scan_valid, self.scan_reason = False, str(exc)
        if self.obstacle.active:
            valid, clearance = self.geometry.clearance(*self.obstacle.direction) if self.lidar_ok() else (False, 0.0)
            self.obstacle.observe(valid, clearance, monotonic())
        self.tick()

    def on_command(self, source, msg):
        if source != self.gate.owner:
            return
        if msg.header.frame_id != 'base_link' or not fresh_stamp(self, msg.header.stamp, self.gate.command_timeout):
            return
        t = msg.twist
        if (any(v != 0 for v in (t.linear.z, t.angular.x, t.angular.y))
                or (t.linear.y != 0 and (source not in ('manual', 'web') or not self.get('allow_lateral')))):
            self.gate.revoke('Unsupported chassis velocity component')
            self.tick()
            return
        if self.count_publishers(f'/control/{source}/cmd_vel') != 1:
            self.gate.revoke('Multiple controllers on one input')
            self.tick()
            return
        self.gate.receive(source, t.linear.x, t.angular.z, monotonic(),
                          stamp_seconds(msg.header.stamp), lateral=t.linear.y)
        self.tick()

    def on_error(self, msg):
        if (self.gate.owner == 'tracking' and msg.header.frame_id == 'map'
                and fresh_stamp(self, msg.header.stamp, self.gate.command_timeout)
                and stamp_seconds(msg.header.stamp) >= self.gate.granted_stamp
                and math.isfinite(msg.vector.x)):
            self.gate.error, self.gate.error_time = msg.vector.x, monotonic()

    def set_stop(self, active):
        if active and self.gate.emergency_stop:
            self.tick()
            return True
        reset_ok = (not self.joy_pressed and self.count_publishers('/cmd_vel') == 1
                    and (not self.get('require_joy') or (self.joystick_ok() and self.joy_neutral)))
        result = self.gate.stop(active, reset_ok)
        if result:
            self.event('EMERGENCY_STOP' if active else 'EMERGENCY_RESET', 'Explicit operator request')
        self.tick()
        return result

    def on_stop(self, msg):
        self.set_stop(msg.data)

    def stop_service(self, request, response):
        response.success = self.set_stop(request.data)
        response.message = self.gate.reason if response.success else 'Release estop and center connected joystick before reset'
        return response

    def reset_obstacle(self, request, response):
        response.success = self.obstacle.reset(monotonic(), self.lidar_ok())
        response.message = 'Obstacle latch cleared; task remains paused' if response.success else 'Require continuous valid clearance above resume distance'
        if response.success:
            self.gate.samples.clear()
            self.gate.granted_at = monotonic()
            self.event('OBSTACLE_RESET', response.message)
        self.tick()
        return response

    def enable(self, source, request, response):
        health = self.healthy(source)
        can_move = (health and not self.obstacle.active and (not self.get('require_lidar') or self.lidar_ok()))
        if source == 'manual' and request.data:
            can_move = can_move and self.joy_neutral and not self.joy_pressed
            if can_move and not self.gate.emergency_stop and self.gate.owner in ('waypoint', 'tracking', 'web'):
                self.event('MANUAL_TAKEOVER', self.gate.owner)
                self.gate.revoke('Explicit manual takeover')
        response.success = self.gate.enable(source, request.data, can_move, monotonic(),
                                            self.get_clock().now().nanoseconds * 1e-9)
        response.message = self.gate.reason if response.success else 'Busy, stopped, not neutral, or required sensor unavailable'
        self.tick()
        return response

    def tick(self):
        if ((self.get('require_joy') and self.joy_seen) or self.gate.owner == 'manual') and not self.joystick_ok():
            if not self.gate.emergency_stop:
                self.event('EMERGENCY_STOP', 'Joystick/control heartbeat lost')
                self.gate.stop(True, True)
        if self.get('require_lidar') and self.gate.owner and not self.obstacle.active:
            sample = self.gate.samples.get(self.gate.owner)
            direction = (sample[0], sample[3], sample[1]) if sample else (0.0, 0.0, 0.0)
            if not self.lidar_ok():
                self.obstacle.latch('LaserScan invalid or timed out', direction)
                self.event('OBSTACLE_STOP', self.obstacle.reason)
            elif sample and any(abs(v) > 1e-4 for v in direction):
                valid, clearance = self.geometry.clearance(*direction)
                if not valid or clearance < self.obstacle.stop_distance:
                    self.obstacle.latch('Motion sector unknown' if not valid else 'Obstacle inside stop clearance', direction)
                    self.event('OBSTACLE_STOP', self.obstacle.reason)
        if not self.lidar_ok():
            self.obstacle.clear_since = None
        if self.obstacle.active and self.gate.owner in ('manual', 'web'):
            self.gate.revoke('Obstacle stop; explicit manual rearm required')
        v, w = self.gate.output(monotonic(), self.healthy())
        msg = Twist()
        if not self.obstacle.active:
            msg.linear.x, msg.linear.y, msg.angular.z = v, self.gate.lateral_output, w
        self.pub.publish(msg)
        inhibited = self.gate.emergency_stop or self.obstacle.active
        self.status_pub.publish(String(data=json.dumps({
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'owner': self.gate.owner, 'emergency_stop': self.gate.emergency_stop,
            'estop_pressed': self.joy_pressed, 'obstacle_stop': self.obstacle.active,
            'motion_inhibited': inhibited, 'reason': self.gate.reason if self.gate.emergency_stop
                else (self.obstacle.reason if self.obstacle.active else self.gate.reason),
            'linear': msg.linear.x, 'lateral': msg.linear.y, 'angular': msg.angular.z,
            'localization_valid': self.localization_ok(), 'joy_connected': self.joystick_ok(),
            'lidar_connected': self.lidar_ok(), 'allow_lateral': self.get('allow_lateral')}, allow_nan=False)))
        self.estop_pub.publish(Bool(data=self.gate.emergency_stop))
        self.obstacle_pub.publish(Bool(data=self.obstacle.active))
        valid = self.lidar_ok()
        data = {'stamp': self.get_clock().now().nanoseconds * 1e-9, 'frame_id': 'base_link',
                'valid': valid, 'reason': (self.scan_reason or 'LaserScan timeout or publisher conflict') if not valid else '',
                'global_range': self.geometry.global_distance if valid else None,
                'front_distance': self.geometry.nearest_sector(0, self.geometry.front_angle) if valid else None,
                'rear_distance': self.geometry.nearest_sector(math.pi, self.geometry.rear_angle) if valid else None,
                'front_valid': valid and self.geometry.sector_valid(0, self.geometry.front_angle),
                'rear_valid': valid and self.geometry.sector_valid(math.pi, self.geometry.rear_angle),
                'obstacle_stop': self.obstacle.active,
                'recovery_ready': bool(valid and self.obstacle.clear_since is not None
                    and monotonic() - self.obstacle.clear_since >= self.obstacle.clear_duration)}
        self.distance_pub.publish(String(data=json.dumps(data, allow_nan=False)))
        self.nearest_pub.publish(Float32(data=float(data['global_range']) if data['global_range'] is not None else math.nan))

    def destroy_node(self):
        self.pub.publish(Twist())
        if self.event_file is not None:
            self.event_file.close()
        return super().destroy_node()


def main(args=None):
    run(Safety, args)
