"""RTK/INS pose adapter. No wheel odometry integration or EKF."""

import json
import math
from time import monotonic

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .coordinates import frame_from_node, finite_positive, wrap_angle
from .runtime import LATCHED, fresh_stamp, run, stamp_seconds


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.frame = frame_from_node(self)
        for name, default in {'sensor_timeout': 0.5, 'heading_offset': 0.0,
                              'antenna_x': 0.0, 'antenna_y': 0.0,
                              'require_fix_status': False}.items():
            self.declare_parameter(name, default)
        self.timeout = finite_positive(self.get_parameter('sensor_timeout').value, 'sensor_timeout')
        self.offset = float(self.get_parameter('heading_offset').value)
        self.ax = float(self.get_parameter('antenna_x').value)
        self.ay = float(self.get_parameter('antenna_y').value)
        if not all(math.isfinite(v) for v in (self.offset, self.ax, self.ay)):
            raise ValueError('Invalid INS extrinsics')
        self.require_fix = self.get_parameter('require_fix_status').value
        self.fixes, self.attitudes, self.velocities = {}, {}, {}
        self.last_stamp = -math.inf
        self.last_pair = -math.inf
        self.valid = False
        self.previous_yaw = None
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
        self.valid_pub = self.create_publisher(Bool, '/localization/valid', 1)
        self.origin_pub = self.create_publisher(String, '/localization/origin', LATCHED)
        self.origin_pub.publish(String(data=json.dumps(self.frame.identity)))
        self.static_tf = StaticTransformBroadcaster(self)
        self.dynamic_tf = TransformBroadcaster(self)
        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id, static.child_frame_id = 'map', 'odom'
        static.transform.rotation.w = 1.0
        self.static_tf.sendTransform(static)
        self.create_subscription(NavSatFix, '/mgx23/position', self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, '/mgx23/attitude_stamped',
                                 self.on_attitude, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, '/mgx23/velocity_stamped',
                                 self.on_velocity, qos_profile_sensor_data)
        self.create_timer(0.05, self.check_health)

    def insert(self, cache, msg):
        cache[stamp_seconds(msg.header.stamp)] = msg
        while len(cache) > 30:
            del cache[next(iter(cache))]

    def on_fix(self, msg):
        self.insert(self.fixes, msg)
        self.pair(stamp_seconds(msg.header.stamp))

    def on_attitude(self, msg):
        self.insert(self.attitudes, msg)
        self.pair(stamp_seconds(msg.header.stamp))

    def on_velocity(self, msg):
        self.insert(self.velocities, msg)

    def pair(self, stamp):
        if stamp not in self.fixes or stamp not in self.attitudes or stamp <= self.last_stamp:
            return
        fix, att = self.fixes.pop(stamp), self.attitudes.pop(stamp)
        try:
            if (not fresh_stamp(self, fix.header.stamp, self.timeout)
                    or (self.require_fix and fix.status.status < 0)
                    or not -math.pi <= att.vector.z <= math.pi):
                raise ValueError('Invalid or stale GNSS/INS sample')
            x, y = self.frame.xy(fix.latitude, fix.longitude)
            yaw = self.frame.yaw(fix.latitude, fix.longitude, att.vector.z, self.offset)
            # Antenna position -> base_link origin, using calibrated body offsets.
            x -= math.cos(yaw) * self.ax - math.sin(yaw) * self.ay
            y -= math.sin(yaw) * self.ax + math.cos(yaw) * self.ay
            odom = Odometry()
            odom.header.stamp = fix.header.stamp
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            q = odom.pose.pose.orientation
            q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
            # Unknown covariance must not appear as perfect knowledge.
            for i in (0, 7, 14, 21, 28, 35):
                odom.pose.covariance[i] = 1e6
                odom.twist.covariance[i] = 1e6
            yaw_rate = 0.0
            dt = stamp - self.last_stamp
            if self.previous_yaw is not None and 0.005 < dt < self.timeout:
                yaw_rate = wrap_angle(yaw - self.previous_yaw) / dt
                odom.twist.twist.angular.z = yaw_rate
            vel = self.velocities.pop(stamp, None)
            if vel is not None and all(math.isfinite(v) for v in (vel.vector.x, vel.vector.y)):
                north_yaw = self.frame.yaw(fix.latitude, fix.longitude, 0.0)
                vx = math.cos(north_yaw) * vel.vector.x + math.sin(north_yaw) * vel.vector.y
                vy = math.sin(north_yaw) * vel.vector.x - math.cos(north_yaw) * vel.vector.y
                odom.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
                odom.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
                odom.twist.twist.linear.x += yaw_rate * self.ay
                odom.twist.twist.linear.y -= yaw_rate * self.ax
            self.odom_pub.publish(odom)
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation = q
            self.dynamic_tf.sendTransform(tf)
            self.last_pair, self.last_stamp, self.valid = monotonic(), stamp, True
            self.previous_yaw = yaw
        except ValueError as exc:
            self.valid = False
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
        self.check_health()

    def check_health(self):
        self.valid_pub.publish(Bool(data=self.valid and monotonic() - self.last_pair < self.timeout))


def main(args=None):
    run(Localization, args)
