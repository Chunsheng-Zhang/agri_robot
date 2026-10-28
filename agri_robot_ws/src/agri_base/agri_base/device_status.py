"""Camera receive health and calibrated laser static frame; no image processing."""

import json
import math
from time import monotonic

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from .coordinates import finite_positive
from .runtime import LATCHED, fresh_stamp, run


class CameraStatus(Node):
    def __init__(self):
        super().__init__('camera_status')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_timeout', 1.0)
        self.timeout = finite_positive(self.get_parameter('image_timeout').value, 'image_timeout')
        self.received, self.stamp = -math.inf, None
        self.width, self.height, self.encoding, self.frames = 0, 0, '', 0
        self.reason = 'Waiting for image'
        self.pub = self.create_publisher(String, '/camera/status', LATCHED)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.2, self.tick)

    def on_image(self, msg):
        if (msg.width <= 0 or msg.height <= 0 or msg.step <= 0
                or len(msg.data) < msg.step * msg.height or not msg.encoding
                or not fresh_stamp(self, msg.header.stamp, self.timeout)):
            self.reason, self.received = 'Malformed or stale image', -math.inf
            return
        self.received, self.stamp = monotonic(), msg.header.stamp
        self.width, self.height, self.encoding = msg.width, msg.height, msg.encoding
        self.reason, self.frames = '', self.frames + 1

    def tick(self):
        connected = (monotonic() - self.received < self.timeout and self.stamp is not None
                     and fresh_stamp(self, self.stamp, self.timeout)
                     and self.count_publishers(self.get_parameter('image_topic').value) == 1)
        self.pub.publish(String(data=json.dumps({
            'stamp': self.get_clock().now().nanoseconds * 1e-9, 'connected': connected,
            'topic': self.get_parameter('image_topic').value, 'width': self.width,
            'height': self.height, 'encoding': self.encoding, 'frames_received': self.frames,
            'reason': self.reason or ('' if connected else 'Image receive timeout')})))


class SensorFrames(Node):
    def __init__(self):
        super().__init__('sensor_frames')
        for name, value in {'laser_x': 0.0435, 'laser_y': 0.00005258, 'laser_z': 0.11,
                            'laser_yaw': 0.0, 'laser_frame': 'laser'}.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        if not all(math.isfinite(get(n)) for n in ('laser_x', 'laser_y', 'laser_z', 'laser_yaw')):
            raise ValueError('Invalid laser extrinsics')
        if get('laser_frame') in ('map', 'odom', 'base_link', ''):
            raise ValueError('Invalid laser child frame')
        self.broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', get('laser_frame')
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform.translation.x, tf.transform.translation.y = get('laser_x'), get('laser_y')
        tf.transform.translation.z = get('laser_z')
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(get('laser_yaw') / 2), math.cos(get('laser_yaw') / 2)
        self.broadcaster.sendTransform(tf)


def camera_main(args=None):
    run(CameraStatus, args)


def frames_main(args=None):
    run(SensorFrames, args)
