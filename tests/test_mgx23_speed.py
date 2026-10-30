"""MGX23-only speed, asynchronous NAVI assembly; no ROS context or hardware."""
import itertools
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from agri_base import localization
from agri_base.coordinates import LocalFrame


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    sendTransform = publish


class LocalizationHarness:
    insert = localization.Localization.insert
    pair = localization.Localization.pair
    on_fix = localization.Localization.on_fix
    on_attitude = localization.Localization.on_attitude
    on_velocity = localization.Localization.on_velocity
    check_health = localization.Localization.check_health
    publish_health = localization.Localization.publish_health

    def __init__(self):
        self.frame = LocalFrame(36.0, 117.0)
        self.fixes, self.attitudes, self.velocities, self.pending_velocity = {}, {}, {}, {}
        self.last_stamp = self.last_pair = -math.inf
        self.previous_yaw = None
        self.valid, self.require_fix = False, False
        self.timeout, self.velocity_wait = 0.5, 0.02
        self.offset = self.ax = self.ay = 0.0
        self.odom_pub, self.valid_pub, self.dynamic_tf = Publisher(), Publisher(), Publisher()
        self.now_ns = 100_000_000_000

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))

    def get_logger(self):
        return SimpleNamespace(warning=lambda *args, **kwargs: None)


def samples(sec=100):
    fix, att, vel = NavSatFix(), Vector3Stamped(), Vector3Stamped()
    for msg in (fix, att, vel):
        msg.header.stamp.sec = sec
    fix.latitude, fix.longitude = 36.0, 117.0
    vel.vector.x, vel.vector.y, vel.vector.z = 0.3, -0.4, 2.0
    return dict(fix=fix, attitude=att, velocity=vel)


@pytest.mark.parametrize('order', list(itertools.permutations(('fix', 'attitude', 'velocity'))))
def test_all_callback_orders_preserve_same_frame_velocity(order):
    node = LocalizationHarness()
    messages = samples()
    for name in order:
        getattr(node, 'on_' + name)(messages[name])
    assert len(node.odom_pub.messages) == 1
    v = node.odom_pub.messages[0].twist.twist.linear
    assert math.hypot(v.x, v.y) == pytest.approx(0.5)
    assert node.valid
    # Duplicate/late delivery must neither republish nor refill old caches.
    for name, msg in messages.items():
        getattr(node, 'on_' + name)(msg)
    assert len(node.odom_pub.messages) == 1
    assert not any((node.fixes, node.attitudes, node.velocities, node.pending_velocity))


def test_missing_velocity_keeps_pose_alive_without_reusing_old_velocity(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(localization, 'monotonic', lambda: clock[0])
    node = LocalizationHarness()
    messages = samples()
    node.on_fix(messages['fix'])
    node.on_attitude(messages['attitude'])
    assert not node.odom_pub.messages
    clock[0] += 0.06
    node.check_health()
    assert len(node.odom_pub.messages) == 1 and node.valid
    assert node.odom_pub.messages[0].twist.covariance[0] == 1e6
    node.on_velocity(messages['velocity'])  # Too late for the already published frame.
    node.now_ns += 100_000_000
    next_messages = samples()
    for msg in next_messages.values():
        msg.header.stamp.nanosec = 100_000_000
    node.on_fix(next_messages['fix'])
    node.on_attitude(next_messages['attitude'])
    clock[0] += 0.06
    node.check_health()
    assert len(node.odom_pub.messages) == 2
    assert node.odom_pub.messages[-1].twist.twist.linear.x == 0.0
    assert not node.pending_velocity


@pytest.fixture
def backend(monkeypatch):
    from agri_backend import main
    monkeypatch.setattr(main, 'robot_state', main.RobotState())
    monkeypatch.setattr(main, 'monotonic', lambda: 10.0)
    return main


def test_web_speed_is_mgx_horizontal_speed_independent_of_odom_and_commands(backend):
    node = LocalizationHarness()
    msg = samples()['velocity']
    backend.RobotStatusNode.velocity_callback(node, msg)
    odom = Odometry()  # Missing odom velocity must not overwrite MGX23 speed.
    odom.pose.pose.orientation.w = 1.0
    backend.RobotStatusNode.odom_callback(None, odom)
    backend.robot_state.update(commanded_speed=0.9)
    status = backend.get_robot_status()
    assert status['speed'] == pytest.approx(0.5)  # Excludes down=2 m/s.
    assert status['speed_source'] == 'mgx23_ned'
    assert status['velocity_ned'] == {'north': 0.3, 'east': -0.4, 'down': 2.0}
    assert status['commanded_speed'] == 0.9
    # Velocity availability does not depend on position freshness.
    backend.robot_state.update(last_odom_monotonic=None)
    assert backend.get_robot_status()['speed'] == pytest.approx(0.5)
    backend.monotonic = lambda: 10.51
    status = backend.get_robot_status()
    assert status['speed'] is None and status['speed_source'] is None


@pytest.mark.parametrize('stamp', [0, 99, 101])
def test_bad_timestamp_does_not_invent_zero_speed(backend, stamp):
    node = LocalizationHarness()
    backend.RobotStatusNode.velocity_callback(node, samples(stamp)['velocity'])
    assert backend.get_robot_status()['speed'] is None


def test_duplicate_old_and_invalid_velocity_cannot_refresh_speed(backend):
    node = LocalizationHarness()
    msg = samples()['velocity']
    backend.RobotStatusNode.velocity_callback(node, msg)
    backend.monotonic = lambda: 11.0
    backend.RobotStatusNode.velocity_callback(node, msg)
    assert backend.get_robot_status()['speed'] is None
    node.now_ns += 100_000_000
    msg.header.stamp.nanosec = 100_000_000
    msg.vector.x = float('nan')
    backend.RobotStatusNode.velocity_callback(node, msg)
    assert backend.robot_state.snapshot()['speed'] is None
    assert backend.get_robot_status()['velocity_ned'] is None


def test_stale_mgx_packet_does_not_refresh_existing_speed(backend):
    node = LocalizationHarness()
    backend.RobotStatusNode.velocity_callback(node, samples()['velocity'])
    node.now_ns += 2_000_000_000
    msg = samples(101)['velocity']
    backend.RobotStatusNode.velocity_callback(node, msg)
    assert backend.robot_state.snapshot()['last_velocity_stamp'] == 100_000_000_000
