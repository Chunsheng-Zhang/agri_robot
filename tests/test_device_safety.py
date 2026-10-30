import math

import pytest

from agri_base.joy_adapter import TypedPublisher, validate_buttons
from agri_safety.obstacles import ScanGeometry, ObstacleLatch
from agri_safety.gate import CommandGate
from std_msgs.msg import Bool, Int32


def scan(geometry=None, overrides=None, invalid_sector=None):
    geometry = geometry or ScanGeometry(laser_x=0, laser_y=0)
    values = [5.0] * 720
    for angle, distance in (overrides or []):
        values[round((angle + math.pi) / (2 * math.pi) * 720) % 720] = distance
    if invalid_sector:
        for i in range(720):
            angle = -math.pi + i * math.pi / 360
            if abs(angle) < invalid_sector:
                values[i] = math.inf
    geometry.update(values, -math.pi, math.pi - math.pi / 360,
                    math.pi / 360, 0.05, 10.0)
    return geometry


def test_forward_ignores_close_rear_obstacle():
    geometry = scan(overrides=[(math.pi, 0.5)])
    assert geometry.global_distance == pytest.approx(0.5)
    assert geometry.clearance(0.2, 0, 0)[1] > 0.8
    assert geometry.clearance(-0.2, 0, 0)[1] < 0.8


def test_lateral_mecanum_motion_uses_side_corridor():
    geometry = scan(overrides=[(math.pi / 2, 0.6)])
    assert geometry.clearance(0.2, 0, 0)[1] > 0.8
    assert geometry.clearance(0, 0.2, 0)[1] < 0.8
    assert geometry.clearance(0, -0.2, 0)[1] > 0.8


def test_rotation_uses_whole_body_sweep():
    geometry = scan(overrides=[(-math.pi / 2, 0.7)])
    assert geometry.clearance(0.2, 0, 0)[1] > 0.8
    assert geometry.clearance(0, 0, 0.2)[1] < 0.8


def test_offset_rotation_of_sensor_and_body_clearance():
    g = ScanGeometry(laser_x=0.1, laser_y=0, laser_yaw=math.pi)
    geometry = scan(g, [(math.pi, 1.0)])
    assert geometry.nearest_sector(0, 0.01) == pytest.approx(1.1)
    assert geometry.clearance(0.2, 0, 0)[1] == pytest.approx(1.1 - 0.225, abs=0.001)


def test_invalid_values_filtered_not_reported_as_obstacles():
    geometry = scan(overrides=[(0, math.nan), (0.1, math.inf), (0.2, -1), (0.3, 100)])
    assert geometry.global_distance == 5.0
    assert geometry.clearance(0.2, 0, 0)[0]


def test_unknown_sector_fails_closed():
    geometry = scan(invalid_sector=1.1)
    assert not geometry.clearance(0.2, 0, 0)[0]
    assert geometry.clearance(-0.2, 0, 0)[0]


def test_partial_scan_cannot_claim_rotation_coverage():
    geometry = ScanGeometry(laser_x=0, laser_y=0)
    geometry.update([5.0] * 361, -math.pi / 2, math.pi / 2, math.pi / 360, 0.05, 10)
    assert geometry.clearance(0.2, 0, 0)[0]
    assert not geometry.clearance(0, 0, 0.2)[0]
    assert not geometry.clearance(-0.2, 0, 0)[0]


@pytest.mark.parametrize('ranges', [[math.inf] * 720, [math.nan] * 720, [0.0] * 720, []])
def test_all_invalid_scan_is_fault(ranges):
    with pytest.raises(ValueError):
        ScanGeometry().update(ranges, -math.pi, math.pi - math.pi / 360, math.pi / 360, 0.05, 10)


def test_obstacle_recovery_is_explicit_and_direction_is_retained():
    latch = ObstacleLatch()
    assert latch.latch('front', (0.2, 0, 0))
    assert not latch.latch('rear', (-0.2, 0, 0))
    assert latch.direction == (0.2, 0, 0)
    latch.observe(True, 2.0, 1.0)
    assert latch.active
    assert not latch.reset(1.5, True)
    assert latch.reset(2.1, True)
    assert not latch.active


def test_bad_or_close_scan_resets_recovery_timer():
    latch = ObstacleLatch()
    latch.latch('front', (0.2, 0, 0))
    latch.observe(True, 2.0, 1.0)
    latch.observe(False, 2.0, 1.5)
    assert not latch.reset(5.0, True)
    latch.observe(True, 2.0, 6.0)
    latch.observe(True, 1.0, 6.5)
    assert not latch.reset(8, True)


def test_manual_lateral_limit_and_estop_global_lock():
    gate = CommandGate(max_lateral=0.2)
    assert gate.stop(False, True)
    assert gate.enable('manual', True, True, 1, 100)
    gate.receive('manual', 0.1, 0.2, 1.01, 100.01, lateral=2.0)
    assert gate.output(1.02, True) == (0.1, 0.2)
    assert gate.lateral_output == 0.2
    gate.stop(True, True)
    assert gate.output(1.03, True) == (0, 0) and gate.lateral_output == 0
    for source in ('waypoint', 'tracking', 'web', 'manual'):
        assert not gate.enable(source, True, True, 2, 101)


def test_out_of_order_motion_cannot_replace_newer_stop():
    gate = CommandGate()
    gate.stop(False, True)
    gate.enable('manual', True, True, 1, 100)
    gate.receive('manual', 0, 0, 1.1, 100.1)
    gate.receive('manual', 0.3, 0, 1.11, 100.05)
    assert gate.output(1.12, True) == (0, 0)


def test_factory_speed_configuration_preserves_stop_and_timeout():
    from pathlib import Path
    import yaml
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
        'agri_robot_ws/src/agri_bringup/config/system.yaml').read_text())
    joystick = config['joy_adapter']['ros__parameters']
    safety = config['safety']['ros__parameters']
    limits = (safety['max_linear_speed'], safety['max_lateral_speed'], safety['max_angular_speed'])
    assert limits == (joystick['xspeed_limit'], joystick['yspeed_limit'], joystick['angular_speed_limit']) == (1, 1, 5)
    gate = CommandGate(max_linear=limits[0], max_lateral=limits[1], max_angular=limits[2])
    assert gate.stop(False, True)
    assert gate.enable('manual', True, True, 1, 100)
    gate.receive('manual', 1, 5, 1.01, 100.01, lateral=1)
    assert gate.output(1.02, True) == (1, 5) and gate.lateral_output == 1
    gate.receive('manual', -2, -9, 1.03, 100.03, lateral=-2)
    assert gate.output(1.04, True) == (-1, -5) and gate.lateral_output == -1
    assert gate.output(1.6, True) == (0, 0) and not gate.owner
    gate.receive('manual', 1, 5, 1.61, 100.61, lateral=1)
    assert gate.output(1.62, True) == (0, 0)  # No automatic restart after timeout.
    assert gate.stop(True, True)
    assert not gate.enable('manual', True, True, 2, 101)


@pytest.mark.parametrize('profile', ['jetson', 'pc'])
def test_new_buttons_are_unused_in_both_mappings(profile):
    validate_buttons(profile, 6, 8)
    with pytest.raises(ValueError):
        validate_buttons(profile, 7, 8)
    with pytest.raises(ValueError):
        validate_buttons(profile, 6, 6)


def test_vendor_pc_publish_arguments_are_ros2_messages():
    messages = []
    class Publisher:
        def publish(self, value):
            messages.append(value)
    TypedPublisher(Publisher(), Int32).publish(3)
    TypedPublisher(Publisher(), Bool).publish(True)
    assert isinstance(messages[0], Int32) and messages[0].data == 3
    assert isinstance(messages[1], Bool) and messages[1].data
