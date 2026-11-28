import math
import struct

import pytest
from shapely.geometry import Polygon

from agri_base.coordinates import LocalFrame, track_speeds, wrap_angle
from agri_waypoint.controller import WaypointController
from agri_path_tracking.controller import Polyline, PurePursuit
from agri_path_planning.field import load_field, validate_path
from agri_safety.gate import CommandGate
from mgx23_driver.mgx23_parser import parse_frame


@pytest.mark.parametrize('heading,expected', [(0, math.pi / 2), (math.pi / 2, 0),
                                            (-math.pi / 2, math.pi), (math.pi, -math.pi / 2)])
def test_heading_cardinal_directions(heading, expected):
    frame = LocalFrame(36, 117)
    assert abs(wrap_angle(frame.yaw(36, 117, heading) - expected)) < 1e-6


def test_fixed_origin_and_projection_convergence():
    frame = LocalFrame(50, 119)
    assert frame.xy(50, 119) == (0, 0)
    assert frame.xy(50, 119.00001)[0] > 0
    # Away from central meridian, true north is not exactly grid north.
    assert abs(frame.yaw(50, 119, 0) - math.pi / 2) > 0.01
    with pytest.raises(ValueError):
        frame.xy(30.201, 120.101)


@pytest.mark.parametrize('lat,lon', [(math.nan, 117), (85, 117), (36, 181)])
def test_reject_invalid_origin(lat, lon):
    with pytest.raises(ValueError):
        LocalFrame(lat, lon)


def test_track_speeds():
    assert track_speeds(0.2, 1, 0.4) == pytest.approx((0, 0.4))
    with pytest.raises(ValueError):
        track_speeds(0, 0, 0)


def test_waypoint_rotates_stops_and_slows():
    c = WaypointController()
    v, w, _, _, done = c.command(0, 0, math.pi, (1, 0))
    assert v == 0 and abs(w) <= c.max_angular and not done
    assert c.command(0.9, 0, 0, (1, 0))[-1]
    far = c.command(0, 0, 0, (1, 0))[0]
    near = c.command(0.7, 0, 0, (1, 0))[0]
    assert 0 < near < far


def test_waypoint_closed_loop_converges():
    c = WaypointController()
    x, y, yaw = 0, 0, -2.0
    for _ in range(3000):
        v, w, _, _, done = c.command(x, y, yaw, (2, 1))
        if done:
            break
        x += v * math.cos(yaw) * 0.05
        y += v * math.sin(yaw) * 0.05
        yaw += w * 0.05
    assert done and math.hypot(x - 2, y - 1) <= c.arrival


def test_closed_path_does_not_complete_at_start():
    c = PurePursuit(Polyline([(0, 0), (5, 0), (5, 1), (0, 1), (0, 0)]))
    assert not c.command(0, 0, 0)[-1]
    assert c.progress == 0


def test_progress_does_not_jump_to_adjacent_row():
    c = PurePursuit(Polyline([(0, 0), (20, 0), (20, 0.4), (0, 0.4)]))
    c.command(0.1, 0.39, 0)
    assert c.progress < 1.0
    old = c.progress
    c.command(-0.1, 0, 0)
    assert c.progress >= old


def test_pure_pursuit_recovers_cross_track_error():
    c = PurePursuit(Polyline([(0, 0), (3, 0)]), lookahead=0.5)
    x, y, yaw = 0, 0.3, 0
    for _ in range(1500):
        v, w, _, _, _, done = c.command(x, y, yaw)
        if done:
            break
        assert abs(w) <= c.max_angular + 1e-9
        x += v * math.cos(yaw) * 0.05
        y += v * math.sin(yaw) * 0.05
        yaw += w * 0.05
    assert done and abs(y) < 0.08


@pytest.mark.parametrize('parameters', [{'speed': 1.001}, {'speed': -1.0},
    {'max_angular': 5.001}, {'max_angular': float('nan')}])
def test_coverage_velocity_limits_reject_out_of_range(parameters):
    with pytest.raises(ValueError):
        PurePursuit(Polyline([(0, 0), (5, 0)]), **parameters)


@pytest.mark.parametrize('offset', [-.08, .08])
@pytest.mark.parametrize('angular_limit', [1., 5.])
def test_coverage_curvature_preserved_at_requested_velocity_limits(offset, angular_limit):
    controller = PurePursuit(Polyline([(0, 0), (5, 0)]), speed=1.0,
        max_angular=angular_limit, lookahead=.1)
    v, w, *_ = controller.command(0, offset, 0)
    assert 0 < v <= 1.0
    assert abs(w) == pytest.approx(angular_limit)
    # At a tight curve the controller reduces forward speed, preserving curvature.
    assert v < 1.0
    assert math.copysign(1, w) == -math.copysign(1, offset)
    assert controller.command(0, 0, math.pi)[:2] == (0.0, -min(angular_limit, math.pi))


@pytest.mark.parametrize('points', [[], [(0, 0)], [(0, 0), (0, 0)], [(math.nan, 0), (1, 1)]])
def test_invalid_paths(points):
    with pytest.raises(ValueError):
        Polyline(points)


def acquired_gate(source='waypoint'):
    gate = CommandGate()
    assert gate.stop(False, True)
    assert gate.enable(source, True, True, 1.0, 100.0)
    return gate


def test_exclusive_ownership_and_release():
    gate = acquired_gate()
    assert not gate.enable('tracking', True, True, 1.1, 100.1)
    assert not gate.enable('tracking', False, True, 1.1, 100.1)
    assert gate.owner == 'waypoint'


def test_limits_and_unselected_input():
    gate = acquired_gate()
    gate.receive('tracking', 0.3, 0.3, 1.1, 100.1)
    assert gate.output(1.1, True) == (0, 0)
    gate.receive('waypoint', 20, -10, 1.1, 100.1)
    assert gate.output(1.1, True) == (0.3, -0.6)


def test_timeout_revokes_no_automatic_restart():
    gate = acquired_gate()
    gate.receive('waypoint', 0.2, 0, 1.01, 100.01)
    assert gate.output(1.7, True) == (0, 0)
    assert not gate.owner
    gate.receive('waypoint', 0.2, 0, 1.8, 100.8)
    assert gate.output(1.8, True) == (0, 0)


def test_localization_fault_and_estop():
    gate = acquired_gate()
    gate.receive('waypoint', 0.2, 0, 1.01, 100.01)
    assert gate.output(1.02, False) == (0, 0)
    assert not gate.owner
    assert gate.stop(True, False)
    assert not gate.stop(False, False)
    assert gate.emergency_stop


def test_old_command_cannot_cross_ownership_change():
    gate = acquired_gate()
    gate.receive('waypoint', 0.2, 0, 1.01, 99.0)
    assert gate.output(1.02, True) == (0, 0)


def test_nan_revokes():
    gate = acquired_gate()
    gate.receive('waypoint', math.nan, 0, 1.01, 100.01)
    assert not gate.owner


def test_path_deviation_and_feedback_timeout():
    gate = acquired_gate('tracking')
    gate.receive('tracking', 0.2, 0, 1.01, 100.01)
    gate.error, gate.error_time = 3.0, 1.01
    assert gate.output(1.02, True) == (0, 0) and not gate.owner
    gate = acquired_gate('tracking')
    gate.receive('tracking', 0.2, 0, 1.59, 100.59)
    assert gate.output(1.6, True) == (0, 0) and not gate.owner


def test_clearance_and_holes():
    field = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)],
                    [[(4, 4), (6, 4), (6, 6), (4, 6)]])
    validate_path([(1, 1), (9, 1)], field, 0.4)
    with pytest.raises(ValueError):
        validate_path([(1, 5), (9, 5)], field, 0.4)
    with pytest.raises(ValueError):
        validate_path([(0.05, 1), (0.05, 9)], field, 0.4)


def test_existing_frontend_boundary_rejects_mismatched_field():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    with pytest.raises(ValueError, match='radius'):
        load_field(root / 'data/field_boundary.json', LocalFrame(36.6468527633281, 117.021433066391))


def test_navi_checksum_is_not_implicitly_valid():
    frame = bytearray(100)
    frame[:4], frame[98:] = b'fmin', b'ed'
    struct.pack_into('<d', frame, 12, 36.0)
    struct.pack_into('<d', frame, 20, 117.0)
    struct.pack_into('<H', frame, 96, sum(frame[:96]) & 0xffff)
    assert parse_frame(frame)['checksum_ok']
    frame[60] ^= 0xff
    assert not parse_frame(frame)['checksum_ok']
    assert parse_frame(frame[:99]) is None
