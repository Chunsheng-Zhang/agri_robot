"""Closed-loop approach/tracking behavior, without ROS or hardware."""
import math

import pytest

from agri_path_tracking.controller import Polyline
from agri_path_tracking.coverage import CoverageController


@pytest.mark.parametrize('start', [(-15, 2, 2.5), (4, -5, -1), (3, 0.2, math.pi)])
def test_distant_start_approaches_first_point_before_tracking(start):
    controller = CoverageController(Polyline([(1, 0), (3, 0)]), start[:2],
                                    speed=.5, max_angular=.3)
    assert controller.phase == 'APPROACHING'
    x, y, yaw = start
    transitioned = False
    for _ in range(6000):
        phase = controller.phase
        v, w, deviation, _, _, done = controller.command(x, y, yaw)
        assert 0 <= v <= .5 and abs(w) <= .3 + 1e-9
        if phase == 'APPROACHING':
            assert not done and controller.progress == 0
            if controller.phase == 'TRACKING':
                transitioned = True
                assert v == w == 0
                assert math.hypot(x - 1, y) <= .15
        if done:
            break
        x += v * math.cos(yaw) * .05
        y += v * math.sin(yaw) * .05
        yaw += w * .05
    assert transitioned and done
    assert math.dist((x, y), (3, 0)) <= .2


def test_near_start_skips_approach_without_completing_a_loop():
    line = Polyline([(0, 0), (3, 0), (3, 1), (0, 1), (0, 0)])
    controller = CoverageController(line, (.1, 0))
    assert controller.phase == 'TRACKING' and controller.approach_path is None
    assert not controller.command(.1, 0, 0)[-1]
    assert controller.progress < .2


def test_approach_error_is_corridor_deviation_not_remaining_distance():
    controller = CoverageController(Polyline([(10, 0), (12, 0)]), (-5, 0))
    assert controller.distance_to_start == 15
    v, w, error, _, _, done = controller.command(-5, .05, 0)
    assert error == pytest.approx(.05) and not done
    assert controller.error(-5, 2.1)[0] == pytest.approx(2.1)
    # Overshooting the finite approach segment must also register deviation.
    assert controller.error(-8, 0)[0] == pytest.approx(3)


def test_tight_start_tolerance_is_respected_at_handover():
    controller = CoverageController(Polyline([(0, 0), (2, 0)]), (-1, 0), start_tolerance=.05)
    assert not controller.command(-.04, 0, 0)[-1]
    assert controller.phase == 'APPROACHING'
    assert controller.command(-.02, 0, 0)[:2] == (0, 0)
    assert controller.phase == 'TRACKING'


def test_approach_speed_can_be_lower_than_waypoint_default_minimum():
    controller = CoverageController(Polyline([(1, 0), (2, 0)]), (-1, 0), speed=.01)
    assert 0 < controller.command(-1, 0, 0)[0] <= .01


def test_nonfinite_approach_start_is_rejected():
    with pytest.raises(ValueError, match='Invalid approach start'):
        CoverageController(Polyline([(1, 0), (2, 0)]), (math.nan, 0))
