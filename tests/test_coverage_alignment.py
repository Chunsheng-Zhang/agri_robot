"""Heading alignment invariants; no ROS context, publishers or vehicle."""
import math

import pytest

from agri_path_tracking.controller import Polyline
from agri_path_tracking.coverage import AlignmentTimeout, CoverageController


def controller(**kwargs):
    return CoverageController(Polyline([(0, 0), (5, 0), (5, 5)]), (0, 0), **kwargs)


def test_initial_alignment_has_zero_forward_speed_and_limited_rotation():
    c = controller(speed=.5, max_angular=.1)
    assert c.phase == 'ALIGNING'
    v, w, *_, done = c.command(0, 0, math.pi/2, now=0)
    assert v == 0 and w == -.1 and not done
    assert c.progress == 0
    assert c.command(0, 0, 0, now=1)[:2] == (0, 0)
    assert c.phase == 'ALIGNING'
    assert c.command(0, 0, 0, now=1.49)[:2] == (0, 0)
    assert c.phase == 'ALIGNING'
    assert c.command(0, 0, 0, now=1.5)[:2] == (0, 0)
    assert c.phase == 'TRACKING'
    assert c.command(0, 0, 0, now=1.6)[0] > 0


def test_alignment_dwell_restarts_after_heading_leaves_tolerance():
    c = controller()
    c.command(0, 0, 0, now=0)
    assert c.command(0, 0, math.radians(11), now=.4)[1] < 0
    c.command(0, 0, 0, now=.5)
    c.command(0, 0, 0, now=.9)
    assert c.phase == 'ALIGNING'
    c.command(0, 0, 0, now=1)
    assert c.phase == 'TRACKING'


def test_repeated_pose_does_not_complete_alignment():
    c = controller()
    c.command(0, 0, 0, now=0, sample_stamp=10)
    c.command(0, 0, 0, now=.7, sample_stamp=10)
    assert c.phase == 'ALIGNING'
    c.command(0, 0, 0, now=.8, sample_stamp=11)
    assert c.phase == 'TRACKING'


def test_alignment_timeout_is_explicit_and_progress_does_not_advance():
    c = controller(alignment_timeout=2)
    c.command(0, 0, math.pi, now=0)
    with pytest.raises(AlignmentTimeout, match='timed out'):
        c.command(0, 0, math.pi, now=2)
    assert c.progress == 0


def test_resume_uses_current_path_direction_and_keeps_progress():
    c = controller()
    c.command(0, 0, 0, now=0)
    c.command(0, 0, 0, now=.5)
    c.tracker.progress = 5.5
    c.align_for_resume()
    v, w, *_ = c.command(5, .6, 0, now=10)
    assert c.phase == 'ALIGNING' and v == 0 and w == pytest.approx(.3)
    assert c.alignment_yaw == pytest.approx(math.pi/2)
    assert c.progress == 5.5
    c.command(5, .6, math.pi/2, now=11)
    c.command(5, .6, math.pi/2, now=11.5)
    assert c.phase == 'TRACKING' and c.progress == 5.5
    assert c.command(5, .6, math.pi/2, now=11.6)[0] > 0
    assert c.progress >= 5.5


def test_resume_during_approach_aligns_to_target_and_preserves_approach_leg():
    c = CoverageController(Polyline([(1, 0), (3, 0)]), (-2, 0))
    leg = c.approach_path
    c.align_for_resume()
    v, w, distance, *_ = c.command(-1, .5, math.pi/2, now=0)
    assert v == 0 and w < 0 and distance == pytest.approx(.5)
    assert c.alignment_yaw == pytest.approx(math.atan2(-.5, 2))
    assert c.alignment_next_phase == 'APPROACHING' and c.approach_path is leg
    c.command(-1, .5, c.alignment_yaw, now=1)
    c.command(-1, .5, c.alignment_yaw, now=1.5)
    assert c.phase == 'APPROACHING' and c.progress == 0


def test_resume_during_alignment_resets_dwell_and_timeout():
    c = controller(alignment_timeout=2)
    c.command(0, 0, 0, now=0)
    c.align_for_resume()
    c.command(0, 0, 0, now=100)
    assert c.phase == 'ALIGNING'
    c.command(0, 0, 0, now=100.5)
    assert c.phase == 'TRACKING'


def test_alignment_uses_wrapped_heading_error():
    c = CoverageController(Polyline([(0, 0), (-5, -.01)]), (0, 0))
    assert c.command(0, 0, math.pi-.01, now=0)[:2] == (0, 0)
    assert abs(c.alignment_error) < .02


@pytest.mark.parametrize('kwargs', [dict(alignment_tolerance=math.pi/2),
    dict(alignment_hold_time=0), dict(alignment_timeout=float('nan')),
    dict(alignment_max_angular=-.1)])
def test_invalid_alignment_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        controller(**kwargs)
