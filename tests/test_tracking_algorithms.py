"""Production algorithms: geometry, segment transitions and failure behavior; no ROS IO."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from agri_path_tracking.algorithms import (
    ALGORITHMS, DEFAULT_TUNING, ControllerFault, controller_metadata, make_tracker,
)
from agri_path_tracking.controller import Polyline
from agri_path_tracking.coverage import CoverageController
from agri_backend.missions import CoverageParameters
from agri_backend.taught_paths import FollowingParameters


@pytest.mark.parametrize('name', ALGORITHMS)
def test_api_choice_validation_and_recording(name):
    for model in (CoverageParameters, FollowingParameters):
        assert model(controller_algorithm=name).controller_algorithm == name
        assert model().controller_algorithm == 'pp'
        for invalid in ('legacy_pp', 'PP', 'unknown', '', None):
            with pytest.raises(ValueError):
                model(controller_algorithm=invalid)
    meta = controller_metadata(name)
    assert meta['algorithm'] == name and meta['tuning'] == DEFAULT_TUNING[name]
    meta['tuning'].clear()
    assert controller_metadata(name)['tuning']


@pytest.mark.parametrize('name', ALGORITHMS)
def test_forward_correction_reverse_and_velocity_bounds(name):
    for direction, yaw in ((1, 0.), (-1, math.pi)):
        path = Polyline([(0., 0.), (3., 0.)], [direction])
        tracker = make_tracker(path, name, speed=.3, max_angular=.4)
        v, w, _, lateral, _, done = tracker.command(.2, .1, yaw)
        assert direction * v > 0 and abs(v) <= .3
        assert -.4 <= w < 0 and lateral > 0 and not done
        assert tracker.algorithm_id == name


@pytest.mark.parametrize('name', ALGORITHMS)
def test_point_turn_and_cusp_stop_before_advancing(name):
    for directions in ([1, 1], [1, -1]):
        path = Polyline([(0., 0.), (2., 0.), (2., 2.)], directions, [1])
        tracker = make_tracker(path, name, speed=.3)
        tracker.progress = 1.8
        # The controller must not look or predict through the marked turn.
        v, w, *_ = tracker.command(1.9, 0., 0.)
        assert v > 0 and abs(w) < 1e-6
        v, w, *_, done = tracker.command(1.98, 0., 0.)
        assert (v, w) == (0., 0.) and tracker.needs_switch and not done
        tracker.advance_leg()
        assert tracker.leg_index == 1 and not tracker.needs_switch
        yaw = math.pi/2 + (math.pi if directions[1] < 0 else 0)
        assert directions[1] * tracker.command(2., .1, yaw)[0] > 0


@pytest.mark.parametrize('name', ALGORITHMS)
@pytest.mark.parametrize('reverse', [False, True])
def test_closed_loop_point_turn_completion_and_resume_alignment(name, reverse):
    points = [(0., 0.), (1., 0.), (0., 0.)] if reverse else [(0., 0.), (1., 0.), (1., 1.)]
    path = Polyline(points, [1, -1] if reverse else [1, 1], stop_indices=[1])
    control = CoverageController(path, (0., 0.), controller_algorithm=name,
                                 speed=.3, completion_tolerance=.1)
    x, y, yaw, resumed = 0., .03, 0., False
    phases = set()
    for step in range(1800):
        t = step * .05
        phases.add(control.phase)
        if t > 2 and not resumed:
            control.align_for_resume()
            resumed = True
            assert control.phase == 'ALIGNING'
        if hasattr(control.tracker, 'set_speed_feedback'):
            control.tracker.set_speed_feedback(.3 if control.phase == 'TRACKING' else 0.)
        v, w, *_, done = control.command(x, y, yaw, now=t, sample_stamp=t)
        assert math.isfinite(v+w) and abs(v) <= .3 and abs(w) <= 1.
        if control.phase == 'ALIGNING':
            assert v == 0
        if done:
            break
        x += v*math.cos(yaw)*.05
        y += v*math.sin(yaw)*.05
        yaw = math.atan2(math.sin(yaw+w*.05), math.cos(yaw+w*.05))
    else:
        pytest.fail(f'{name} did not finish: {(x,y,yaw,control.phase,control.progress)}')
    assert phases == {'ALIGNING', 'TRACKING'} and control.tracker.leg_index == 1
    assert math.dist((x, y), points[-1]) <= .1


@pytest.mark.parametrize('name', ALGORITHMS)
def test_ported_forward_law_matches_simulation_reference(name):
    filename = Path(__file__).resolve().parents[1]/'tracking_simulation/four_controler/four_controler.py'
    spec = importlib.util.spec_from_file_location('reference_four_controllers', filename)
    ref = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ref)
    points = [(i*.1, .03*math.sin(i*.1)) for i in range(31)]
    path = Polyline(points)
    settings = dict(lookahead=.5, speed=.3, max_angular=1., completion_tolerance=.1)
    a = make_tracker(path, name, **settings)
    factory = ref.build_controller(name, dt=.05, tuning=DEFAULT_TUNING)
    b = factory(path, **settings)
    for x, y, yaw in ((.2, .06, .02), (.5, -.05, .01), (1., .03, .02)):
        if hasattr(a, 'set_speed_feedback'):
            a.set_speed_feedback(.25)
            b.set_speed_feedback(.25)
        assert np.allclose(a.command(x, y, yaw), b.command(x, y, yaw), atol=1e-7)


def test_new_pp_speed_feedback_changes_lookahead_without_speed_pid():
    tracker = make_tracker(Polyline([(0,0), (4,0)]), 'pp', lookahead=.8, speed=.3)
    tracker.set_speed_feedback(.4)
    v, *_ = tracker.command(0, 0, 0)
    assert tracker.effective_lookahead == pytest.approx(.94)
    assert v == .3
    with pytest.raises(ValueError):
        tracker.set_speed_feedback(float('nan'))


def test_mpc_nonconvergence_is_a_fault_not_a_command(monkeypatch):
    from agri_path_tracking import algorithms
    tracker = make_tracker(Polyline([(0,0), (4,0)]), 'mpc')
    monkeypatch.setattr(algorithms, 'box_qp', lambda *a, **kw: (np.zeros(20), False, 200))
    with pytest.raises(ControllerFault, match='converge'):
        tracker.command(.1, .1, .05)
    assert tracker.optimization_unconverged == 1


def test_numerical_solver_exception_is_a_control_fault(monkeypatch):
    def singular(*args, **kwargs):
        raise np.linalg.LinAlgError('injected singular matrix')
    monkeypatch.setattr(np.linalg, 'solve', singular)
    path = Polyline([(0,0), (4,0)])
    with pytest.raises(ControllerFault, match='initialization'):
        make_tracker(path, 'lqr')
    tracker = make_tracker(path, 'mpc')
    with pytest.raises(ControllerFault, match='numerical solve'):
        tracker.command(.1, .1, .05)
