"""Noise, causal actuator response and shared closed-loop behavior tests."""
from dataclasses import replace
import math
from pathlib import Path
import statistics
import sys

import pytest

SIM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM))
from disturbances import FirstOrderActuator, GaussianPoseNoise
from simulate import SimulationConfig, forward_poses, integrate, simulate
from run_tracking import read_config
from four_controler.four_controler import NAMES, build_controller


def test_noise_is_seeded_independent_zero_mean_with_requested_units():
    noise = GaussianPoseNoise(.03, .5, 42)
    same = GaussianPoseNoise(.03, .5, 42)
    assert noise.sample(1., 2., .3) == same.sample(1., 2., .3)
    errors = [noise.sample(0., 0., 0.) for _ in range(20000)]
    for i, std in enumerate((.03, .03, math.radians(.5))):
        values = [e[i] for e in errors]
        assert abs(statistics.mean(values)) < .03*std
        assert statistics.pstdev(values) == pytest.approx(std, rel=.025)
    wrapped = GaussianPoseNoise(0., 20., 1)
    assert all(-math.pi <= wrapped.sample(0., 0., math.pi-.001)[2] < math.pi for _ in range(100))


def test_actuator_step_response_and_causality():
    actuator = FirstOrderActuator(.15, .3)
    assert actuator.command(1., 2., 0.) == (0., 0.)
    v, w = actuator.advance(.15)
    assert v == pytest.approx(1-math.exp(-1))
    assert w == pytest.approx(2*(1-math.exp(-.5)))
    assert actuator.command(0., 0., .15) == pytest.approx((v, w))
    assert actuator.advance(.3) == pytest.approx((v*math.exp(-1), w*math.exp(-.5)))


def test_actuator_partition_invariance_zero_tau_and_fault_reset():
    one, many = FirstOrderActuator(.15, .15), FirstOrderActuator(.15, .15)
    one.command(1., -1., 0.)
    many.command(1., -1., 0.)
    expected = one.advance(.15)
    for t in (.05, .1, .15):
        result = many.advance(t)
    assert result == pytest.approx(expected)
    assert many.reset(.15) == (0., 0.)
    assert many.advance(1.) == (0., 0.)  # no stale command can revive motion
    direct = FirstOrderActuator()
    assert direct.command(.5, -.2, 0.) == (.5, -.2)


@pytest.mark.parametrize('bad', [-.1, math.nan, math.inf, True])
def test_invalid_lag_constants_rejected(bad):
    with pytest.raises(ValueError):
        FirstOrderActuator(bad, .15)


def test_actual_motion_uses_lagged_output_and_zero_tau_remains_direct():
    path = forward_poses([(i*.05, 0.) for i in range(101)])
    config = SimulationConfig(speed=.5, initial_lateral=0., initial_heading_deg=0., duration=.5)
    direct, _ = simulate(path, config)
    delayed, _ = simulate(path, replace(config, actuator_linear_tau=.15, actuator_angular_tau=.15))
    assert direct[1]['x_true_m'] == pytest.approx(.025)
    first = delayed[1]
    expected = .5*(.05-.15*(1-math.exp(-.05/.15)))
    assert first['x_true_m'] == pytest.approx(expected, abs=1e-5)
    assert 0 < first['v_actual_body_m_s'] < first['v_command_m_s']
    assert delayed[-1]['phase'] == 'BRAKING'
    assert abs(delayed[-1]['v_actual_body_m_s']) < .001
    assert all(r['v_command_m_s'] == r['w_command_rad_s'] == 0 for r in delayed if r['time_s'] >= .5)


def test_point_turn_reverse_and_pause_keep_working_with_actuator_lag():
    poses = [(0., 0., 0.), (1., 0., 0.), (1., 0., math.pi/2),
             (1., 1., math.pi/2), (1., 0., math.pi/2)]
    rows, metrics = simulate(poses, SimulationConfig(speed=.5, initial_lateral=.1, duration=80.,
        actuator_linear_tau=.15, actuator_angular_tau=.15))
    assert metrics['state'] == 'COMPLETED'
    assert any(r['v_actual_body_m_s'] < -.01 for r in rows)
    assert any(r['phase'] == 'ALIGNING' for r in rows)


@pytest.mark.parametrize('name', NAMES)
def test_four_controllers_close_loop_with_noise_and_lag(name):
    data, _, _, _ = read_config(SIM/'config.yaml')
    path = forward_poses([(i*.05, .2*math.sin(i*.05)) for i in range(101)])
    # A regression test must not change its disturbance scenario when the user
    # disables noise/lag in their everyday run profile.
    config = SimulationConfig(speed=.5, initial_lateral=.1, initial_heading_deg=5.,
        feedback_period=.1, position_noise=.03, heading_noise_deg=.5,
        actuator_linear_tau=.15, actuator_angular_tau=.15, duration=40., seed=0)
    rows, metrics = simulate(path, config,
        controller_factory=build_controller(name, dt=config.dt, tuning=data['tuning']))
    assert metrics['state'] == 'COMPLETED'
    assert metrics['optimizer']['unconverged'] == 0
    assert any(abs(r['x_observed_m']-r['x_true_m'])>.01 for r in rows)
    assert any(abs(r['yaw_observed_rad']-r['yaw_true_rad'])>math.radians(.1) for r in rows)
    assert any(abs(r['v_actuator_m_s']-r['v_command_m_s'])>.05 for r in rows)
    paused, _ = simulate(path, replace(config, duration=3.), pause_intervals=((1., 2.),))
    assert all(r['v_actuator_m_s'] == r['w_actuator_rad_s'] == 0 for r in paused if r['phase'] == 'PAUSED')
