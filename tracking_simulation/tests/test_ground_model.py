"""Causal response, archived data semantics and independent validation checks."""
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
from ground_model import (GroundActuator, GroundRun, fit_channel,
                          pose_window_check, read_ground_run, replay_channel)
from calibrate_ground import identify
from run_tracking import run
from simulate import SimulationConfig, forward_poses, simulate


def profile(**changes):
    result = dict(schema_version=1, kind='X3_GROUND_EQUIVALENT_RESPONSE',
        linear=dict(gain=.8, limit=.5, tau_s=.2, delay_s=.1),
        angular=dict(gain=1., limit=1., tau_s=.3, delay_s=.2), feedback_period_s=.1)
    result.update(changes)
    return result


def test_delayed_step_is_causal_and_independent_per_channel():
    actuator = GroundActuator(profile())
    assert actuator.command(1., -2., 0.) == (0., 0.)
    assert actuator.advance(.1) == (0., 0.)
    v, w = actuator.advance(.2)
    assert v == pytest.approx(.5*(1-math.exp(-.1/.2)))
    assert w == 0.
    v, w = actuator.advance(.5)
    assert v == pytest.approx(.5*(1-math.exp(-.4/.2)))
    assert w == pytest.approx(-(1-math.exp(-.3/.3)))


def test_fault_reset_purges_delayed_targets_and_no_old_motion_revives():
    actuator = GroundActuator(profile())
    actuator.command(.5, .5, 0.)
    actuator.command(.2, -.5, .05)
    assert actuator.reset(.075) == (0., 0.)
    assert actuator.advance(2.) == (0., 0.)
    assert actuator.command(.25, 0., 2.) == (0., 0.)
    assert actuator.advance(2.3)[0] > 0


def test_exact_partition_invariance_with_multiple_commands():
    left, right = GroundActuator(profile()), GroundActuator(profile())
    for actuator in (left, right):
        actuator.command(.4, .8, 0.)
        actuator.command(-.3, 0., .05)
    for t in np.arange(.06, .501, .01):
        right.advance(float(t))
    assert left.advance(.5) == pytest.approx(right.output)


@pytest.mark.parametrize('value', [-1, math.nan, math.inf, True])
def test_profile_rejects_invalid_values(value):
    item = profile()
    item['linear']['delay_s'] = value
    with pytest.raises(ValueError):
        GroundActuator(item)


def test_zero_tau_delay_still_defers_motion_and_zero_delay_applies_gain():
    item = profile()
    item['linear']['tau_s'] = 0.
    model = GroundActuator(item)
    model.command(-.25, 0., 0.)
    assert model.advance(.099)[0] == 0
    assert model.advance(.1)[0] == pytest.approx(-.2)
    item['linear']['delay_s'] = 0.
    model = GroundActuator(item)
    assert model.command(-.25, 0., 0.)[0] == pytest.approx(-.2)


def synthetic_run():
    t = np.arange(0., 24., .01)
    u = np.where((t % 8) < 2, 0., np.where((t % 8) < 5, .4, .9))
    item = GroundRun('synthetic', t, np.column_stack((u, u)), t,
                     np.zeros((len(t), 2)), np.zeros((len(t), 3)), t > 2., {})
    params = dict(gain=.9, limit=.6, tau_s=.25, delay_s=.075)
    item.measured[:, 0] = replay_channel(item, 0, params)
    item.measured[:, 1] = replay_channel(item, 1, params)
    return item, params


def test_identification_recovers_known_excited_response_without_measurement_resets():
    data, expected = synthetic_run()
    result = fit_channel([data], 0)
    for key in expected:
        assert result[key] == pytest.approx(expected[key], abs=.002)
    prediction = replay_channel(data, 0, result)
    assert np.sqrt(np.mean((prediction-data.measured[:, 0])**2)) < .001
    data.measured[:] = 50.  # replay must depend only on input, not future feedback
    assert replay_channel(data, 0, result) == pytest.approx(prediction)


def test_synthetic_holdout_and_five_second_pose_windows():
    t = np.arange(0., 12.01, .1)
    speed = np.column_stack((np.full(len(t), .5), np.zeros(len(t))))
    poses = np.column_stack((.5*t, np.zeros((len(t), 2))))
    data = GroundRun('constant', t, speed, t, speed, poses, np.ones(len(t), dtype=bool), {})
    metrics, windows = pose_window_check(data, speed, speed)
    assert metrics['windows'] == 2 and len(windows) == 2
    assert metrics['model_endpoint_rmse_m'] < 1e-10
    data.valid[:60] = False
    assert pose_window_check(data, speed, speed)[0]['windows'] == 0


def test_training_and_validation_disjoint_before_any_io(tmp_path):
    with pytest.raises(ValueError, match='互不重合'):
        identify([tmp_path/'same'], [tmp_path/'same'])


def test_validation_not_given_to_fitting(monkeypatch, tmp_path):
    import calibrate_ground
    names = []
    def load(path):
        data, _ = synthetic_run()
        data.name = path.name
        data.metadata = dict(projection={'epsg': 32650}, feedback_period_median_s=.1,
                             source_sha256={}, max_abs_command=[.9, .9])
        return data
    def fit(runs, channel):
        names.extend(r.name for r in runs)
        return dict(gain=1., limit=1., tau_s=.1, delay_s=0.)
    monkeypatch.setattr(calibrate_ground, 'read_ground_run', load)
    monkeypatch.setattr(calibrate_ground, 'fit_channel', fit)
    result, training, validation = identify([tmp_path/'train'], [tmp_path/'heldout'])
    assert names == ['train', 'train']
    assert result['training_runs'] == ['train'] and result['validation_runs'] == ['heldout']


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def test_mgx_projection_exact_pairing_signed_velocity_and_snapshot_unchanged(tmp_path):
    from agri_base.coordinates import LocalFrame
    projection = dict(latitude=36.6468527633281, longitude=117.021433066391, epsg=32650)
    frame = LocalFrame(projection['latitude'], projection['longitude'])
    north_yaw = frame.yaw(projection['latitude'], projection['longitude'], 0.)
    (tmp_path/'run.json').write_text(json.dumps(dict(mode='coverage', state='COMPLETED', projection=projection)))
    (tmp_path/'config').mkdir()
    (tmp_path/'config/system.json').write_text(json.dumps(dict(localization=dict(ros__parameters=dict(antenna_x=.2, antenna_y=.1)))))
    motion, mgx, odom = [], [], []
    stamp0 = 1793854440000000000
    for i in range(160):
        stamp = stamp0+i*100_000_000
        # Point south, body speed is negative despite positive north velocity.
        yaw = north_yaw+math.pi
        odom.append(dict(source_stamp_ns=str(stamp), x=1, y=2, yaw=yaw, angular_z=.2))
        motion.append(dict(received_ros_ns=str(stamp), topic='/base/driver_cmd_vel', linear_x=.1, linear_y=0., angular_z=.2))
        mgx += [dict(source_stamp_ns=str(stamp), received_ros_ns=str(stamp+10_000_000), topic='/mgx23/position', v1=projection['latitude'], v2=projection['longitude']),
                dict(source_stamp_ns=str(stamp), received_ros_ns=str(stamp+10_000_000), topic='/mgx23/velocity_stamped', v1=.3, v2=0.)]
    del odom[100]  # must be excluded, not paired to a neighbor/future pose
    write_csv(tmp_path/'telemetry/motion.csv', motion)
    write_csv(tmp_path/'telemetry/mgx23.csv', mgx)
    write_csv(tmp_path/'telemetry/odometry.csv', odom)
    snapshot = {p: p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    data = read_ground_run(tmp_path)
    assert data.metadata['unmatched_velocity_frames'] == 1
    assert data.metadata['samples'] == 159
    assert data.measured[:, 0] == pytest.approx(np.full(159, -.3+.2*.1))
    assert data.measured[:, 1] == pytest.approx(np.full(159, .2))
    assert data.metadata['velocity_receive_age_p95_s'] == pytest.approx(.01)
    assert all(p.read_bytes() == content for p, content in snapshot.items())


def test_ground_simulation_uses_delayed_output_and_keeps_original_defaults():
    before = (SIM/'config.yaml').read_bytes()
    config = SimulationConfig(duration=2., speed=.3, initial_lateral=0., initial_heading_deg=0.)
    rows, metrics = simulate(forward_poses([(0., 0.), (20., 0.)]), config, model_profile=profile())
    assert rows[0]['v_command_m_s'] == .3
    assert rows[0]['v_actual_body_m_s'] == 0
    assert rows[1]['x_true_m'] == 0
    assert 0 < rows[10]['v_actual_body_m_s'] < .3*.8
    assert metrics['model_profile']['linear']['gain'] == .8
    assert metrics['reference_regions']['turn']['samples'] == 0
    assert before == (SIM/'config.yaml').read_bytes()


def test_ideal_and_ground_profile_cannot_be_accidentally_combined():
    with pytest.raises(ValueError, match='不能同时'):
        run(SIM/'config.yaml', ideal=True, model_profile=SIM/'not_needed.json')
