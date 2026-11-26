"""Offline contract tests; physical closure is verified by separate Gazebo runs."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from model import build_world, DESCRIPTION
from runner import Localization, physical_config
from simulate import SimulationConfig


def test_delayed_localization_uses_acquisition_stamp_and_holds_between_samples():
    model = Localization(SimulationConfig(feedback_period=.1), delay=.2)
    assert model.update((0., 1., 2., .3))[0] is None
    assert model.update((.1, 2., 3., .4))[0] is None
    observation, fresh = model.update((.2, 3., 4., .5))
    assert observation == (0., 1., 2., .3) and fresh
    same, fresh = model.update((.25, 4., 5., .6))
    assert same == observation and not fresh
    observation, fresh = model.update((.3, 4., 5., .6))
    assert observation == (.1, 2., 3., .4) and fresh


def test_noise_seed_is_reproducible_and_never_changes_truth():
    settings = SimulationConfig(position_noise=.02, heading_noise_deg=.5, seed=42)
    truth = (0., 5., 8., .7)
    left, right = Localization(settings, 0), Localization(settings, 0)
    a, _ = left.update(truth)
    b, _ = right.update(truth)
    assert a == b and a != truth
    assert truth == (0., 5., 8., .7)


def test_speed_feedback_has_same_delay_as_pose():
    model = Localization(SimulationConfig(feedback_period=.1), delay=.2)
    model.update((0., 0., 0., 0., .5, 0., 0.))
    model.update((.1, .1, 0., 0., .7, 0., 0.))
    assert model.delivered is None and model.delivered_speed == 0.
    observation, _ = model.update((.2, .2, 0., 0., .9, 0., 0.))
    assert observation[0] == 0.
    assert model.delivered_speed == .5


def test_world_keeps_absolute_map_and_has_real_wheel_collisions(tmp_path):
    before = (DESCRIPTION/'urdf/yahboomcar_X3.urdf').read_bytes()
    filename = tmp_path/'world.sdf'
    initial = (110., -30., .5)
    build_world(filename, [(110., -30., 0.), (111., -30., 0.)], initial, physical_config(HERE/'config.yaml'))
    world = ET.parse(filename).getroot().find('world')
    robot = world.find("model[@name='tracking_robot']")
    assert list(map(float, robot.findtext('pose').split()))[:2] == list(initial[:2])
    assert len(robot.findall('joint')) == 4
    assert len(robot.findall('link/collision/geometry/cylinder')) == 4
    wheel = robot.find("link[@name='front_left_wheel']")
    assert wheel.findtext('collision/surface/friction/ode/fdir1') == '0 0 1'
    drive = robot.find("plugin[@filename='libgazebo_ros_diff_drive.so']")
    assert drive.findtext('ros/namespace') == '/tracking_sim'
    assert drive.findtext('publish_odom_tf') == 'false'
    assert drive.findtext('num_wheel_pairs') == '2'
    truth = robot.find("plugin[@filename='libgazebo_ros_p3d.so']")
    assert truth.findtext('frame_name') == 'map'
    assert not world.findall("model[@name='reference_path']//collision")
    assert 'planar_move' not in filename.read_text()
    assert before == (DESCRIPTION/'urdf/yahboomcar_X3.urdf').read_bytes()


@pytest.mark.parametrize('value', [-1, math.nan, math.inf])
def test_bad_delay_is_rejected(tmp_path, value):
    import yaml
    config = physical_config(HERE/'config.yaml')
    config['physics']['feedback_delay'] = value
    path = tmp_path/'bad.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        physical_config(path)
