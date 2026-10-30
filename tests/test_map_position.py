"""Display-coordinate conversion only; no ROS context or hardware."""
import math

import pytest
from pyproj import Geod, Transformer

from agri_backend.map_position import geographic_pose

ORIGIN = {'latitude': 36.6468527633281, 'longitude': 117.021433066391, 'epsg': 32650}


@pytest.mark.parametrize('bearing', [0, 90, 180, 270, 34])
def test_display_uses_same_base_link_point_and_true_bearing(bearing):
    geod = Geod(ellps='WGS84')
    # A point away from the UTM central meridian also verifies grid convergence.
    origin = {**ORIGIN, 'longitude': 118.8}
    project = Transformer.from_crs(4326, 32650, always_xy=True)
    e0, n0 = project.transform(origin['longitude'], origin['latitude'])
    lon, lat, _ = geod.fwd(origin['longitude'], origin['latitude'], 30, 123)
    lon2, lat2, _ = geod.fwd(lon, lat, bearing, 1)
    e, n = project.transform(lon, lat)
    e2, n2 = project.transform(lon2, lat2)
    grid_yaw = math.degrees(math.atan2(n2 - n, e2 - e))
    result = geographic_pose({'position_x': e - e0, 'position_y': n - n0, 'heading': grid_yaw}, origin)
    assert result['longitude'] == pytest.approx(lon, abs=1e-9)
    assert result['latitude'] == pytest.approx(lat, abs=1e-9)
    assert (result['bearing'] - bearing + 180) % 360 - 180 == pytest.approx(0, abs=1e-5)
    assert result['reference'] == 'base_link'


@pytest.mark.parametrize('data,projection', [
    ({'position_x': None, 'position_y': 0}, ORIGIN),
    ({'position_x': float('nan'), 'position_y': 0}, ORIGIN),
    ({'position_x': 0, 'position_y': 0}, None),
    ({'position_x': 0, 'position_y': 0}, {**ORIGIN, 'epsg': 32649}),
    ({'position_x': 0, 'position_y': 0}, {**ORIGIN, 'latitude': 90}),
])
def test_unknown_or_mismatched_pose_is_not_a_location(data, projection):
    assert geographic_pose(data, projection) is None


def test_unknown_heading_preserves_position_without_inventing_an_arrow():
    result = geographic_pose({'position_x': 0, 'position_y': 0, 'heading': None}, ORIGIN)
    assert result['bearing'] is None
    assert result['longitude'] == pytest.approx(ORIGIN['longitude'])


def test_robot_api_exposes_projected_pose_only_with_fresh_odom_and_ros_origin(monkeypatch):
    from agri_backend import main
    from agri_backend.devices import device_cache
    monkeypatch.setattr(main, 'robot_state', main.RobotState())
    monkeypatch.setattr(main, 'monotonic', lambda: 10.0)
    monkeypatch.setattr(device_cache, 'values', {})
    main.robot_state.update(position_x=1.5, position_y=2, heading=90, last_odom_monotonic=9.9)
    assert main.get_robot_status()['map_position'] is None
    device_cache.update('origin', {'projection': ORIGIN})
    # Raw GNSS can be at an antenna elsewhere; it must not overwrite the vehicle pose.
    device_cache.update('gnss', {'latitude': 30, 'longitude': 110})
    result = main.get_robot_status()
    assert result['latitude'] == 30
    assert result['map_position']['latitude'] > ORIGIN['latitude']
    assert result['map_position']['longitude'] > ORIGIN['longitude']
    main.monotonic = lambda: 10.91
    assert main.get_robot_status()['map_position'] is None
