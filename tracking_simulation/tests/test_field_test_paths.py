"""Analytic field turn geometry and production import compatibility, no ROS IO."""
import math
from pathlib import Path
import sys

import pytest
from shapely.geometry import LineString

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from generate_field_test_paths import geometry, pear_geometry
from agri_backend.path_import import preserve_geometry


@pytest.mark.parametrize('radius', [1.5, 2.])
def test_pear_has_specified_radius_tangency_length_and_no_intersections(radius):
    rows = pear_geometry(10., radius, .05)
    assert rows[0] == (0., 0., 0., 0., 'outbound')
    assert rows[-1][:3] == (0., 2., math.pi)
    alpha = math.acos((2.+2*radius)/(4*radius))
    assert rows[-1][3] == pytest.approx(20+radius*(math.pi+4*alpha))
    assert LineString([r[:2] for r in rows]).is_simple
    centers = [(10., -radius), (10+2*radius*math.sin(alpha), 1.), (10., 2.+radius)]
    regions = ['arc_right_entry', 'arc_left_main', 'arc_right_exit']
    for region, center, sign in zip(regions, centers, [-1, 1, -1]):
        for u, left, yaw, _, label in rows:
            if label == region:
                assert math.dist((u, left), center) == pytest.approx(radius, abs=1e-10)
                assert (u-center[0], left-center[1]) == pytest.approx(
                    (sign*radius*math.sin(yaw), -sign*radius*math.cos(yaw)), abs=1e-10)
    for a, b in zip(rows, rows[1:]):
        ds = b[3]-a[3]
        assert 0 < ds <= .050000001
        assert math.dist(a[:2], b[:2]) <= ds+1e-12
        assert abs(b[2]-a[2]) <= ds/radius+1e-12
        chord = math.atan2(b[1]-a[1], b[0]-a[0])
        assert abs(math.atan2(math.sin(chord-a[2]), math.cos(chord-a[2]))) <= .05/(2*radius)+1e-10
    points = [dict(x=r[0], y=r[1], yaw=math.atan2(math.sin(r[2]), math.cos(r[2]))) for r in rows]
    result = preserve_geometry(points, {})
    assert result['points'] == points
    assert result['statistics']['point_turn_count'] == 0
    assert result['statistics']['reverse_segments'] == 0


@pytest.mark.parametrize('radius', [1.5, 2., 2.5])
def test_u_geometry_remains_the_same(radius):
    rows = geometry(10., radius, .05)
    assert rows[-1][:3] == (0., 2*radius, math.pi)
    assert rows[-1][3] == pytest.approx(20+math.pi*radius)
    assert all(math.dist(a[:2],b[:2]) <= .050000001 for a,b in zip(rows,rows[1:]))


@pytest.mark.parametrize('radius,spacing,distance', [(1.,.05,2.),(1.5,0.,2.),(-1.,.05,2.),(1.5,.05,math.nan)])
def test_invalid_pear_dimensions_are_rejected(radius, spacing, distance):
    with pytest.raises(ValueError):
        pear_geometry(10., radius, spacing, distance)
