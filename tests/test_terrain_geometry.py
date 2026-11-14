"""Native work-area completeness and static forbidden-region semantics."""
import math

import pytest
from shapely.geometry import Polygon, box

from agri_path_planning.field import validate_native_work_area
from agri_path_planning.point_turn_planner import StraightRouter


def neck(width):
    return Polygon([(0,0),(8,0),(8,5),(14,5),(14,0),(22,0),(22,12),(14,12),
                    (14,5+width),(8,5+width),(8,12),(0,12)])


@pytest.mark.parametrize('width',[.2,1.2])
def test_native_headland_split_never_silently_returns_the_first_work_area(width):
    field=neck(width)
    assert field.buffer(-3).geom_type=='MultiPolygon'
    with pytest.raises(ValueError,match='分裂.*拒绝仅规划其中一块'):
        validate_native_work_area(field,3)
    assert validate_native_work_area(field,0).equals(field)
    if width>1:
        # This can still be traversable with the intelligent planner's smaller
        # envelope. Native API rejection must not claim physical impossibility.
        assert validate_native_work_area(field,.296321).geom_type=='Polygon'


@pytest.mark.parametrize('width',[math.inf,math.nan,-1,100])
def test_empty_or_invalid_headland_is_rejected_before_native_call(width):
    with pytest.raises(ValueError):validate_native_work_area(box(0,0,20,16),width)


def test_regular_and_single_hole_work_areas_are_preserved():
    field=Polygon([(0,0),(28,0),(28,22),(0,22)],holes=[[(11,8),(17,8),(17,14),(11,14)]])
    working=validate_native_work_area(field,3)
    assert working.geom_type=='Polygon' and len(working.interiors)==1
    assert working.area<field.area
    assert not working.intersects(box(11,8,17,14))


def test_visibility_connector_detours_a_hole_and_refuses_disconnected_regions():
    from shapely.geometry import LineString
    field=box(0,0,28,22).difference(box(11,8,17,14))
    safe=field.buffer(-.296321)
    route=StraightRouter(safe,lambda:None).route((5,11),(23,11))
    line=LineString(route)
    assert line.length>18 and len(route)>2
    assert safe.buffer(1e-5).covers(line)
    assert not line.intersects(box(11,8,17,14))
    split=neck(.2).buffer(-.296321)
    with pytest.raises(ValueError,match='不连通'):
        StraightRouter(split,lambda:None).route((3,3),(19,3))
