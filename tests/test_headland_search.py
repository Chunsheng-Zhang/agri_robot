"""Automatic range inference and bounded width refinement, no ROS or hardware."""
import copy
from threading import Event
import pytest
from shapely.geometry import box, Polygon
from agri_path_planning import contour
from agri_path_planning.contour import Builder
from agri_path_planning.headland_search import initial_widths,refine_widths


def builder(field=None,**kw):
    p=dict(width=1.,headland=3.,radius=1.,spacing=.1,robot_width=.35,continuity='DUBINS')
    p.update(kw)
    return Builder(box(0,0,20,16) if field is None else field,**p)


def test_manual_width_is_exact_and_does_not_probe_curves(monkeypatch):
    b=builder()
    monkeypatch.setattr(b,'curve_between',lambda *_:pytest.fail('Manual mode must not search'))
    for requested in (0.,1.23456789,5.):
        values,meta=initial_widths(b,'MANUAL',requested)
        assert values==[requested] and meta['range_min_m']==meta['range_max_m']==requested
        assert refine_widths(meta,requested)==[]


def test_range_comes_from_actual_field_not_old_fixed_bounds_and_keeps_usable_interior():
    small=builder(box(0,0,8,6),radius=2.);large=builder(box(0,0,24,20),radius=2.)
    _,a=initial_widths(small,'AUTO',3);values,b=initial_widths(large,'AUTO',3)
    assert a['range_max_m']<3 and b['range_max_m']>4
    assert a['range_max_m']<b['range_max_m']
    assert b['native_turn_probes'] and b['curve_type']=='DUBINS'
    for h in values:
        assert not large.field.buffer(-(h+.5),join_style=2).is_empty


def test_interior_holes_reduce_geometric_width_envelope():
    outer=box(0,0,20,16)
    holed=Polygon(outer.exterior.coords,holes=[box(5,4,15,12).exterior.coords])
    _,whole=initial_widths(builder(outer),'AUTO',3)
    _,split=initial_widths(builder(holed),'AUTO',3)
    assert split['range_max_m']<whole['range_max_m']


def test_vehicle_curve_and_reverse_permission_are_recorded_and_manual_value_is_ignored_in_auto():
    b=builder(radius=1.5,continuity='REEDS_SHEPP_HC',allow_reverse=True)
    values,meta=initial_widths(b,'AUTO',0)
    other,again=initial_widths(builder(radius=1.5,continuity='REEDS_SHEPP_HC',allow_reverse=True),'AUTO',40)
    assert values==other and meta['vehicle_radius_m']==1.5 and meta['allow_reverse'] is True
    assert meta['native_turn_probes'] and meta['curve_type']=='REEDS_SHEPP_HC'


def test_refinement_bisects_neighbours_without_duplicate_trials_and_recovers_without_initial_solution():
    meta={'method':'GEOMETRY_NATIVE_ADAPTIVE','initial_widths_m':[.2,1.4,4.],
          'refinement_widths_m':[],'width_resolution_m':.1,'range_min_m':.2,'range_max_m':4.}
    assert refine_widths(meta,1.4)==[.8,2.7]
    assert refine_widths(meta,1.4)==[1.1,2.05]
    trial=refine_widths(meta,None)
    assert trial and len(set(meta['refinement_widths_m']))==len(meta['refinement_widths_m'])


def test_no_coarse_solution_expands_turn_envelope_only_up_to_geometric_limit():
    meta={'method':'GEOMETRY_NATIVE_ADAPTIVE','initial_widths_m':[.2,1.4,3.],
          'refinement_widths_m':[],'width_resolution_m':.1,'range_min_m':.2,
          'range_max_m':3.,'geometric_limit_m':5.,'operation_width_m':1.}
    assert refine_widths(meta,None)==[5.]
    assert meta['range_max_m']==5.
    assert all(h<=5 for h in refine_widths(meta,None))


def test_geometry_search_honours_cancellation_and_rejects_an_unusable_field(monkeypatch):
    event=Event();event.set();monkeypatch.setattr(contour,'_CANCEL',event)
    with pytest.raises(ValueError,match='取消'):initial_widths(builder(),'AUTO',3)
    event.clear()
    with pytest.raises(ValueError,match='地块过窄'):initial_widths(builder(box(0,0,1,1),width=2),'AUTO',3)
