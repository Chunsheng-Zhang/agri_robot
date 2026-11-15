"""Whole-field and independently oriented cells share one safe execution path."""
import math
from threading import Event
import pytest
from shapely.geometry import Polygon, box, LineString
from shapely.ops import unary_union
from agri_base.path_geometry import rotation_envelope, validate_point_turn_path, point_turn_indices
from agri_path_planning import contour, optimizer
from agri_path_planning.native_layouts import partition_cells, best_rows
from agri_path_planning.native_route import plan_native_route
from agri_path_planning.point_turn_planner import sample_path


def check():pass


def settings():
    return dict(operation_width=6.,robot_width=.2,robot_length=.45,rotation_safety_margin=.05,
                sampling_distance=.1,quality_resolution=.1)


def l_shape():
    return Polygon([(0,0),(24,0),(24,8),(8,8),(8,30),(0,30)])


def safe(field):
    env=rotation_envelope(.45,.2,.05)
    return field.buffer(-env['headland_width_m'],join_style=2),env


def test_whole_field_preserves_hole_and_native_connections_do_not_cross_it():
    field=Polygon([(0,0),(40,0),(40,28),(0,28)],holes=[[(16,8),(23,8),(23,19),(16,19)]])
    work,env=safe(field);before=field.wkb
    part=partition_cells(work,'NONE',None,check)
    assert len(part['polygons'])==1 and len(part['polygons'][0].interiors)==1
    prep=best_rows(part,6.,'NSwathModified',check)
    vertices,details=plan_native_route(prep,check)
    poses,_,_=sample_path(vertices,.1,check)
    validate_point_turn_path(poses,field,env)
    assert details['layout_family']=='NATIVE_WHOLE_FIELD'
    assert not LineString(vertices).intersects(Polygon(field.interiors[0]))
    assert len(details['swath_visits'])==len(prep['expected']) and field.wkb==before


def test_cells_choose_different_directions_without_extra_virtual_headlands():
    field=l_shape();work,env=safe(field)
    part=partition_cells(work,'TRAPEZOIDAL',0.,check)
    union=unary_union(part['polygons'])
    assert union.symmetric_difference(work).area<1e-4
    prep=best_rows(part,6.,'NSwath',check)
    angles={round(d['angle_rad'],5) for d in prep['cell_directions']}
    assert len(angles)==2 and max(angles)-min(angles)>math.pi/4
    vertices,details=plan_native_route(prep,check,guided_local_search=True)
    poses,_,_=sample_path(vertices,.1,check)
    validate_point_turn_path(poses,field,env)
    assert details['direction_mode']=='PER_CELL' and details['native_tsp']['guided_local_search']
    assert not details['continuous_cells_required']


@pytest.fixture(scope='module')
def planned():return optimizer.optimize(l_shape().wkb,settings())


def test_all_families_compete_on_the_same_full_path_objectives(planned):
    report=planned['optimization'];chosen=report['selected'];rows=report['candidates']
    assert planned['metadata']['policy_version']==7
    assert planned['metadata']['path_details']['layout_family']==chosen['layout_family']
    assert planned['metadata']['path_details']['direction_mode']==chosen['direction_mode']
    assert {r.get('layout_family') for r in rows if r.get('executable')}=={
        'EDGE_GLOBAL','NATIVE_WHOLE_FIELD','NATIVE_PER_CELL'}
    assert chosen['score']==min(r['score'] for r in rows if r.get('executable'))
    assert chosen['point_turn_count']==len(point_turn_indices(planned['points']))
    assert chosen['score']<=min(r['score'] for r in rows if r.get('layout_family')=='EDGE_GLOBAL' and r.get('executable'))
    assert report['search_complete'] and not report['candidate_limit_enabled'] and not report['search_time_limit_enabled']
    assert any(r['state']=='DEDUPLICATED' for r in rows)
    assert all(len(r['cell_directions'])==r['cell_count'] for r in rows if r.get('cell_directions'))
    assert chosen['length_m']==pytest.approx(chosen['swath_length_m']+chosen['transition_length_m'])


def test_native_stage_cancellation_never_returns_a_partial_old_winner(monkeypatch):
    event=Event();monkeypatch.setattr(contour,'_CANCEL',event)
    def progress(value):
        if value['phase']=='NATIVE_LAYOUTS':event.set()
    with pytest.raises(ValueError,match='取消'):
        optimizer.optimize(box(0,0,12,8).wkb,settings(),progress)


def test_native_candidate_failure_keeps_old_complete_candidates(monkeypatch):
    def rejected(*args):
        yield {'layout_family':'NATIVE_WHOLE_FIELD','direction_mode':'GLOBAL'},None,'deliberate invalid layout'
    monkeypatch.setattr(optimizer,'native_layouts',rejected)
    result=optimizer.optimize(box(0,0,12,8).wkb,settings())
    assert result['optimization']['state']=='SUCCEEDED'
    assert result['optimization']['selected']['layout_family']=='EDGE_GLOBAL'
    assert any(r.get('reason')=='deliberate invalid layout' for r in result['optimization']['candidates'])
