"""Point-turn planning policy and immutable legacy report compatibility."""
import copy
import json
import math
from threading import Event
import pytest
from shapely.geometry import box, Polygon, LineString
from agri_path_planning import contour
from agri_path_planning.contour import Builder
from agri_path_planning.optimizer import optimize, score, pca_angle
from agri_path_planning.planning_options import options, WEIGHTS
from agri_path_planning.turn_library import NativeTurns, energy
from agri_base.path_geometry import rotation_envelope, directed_poses, point_turn_indices, validate_point_turn_path
from agri_backend.missions import CoverageParameters
from agri_backend.optimization_results import read, summary


def settings(**changes):
    p=dict(operation_width=1.,robot_width=.2,robot_length=.45,rotation_safety_margin=.05,
        sampling_distance=.1,quality_resolution=.1)
    return {**p,**changes}


@pytest.fixture(scope='module')
def planned():return optimize(box(0,0,16,12).wkb,settings())


@pytest.mark.parametrize('values',[{'unexpected':1},None,[]])
def test_options_reject_unknown_fields_or_non_objects(values):
    with pytest.raises(ValueError):options(values)


def test_legacy_strategy_fields_are_discarded_and_input_is_not_rewritten():
    values={'optimization_headland_mode':'MANUAL','optimization_weight_overlap':0,
        'optimization_weight_length':0,'optimization_weight_energy':1,'energy_turn':100,
        'optimization_patterns':'CONTOUR','optimization_search_angle':'FIXED','optimization_min_coverage':100,
        'optimization_max_candidates':1,'optimization_time_budget':.001}
    before=copy.deepcopy(values)
    assert options(values)==options({})
    m=CoverageParameters(**values)
    assert not set(values)&set(m.model_dump()) and before==values
    assert m.robot_length==.45 and m.rotation_safety_margin==.05 and m.robot_width==.2


def test_fixed_score_rewards_coverage_and_penalizes_overlap_and_total_length():
    assert WEIGHTS=={'coverage':.5,'overlap':.3,'length':.2}
    assert score(100,0,100,100)==pytest.approx(.1)
    assert score(90,0,100,100)>score(100,0,100,100)
    assert score(100,10,100,100)>score(100,0,100,100)
    assert score(100,0,120,100)>score(100,0,100,100)
    assert score(50,50,1e9,100)<1


def test_plan_has_one_geometry_model_explicit_turns_and_minimum_score(planned):
    r=planned['optimization'];m=planned['metadata'];p=planned['points']
    assert r['state']=='SUCCEEDED' and r['schema_version']==2
    assert r['turn_mode']=='POINT_TURN' and r['coverage_objective_enabled']
    assert r['weights']==WEIGHTS and not r['allow_reverse'] and 'energy_model' not in r
    expected=rotation_envelope(.45,.2,.05)
    assert m['rotation_envelope']==expected
    assert m['effective_parameters']['headland_width']==expected['headland_width_m']
    assert r['selected']['score']==min(c['score'] for c in r['candidates'] if c.get('executable'))
    assert r['selected']['score']==pytest.approx(score(r['selected']['coverage_percent'],r['selected']['overlap_percent'],r['selected']['length_m'],192))
    assert len({round(c['angle_rad'],5) for c in r['candidates']})>=6
    assert {c['mode'] for c in r['candidates']}=={'BOUSTROPHEDON'}
    clean,directions=directed_poses(p)
    assert all(d==1 for d in directions) and len(point_turn_indices(p))>2
    validate_point_turn_path(p,box(0,0,16,12),expected)
    assert r['selected']['length_m']==pytest.approx(LineString([q[:2] for q in p]).length)
    assert r['search_resolution_m']==r['final_resolution_m']
    assert r['search_complete'] and r['possible_count']==r['evaluated_count']
    assert not r['candidate_limit_enabled'] and not r['search_time_limit_enabled']
    coarse={round(c['angle_rad'],5) for c in r['candidates'] if c['headland_search_phase']=='COARSE'}
    assert {round(math.radians(i),5) for i in range(0,180,15)}<=coarse
    assert any(c['headland_search_phase']=='REFINE' for c in r['candidates'])


def test_old_curve_radius_headland_and_evaluation_width_do_not_override_policy(planned):
    p=settings(headland_width=100.,min_turning_radius=20.,path_continuity='REEDS_SHEPP',
        allow_reverse=True,evaluation_width=15.,optimization_weight_energy=1)
    before=copy.deepcopy(p);v=optimize(box(0,0,16,12).wkb,p)
    assert p==before and v['points']==planned['points']
    assert v['metadata']['effective_parameters']['evaluation_width']==1.
    assert v['optimization']['headland_search']['method']=='MIN_SAFE_POINT_TURN'


def test_concave_field_hole_and_disconnected_clearance_never_get_unsafe_connections():
    field=Polygon([(0,0),(16,0),(16,12),(0,12)],holes=[[(6,4),(10,4),(10,8),(6,8)]])
    v=optimize(field.wkb,settings())
    assert v['optimization']['state']=='SUCCEEDED'
    validate_point_turn_path(v['points'],field,v['metadata']['rotation_envelope'])
    assert not LineString([p[:2] for p in v['points']]).intersects(box(6,4,10,8))
    narrow=Polygon([(0,0),(4,0),(4,2),(8,2),(8,0),(12,0),(12,4),(8,4),(8,2.1),(4,2.1),(4,4),(0,4)])
    v=optimize(narrow.wkb,settings())
    assert v['points']==[] and v['optimization']['state']=='FAILED'


def test_dimensions_margin_cancellation_and_oversized_body(monkeypatch):
    env=rotation_envelope(.45,.2,.05)
    assert env['rotation_radius_m']==pytest.approx(math.hypot(.225,.1)+.05)
    assert rotation_envelope(.45,.2,.05,.1,0)['headland_width_m']>env['headland_width_m']
    v=optimize(box(0,0,2,2).wkb,settings(robot_length=10))
    assert not v['points'] and v['optimization']['state']=='FAILED'
    for key,value in [('robot_width',False),('robot_length',-1),('rotation_safety_margin',-.1)]:
        with pytest.raises(ValueError):optimize(box(0,0,16,12).wkb,settings(**{key:value}))
    event=Event();event.set();monkeypatch.setattr(contour,'_CANCEL',event)
    with pytest.raises(ValueError,match='取消'):optimize(box(0,0,16,12).wkb,settings())
def test_old_search_caps_do_not_change_result_or_stop_after_45_seconds(monkeypatch,planned):
    import itertools
    from agri_path_planning import optimizer
    clock=itertools.count(0,10)
    monkeypatch.setattr(optimizer,'monotonic',lambda:next(clock))
    progress=[]
    result=optimize(box(0,0,16,12).wkb,settings(optimization_max_candidates=1,
        optimization_time_budget=.001),progress=progress.append)
    assert result['points']==planned['points']
    assert result['optimization']['elapsed_seconds']>45
    assert progress[0]['phase']=='INITIALIZING' and progress[-1]['phase']=='COMPLETE'
    assert {p['phase'] for p in progress}>={'COARSE','REFINE','FINALIZING'}
    assert max(p['evaluated_count'] for p in progress)>1
    assert all(0<=p['stage_completed']<=p['stage_total'] for p in progress)
    assert [p['evaluated_count'] for p in progress]==sorted(p['evaluated_count'] for p in progress)


def test_cancel_mid_search_does_not_return_best_so_far(monkeypatch):
    event=Event();monkeypatch.setattr(contour,'_CANCEL',event)
    def progress(value):
        if value['evaluated_count']>=2:event.set()
    with pytest.raises(ValueError,match='取消'):
        optimize(box(0,0,16,12).wkb,settings(),progress=progress)


def test_outer_timeout_rejects_completed_late_result_and_stale_progress():
    from types import SimpleNamespace
    from concurrent.futures import Future
    from queue import Queue
    from agri_path_planning.node import CoverageAdapter
    from time import monotonic
    progress=Queue();progress.put((6,{'phase':'COMPLETE','evaluated_count':999}))
    future=Future();future.set_result({'points':[(1,1,0),(2,1,0)]})
    published=[];cancel=Event()
    n=SimpleNamespace(state='PLANNING',reason='',token=7,contour_token=7,deadline=0.,
        contour_future=future,contour_cancel=cancel,goal_handle=None,progress_queue=progress,
        planning_progress={'phase':'COARSE','evaluated_count':2},planning_started=monotonic(),
        mode_ready=lambda:True,get_parameter=lambda name:SimpleNamespace(value='/test/paths'),
        status=SimpleNamespace(publish=lambda message:published.append(json.loads(message.data))))
    n.cancel=lambda request,response:CoverageAdapter.cancel(n,request,response)
    CoverageAdapter.tick(n)
    assert cancel.is_set() and n.state=='FAILED' and n.contour_future is None
    assert published[-1]['search_progress']['phase']=='FAILED'
    assert published[-1]['search_progress']['evaluated_count']==2


def test_energy_distinguishes_forward_turn_reverse_and_switches():
    straight=energy([(0,0,0),(1,0,0)],.6)
    reverse=energy([(0,0,math.pi,-1),(1,0,math.pi,-1)],.6)
    turn=energy([(0,0,0),(1,0,1/.6)],.6)
    switches=energy([(0,0,0,1),(1,0,0,-1),(0,0,0,1),(1,0,0,1)],.6)
    assert straight['energy_proxy']==pytest.approx(1)
    assert reverse['energy_proxy']==pytest.approx(1.3)
    assert turn['energy_proxy']==pytest.approx(1.5)
    assert switches['switches']==2 and switches['switch_cost']==1
    assert not turn['calibrated'] and turn['unit']=='relative_cost'

def test_native_turn_uses_selected_generator_and_exact_radius_only():
    builder=Builder(box(-10,-10,20,20),1,2,1.,.1,.35,'DUBINS')
    library=NativeTurns(builder,options({}))
    for gap in (.4,1.,2.,4.):
        a=(0.,0.,0.);b=(0.,gap,math.pi)
        value=library.connection(a,b)
        assert value['type']=='DUBINS' and value['radius_m']==1.
        assert value['metrics']==energy(value['points'],builder.radius,library.config)
        points=library.connect(a,b)
        assert all(len(p)==4 and p[3]==1 for p in points)
        assert math.dist(points[-1][:2],b[:2])<1e-4
    assert library.total==4 and len(library.previews)==4

def test_scoped_report_checks_origin_and_reference_fingerprint(tmp_path,planned):
    (tmp_path/'paths').mkdir()
    run={'projection':{'epsg':32650},'reference_fingerprint':'fingerprint'}
    report={**planned['optimization'],'projection':run['projection'],'frame_id':'map',
            'reference_fingerprint':'fingerprint'}
    (tmp_path/'run.json').write_text(json.dumps(run))
    target=tmp_path/'paths/optimization.json';target.write_text(json.dumps(report))
    assert read(tmp_path)['state']=='SUCCEEDED'
    small=summary(tmp_path)
    assert not any('points'in p for p in small['turn_curves'])
    report['reference_fingerprint']='other';target.write_text(json.dumps(report))
    assert summary(tmp_path)['state']=='FAILED'
    report['projection']={'epsg':32649};target.write_text(json.dumps(report))
    with pytest.raises(ValueError,match='原点'):read(tmp_path)

def test_report_and_map_partition_use_selected_width_not_requested_width(tmp_path):
    from test_coverage_quality import make_run
    from agri_backend.missions import atomic_json
    from agri_backend.coverage_quality import completed_quality
    from agri_backend.coverage_map import MapState
    import hashlib
    store,run,directory,origin,points=make_run(tmp_path)
    run['planning_strategy']='SMART'
    run['reference_fingerprint']=hashlib.sha256(json.dumps(['map',points],sort_keys=True).encode()).hexdigest()
    store.save_run(run)
    path=directory/'paths/coverage_test.json'
    saved=json.loads(path.read_text());saved['planning']={'strategy':'SMART','effective_parameters':{'headland_width':1.5}}
    atomic_json(path,saved)
    report={'schema_version':1,'strategy':'SMART','state':'SUCCEEDED','frame_id':'map','projection':origin,
        'reference_fingerprint':run['reference_fingerprint'],'selected':{'mode':'SNAKE','headland_width_m':1.5,
        'length_m':6,'energy_proxy':6},'weights':{'length':1},'evaluated_count':1,'possible_count':1,
        'coverage_constraint_enabled':False}
    atomic_json(directory/'paths/optimization.json',report)
    result=completed_quality(directory,{'task_state':'COMPLETED'})
    assert result['planned']['effective_headland_width_m']==1.5
    assert result['optimization']['selected']['headland_width_m']==1.5
    assert MapState(directory).base['inset_distance_m']==1.5
    assert result['planned']['metrics']['field']['area_m2']==pytest.approx(24,abs=.001)
    text=(directory/'analysis/coverage_quality_report.md').read_text()
    assert '实际选定宽度：1.5' in text and '../paths/optimization.json'in text
    assert '不是 Wh 或实测能源'in text
    assert '本轮不设最低覆盖率门槛'in text
    assert result['optimization']['coverage_constraint_enabled'] is False

def test_adaptive_width_outside_legacy_limits_uses_same_partition_for_map_and_report(tmp_path):
    from test_coverage_quality import make_run
    from agri_backend.missions import atomic_json
    from agri_backend.coverage_quality import planned_quality
    from agri_backend.coverage_map import MapState
    from agri_backend.saved_plans import signature
    store,run,directory,origin,points=make_run(tmp_path)
    run['reference_fingerprint']=signature([(p['x'],p['y']) for p in points]);store.save_run(run)
    path=directory/'paths/coverage_test.json'
    saved=json.loads(path.read_text())
    saved['planning']={'strategy':'SMART','effective_parameters':{'headland_width':.25},
        'headland_search':{'schema_version':1,'method':'GEOMETRY_NATIVE_ADAPTIVE',
            'range_min_m':.1751,'range_max_m':1.8,'selected_width_m':.25}}
    atomic_json(path,saved)
    assert planned_quality(directory)['effective_headland_width_m']==.25
    assert MapState(directory).base['inset_distance_m']==.25
    saved['planning']['headland_search']['range_min_m']=.3;atomic_json(path,saved)
    with pytest.raises(ValueError,match='范围'):planned_quality(directory)