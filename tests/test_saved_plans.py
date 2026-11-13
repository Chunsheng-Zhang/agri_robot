"""Archived path preview/loading rules without ROS or hardware."""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from pyproj.enums import TransformDirection
from agri_backend.missions import MissionStore,Mission,LocalFrame,atomic_json
from agri_backend.saved_plans import SavedPlans,read_plan,signature

ROOT=Path(__file__).resolve().parents[1]


def make_plan(tmp_path):
    store=MissionStore(tmp_path);store.config_path=ROOT/'config/jetson_system.yaml'
    origin=store.configured_origin();frame=LocalFrame(origin['latitude'],origin['longitude'])
    def geo(x,y):
        lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
        return {'longitude':lon,'latitude':lat}
    mission=store.save(Mission(name='保存路径验证',mode='coverage',boundary={'points':[geo(x,y) for x,y in ((0,0),(10,0),(10,8),(0,8))]},
        parameters={'operation_width':1.,'headland_width':1.}),origin)
    run=store.create_run(mission,1,'planning')
    data={'frame_id':'map','projection':origin,'points':[{'x':1.,'y':1.,'yaw':0.},{'x':9.,'y':1.,'yaw':0.}],
          'velocities':[],'is_backward':[False,False]}
    run.update(state='READY',saved_plan_version=1,reference_fingerprint=signature([(1.,1.),(9.,1.)]),planning_strategy='MANUAL')
    store.save_run(run);path=store.run_path(run['id'])/'paths/coverage_saved.json';atomic_json(path,data)
    return store,mission,run,path,copy.deepcopy(data)


def test_saved_plan_preview_is_read_only_and_version_scoped(tmp_path):
    store,mission,run,path,_=make_plan(tmp_path)
    before=path.read_bytes();record=(path.parent.parent/'run.json').read_bytes()
    source,data,points,fingerprint=read_plan(store,mission,run['id'])
    assert fingerprint==run['reference_fingerprint'] and len(points)==2
    assert source['state']=='READY' and path.read_bytes()==before
    assert (path.parent.parent/'run.json').read_bytes()==record
    changed=copy.deepcopy(mission);changed['revision']+=1
    with pytest.raises(ValueError,match='版本'):read_plan(store,changed,run['id'])
    changed=copy.deepcopy(mission);changed['parameters']['speed']=.4
    with pytest.raises(ValueError,match='快照'):read_plan(store,changed,run['id'])


def test_historical_runs_are_ignored_without_backfill(tmp_path):
    store,mission,run,path,_=make_plan(tmp_path)
    run.pop('saved_plan_version');store.save_run(run)
    before=path.read_bytes()
    assert SavedPlans().summaries(store,[mission])[mission['id']]==[]
    with pytest.raises(ValueError,match='未启用'):read_plan(store,mission,run['id'])
    assert path.read_bytes()==before
    assert 'saved_plan_version' not in json.loads((path.parent.parent/'run.json').read_text())


def test_explicit_save_adds_pending_plan_to_catalogue_without_ros_or_rewriting_path(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    store,mission,run,path,_=make_plan(tmp_path)
    run.pop('saved_plan_version');run['plan_save_required']=True;store.save_run(run)
    workspace=main.app.state.workspace
    monkeypatch.setattr(workspace,'store',store);monkeypatch.setattr(workspace,'active',copy.deepcopy(run))
    monkeypatch.setattr(workspace,'origin',lambda:(mission['projection'],'configured_yaml'))
    monkeypatch.setattr(workspace,'saved_plans',SavedPlans())
    def no_ros():pytest.fail('Saving must not call ROS, start a task or acquire control')
    monkeypatch.setattr(workspace,'gateway',no_ros)
    catalogue=workspace.saved_plans
    assert catalogue.summaries(store,[mission])[mission['id']]==[]
    with pytest.raises(ValueError,match='先点击保存'):read_plan(store,mission,run['id'])
    before=path.read_bytes();config=(path.parent.parent/'config/mission.json').read_bytes()
    client=TestClient(main.app);route=f'/api/missions/{mission["id"]}/plans/{run["id"]}/save'
    try:
        response=client.post(route);assert response.status_code==200,response.text
        saved=response.json()
        assert saved['success'] and not saved['already_saved']
        assert saved['run']['state']=='READY' and saved['run']['saved_plan_version']==1
        assert saved['run']['plan_save_required'] is False
        assert saved['run']['plan_saved_at'] and workspace.active['saved_plan_version']==1
        assert [p['id'] for p in catalogue.summaries(store,[mission])[mission['id']]]==[run['id']]
        assert path.read_bytes()==before and (path.parent.parent/'config/mission.json').read_bytes()==config
        assert not (path.parent.parent/'tracks').exists()
        after=(path.parent.parent/'run.json').read_bytes()
        assert client.post(route).json()['already_saved'] is True
        assert (path.parent.parent/'run.json').read_bytes()==after
        assert read_plan(store,mission,run['id'])[0]['saved_plan_version']==1
        # The marker/list survives a backend catalogue restart.
        assert SavedPlans().summaries(store,[mission])[mission['id']][0]['id']==run['id']
    finally:client.close()


@pytest.mark.parametrize('state',['FAILED','RUNNING','PAUSED','CANCELED'])
def test_explicit_save_requires_new_ready_planning_round(tmp_path,monkeypatch,state):
    from fastapi.testclient import TestClient
    from agri_backend import main
    store,mission,run,path,_=make_plan(tmp_path)
    run.pop('saved_plan_version');run.update(plan_save_required=True,state=state);store.save_run(run)
    workspace=main.app.state.workspace
    monkeypatch.setattr(workspace,'store',store);monkeypatch.setattr(workspace,'active',None)
    monkeypatch.setattr(workspace,'origin',lambda:(mission['projection'],'configured_yaml'))
    before=(path.parent.parent/'run.json').read_bytes()
    client=TestClient(main.app)
    try:
        response=client.post(f'/api/missions/{mission["id"]}/plans/{run["id"]}/save')
        assert response.status_code==409,response.text
        assert (path.parent.parent/'run.json').read_bytes()==before
    finally:client.close()


def test_save_rejects_historical_unmarked_or_changed_mission_without_backfill(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    store,mission,run,path,_=make_plan(tmp_path)
    run.pop('saved_plan_version');store.save_run(run)
    workspace=main.app.state.workspace
    monkeypatch.setattr(workspace,'store',store);monkeypatch.setattr(workspace,'active',None)
    monkeypatch.setattr(workspace,'origin',lambda:(mission['projection'],'configured_yaml'))
    client=TestClient(main.app);route=f'/api/missions/{mission["id"]}/plans/{run["id"]}/save'
    try:
        before=path.read_bytes()
        assert client.post(route).status_code==422
        run['plan_save_required']=True;store.save_run(run)
        mission['revision']+=1;atomic_json(store.path(mission['id']),mission)
        assert client.post(route).status_code==422
        assert 'saved_plan_version' not in json.loads((path.parent.parent/'run.json').read_text())
        assert path.read_bytes()==before
    finally:client.close()


@pytest.mark.parametrize('change,reason',[
    (lambda d:d.update(frame_id='odom'),'坐标系'),
    (lambda d:d['projection'].update(longitude=118.),'原点'),
    (lambda d:d['points'][0].update(x=-1.),'boundary'),
    (lambda d:d['points'][0].update(yaw=float('nan')),'无效'),
    (lambda d:d.update(is_backward=[True,False]),'倒车'),
    (lambda d:d.update(velocities=[-.2]),'倒车'),
    (lambda d:d.update(points=[]),'点数'),
])
def test_corrupt_unsafe_or_reverse_paths_cannot_be_loaded(tmp_path,change,reason):
    store,mission,run,path,data=make_plan(tmp_path)
    change(data);path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match=reason):read_plan(store,mission,run['id'])


def test_tampered_fingerprint_and_deleted_records_are_rejected(tmp_path):
    store,mission,run,path,data=make_plan(tmp_path)
    data['points'][1]['x']=8.;atomic_json(path,data)
    with pytest.raises(ValueError,match='指纹'):read_plan(store,mission,run['id'])
    store.delete_run(run['id'])
    with pytest.raises(ValueError,match='删除'):read_plan(store,mission,run['id'])


def test_catalogue_keeps_older_paths_beyond_run_table_limit_and_no_point_arrays(tmp_path):
    store,mission,run,path,_=make_plan(tmp_path)
    for _ in range(51):
        later=store.create_run({**mission,'mode':'waypoint'},1,'execution')
        later['state']='COMPLETED';store.save_run(later)
    assert run['id'] not in {r['id'] for r in store.list_runs()}
    catalogue=SavedPlans();rows=catalogue.summaries(store,[mission])[mission['id']]
    assert [row['id'] for row in rows]==[run['id']]
    assert 'points' not in json.dumps(rows)
    store.delete_run(run['id'])
    assert catalogue.summaries(store,[mission])[mission['id']]==[]


def test_api_preview_then_explicit_load_create_new_round_without_motion_or_overwriting(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    store,mission,run,path,data=make_plan(tmp_path)
    workspace=main.app.state.workspace
    monkeypatch.setattr(workspace,'store',store);monkeypatch.setattr(workspace,'active',None)
    monkeypatch.setattr(workspace,'sync',lambda:None)
    monkeypatch.setattr(workspace,'origin',lambda:(mission['projection'],'active_ros'))
    monkeypatch.setattr(workspace,'require_idle',lambda mode:{'mode':mode,'mode_revision':2})
    monkeypatch.setattr(workspace,'robot_snapshot',lambda:{'reference_sequence':5})
    calls=[];parameters={}
    def configure(node,values):
        assert node=='coverage_adapter';parameters.update(values);calls.append(('parameters',node))
    def trigger(name):
        calls.append(('trigger',name));assert name=='/planning/load'
        saved=json.loads(Path(parameters['saved_path_file']).read_text())
        atomic_json(Path(parameters['output_directory'])/'coverage_reloaded.json',saved)
        return {'success':True,'message':'loaded without motion'}
    monkeypatch.setattr(workspace,'gateway',lambda:SimpleNamespace(parameters=configure,trigger=trigger))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in path.parent.parent.rglob('*') if p.is_file()}
    client=TestClient(main.app)
    try:
        preview=client.get(f'/api/missions/{mission["id"]}/plans/{run["id"]}/path')
        assert preview.status_code==200,preview.text
        assert preview.json()['count']==2 and calls==[] and workspace.active is None
        response=client.post(f'/api/missions/{mission["id"]}/plans/{run["id"]}/load')
        assert response.status_code==200,response.text
        current=response.json()['run'];assert current['id']!=run['id'] and current['source_plan_id']==run['id']
        assert current['planning_strategy']=='REUSED' and current['stage']=='planning'
        assert calls==[('parameters','coverage_adapter'),('trigger','/planning/load')]
        assert not (store.run_path(current['id'])/'tracks').exists()
        assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
        assert client.get('/api/missions/bad/plans/bad/path').status_code in (409,404)
    finally:client.close()
