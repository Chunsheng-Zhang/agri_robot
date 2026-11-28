"""Teach processing, file integrity, dropout semantics and safe API operation."""
import csv
import json
import math
from time import monotonic
from types import SimpleNamespace

import pytest

from agri_base.path_geometry import directed_poses, point_turn_indices
from agri_backend.devices import device_cache
from agri_backend.following import FollowingManager, RecordRequest
from agri_backend.taught_paths import ProcessingParameters, process_samples, TaughtPathStore

ORIGIN = {'epsg':32650,'latitude':36.6468527633281,'longitude':117.021433066391}


def samples(count=81):
    return [{'stamp':100+i*.1,'x':i*.02,'y':0.,'yaw':0.} for i in range(count)]


def test_ordered_processing_removes_spike_and_stationary_jitter_resamples_and_keeps_origin():
    rows = samples()
    rows[30]['y'] = 7.
    rows[10]['x'] = rows[9]['x']+.002
    result = process_samples(rows,{})
    points = result['points']
    assert points[0]['x']==0 and points[-1]['x']==pytest.approx(1.6)
    assert max(abs(p['y']) for p in points)==0
    assert result['statistics']['rejected']['isolated_spike']==1
    assert result['statistics']['rejected']['stationary']>0
    assert all(0<math.dist((a['x'],a['y']),(b['x'],b['y']))<=.101 for a,b in zip(points,points[1:]))
    assert all(a['x']<b['x'] for a,b in zip(points,points[1:]))


def test_smoothing_is_local_bounded_and_preserves_endpoints():
    rows=samples(101)
    for i,r in enumerate(rows):
        r['y']=.015*math.sin(i*.8)
    result=process_samples(rows,{'smoothing':.02,'spacing':.05})
    assert result['points'][0]['x']==rows[0]['x']
    assert result['points'][-1]['y']==pytest.approx(rows[-1]['y'])
    assert max(abs(p['y']) for p in result['points'])<.016
    assert max(p['x'] for p in result['points'])==2.


@pytest.mark.parametrize('kind',['gap','backwards','jump','short','lateral','reverse'])
def test_unreplayable_samples_do_not_become_a_silent_connected_path(kind):
    rows=samples()
    if kind=='gap': rows=rows[:20]+rows[30:]
    if kind=='backwards': rows[10]['stamp']=rows[9]['stamp']-.1
    if kind=='jump':
        for r in rows[40:]:r['y']+=10
    if kind=='short':rows=rows[:5]
    if kind=='lateral':
        for r in rows:r['yaw']=math.pi/2
    if kind=='reverse':
        for r in rows:r['yaw']=math.pi
    with pytest.raises(ValueError):process_samples(rows,{})


def test_sharp_corner_is_preserved_as_stopped_turn_and_reverse_requires_permission():
    rows=samples(51)
    rows[-1]['yaw']=math.pi/2
    rows += [{'stamp':105+i*.1,'x':1.,'y':i*.02,'yaw':math.pi/2} for i in range(1,51)]
    result=process_samples(rows,{})
    poses=[(p['x'],p['y'],p['yaw']) for p in result['points']]
    assert len(point_turn_indices(poses))==1
    assert all(d==1 for d in directed_poses(poses)[1])
    # A true backtrack retains visit order and the reverse leg.
    back=samples(51)
    back += [{'stamp':105+i*.1,'x':1.-i*.02,'y':0.,'yaw':0.} for i in range(1,51)]
    with pytest.raises(ValueError,match='倒车'):process_samples(back,{})
    reverse=process_samples(back,{'allow_reverse':True})
    directions=directed_poses([(p['x'],p['y'],p['yaw']) for p in reverse['points']])[1]
    assert 1 in directions and -1 in directions
    assert reverse['points'][0]['x']==pytest.approx(reverse['points'][-1]['x'])


def test_library_integrity_rename_delete_and_original_record_are_separate(tmp_path):
    store=TaughtPathStore(tmp_path)
    meta=store.create('试验道路',ORIGIN,ProcessingParameters().model_dump())
    store.finish(meta,samples())
    first=store.get(meta['id'])
    assert first['projection']==ORIGIN and first['frame_id']=='map'
    filename=store.directory(meta['id'])/'path.json'
    original=filename.read_bytes()
    store.update(meta,name='新名称')
    assert store.get(meta['id'])['name']=='新名称' and filename.read_bytes()==original
    corrupted=json.loads(original);corrupted['points'][1]['x']+=.01
    filename.write_text(json.dumps(corrupted))
    with pytest.raises(ValueError,match='指纹'):store.get(meta['id'])
    filename.write_bytes(original)
    store.update(meta,deleted_at='2026-11-13')
    assert not store.list() and filename.exists()
    with pytest.raises(ValueError):store.get(meta['id'])
    with pytest.raises(ValueError):store.directory('../../x')


def test_backend_restart_marks_incomplete_teaching_fault_without_touching_saved_path(tmp_path):
    store=TaughtPathStore(tmp_path)
    interrupted=store.create('中断',ORIGIN,ProcessingParameters().model_dump())
    directory=store.directory(interrupted['id'])
    (directory/'raw.csv').write_text('stamp,x,y,yaw\n100,1,2,0\n')
    saved=store.create('完整',ORIGIN,ProcessingParameters().model_dump())
    store.finish(saved,samples())
    original=store.directory(saved['id']).joinpath('path.json').read_bytes()
    store.recover_interrupted()
    assert store.get(interrupted['id'],ready=False)['state']=='FAULT'
    assert (directory/'raw.csv').read_text().endswith('100,1,2,0\n')
    assert store.get(saved['id'])['state']=='READY'
    assert store.directory(saved['id']).joinpath('path.json').read_bytes()==original


def ready_manager(tmp_path,monkeypatch):
    monkeypatch.setattr(device_cache,'values',{})
    device_cache.update('safety',{'mode':'base','owner':'manual','motion_inhibited':False,'joy_connected':True})
    device_cache.update('origin',{'projection':ORIGIN})
    m=FollowingManager(tmp_path)
    m.valid,m.valid_at=True,monotonic()
    m.accept_pose(samples()[0])
    return m


def test_recording_does_not_acquire_control_and_preserves_raw_on_fault(tmp_path,monkeypatch):
    m=ready_manager(tmp_path,monkeypatch)
    identity=m.start_record(RecordRequest(name='示教'),ORIGIN)
    for row in samples()[1:15]:m.accept_pose(row)
    m.work_once()
    device_cache.update('safety',{'mode':'base','owner':'','motion_inhibited':True,'joy_connected':True})
    m.work_once()
    assert m.state=='FAULT' and m.publisher is None
    raw=list(csv.DictReader((m.store.directory(identity)/'raw.csv').open()))
    assert len(raw)==15
    assert m.store.get(identity,ready=False)['state']=='FAULT'
    assert not (m.store.directory(identity)/'path.json').exists()
    m.accept_pose(samples()[20])
    assert len(m.samples)==15  # No automatic recovery/bridging.
    m.close()


def test_record_save_and_time_gap_have_distinct_results(tmp_path,monkeypatch):
    m=ready_manager(tmp_path,monkeypatch)
    identity=m.start_record(RecordRequest(name='道路'),ORIGIN)
    for row in samples()[1:]:m.accept_pose(row)
    m.stop_record()
    assert m.state=='PROCESSING'
    m.work_once()
    assert m.state=='READY' and m.store.get(identity)['statistics']['length_m']>1
    m.valid_at=monotonic();m.accept_pose(samples()[0])
    failed=m.start_record(RecordRequest(name='中断道路'),ORIGIN)
    m.accept_pose(samples()[10])
    assert m.state=='FAULT'
    m.work_once()
    assert m.store.get(failed,ready=False)['state']=='FAULT'
    m.close()


def test_following_gate_has_exclusive_lease_and_error_watchdog():
    from agri_safety.gate import CommandGate
    gate=CommandGate(command_timeout=.25,max_path_error=.5)
    gate.stop(False,True)
    assert gate.enable('following',True,True,0,100)
    assert not gate.enable('tracking',True,True,.1,100.1)
    gate.receive('following',.1,.2,.1,100.1)
    assert gate.output(.1,True)==(0.,0.)  # Need fresh error feedback.
    gate.error,gate.error_time=.1,.1
    assert gate.output(.1,True)==(.1,.2)
    assert gate.output(.7,True)==(0.,0.) and gate.owner==''


def test_following_context_hides_restored_run_internal_timestamps(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.following import following_manager as manager
    workspace=main.app.state.workspace
    # sync() restores this sentinel after reconnecting a completed ROS run.
    run={'id':'finished-following','mode':'following','state':'COMPLETED',
         '_requested':-float('inf')}
    monkeypatch.setattr(workspace,'active',run)
    monkeypatch.setattr(workspace,'sync',lambda:None)
    monkeypatch.setattr(manager,'snapshot',lambda:{'recording':{'state':'IDLE'}})
    monkeypatch.setattr(manager,'store',TaughtPathStore(tmp_path))
    response=TestClient(main.app).get('/api/following/context')
    assert response.status_code==200
    assert response.json()['active_run']=={
        'id':'finished-following','mode':'following','state':'COMPLETED'}
    assert workspace.active is run and math.isinf(run['_requested'])
    monkeypatch.setattr(workspace,'active',{**run,'mode':'coverage'})
    assert TestClient(main.app).get('/api/following/context').json()['active_run'] is None


def test_following_api_file_library_and_start_rejection(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.following import following_manager as m
    from agri_backend.missions import MissionStore
    workspace=main.app.state.workspace
    monkeypatch.setattr(workspace,'active',None)
    monkeypatch.setattr(workspace,'store',MissionStore(tmp_path))
    monkeypatch.setattr(m,'store',TaughtPathStore(tmp_path))
    monkeypatch.setattr(m,'state','IDLE')
    monkeypatch.setattr(m,'pending_finish',False)
    monkeypatch.setattr(m,'loaded',None)
    monkeypatch.setattr(m,'reference',[])
    monkeypatch.setattr(m,'track',[])
    monkeypatch.setattr(device_cache,'values',{})
    device_cache.update('following',{'state':'IDLE'})
    device_cache.update('safety',{'mode':'following','mode_revision':3,'owner':'','motion_inhibited':False,'limits':{'linear':1,'angular':5,'path_error':2}})
    device_cache.update('origin',{'projection':ORIGIN})
    sent=[]
    gateway=SimpleNamespace(parameters=lambda *args:sent.append(args))
    monkeypatch.setattr(main.app.state,'ros_bridge',SimpleNamespace(started=True,node=SimpleNamespace(workspace_ros=gateway)),raising=False)
    monkeypatch.setattr(m,'publish',lambda points:sent.append(points))
    meta=m.store.create('路 A',ORIGIN,ProcessingParameters().model_dump())
    m.store.finish(meta,samples())
    client=TestClient(main.app)
    assert client.get('/api/following/context').json()['paths'][0]['name']=='路 A'
    assert client.post('/api/following/start',json={}).status_code==409
    assert client.post(f'/api/following/paths/{meta["id"]}/load').status_code==200
    assert sent and m.loaded['id']==meta['id']
    # A publish request alone does not prove controller receipt.
    assert client.post('/api/following/start',json={}).status_code==409
    assert client.patch(f'/api/following/paths/{meta["id"]}',json={'name':'路 B'}).status_code==200
    device_cache.update('following',{'state':'RUNNING'})
    assert client.delete(f'/api/following/paths/{meta["id"]}').status_code==409
    device_cache.update('following',{'state':'IDLE'})
    assert client.delete(f'/api/following/paths/{meta["id"]}').status_code==200
    assert m.loaded is None and m.store.list()==[]
