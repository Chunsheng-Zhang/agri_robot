"""Explicit saved-path reload and repeat execution, isolated synthetic ROS only."""
import hashlib
import json
import os
import time
import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS')!='1',reason='Opt-in isolated synthetic ROS integration')


def test_saved_plan_survives_mode_change_then_loads_into_new_execution_round():
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from web_test_runtime import SyntheticSystem
    system=SyntheticSystem();client=TestClient(system.app)
    def get(route):
        r=client.get(route);assert r.status_code==200,r.text
        return r.json()
    def post(route,data=None):
        r=client.post(route,json=data) if data is not None else client.post(route)
        assert r.status_code==200,r.text
        return r.json()
    def until(predicate,timeout=35):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            time.sleep(.06)
        raise AssertionError(f'timeout: planner={system.planner.state}, tracker={system.tracker.state}')
    def hashes(directory):
        return {str(p.relative_to(directory)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in directory.rglob('*') if p.is_file()}
    try:
        until(lambda:get('/api/devices')['safety']['fresh'])
        post('/api/system/mode',{'mode':'coverage'});until(system.planner.mode_ready)
        until(lambda:system.planner.client.server_is_ready())
        frame=system.localization.frame;points=[]
        for x,y in [(0,0),(16,0),(16,12),(0,12)]:
            lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
            points.append({'longitude':lon,'latitude':lat})
        mission=post('/api/missions',{'name':'保存路径复用验证','mode':'coverage','boundary':{'points':points},
            'parameters':{'route_mode':'CONTOUR','headland_width':1.,'operation_width':1.,'min_turning_radius':.6}})
        source=post(f'/api/missions/{mission["id"]}/plan')['run']
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        until(lambda:((get('/api/workspace').get('active_run') or {}).get('coverage_quality') or {}).get('state')=='SUCCEEDED')
        source=get('/api/workspace')['active_run'];directory=system.workspace.store.run_path(source['id'])
        route=f'/api/missions/{mission["id"]}/plans/{source["id"]}'
        assert not source.get('saved_plan_version') and source['plan_save_required'] is True
        assert not next(m for m in get('/api/workspace')['missions'] if m['id']==mission['id']).get('coverage_plans')
        assert client.get(route+'/path').status_code==409
        state_before=dict(system.state)
        result=post(route+'/save');assert result['run']['saved_plan_version']==1
        assert system.state==state_before and not system.safety.gate.owner
        until(lambda:bool(next(m for m in get('/api/workspace')['missions'] if m['id']==mission['id']).get('coverage_plans')))
        assert post(route+'/save')['already_saved'] is True
        preview=get(route+'/path');assert preview['count']>2
        assert preview['fingerprint']==source['reference_fingerprint']
        assert get('/api/devices')['safety']['emergency_stop'] and not system.safety.gate.owner
        post('/api/system/mode',{'mode':'base'});until(lambda:system.planner.mode=='base')
        assert client.post(route+'/load').status_code==409
        post('/api/system/mode',{'mode':'coverage'});until(system.planner.mode_ready)
        until(lambda:get('/api/robot/path/reference')['count']==0)
        get('/api/workspace');before=hashes(directory)
        loaded=post(route+'/load')['run']
        assert loaded['id']!=source['id'] and loaded['source_plan_id']==source['id']
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        assert get('/api/robot/path/reference')['fingerprint']==preview['fingerprint']
        assert system.state['v']==system.state['w']==0 and not system.safety.gate.owner
        assert client.post(f'/api/missions/{mission["id"]}/start').status_code==409
        # Align only the synthetic robot. Start is a separate explicit API call.
        post('/api/test/align-path-start');time.sleep(.25)
        post('/api/robot/emergency-stop/reset')
        until(lambda:not get('/api/devices')['safety']['motion_inhibited'])
        post(f'/api/missions/{mission["id"]}/start')
        until(lambda:system.tracker.state=='RUNNING')
        assert client.post(route+'/load').status_code==409
        post('/api/tasks/tracking/cancel')
        until(lambda:system.tracker.state=='CANCELED' and not system.safety.gate.owner)
        new_directory=system.workspace.store.run_path(loaded['id'])
        until(lambda:not system.backend.workspace_ros.recorder.busy(new_directory),45)
        assert list((new_directory/'tracks').glob('*.csv'))
        assert hashes(directory)==before
        repeated=post(route+'/load')['run']
        assert repeated['id'] not in (source['id'],loaded['id'])
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        assert hashes(directory)==before
        updated={k:mission[k] for k in ('name','mode','boundary','parameters','waypoints','revision')}
        updated['parameters']={**updated['parameters'],'operation_width':1.2}
        response=client.put(f'/api/missions/{mission["id"]}',json=updated)
        assert response.status_code==200,response.text
        assert client.post(route+'/load').status_code==422
        assert client.get(route+'/path').status_code==409
        assert system.state['v']==system.state['w']==0
        until(lambda:not system.workspace.quality_jobs.busy(system.workspace.store.run_path(repeated['id'])))
    finally:client.close();system.close()
