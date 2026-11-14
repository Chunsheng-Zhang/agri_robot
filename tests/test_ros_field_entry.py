"""Real OpenNav/ROS services with synthetic sensors; no physical drivers."""
import json
import math
import os
import time

import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS')!='1',reason='Opt-in isolated ROS/Web test')


def test_manual_and_smart_entry_keep_original_single_waypoint_approach(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from web_test_runtime import SyntheticSystem
    from agri_backend.following import following_manager
    from agri_backend.taught_paths import TaughtPathStore
    monkeypatch.setattr(following_manager,'store',TaughtPathStore(tmp_path))
    system=SyntheticSystem();client=TestClient(system.app)
    def until(predicate,timeout=25):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.03)
        raise AssertionError(f'入口联调超时：{system.planner.state} {system.planner.reason}, tracker={system.tracker.state}')
    def post(path,body=None):
        response=client.post(path,json=body) if body is not None else client.post(path)
        assert response.status_code==200,response.text
        return response.json()
    def get(path):
        response=client.get(path);assert response.status_code==200,response.text;return response.json()
    def geo(x,y):
        frame=system.localization.frame
        lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
        return {'longitude':lon,'latitude':lat}
    def create(**parameters):
        return post('/api/missions',{'name':'人工入口隔离任务','mode':'coverage',
            'boundary':{'points':[geo(0,0),geo(16,0),geo(16,12),geo(0,12)]},
            'entry':{**geo(1,1),'source':'map'},'parameters':{
                'operation_width':1.5,'headland_width':2.,'min_turning_radius':.5,
                'sampling_distance':.15,'path_continuity':'DUBINS','speed':.3,'lookahead':.25,
                'quality_resolution':.15,**parameters}})
    try:
        until(lambda:get('/api/devices')['safety']['fresh'])
        post('/api/system/mode',{'mode':'coverage'});until(system.planner.mode_ready)
        until(lambda:system.planner.client.server_is_ready())
        mission=create()
        post(f'/api/missions/{mission["id"]}/plan')
        assert system.planner.get_parameter('min_turning_radius').value==.5
        assert system.planner.get_parameter('operation_width').value==1.5
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        run=get('/api/workspace')['active_run'];directory=system.workspace.store.run_path(run['id'])
        saved=json.loads(next((directory/'paths').glob('coverage_*.json')).read_text())
        assert saved['planning']['entry']['reference']['source']=='map'
        assert saved['planning']['entry_variant'] in range(4)
        assert math.dist((saved['points'][0]['x'],saved['points'][0]['y']),(1,1))<4
        assert run['entry']['connection_length_m']>=0
        assert system.state['v']==0 and not system.safety.gate.owner
        post(f'/api/missions/{mission["id"]}/plans/{run["id"]}/save')
        post(f'/api/missions/{mission["id"]}/plans/{run["id"]}/load')
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        loaded=get('/api/workspace')['active_run']
        assert loaded['id']!=run['id'] and loaded['entry']['reference']==run['entry']['reference']
        post('/api/robot/emergency-stop/reset')
        start=saved['points'][0]
        assert math.hypot(system.state['x'],system.state['y'])<.01
        assert math.dist((system.state['x'],system.state['y']),(start['x'],start['y']))>2.
        post(f'/api/missions/{mission["id"]}/start')
        until(lambda:system.tracker.state=='RUNNING' and system.tracker.controller.phase=='APPROACHING')
        from agri_waypoint.controller import WaypointController
        assert isinstance(system.tracker.controller.approach,WaypointController)
        assert math.dist(system.tracker.controller.target,(start['x'],start['y']))<1e-6
        assert math.dist(system.tracker.controller.target,(1,1))>.5  # Target is coverage start, not manual entry.
        until(lambda:system.state['v']>0)
        assert system.tracker.controller.progress==0
        post('/api/tasks/tracking/pause');until(lambda:system.state['v']==0)
        post('/api/tasks/tracking/resume')
        until(lambda:system.tracker.controller.phase=='TRACKING',30)
        assert system.tracker.state=='RUNNING'
        post('/api/tasks/tracking/cancel');until(lambda:not system.safety.gate.owner and system.state['v']==0)
        directory=system.workspace.store.run_path(loaded['id'])
        assert not (directory/'config/entry_approach.json').exists()
        assert list((directory/'tracks').glob('approach_*.csv'))
        assert list((directory/'tracks').glob('alignment_*.csv'))
        until(lambda:not system.backend.workspace_ros.recorder.busy(directory),20)
        # A task edit cannot reuse an old entry's saved plan.
        edited=post('/api/missions/preview',{k:mission[k] for k in ('name','mode','boundary','parameters','entry')})
        assert edited['entry']['x']==pytest.approx(1,abs=1e-6)
        smart=create(operation_width=2.5)
        post(f'/api/missions/{smart["id"]}/optimize')
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY',40)
        smart_run=get('/api/workspace')['active_run']
        report=get(f'/api/runs/{smart_run["id"]}/optimization')
        assert {v['variant'] for v in report['candidates']}=={0,1,2,3}
        assert smart_run['entry']['reference']['source']=='map'
        assert report['selected']['entry_length_m']==pytest.approx(smart_run['entry']['distance_m'])
        assert not system.safety.gate.owner and system.state['v']==0
    finally:
        system.close()
