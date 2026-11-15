"""Explicitly isolated ROS roundtrip of native layouts; no hardware or motion."""
import json
import os
import time
import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS')!='1',reason='Opt-in localhost synthetic domain')


def test_native_layout_metadata_progress_entry_and_save_reload_without_motion():
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from agri_base.path_geometry import point_turn_indices
    from web_test_runtime import SyntheticSystem
    system=SyntheticSystem();client=TestClient(system.app)
    def get(route):
        r=client.get(route);assert r.status_code==200,r.text
        return r.json()
    def post(route,payload=None):
        r=client.post(route,json=payload) if payload is not None else client.post(route)
        assert r.status_code==200,r.text
        return r.json()
    def until(fn,timeout=60):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if fn():return
            time.sleep(.05)
        raise AssertionError(f'{system.planner.state}: {system.planner.reason}')
    try:
        until(lambda:get('/api/devices')['safety']['fresh'])
        post('/api/system/mode',{'mode':'coverage'});until(system.planner.mode_ready)
        frame=system.localization.frame
        def geo(x,y):
            lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
            return {'longitude':lon,'latitude':lat}
        mission=post('/api/missions',{'name':'三类布局隔离验证','mode':'coverage',
            'boundary':{'points':[geo(x,y) for x,y in [(0,0),(24,0),(24,8),(8,8),(8,30),(0,30)]]},
            'entry':{**geo(-2,-2),'source':'coordinates'},
            'parameters':{'operation_width':6.,'quality_resolution':.1}})
        post(f'/api/missions/{mission["id"]}/optimize')
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        run=get('/api/workspace')['active_run'];report=get(f'/api/runs/{run["id"]}/optimization')
        assert report['layout_search']['families']==['EDGE_GLOBAL','NATIVE_WHOLE_FIELD','NATIVE_PER_CELL']
        directory=system.workspace.store.run_path(run['id'])
        full=json.loads((directory/'paths/optimization.json').read_text())
        candidates=[c for c in full['candidates'] if c.get('executable') and c.get('layout_family')=='NATIVE_PER_CELL']
        assert any(len({round(d['angle_rad'],3) for d in c['cell_directions']})>1 for c in candidates)
        assert {c['variant'] for c in candidates}=={0,1,2,3}
        saved=json.loads(next((directory/'paths').glob('coverage_*.json')).read_text())
        assert saved['planning']['policy_version']==7
        selected=full['selected'];details=saved['planning']['path_details']
        assert details['layout_family']==selected['layout_family']
        if selected['direction_mode']=='PER_CELL':assert details['cell_directions']==selected['cell_directions']
        assert selected['entry_length_m']==pytest.approx(saved['planning']['entry']['distance_m'])
        poses=[(v['x'],v['y'],v['yaw']) for v in saved['points']]
        assert len(point_turn_indices(poses))==selected['point_turn_count']
        assert system.state['v']==system.state['w']==0
        assert system.safety.gate.emergency_stop and not system.safety.gate.owner
        route=f'/api/missions/{mission["id"]}/plans/{run["id"]}'
        post(route+'/save');post(route+'/load')
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        loaded=get('/api/workspace')['active_run']
        assert loaded['source_plan_id']==run['id']
        assert loaded['optimization']['selected']['layout_family']==selected['layout_family']
        assert loaded['optimization']['selected'].get('cell_directions')==selected.get('cell_directions')
        assert system.state['v']==system.state['w']==0 and not system.safety.gate.owner
    finally:
        client.close();system.close()
