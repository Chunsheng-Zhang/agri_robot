"""Opt-in Web/ROS contour pipeline in a verified localhost synthetic domain."""
import json
import os
import time

import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS')!='1',reason='Opt-in isolated synthetic ROS integration')


@pytest.mark.parametrize('vertices',[
    [(0,0),(20,0),(20,14),(0,14)],
    [(0.,0.),(-.0903990528313443,-16.356757320463657),
     (12.755411364603788,-16.490246892441064),(10.91983889346011,-5.4276303220540285),
     (10.95955598494038,-.14654403179883957)],
],ids=['rectangle','actual_concave_five_point'])
def test_web_inside_out_contour_and_cancel_without_motion(vertices):
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from shapely.geometry import Polygon,Point
    from web_test_runtime import SyntheticSystem
    system=SyntheticSystem();client=TestClient(system.app)
    def get(route):
        response=client.get(route);assert response.status_code==200,response.text
        return response.json()
    def post(route,payload=None):
        response=client.post(route,json=payload) if payload is not None else client.post(route)
        assert response.status_code==200,response.text
        return response.json()
    def until(fn,timeout=25):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if fn():return
            time.sleep(.05)
        raise AssertionError(f'timeout; planner={system.planner.state}: {system.planner.reason}')
    frame=system.localization.frame
    def boundary(width=None,height=None):
        points=[]
        for x,y in (vertices if width is None else [(0,0),(width,0),(width,height),(0,height)]):
            lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
            points.append({'longitude':lon,'latitude':lat})
        return {'points':points}
    try:
        until(lambda:get('/api/devices')['safety']['fresh'])
        post('/api/system/mode',{'mode':'coverage'});until(system.planner.mode_ready)
        until(lambda:system.planner.client.server_is_ready())
        for continuity in ['CONTINUOUS','DISCONTINUOUS']:
            mission=post('/api/missions',{'name':'回形离线ROS验证','mode':'coverage','boundary':boundary(),
                'parameters':{'route_mode':'CONTOUR','operation_width':1.,'headland_width':1.,
                              'min_turning_radius':.6,'path_continuity':continuity}})
            post(f'/api/missions/{mission["id"]}/plan')
            until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
            until(lambda:((get('/api/workspace').get('active_run') or {}).get('coverage_quality') or {}).get('state')=='SUCCEEDED')
            run=get('/api/workspace')['active_run'];directory=system.workspace.store.run_path(run['id'])
            files=list((directory/'paths').glob('coverage_*.json'));assert len(files)==1
            saved=json.loads(files[0].read_text());metadata=saved['planning']
            assert metadata['mode']=='CONTOUR' and metadata['direction']=='INSIDE_OUT'
            assert metadata['topology']=='OPEN_SPIRAL' and metadata['closed_laps'] is False
            assert metadata['ordering']=='REFERENCE_O_REVERSED'
            assert all(not layer['closed'] for layer in metadata['layers'])
            for a,b,join in zip(metadata['layers'],metadata['layers'][1:],metadata['connections']):
                assert join['start_index']==a['end_index'] and join['end_index']==b['start_index']
            assert metadata['ring_count']>=2 and metadata['layers'][0]['level']>metadata['layers'][-1]['level']
            assert saved['projection']==run['projection'] and saved['frame_id']=='map'
            assert len(saved['is_backward'])==len(saved['points']) and not any(saved['is_backward'])
            path=get('/api/robot/path/reference')
            assert path['count']==len(saved['points']) and path['fingerprint']==run['reference_fingerprint']
            field=Polygon(vertices)
            assert field.boundary.distance(Point(path['points'][0]['x'],path['points'][0]['y']))>field.boundary.distance(Point(path['points'][-1]['x'],path['points'][-1]['y']))+.5
            assert run['coverage_quality']['planned']['metrics']['field']['coverage_percent']>0
            post(f'/api/missions/{mission["id"]}/plans/{run["id"]}/save')
            preview=get(f'/api/missions/{mission["id"]}/plans/{run["id"]}/path')
            assert preview['points']==path['points']
            safety=get('/api/devices')['safety']
            assert safety['fresh'] and safety['emergency_stop'] and not safety['owner']
            assert system.state['v']==system.state['w']==0
        # A large planning request is cancelled cooperatively; no late reference
        # or partial file may overwrite the empty reference after cancellation.
        mission=post('/api/missions',{'name':'取消回形验证','mode':'coverage','boundary':boundary(100,80),
            'parameters':{'route_mode':'CONTOUR','operation_width':.1,'headland_width':0}})
        result=post(f'/api/missions/{mission["id"]}/plan')
        post('/api/planning/cancel')
        until(lambda:system.planner.contour_future is None,10)
        assert system.planner.state=='CANCELED'
        directory=system.workspace.store.run_path(result['run']['id'])
        assert not list((directory/'paths').glob('coverage_*.json'))
        assert get('/api/robot/path/reference')['count']==0
        assert system.state['v']==system.state['w']==0 and not system.safety.gate.owner
    finally:
        client.close();system.close()
