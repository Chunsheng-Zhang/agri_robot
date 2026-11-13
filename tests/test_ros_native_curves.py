"""Four native OpenNav forms and reverse execution in an isolated fake chassis."""
import json
import math
import os
import time
import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS')!='1',reason='Opt-in isolated synthetic ROS integration')


def test_native_planning_and_explicit_reverse_execution():
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from shapely.geometry import box
    from geometry_msgs.msg import Twist
    from agri_base.path_geometry import backward_flags
    from agri_backend.missions import atomic_json
    from agri_backend.saved_plans import signature
    from agri_path_planning.contour import Builder
    from agri_path_planning.turn_library import NativeTurns
    from agri_path_planning.planning_options import options
    from web_test_runtime import SyntheticSystem
    system=SyntheticSystem();client=TestClient(system.app)
    def get(route):
        r=client.get(route);assert r.status_code==200,r.text
        return r.json()
    def post(route,data=None):
        r=client.post(route,json=data) if data is not None else client.post(route)
        assert r.status_code==200,r.text
        return r.json()
    def until(fn,seconds=35):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if fn():return
            time.sleep(.05)
        raise AssertionError(f'planner={system.planner.state}:{system.planner.reason}; tracker={system.tracker.state}:{system.tracker.reason}')
    def mission(curve,allowed=False):
        frame=system.localization.frame;points=[]
        for x,y in [(-10,-8),(10,-8),(10,8),(-10,8)]:
            lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
            points.append({'longitude':lon,'latitude':lat})
        return post('/api/missions',{'name':'原生曲线隔离验证','mode':'coverage','boundary':{'points':points},
            'parameters':{'path_continuity':curve,'allow_reverse':allowed,'headland_width':4.,
                'min_turning_radius':1.,'operation_width':1.,'speed':.5,'lookahead':.2,
                'sampling_distance':.05,'completion_tolerance':.06,'search_window':.4}})
    try:
        until(lambda:get('/api/devices')['safety']['fresh'])
        post('/api/system/mode',{'mode':'coverage'});until(system.planner.mode_ready)
        until(system.planner.client.server_is_ready)
        for curve in ('DUBINS','DUBINS_CC','REEDS_SHEPP','REEDS_SHEPP_HC'):
            m=mission(curve,curve.startswith('REEDS'))
            post(f'/api/missions/{m["id"]}/plan')
            until(lambda:(get('/api/workspace').get('active_run') or {}).get('state') in ('READY','FAILED'))
            run=get('/api/workspace')['active_run']
            assert run['state']=='READY',(curve,run)
            data=json.loads(next((system.workspace.store.run_path(run['id'])/'paths').glob('coverage_*.json')).read_text())
            assert data['planning']['curve_type']==curve
            assert not system.safety.gate.owner and system.state['v']==system.state['w']==0
        # The same native RS planner must reject reverse when the option is off.
        m=mission('REEDS_SHEPP',False)
        post(f'/api/missions/{m["id"]}/plan')
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='FAILED')
        assert '倒车' in get('/api/workspace')['active_run']['reason']
        assert get('/api/robot/path/reference')['count']==0
        # A short, genuine native curve exercises complete F/B/F tracking quickly.
        m=mission('REEDS_SHEPP',True)
        builder=Builder(box(-5,-5,5,5),1,2,1,.05,.35,'REEDS_SHEPP',allow_reverse=True)
        poses=NativeTurns(builder,options({}),True).connect((0.,0.,0.),(0.,.4,math.pi))
        source=system.workspace.store.create_run(m,system.planner.mode_revision,'planning')
        source.update(state='READY',saved_plan_version=1,reference_fingerprint=signature(poses))
        system.workspace.store.save_run(source)
        atomic_json(system.workspace.store.run_path(source['id'])/'paths/coverage_native.json',{
            'frame_id':'map','projection':m['projection'],
            'points':[dict(zip(('x','y','yaw'),p[:3])) for p in poses],
            'is_backward':backward_flags(poses),'velocities':[],
            'planning':{'curve_type':'REEDS_SHEPP','allow_reverse':True}})
        route=f'/api/missions/{m["id"]}'
        loaded=post(f'{route}/plans/{source["id"]}/load')['run']
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        post('/api/test/align-path-start');time.sleep(.25)
        assert client.post(route+'/start').status_code==409
        post('/api/robot/emergency-stop/reset')
        until(lambda:not get('/api/devices')['safety']['motion_inhibited'])
        commands=[]
        system.bench.create_subscription(Twist,'/base/driver_cmd_vel',lambda msg:commands.append(msg.linear.x),10)
        post(route+'/start')
        until(lambda:system.tracker.state=='COMPLETED',50)
        assert min(commands)<-.01 and max(commands)>.01
        assert not system.safety.gate.owner and system.state['v']==system.state['w']==0
        directory=system.workspace.store.run_path(loaded['id'])
        saved=json.loads(next((directory/'paths').glob('coverage_*.json')).read_text())
        assert any(saved['is_backward'])
        assert system.tracker.controller.tracker.leg_index>=2
        sidecar=json.loads(next((directory/'tracks').glob('track_*.json')).read_text())
        assert sidecar['allow_reverse'] is True and -1 in sidecar['segment_directions']
        until(lambda:not system.backend.workspace_ros.recorder.busy(directory),45)
    finally:
        client.close();system.close()
