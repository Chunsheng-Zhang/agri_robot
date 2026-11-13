"""Point-turn planner/API/save/load on explicitly isolated ROS, no hardware."""
import json
import os
import time
import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS')!='1',reason='Opt-in localhost synthetic domain')


def test_web_smart_planning_success_save_load_failure_and_cancel_without_motion():
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from web_test_runtime import SyntheticSystem
    from agri_base.path_geometry import point_turn_indices,rotation_envelope,motion_signature
    system=SyntheticSystem();client=TestClient(system.app)
    def get(route):
        r=client.get(route);assert r.status_code==200,r.text
        return r.json()
    def post(route,payload=None):
        r=client.post(route,json=payload) if payload is not None else client.post(route)
        assert r.status_code==200,r.text
        return r.json()
    def until(fn,timeout=35):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if fn():return
            time.sleep(.05)
        raise AssertionError(f'timeout: {system.planner.state}: {system.planner.reason}')
    def create(width=16,height=12,**changes):
        frame=system.localization.frame;points=[]
        for x,y in [(0,0),(width,0),(width,height),(0,height)]:
            lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
            points.append({'longitude':lon,'latitude':lat})
        return post('/api/missions',{'name':'原地转向智能规划隔离验证','mode':'coverage','boundary':{'points':points},
            'parameters':{**dict(operation_width=1.,headland_width=2.,min_turning_radius=.6,
                evaluation_width=3.,optimization_max_candidates=1,optimization_time_budget=.001),**changes}})
    try:
        until(lambda:get('/api/devices')['safety']['fresh'])
        post('/api/system/mode',{'mode':'coverage'});until(system.planner.mode_ready)
        mission=create(optimization_min_coverage=100,optimization_patterns='CONTOUR',
                       optimization_headland_min=100,optimization_headland_max=100,
                       optimization_weight_energy=1,optimization_headland_mode='MANUAL',allow_reverse=True)
        for key in ('optimization_min_coverage','optimization_patterns','optimization_headland_min',
                    'optimization_weight_energy','optimization_headland_mode','optimization_max_candidates','optimization_time_budget'):
            assert key not in mission['parameters']
        from std_msgs.msg import String
        feedback=[]
        system.bench.create_subscription(String,'/planning/status',lambda message:feedback.append(json.loads(message.data)),10)
        post(f'/api/missions/{mission["id"]}/optimize')
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        until(lambda:((get('/api/workspace').get('active_run') or {}).get('coverage_quality') or {}).get('state')=='SUCCEEDED')
        run=get('/api/workspace')['active_run'];directory=system.workspace.store.run_path(run['id'])
        report=get(f'/api/runs/{run["id"]}/optimization')
        assert report['state']=='SUCCEEDED' and report['schema_version']==2
        assert report['turn_mode']=='POINT_TURN' and report['coverage_objective_enabled']
        assert report['weights']=={'coverage':.5,'overlap':.3,'length':.2}
        assert report['search_complete'] and report['evaluated_count']>1
        assert not report['candidate_limit_enabled'] and not report['search_time_limit_enabled']
        assert report['configuration']=={}
        assert any(v['state']=='PLANNING' and (v.get('search_progress') or {}).get('evaluated_count',0)>0 for v in feedback)
        status=get('/api/devices')['planning']
        assert status['search_progress']['phase']=='COMPLETE'
        assert status['search_progress']['evaluated_count']==report['evaluated_count']
        elapsed=status['search_progress']['elapsed_seconds'];time.sleep(.25)
        assert get('/api/devices')['planning']['search_progress']['elapsed_seconds']==elapsed
        assert {c['mode'] for c in report['candidates']}=={'BOUSTROPHEDON'}
        assert report['reference_fingerprint']==run['reference_fingerprint']
        assert report['projection']==run['projection'] and not report['allow_reverse']
        assert 'energy_model' not in report and report['curve_type']=='POINT_TURN'
        saved=json.loads(next((directory/'paths').glob('coverage_*.json')).read_text())
        poses=[(p['x'],p['y'],p['yaw']) for p in saved['points']]
        stops=point_turn_indices(poses);assert len(stops)>2
        assert saved['motion_signature']==motion_signature(poses)
        h=rotation_envelope(.45,.2,.05)['headland_width_m']
        assert report['selected']['headland_width_m']==h
        assert saved['planning']['rotation_envelope']==rotation_envelope(.45,.2,.05)
        assert not any(saved['is_backward'])
        assert run['coverage_quality']['planned']['effective_headland_width_m']==h
        assert run['coverage_quality']['planned']['width_m']==1.
        assert json.loads((directory/'analysis/coverage_quality_plan.json').read_text())['width_source']=='operation_width_effective'
        until(lambda:get(f'/api/runs/{run["id"]}/coverage-map').get('state')=='READY')
        assert get(f'/api/runs/{run["id"]}/coverage-map')['inset_distance_m']==h
        assert run['coverage_quality']['planned']['metrics']['field']['coverage_percent']==pytest.approx(report['selected']['coverage_percent'],abs=.001)
        assert json.loads((directory/'config/mission.json').read_text())['parameters']['headland_width']==2.
        # Finishing and explicitly saving/loading never unlocks or drives.
        s=get('/api/devices')['safety'];assert s['emergency_stop'] and not s['owner']
        assert system.state['v']==system.state['w']==0
        route=f'/api/missions/{mission["id"]}/plans/{run["id"]}'
        post(route+'/save');assert get(route+'/path')['count']==len(poses)
        loaded=post(route+'/load')['run']
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='READY')
        assert loaded['source_plan_id']==run['id'] and loaded['planning_policy']=='POINT_TURN_ANGLE'
        latest=system.tracker.pending_path
        from agri_base.runtime import yaw_of
        loaded_poses=[(p.pose.position.x,p.pose.position.y,yaw_of(p.pose.orientation)) for p in latest.poses]
        assert point_turn_indices(loaded_poses)==stops
        assert system.state['v']==system.state['w']==0 and not system.safety.gate.owner
        # Execute only inside this hardware-free testbench. Verify that the ROS
        # tracker actually consumes the saved stop markers and pause semantics.
        post('/api/test/align-path-start');time.sleep(.25)
        post('/api/robot/emergency-stop/reset')
        until(lambda:not get('/api/devices')['safety']['motion_inhibited'])
        post(f'/api/missions/{mission["id"]}/start')
        until(lambda:system.tracker.state=='RUNNING' and system.tracker.controller.phase=='TRACKING')
        control=system.tracker.controller
        assert len(control.path.legs)==len(stops)+1
        hi=control.tracker.leg[1];x,y,_=control.path.at(hi)
        yaw=control.path.at(hi-1e-6)[2]
        control.tracker.progress=hi-.001
        system.state.update(x=x,y=y,yaw=yaw,v=0.,w=0.)
        until(lambda:control.phase=='ALIGNING' and control.alignment_reason=='point_turn')
        until(lambda:abs(system.state['w'])>.05)
        assert system.state['v']==0 and abs(system.state['w'])<=.3
        index=control.tracker.leg_index
        post('/api/tasks/tracking/pause')
        until(lambda:system.tracker.state=='PAUSED' and system.state['v']==system.state['w']==0)
        post('/api/tasks/tracking/resume')
        until(lambda:control.phase=='ALIGNING' and control.alignment_reason=='resume')
        assert control.tracker.leg_index==index and system.state['v']==0
        post('/api/tasks/tracking/cancel')
        until(lambda:system.tracker.state=='CANCELED' and not system.safety.gate.owner)
        execution=system.workspace.store.run_path(loaded['id'])
        until(lambda:not system.backend.workspace_ros.recorder.busy(execution),45)
        assert list((execution/'tracks').glob('alignment_*.csv'))
        quality_report=execution/'analysis/coverage_quality_report.md'
        until(quality_report.exists,45)
        text=quality_report.read_text()
        assert '本轮原地转向智能规划' in text and '0.5×' in text
        assert '相对能耗代理：' not in text
        # Huge chassis cannot fit; there is no partial reference to execute.
        mission=create(width=2,height=2,robot_length=10.)
        post(f'/api/missions/{mission["id"]}/optimize')
        until(lambda:(get('/api/workspace').get('active_run') or {}).get('state')=='FAILED')
        failed=get('/api/workspace')['active_run'];folder=system.workspace.store.run_path(failed['id'])
        assert get(f'/api/runs/{failed["id"]}/optimization')['state']=='FAILED'
        assert not list((folder/'paths').glob('coverage_*.json'))
        assert get('/api/robot/path/reference')['count']==0
        mission=create(width=100,height=80,operation_width=.1)
        result=post(f'/api/missions/{mission["id"]}/optimize')
        post('/api/planning/cancel');until(lambda:system.planner.contour_future is None,10)
        assert system.planner.state=='CANCELED'
        until(lambda:get('/api/devices')['planning']['search_progress']['phase']=='CANCELED')
        assert not list((system.workspace.store.run_path(result['run']['id'])/'paths').glob('coverage_*.json'))
        assert get('/api/robot/path/reference')['count']==0
        assert system.state['v']==system.state['w']==0 and not system.safety.gate.owner
    finally:
        client.close();system.close()
