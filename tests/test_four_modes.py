"""Mode contracts, idempotent switching and recording epoch invalidation."""
from types import SimpleNamespace
from typing import get_args

import pytest
from fastapi import HTTPException

from agri_base.modes import WORK_MODES, WorkMode, MODE_SOURCE, source_allowed
from agri_backend.devices import device_cache
from agri_backend.workspace import Workspace, ModeCommand


def test_four_modes_have_distinct_autonomous_authorities():
    assert WORK_MODES == ('base','waypoint','coverage','following')
    assert set(get_args(WorkMode)) == set(WORK_MODES)
    for mode in WORK_MODES:
        assert ModeCommand(mode=mode).mode==mode
        assert source_allowed(mode,'manual')  # explicit joystick takeover
        assert source_allowed(mode,'web')==(mode=='base')
        allowed=[source for source in MODE_SOURCE.values() if source_allowed(mode,source)]
        assert allowed==([] if mode=='base' else [MODE_SOURCE[mode]])
    assert not source_allowed('following','planner')
    assert not source_allowed('manual','manual')  # authority is not a work mode
    with pytest.raises(ValueError):ModeCommand(mode='teaching')


@pytest.mark.parametrize('mode',WORK_MODES)
def test_repeated_mode_selection_does_not_stop_or_release_current_control(mode,tmp_path,monkeypatch):
    monkeypatch.setattr(device_cache,'values',{})
    device_cache.update('safety',{'mode':mode,'owner':MODE_SOURCE.get(mode,'web'),'mode_revision':8})
    from agri_backend.missions import MissionStore
    workspace=Workspace(lambda:None,lambda:{},store=MissionStore(tmp_path))
    actions=[]
    monkeypatch.setattr(workspace,'gateway',lambda:SimpleNamespace(
        stop=lambda:actions.append('stop'),trigger=lambda path:actions.append(path)))
    try:
        result=workspace.select_mode(mode)
        assert result['success'] and result['changed'] is False and actions==[]
        with pytest.raises(HTTPException):workspace.select_mode('bad')
        assert actions==[]
        target='following' if mode!='following' else 'base'
        workspace.select_mode(target)
        assert actions==['stop',f'/system/mode/{target}']
    finally:
        workspace.quality_jobs.close();workspace.coverage_maps.close()


def test_teaching_cannot_span_a_mode_switch_and_quick_return_to_base(tmp_path,monkeypatch):
    from agri_backend.following import FollowingManager, RecordRequest
    from time import monotonic
    origin={'epsg':32650,'latitude':36.6468527633281,'longitude':117.021433066391}
    monkeypatch.setattr(device_cache,'values',{})
    safety={'mode':'base','owner':'manual','motion_inhibited':False,'joy_connected':True,'mode_revision':1}
    device_cache.update('safety',safety)
    device_cache.update('origin',{'projection':origin})
    manager=FollowingManager(tmp_path)
    manager.valid,manager.valid_at=True,monotonic()
    manager.accept_pose({'stamp':100.,'x':0.,'y':0.,'yaw':0.})
    identity=manager.start_record(RecordRequest(name='跨模式示教'),origin)
    device_cache.update('safety',{**safety,'mode_revision':3})  # base→following→base between writer ticks
    with pytest.raises(ValueError,match='切换'):manager.stop_record()
    manager.work_once()
    assert manager.state=='FAULT'
    assert manager.store.get(identity,ready=False)['state']=='FAULT'
    assert not manager.store.directory(identity).joinpath('path.json').exists()
    manager.close()


def test_status_and_monitor_trajectory_follow_selected_mode(tmp_path,monkeypatch):
    from agri_backend import main
    from agri_backend.following import following_manager
    monkeypatch.setattr(device_cache,'values',{})
    device_cache.update('safety',{'mode':'following','owner':'','mode_revision':1})
    device_cache.update('following',{'state':'PAUSED'})
    monkeypatch.setattr(following_manager,'reference',[{'x':1.,'y':2.}]*3)
    monkeypatch.setattr(following_manager,'track',[{'x':1.,'y':2.}]*2)
    data={**main.robot_state.data,'reference_path_points':99,'tracked_path_points':88,
          'tracked_source':'tracking','tracked_path':[{'x':100.,'y':100.}], 'tracked_frame':'map'}
    monkeypatch.setattr(main.robot_state,'data',data)
    status=main.get_robot_status()
    assert status['work_mode']=='following' and status['control_mode']==''
    assert status['task_state']=='PAUSED'
    assert (status['reference_path_points'],status['tracked_path_points'])==(3,2)
    assert main.get_tracked_path('waypoint')['points']==[]
    assert main.get_tracked_path('tracking')['count']==1
    device_cache.update('safety',{'mode':'base','owner':''})
    assert main.get_robot_status()['reference_path_points']==0


def test_generic_following_control_does_not_bypass_validated_start(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main, devices
    from agri_backend.missions import MissionStore
    calls=[]
    monkeypatch.setattr(main.app.state,'ros_bridge',SimpleNamespace(started=True,node=object()),raising=False)
    monkeypatch.setattr(main.app.state.workspace,'store',MissionStore(tmp_path))
    monkeypatch.setattr(devices,'service_call',lambda node,path: calls.append(path) or {'success':True})
    client=TestClient(main.app)
    assert client.post('/api/tasks/following/start').status_code==404
    for action in ('pause','resume','cancel'):
        assert client.post(f'/api/tasks/following/{action}').status_code==200
    assert calls==['/following/pause','/following/resume','/following/cancel']
