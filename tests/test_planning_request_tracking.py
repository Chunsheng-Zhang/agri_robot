"""Late planner heartbeats cannot terminate a different planning request."""
import json
from contextlib import nullcontext
from types import SimpleNamespace
import pytest
from fastapi import HTTPException
from agri_backend.workspace import Workspace
from agri_backend.devices import device_cache


def test_delayed_previous_failure_is_ignored_but_current_failure_is_applied(monkeypatch):
    saved=[]
    run={'id':'new-request','stage':'planning','state':'PLANNING','reason':'','mode_revision':7}
    status={'fresh':True,'age_seconds':0.,'state':'FAILED','reason':'old failure','request_id':'old-request'}
    monkeypatch.setattr(device_cache,'snapshot',lambda:{'safety':{'fresh':True,'mode_revision':7},'planning':status})
    workspace=SimpleNamespace(active=run,robot_snapshot=lambda:{},plan_requested=0.,plan_sequence=0,
                              store=SimpleNamespace(save_run=lambda v:saved.append(dict(v))))
    Workspace.sync(workspace)
    assert run['state']=='PLANNING' and saved==[]
    status.update(request_id='new-request',reason='current failure')
    Workspace.sync(workspace)
    assert run['state']=='FAILED' and run['reason']=='current failure' and len(saved)==1


@pytest.mark.parametrize('policy',['POINT_TURN_ANGLE','POINT_TURN_NATIVE_TSP'])
def test_all_point_turn_policies_require_motion_fingerprint_even_without_reverse(policy,tmp_path):
    folder=tmp_path/'paths';folder.mkdir()
    points=[{'x':0.,'y':0.,'yaw':0.},{'x':1.,'y':0.,'yaw':0.},
            {'x':1.,'y':0.,'yaw':1.5707963267948966},{'x':1.,'y':1.,'yaw':1.5707963267948966}]
    (folder/'coverage_test.json').write_text(json.dumps({'points':points}))
    mission={'id':'mission','mode':'coverage','revision':1,'parameters':{'speed':.1,'max_angular':.3,'max_path_error':.2,'allow_reverse':False}}
    run={'id':'request','stage':'planning','state':'READY','mission_id':'mission','mission_revision':1,
         'mode_revision':7,'reference_fingerprint':'geometry','planning_policy':policy}
    calls=[]
    gateway=SimpleNamespace(parameters=lambda *args:calls.append(args))
    workspace=SimpleNamespace(operation=lambda:nullcontext(),store=SimpleNamespace(get=lambda _:mission,run_path=lambda _:tmp_path),
        require_idle=lambda _:{'mode_revision':7,'limits':{'linear':1.,'angular':1.,'path_error':2.}},
        validate_mission=lambda *_:mission,gateway=lambda:gateway,sync=lambda:None,active=run,
        robot_snapshot=lambda:{'reference_fingerprint':'geometry','reference_motion_fingerprint':'tampered'})
    with pytest.raises(HTTPException,match='参考路径航向或方向不一致'):
        Workspace.start(workspace,'mission')
    assert calls==[]
