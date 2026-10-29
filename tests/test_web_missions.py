import json
from pathlib import Path

import pytest
from pyproj.enums import TransformDirection
from pydantic import ValidationError

from agri_backend.missions import Mission, MissionStore
from agri_base.coordinates import LocalFrame

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def store(tmp_path):
    result = MissionStore(tmp_path)
    result.config_path = ROOT / 'agri_robot_ws/src/agri_bringup/config/system.yaml'
    return result


def geo(frame, x, y):
    lon, lat = frame.transformer.transform(frame.e0 + x, frame.n0 + y, direction=TransformDirection.INVERSE)
    return {'longitude': lon, 'latitude': lat}


def test_web_boundaries_and_waypoints_keep_global_origin(store):
    origin = store.configured_origin()
    frame = LocalFrame(origin['latitude'], origin['longitude'])
    outer = [geo(frame, x, y) for x, y in ((10, 10), (30, 10), (30, 30), (10, 30))]
    hole = [geo(frame, x, y) for x, y in ((15, 15), (20, 15), (20, 20), (15, 20))]
    field = store.save(Mission(name='实验田', mode='coverage', boundary={'rings': [outer, hole]}), origin)
    assert field['preview']['area'] == pytest.approx(375, abs=1e-5)
    assert field['preview']['rings'][0][0]['x'] == pytest.approx(10, abs=1e-7)
    targets = store.save(Mission(name='目标点', mode='waypoint', waypoints=[outer[0]]), origin)
    assert targets['preview']['points'][0]['x'] == pytest.approx(10, abs=1e-7)
    assert targets['projection'] == field['projection'] == origin


def test_mission_revisions_and_import_origin_are_checked(store):
    origin = store.configured_origin()
    task = Mission(name='第一版', mode='waypoint', waypoints=[{'latitude': origin['latitude'], 'longitude': origin['longitude']}])
    first = store.save(task, origin)
    with pytest.raises(ValueError, match='已被修改'):
        store.save(task, origin, first['id'])
    task.revision, task.name = first['revision'], '第二版'
    assert store.save(task, origin, first['id'])['revision'] == 2
    task.import_origin = {**origin, 'latitude': origin['latitude'] + 0.01}
    with pytest.raises(ValueError, match='原点不同'):
        store.save(task, origin)


def test_unsafe_parameters_and_file_paths_rejected(store):
    origin = store.configured_origin()
    point = {'latitude': origin['latitude'], 'longitude': origin['longitude']}
    for parameters in ({'origin_latitude': 0}, {'max_linear': 0.1, 'min_linear': 0.2}, {'max_angular': float('nan')}):
        with pytest.raises((ValidationError, ValueError)):
            store.save(Mission(name='bad', mode='waypoint', waypoints=[point], parameters=parameters), origin)
    with pytest.raises(ValidationError):
        store.save(Mission(name='bad reverse', mode='coverage', boundary={'points': [point] * 3},
                           parameters={'path_continuity': 'REEDS_SHEPP'}), origin)
    with pytest.raises(ValueError):
        store.path('../../secret')
    with pytest.raises(ValueError):
        store.run_path('../../secret')


def test_workspace_http_offline_edits_but_never_grants_motion(store, monkeypatch):
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.devices import device_cache
    monkeypatch.setattr(device_cache, 'values', {})
    monkeypatch.setattr(main.app.state, 'ros_bridge', None, raising=False)
    workspace = main.app.state.workspace
    monkeypatch.setattr(workspace, 'store', store)
    monkeypatch.setattr(workspace, 'active', None)
    client = TestClient(main.app)  # No lifespan: no ROS node or hardware.
    origin = store.configured_origin()
    payload = {'name': '室外测试', 'mode': 'waypoint', 'waypoints': [
        {'latitude': origin['latitude'], 'longitude': origin['longitude'], 'dwell_time': 1}]}
    saved = client.post('/api/missions', json=payload)
    assert saved.status_code == 200, saved.text
    identity = saved.json()['id']
    context = client.get('/api/workspace').json()
    assert context['origin_source'] == 'configured_yaml'
    assert context['missions'][0]['id'] == identity
    assert client.post('/api/system/mode', json={'mode': 'coverage'}).status_code == 503
    assert client.post('/api/control/arm').status_code == 503
    assert client.post(f'/api/missions/{identity}/start').status_code == 503
    assert client.post('/api/system/mode', json={'mode': 'shell command'}).status_code == 422
    invalid = client.post('/api/control/command', content='{"token":"' + 'a' * 32 +
        '","seq":1,"linear":NaN,"angular":0,"expires_at":1}', headers={'Content-Type': 'application/json'})
    assert invalid.status_code == 422 and 'linear' in invalid.json()['detail']
    assert client.post('/api/missions/preview', json={**payload, 'waypoints': [{'latitude': 0, 'longitude': 0}]}).status_code == 422
    assert client.put(f'/api/missions/{identity}', json=payload).status_code == 422  # Missing revision.
    assert client.delete(f'/api/missions/{identity}').status_code == 200
    assert not list(store.documents.glob('*.json'))


def test_run_archives_are_separate_from_profiles(store):
    origin = store.configured_origin()
    mission = store.save(Mission(name='记录', mode='waypoint', waypoints=[
        {'latitude': origin['latitude'], 'longitude': origin['longitude']}]), origin)
    first, second = (store.create_run(mission, 3, 'execution') for _ in range(2))
    assert first['id'] != second['id']
    for run in (first, second):
        saved = json.loads((store.run_path(run['id']) / 'config/mission.json').read_text())
        assert saved['projection'] == origin
        assert saved['revision'] == mission['revision']


def test_mode_switch_does_not_rewrite_finished_run(store, monkeypatch):
    from agri_backend.workspace import Workspace
    from agri_backend.devices import device_cache
    origin = store.configured_origin()
    mission = store.save(Mission(name='已完成', mode='waypoint', waypoints=[
        {'latitude': origin['latitude'], 'longitude': origin['longitude']}]), origin)
    run = store.create_run(mission, 1, 'execution')
    run['state'] = 'COMPLETED'
    store.save_run(run)
    workspace = Workspace(lambda: None, lambda: {}, store)
    workspace.active = run
    monkeypatch.setattr(device_cache, 'values', {})
    device_cache.update('safety', {'mode': 'coverage', 'mode_revision': 2})
    workspace.sync()
    assert run['state'] == store.list_runs()[0]['state'] == 'COMPLETED'
