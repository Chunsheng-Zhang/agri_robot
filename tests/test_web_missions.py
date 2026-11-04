import json
from pathlib import Path

import pytest
from pyproj.enums import TransformDirection
from pydantic import ValidationError

from agri_backend.missions import Mission, MissionStore, CoverageParameters, WaypointParameters
from agri_base.coordinates import LocalFrame

ROOT = Path(__file__).resolve().parents[1]


def test_coverage_accepts_new_velocity_bounds_without_changing_waypoint_bounds():
    assert CoverageParameters(speed=1, max_angular=1).max_angular == 1
    for values in ({'speed': 1.001}, {'speed': -1}, {'max_angular': 1.001}):
        with pytest.raises(ValidationError):
            CoverageParameters(**values)
    with pytest.raises(ValidationError):
        WaypointParameters(max_angular=5)


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


def test_archive_includes_raw_and_analysis_and_blocks_live_analysis(store, monkeypatch):
    import io
    import zipfile
    from types import SimpleNamespace
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.devices import device_cache
    workspace = main.app.state.workspace
    monkeypatch.setattr(workspace, 'store', store)
    monkeypatch.setattr(workspace, 'active', None)
    monkeypatch.setattr(device_cache, 'values', {})
    run, _ = create_record(store)
    folder = store.run_path(run['id'])
    for name, content in (('raw/mgx23.bin', b'original\x00bytes'),
                          ('analysis/heading_error.png', b'png-data'),
                          ('analysis/README.md', b'report')):
        path = folder / name
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(content)
    pending = [True]
    recorder = SimpleNamespace(busy=lambda path: pending[0])
    monkeypatch.setattr(main.app.state, 'ros_bridge', SimpleNamespace(
        node=SimpleNamespace(workspace_ros=SimpleNamespace(recorder=recorder))), raising=False)
    client = TestClient(main.app)
    url = f'/api/runs/{run["id"]}/download'
    assert client.get(url).status_code == 409
    pending[0] = False
    response = client.get(url)
    assert response.status_code == 200
    with zipfile.ZipFile(io.BytesIO(response.content)) as archive:
        assert archive.read('raw/mgx23.bin') == b'original\x00bytes'
        assert archive.read('analysis/heading_error.png') == b'png-data'
        assert 'analysis/README.md' in archive.namelist()
    assert not list((store.root / '.runtime/archives').glob('*.zip'))


def create_record(store, state='COMPLETED', mode='waypoint'):
    origin = store.configured_origin()
    mission = store.save(Mission(name='记录清理测试', mode='waypoint', waypoints=[
        {'latitude': origin['latitude'], 'longitude': origin['longitude']}]), origin)
    run = store.create_run({**mission, 'mode': mode}, 1, 'planning' if mode == 'coverage' else 'execution')
    run['state'] = state
    store.save_run(run)
    return run, mission


@pytest.fixture
def records_client(store, monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.devices import device_cache
    workspace = main.app.state.workspace
    monkeypatch.setattr(device_cache, 'values', {})
    monkeypatch.setattr(workspace, 'store', store)
    monkeypatch.setattr(workspace, 'active', None)
    monkeypatch.setattr(workspace, 'plan_requested', None)
    monkeypatch.setattr(workspace, 'plan_sequence', None)
    def no_control():
        pytest.fail('Record deletion must never call ROS or take control')
    monkeypatch.setattr(workspace, 'gateway', no_control)
    return TestClient(main.app), workspace  # No lifespan, ROS executor or hardware.


def test_delete_record_preserves_data_mission_and_persists_across_store_restart(store, records_client):
    client, workspace = records_client
    run, mission = create_record(store)
    other, _ = create_record(store)
    directory = store.run_path(run['id'])
    (directory / 'tracks').mkdir()
    (directory / 'tracks/waypoint_test.csv').write_text('stamp,x,y\n1,2,3\n')
    before = {path.relative_to(directory): path.read_bytes() for path in directory.rglob('*') if path.is_file()}
    workspace.active = run
    response = client.delete(f'/api/runs/{run["id"]}')
    assert response.status_code == 200, response.text
    assert response.json() == {'success': True, 'data_preserved': True}
    assert workspace.active is None
    assert {path.relative_to(directory): path.read_bytes() for path in directory.rglob('*') if path.is_file()} == before
    assert store.get(mission['id']) == mission
    assert [record['id'] for record in store.list_runs()] == [other['id']]
    assert [record['id'] for record in MissionStore(store.root).list_runs()] == [other['id']]
    assert [record['id'] for record in client.get('/api/workspace').json()['runs']] == [other['id']]
    assert client.delete(f'/api/runs/{run["id"]}').status_code == 200  # Safe repeat.
    assert 'RUN_RECORD_DELETED' in (store.root / 'data/web/events.jsonl').read_text()


@pytest.mark.parametrize('state', ['PREPARING', 'PLANNING', 'ACQUIRING', 'RUNNING', 'PAUSED', 'WAITING_PATH', 'UNKNOWN'])
def test_unfinished_record_cannot_be_deleted(store, records_client, state):
    client, _ = records_client
    run, _ = create_record(store, state)
    response = client.delete(f'/api/runs/{run["id"]}')
    assert response.status_code == 409
    assert not store.deleted_run_path(run['id']).exists()
    assert store.list_runs()[0]['id'] == run['id']


@pytest.mark.parametrize('source', ['waypoint', 'tracking', 'planning'])
def test_live_writer_blocks_deletion_even_if_stored_record_says_finished(store, records_client, source):
    from agri_backend.devices import device_cache
    client, _ = records_client
    run, _ = create_record(store)
    directory = store.run_path(run['id'])
    device_cache.update(source, {'state': 'PLANNING' if source == 'planning' else 'RUNNING',
        'output_directory' if source == 'planning' else 'run_directory': str(directory / ('paths' if source == 'planning' else 'tracks'))})
    assert client.delete(f'/api/runs/{run["id"]}').status_code == 409
    assert not store.deleted_run_path(run['id']).exists()


def test_delete_ready_plan_clears_only_its_binding_and_does_not_reattach(store, records_client):
    client, workspace = records_client
    run, _ = create_record(store, 'READY', 'coverage')
    workspace.active = run
    workspace.plan_requested, workspace.plan_sequence = 123, 5
    assert client.delete(f'/api/runs/{run["id"]}').status_code == 200
    assert workspace.active is None
    assert workspace.plan_requested is workspace.plan_sequence is None
    assert client.get('/api/workspace').json()['active_run'] is None
    assert store.run_path(run['id']).is_dir()


def test_deleting_old_record_does_not_clear_another_active_run(store, records_client):
    client, workspace = records_client
    old, _ = create_record(store)
    active, _ = create_record(store, 'RUNNING')
    workspace.active = active
    assert client.delete(f'/api/runs/{old["id"]}').status_code == 200
    assert workspace.active is active
    assert store.list_runs()[0]['id'] == active['id']


def test_delete_invalid_missing_record_and_busy_lock_leave_store_unchanged(store, records_client):
    client, workspace = records_client
    run, _ = create_record(store)
    assert client.delete('/api/runs/not-a-run').status_code == 422
    assert client.delete('/api/runs/20261030_010101_000000_web_00000000').status_code == 404
    workspace.lock.acquire()
    try:
        assert client.delete(f'/api/runs/{run["id"]}').status_code == 409
    finally:
        workspace.lock.release()
    assert not store.deleted_run_path(run['id']).exists()


def test_deleted_entries_are_filtered_before_the_50_record_limit(store):
    run, mission = create_record(store)
    for _ in range(50):
        run = store.create_run(mission, 1, 'execution')
        run['state'] = 'COMPLETED'
        store.save_run(run)
    store.delete_run(run['id'])
    assert len(store.list_runs()) == 50
    assert run['id'] not in {record['id'] for record in store.list_runs()}
