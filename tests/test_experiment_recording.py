"""Read-only task capture and analysis, with synthetic data and no chassis node."""
import base64
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from agri_backend.experiment_recording import CaptureSession, ExperimentRecorder
from agri_backend.experiment_analysis import analyze_run, statistics
from agri_backend.missions import atomic_json


ORIGIN = {'epsg': 32650, 'latitude': 36.6468527633281, 'longitude': 117.021433066391}


def prepared_run(tmp_path):
    atomic_json(tmp_path / 'run.json', {'id': 'synthetic-run', 'mode': 'coverage',
        'state': 'COMPLETED', 'projection': ORIGIN})
    atomic_json(tmp_path / 'config/mission.json', {'parameters': {'speed': .2},
        'projection': ORIGIN, 'preview': {'rings': [[{'x': -1, 'y': -1}, {'x': 3, 'y': -1},
            {'x': 3, 'y': 1}, {'x': -1, 'y': 1}, {'x': -1, 'y': -1}]]}})
    points = [{'x': 0., 'y': 0.}, {'x': 2., 'y': 0.}]
    atomic_json(tmp_path / 'paths/coverage_test.json', {'projection': ORIGIN, 'frame_id': 'map', 'points': points})
    atomic_json(tmp_path / 'tracks/track_test.json', {'projection': ORIGIN, 'frame_id': 'map',
        'reference_path': {'frame_id': 'map', 'points': points}})
    with (tmp_path / 'tracks/track_test.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['stamp','x','y','yaw','cross_track','heading_error','progress'])
        writer.writerows([[100,0,.1,0,.1,.2,0], [101,1,-.1,0,-.1,-.2,1], [102,1.9,.05,0,.05,.1,1.9]])
    atomic_json(tmp_path / 'tracks/approach_test.json', {'projection': ORIGIN, 'frame_id': 'map'})
    (tmp_path / 'tracks/approach_test.csv').write_text('stamp,x,y,yaw,target_index,distance,heading_error\n99,-1,0,0,0,1,2\n')
    return tmp_path


def packet(data, seq, session='device-1'):
    return json.dumps({'schema_version': 1, 'session': session, 'sequence': seq,
        'stamp_ns': seq * 1000000000, 'unix_ns': seq * 1000000000,
        'transport': 'tcp', 'data_base64': base64.b64encode(data).decode()})


def finish(session):
    session.request_stop('COMPLETED', tail=0)
    session.stop()
    assert session.finished.wait(3)
    session.thread.join(1)


def test_raw_bytes_preserved_sequence_gaps_and_missing_topics_are_explicit(tmp_path):
    session = CaptureSession(tmp_path, 'tracking')
    session.put('raw', '/mgx23/raw', ([100,100,100], packet(b'\x00fmin\xff', 10)))
    session.put('raw', '/mgx23/raw', ([101,101,101], packet(b'ed\x00', 12)))
    session.put('raw', '/mgx23/raw', ([102,102,102], packet(b'new', 1, 'device-2')))
    finish(session)
    assert (tmp_path / 'raw/mgx23.bin').read_bytes() == b'\x00fmin\xffed\x00new'
    manifest = json.loads((tmp_path / 'telemetry/recording.json').read_text())
    assert manifest['state'] == 'CLOSED'
    assert manifest['raw_sequence_gaps'] == [{'session': 'device-1', 'after': 10, 'before': 12}]
    assert manifest['raw_sessions'] == ['device-1','device-2']
    chunks = list(csv.DictReader((tmp_path / 'raw/chunks.csv').open()))
    assert [int(r['offset']) for r in chunks] == [0,6,9]
    with pytest.raises(FileExistsError):
        CaptureSession(tmp_path, 'tracking')


def test_bounded_queue_reports_losses_and_stop_rejects_late_data(tmp_path):
    session = CaptureSession(tmp_path, 'tracking', queue_size=1)
    # Stop first so the consumer exits, then replace the bounded queue's insertion.
    from queue import Full
    def full(item):
        raise Full
    session.queue.put_nowait = full
    session.put('raw', '/mgx23/raw', ([], '{}'))
    assert session.dropped['/mgx23/raw'] == 1
    finish(session)
    session.put('raw', '/mgx23/raw', ([], '{}'))
    assert session.dropped['/mgx23/raw'] == 1


def test_analysis_four_plots_separates_phases_and_uses_real_speed_samples(tmp_path, monkeypatch):
    monkeypatch.setenv('MPLCONFIGDIR', str(tmp_path / 'mpl'))
    prepared_run(tmp_path)
    atomic_json(tmp_path / 'tracks/alignment_test.json', {'projection': ORIGIN, 'frame_id': 'map'})
    (tmp_path / 'tracks/alignment_test.csv').write_text(
        'stamp,x,y,yaw,target_yaw,heading_error,progress\n99.5,0,0,1,0,-1,0\n')
    session = CaptureSession(tmp_path, 'tracking')
    session.put('motion', '/cmd_vel', [100000000000,100,100,'','/cmd_vel','base_link',.2,0,.1])
    session.put('motion', '/vel_raw', [100000000001,101,101,'','/vel_raw','base_link',.23,0,.12])
    session.put('mgx', '/mgx23/velocity_stamped', [100000000002,102,102,100000000000,
        '/mgx23/velocity_stamped','gnss',.3,.4,0,''])
    session.put('raw', '/mgx23/raw', ([100000000003,103,103], packet(b'raw-received', 1)))
    finish(session)
    result = analyze_run(tmp_path)
    assert result['phases']['tracking']['cross_track_m']['mae'] == pytest.approx(1/12)
    assert result['phases']['tracking']['heading_error_rad']['mae'] == pytest.approx(1/6)
    assert result['phases']['approach']['heading_error_rad']['mae'] == 2
    assert result['phases']['alignment']['samples'] == 1
    assert result['phases']['alignment']['heading_error_rad']['mae'] == 1
    assert result['motion_by_topic']['/vel_raw']['linear_x_m_s']['abs_max'] == .23
    assert '/mgx23/raw' not in result['missing_topics']
    assert '/mgx23/accuracy' in result['missing_topics']
    assert result['phases']['tracking']['final_endpoint_distance_m'] == pytest.approx(math.hypot(.1,.05))
    assert (tmp_path / 'analysis/trajectory_and_speed.png').stat().st_size > 10000
    for name in ('trajectory','speed','cross_track_error','heading_error'):
        assert (tmp_path / f'analysis/{name}.png').is_file()
    assert (tmp_path / 'analysis/metrics.csv').is_file()
    assert json.loads((tmp_path / 'analysis/status.json').read_text())['state'] == 'SUCCEEDED'


def test_analysis_refuses_active_recording_and_coordinate_mismatch(tmp_path):
    prepared_run(tmp_path)
    session = CaptureSession(tmp_path, 'tracking')
    try:
        with pytest.raises(ValueError, match='尚未结束'):
            analyze_run(tmp_path)
    finally:
        finish(session)
    path = tmp_path / 'tracks/track_test.json'
    value = json.loads(path.read_text())
    value['projection']['latitude'] = 0
    atomic_json(path,value)
    with pytest.raises(ValueError, match='origin/frame mismatch'):
        analyze_run(tmp_path)


def test_statistics_empty_nan_and_p95():
    assert statistics([])['mae'] is None
    assert statistics([float('nan'), None, -.1, .2])['mae'] == pytest.approx(.15)
    assert statistics([-.1,.2])['abs_p95'] == pytest.approx(.195)


def test_unconfirmed_start_does_not_record_forever_on_old_idle_status(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Lock
    from time import monotonic
    recorder = ExperimentRecorder.__new__(ExperimentRecorder)
    recorder.lock, recorder.closed, recorder.pending_analysis = Lock(), False, {}
    recorder.analysis = ThreadPoolExecutor(max_workers=1)
    recorder.analyze = lambda directory: None
    recorder.session = session = CaptureSession(tmp_path, 'tracking')
    session.armed_at = monotonic() - 11
    session.last_status = monotonic()  # An idle controller still publishes periodically.
    try:
        recorder.tick()
        assert session.terminal['state'] == 'START_UNCONFIRMED'
        assert session.finished.wait(3)
        recorder.tick()
    finally:
        recorder.close()


def test_analysis_runs_in_separate_process_without_ros(tmp_path, monkeypatch):
    prepared_run(tmp_path)
    session = CaptureSession(tmp_path, 'tracking')
    finish(session)
    ExperimentRecorder.analyze(tmp_path)
    status = json.loads((tmp_path / 'analysis/status.json').read_text())
    assert status['state'] == 'SUCCEEDED', (tmp_path / 'analysis/analysis.log').read_text()
    assert (tmp_path / 'analysis/heading_error.png').is_file()


@pytest.mark.parametrize('failure', ['open', 409, 503, 504])
def test_capture_start_failures_preserve_data_and_do_not_infer_a_stop(tmp_path, monkeypatch, failure):
    from fastapi import HTTPException
    from agri_backend.missions import Mission, MissionStore, ROOT
    from agri_backend import workspace as module
    store = MissionStore(tmp_path)
    store.config_path = ROOT / 'config/jetson_system.yaml'
    mission = store.save(Mission(name='采集故障验证', mode='waypoint', waypoints=[
        {'latitude': ORIGIN['latitude'], 'longitude': ORIGIN['longitude']}]), ORIGIN)
    events = []
    def begin(*args):
        events.append('begin')
        if failure == 'open':
            raise OSError('synthetic disk failure')
    def service(*args):
        assert failure != 'open', 'Cannot start when capture files cannot be opened'
        events.append('service')
        raise HTTPException(failure, 'synthetic service failure')
    gateway = SimpleNamespace(parameters=lambda *args: None,
        recorder=SimpleNamespace(begin=begin, abort=lambda reason: events.append('abort')))
    workspace = module.Workspace(lambda: SimpleNamespace(node=None), lambda: {}, store)
    monkeypatch.setattr(workspace, 'gateway', lambda: gateway)
    monkeypatch.setattr(workspace, 'validate_mission', lambda *args: mission)
    monkeypatch.setattr(workspace, 'require_idle', lambda *args: {
        'mode_revision':1,'limits':{'linear':1.,'angular':2.,'path_error':2.}})
    monkeypatch.setattr(module, 'service_call', service)
    with pytest.raises(HTTPException) as caught:
        workspace.start(mission['id'])
    assert caught.value.status_code == (503 if failure == 'open' else failure)
    assert events == (['begin'] if failure == 'open' else ['begin', 'service', 'abort'] if failure == 409 else ['begin', 'service'])
    assert workspace.active['state'] == 'FAILED'


def test_observer_ignores_wrong_run_and_closes_after_terminal_without_browser(tmp_path, monkeypatch):
    """Exercise adapter callbacks with real ROS messages, but no DDS/driver nodes."""
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist, Vector3Stamped
    class Node:
        def create_subscription(self, *args):
            return args
        def create_timer(self, *args):
            return args
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100000000000))
    recorder = ExperimentRecorder(Node())
    analyzed = []
    monkeypatch.setattr(recorder, 'analyze', lambda directory: analyzed.append(directory))
    try:
        recorder.begin(tmp_path, 'tracking')
        session = recorder.session
        recorder.on_status('/tracking/status', String(data=json.dumps({'state':'COMPLETED',
            'run_directory':str(tmp_path / 'tracks')})))
        assert session.terminal is None
        recorder.on_status('/tracking/status', String(data=json.dumps({'state':'RUNNING',
            'run_directory':str(tmp_path / 'other')})))
        assert not session.seen_active
        recorder.on_status('/tracking/status', String(data=json.dumps({'state':'RUNNING',
            'run_directory':str(tmp_path / 'tracks')})))
        msg = Twist()
        msg.linear.x = .2
        recorder.on_motion('/cmd_vel', msg)
        v = Vector3Stamped()
        v.header.stamp.sec = 100
        v.vector.x, v.vector.y = .3,.4
        recorder.on_vector('/mgx23/velocity_stamped',v)
        recorder.on_raw(String(data=packet(b'data',1)))
        recorder.on_status('/tracking/status', String(data=json.dumps({'state':'COMPLETED',
            'run_directory':str(tmp_path / 'tracks')})))
        assert session.terminal['state'] == 'COMPLETED'
        session.stop_at = 0
        recorder.tick()
        assert session.finished.wait(3)
        recorder.tick()
        recorder.analysis.shutdown(wait=True)
        assert analyzed == [tmp_path.resolve()]
        motion = list(csv.DictReader((tmp_path / 'telemetry/motion.csv').open()))
        assert motion[0]['source_stamp_ns'] == ''  # Never fabricate a source timestamp.
        assert float(motion[0]['linear_x']) == .2
        assert (tmp_path / 'raw/mgx23.bin').read_bytes() == b'data'
    finally:
        recorder.close()
