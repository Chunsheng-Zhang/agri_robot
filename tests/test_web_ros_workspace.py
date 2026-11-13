"""Real ROS safety/mode services and planner behind FastAPI, synthetic hardware only."""
import io
import os
import time
import zipfile

import pytest

pytestmark = pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS') != '1', reason='Opt-in isolated Web/ROS integration')


def test_web_modes_missions_planning_recordings_and_deadman():
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from agri_interfaces.srv import AcquireControl
    from agri_backend.ros_workspace import wait_result
    from web_test_runtime import SyntheticSystem
    system = SyntheticSystem()
    client = TestClient(system.app)  # Explicit fixture owns the ROS executor, no app lifespan.

    def until(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.03)
        raise AssertionError(f'timeout: {client.get("/api/devices").json()}')

    def post(path, body=None):
        result = client.post(path, json=body) if body is not None else client.post(path)
        assert result.status_code == 200, result.text
        return result.json()

    try:
        until(lambda: client.get('/api/devices').json()['safety']['fresh'])
        assert client.get('/api/devices').json()['safety']['mode'] == 'base'
        from rclpy.parameter import Parameter
        assert system.tracker.set_parameters([Parameter('max_angular', value=1.0)])[0].successful
        assert not system.tracker.set_parameters([Parameter('max_angular', value=1.001)])[0].successful
        assert not system.tracker.set_parameters([Parameter('speed', value=1.001)])[0].successful
        frame = system.localization.frame
        lon, lat = frame.transformer.transform(frame.e0 + 1, frame.n0, direction=TransformDirection.INVERSE)
        waypoint = post('/api/missions', {'name': '实际接口多点测试', 'mode': 'waypoint',
            'waypoints': [{'latitude': lat, 'longitude': lon, 'dwell_time': 0}]})
        assert client.post(f'/api/missions/{waypoint["id"]}/start').status_code == 409
        post('/api/system/mode', {'mode': 'waypoint'})
        until(lambda: system.waypoint.mode_matches)
        assert client.post(f'/api/missions/{waypoint["id"]}/start').status_code == 409  # Estop stays locked.
        post('/api/robot/emergency-stop/reset')
        until(lambda: not client.get('/api/devices').json()['safety']['emergency_stop'])
        post(f'/api/missions/{waypoint["id"]}/start')
        until(lambda: system.waypoint.state == 'RUNNING' and system.state['v'] > 0)
        # Policy changes stop a real task, without resetting estop or resuming it.
        post('/api/robot/lidar-protection', {'enabled': True})
        until(lambda: system.waypoint.state == 'FAULT' and system.state['v'] == 0)
        assert not system.safety.gate.owner and system.safety.lidar_required
        post('/api/robot/lidar-protection', {'enabled': False})
        until(lambda: system.waypoint.mode_revision == system.safety.mode_revision)
        until(lambda: client.get('/api/devices').json()['waypoint'].get('state') == 'FAULT')
        assert system.waypoint.state == 'FAULT' and system.state['v'] == 0
        post(f'/api/missions/{waypoint["id"]}/start')
        until(lambda: system.waypoint.state == 'RUNNING' and system.state['v'] > 0)
        post('/api/tasks/waypoint/pause')
        until(lambda: system.state['v'] == 0)
        post('/api/tasks/waypoint/resume')
        until(lambda: system.state['v'] > 0)
        old_revision = system.safety.mode_revision
        post('/api/system/mode', {'mode': 'base'})
        until(lambda: system.waypoint.state == 'CANCELED' and system.state['v'] == 0)
        # A queued request from the previous mode cannot acquire control.
        response = wait_result(system.backend.workspace_ros.clients['/safety/acquire_control'].call_async(
            AcquireControl.Request(source='waypoint', enable=True, mode_revision=old_revision, task_generation=1)))
        assert not response.success
        post('/api/system/mode', {'mode': 'waypoint'})
        until(lambda: system.waypoint.mode_matches)
        assert not system.safety.gate.owner and system.state['v'] == 0
        archive_run = client.get('/api/workspace').json()['runs'][0]
        from pathlib import Path
        directory = system.workspace.store.run_path(archive_run['id'])
        until(lambda: not system.backend.workspace_ros.recorder.busy(directory))
        archive = client.get(f'/api/runs/{archive_run["id"]}/download')
        assert archive.status_code == 200, archive.text
        with zipfile.ZipFile(io.BytesIO(archive.content)) as data:
            assert any(name.startswith('tracks/waypoint_') and name.endswith('.csv') for name in data.namelist())
            assert any(name.startswith('tracks/waypoint_') and name.endswith('.json') for name in data.namelist())
            assert 'raw/mgx23.bin' in data.namelist()
            assert b'synthetic-mgx23-bytes' in data.read('raw/mgx23.bin')
            assert 'telemetry/motion.csv' in data.namelist()
            assert 'analysis/heading_error.png' in data.namelist()
            assert 'analysis/trajectory_and_speed.png' in data.namelist()

        post('/api/system/mode', {'mode': 'coverage'})
        until(system.planner.mode_ready)
        import yaml
        from pathlib import Path
        boundary = yaml.safe_load((Path(__file__).resolve().parents[1] /
            'agri_robot_ws/src/agri_bringup/config/field.yaml').read_text())
        coverage = post('/api/missions', {'name': '实际 OpenNav 测试', 'mode': 'coverage', 'boundary': boundary,
            'parameters': {'min_turning_radius': 0.65}})
        until(lambda: system.planner.client.server_is_ready() and system.backend.workspace_ros.clients[
            '/coverage_lifecycle_manager/manage_nodes'].service_is_ready())
        post(f'/api/missions/{coverage["id"]}/plan')
        until(lambda: (client.get('/api/workspace').json().get('active_run') or {}).get('state') == 'READY', 30)
        until(lambda: ((client.get('/api/workspace').json().get('active_run') or {}).get('coverage_quality') or {}).get('state') == 'SUCCEEDED',30)
        quality_run = client.get('/api/workspace').json()['active_run']
        quality = quality_run['coverage_quality']['planned']['metrics']['field']
        assert 0 <= quality['overlap_percent'] <= quality['coverage_percent'] <= 100
        assert abs(quality['coverage_percent'] + quality['miss_percent'] - 100) < 1e-6
        assert client.get(f'/api/runs/{quality_run["id"]}/coverage-quality/plan.png').status_code == 200
        map_url = f'/api/runs/{quality_run["id"]}/coverage-map'
        until(lambda: client.get(map_url).json()['state'] == 'READY', 20)
        layers = client.get(map_url).json()
        assert layers['reference_fingerprint'] == quality_run['reference_fingerprint']
        assert layers['projection'] == quality_run['projection']
        assert layers['sources']['actual']['status'] == 'NOT_STARTED'
        assert layers['inset_polygons']
        for layer in ('completed', 'remaining', 'overlap'):
            assert client.get(f'{map_url}/planned/{layer}.png?v={layers["sources"]["planned"]["version"]}').status_code == 200
        assert system.backend.workspace_ros.get_parameters('coverage_server', ['min_turning_radius'])['min_turning_radius'] == 0.65
        assert not system.safety.gate.owner  # Planning never starts motion.
        # Use the actual current synthetic pose, without teleporting to the path.
        until(lambda: system.tracker.pending_path is not None)
        import math
        start = system.tracker.pending_path.poses[0].pose.position
        assert math.hypot(system.state['x'] - start.x, system.state['y'] - start.y) > .8
        post(f'/api/missions/{coverage["id"]}/start')
        until(lambda: system.tracker.state == 'RUNNING' and system.state['v'] > 0)
        assert system.tracker.controller.phase == 'APPROACHING'
        status = client.get('/api/devices').json()['tracking']
        assert status['phase'] == 'APPROACHING' and status['approach_distance'] > .8
        assert status['progress'] == 0 and system.safety.system_mode == 'coverage'
        post('/api/tasks/tracking/pause')
        until(lambda: system.state['v'] == system.state['w'] == 0)
        assert system.safety.gate.owner == 'tracking'
        post('/api/tasks/tracking/resume')
        assert system.tracker.controller.phase == 'ALIGNING'
        assert system.state['v'] == 0
        until(lambda: system.state['v'] > 0)
        post('/api/tasks/tracking/cancel')
        until(lambda: system.tracker.state == 'CANCELED' and not system.safety.gate.owner)
        coverage_run = client.get('/api/workspace').json()['active_run']
        directory = system.workspace.store.run_path(coverage_run['id'])
        until(lambda: not system.backend.workspace_ros.recorder.busy(directory))
        archive = client.get(f'/api/runs/{coverage_run["id"]}/download')
        assert archive.status_code == 200
        with zipfile.ZipFile(io.BytesIO(archive.content)) as data:
            assert any(name.startswith('tracks/approach_') and name.endswith('.csv') for name in data.namelist())
            import json
            recording = json.loads(data.read('telemetry/recording.json'))
            assert recording['terminal']['state'] == 'CANCELED'
            assert recording['samples_by_topic']['/cmd_vel'] > 0
            assert recording['samples_by_topic']['/base/driver_cmd_vel'] > 0
            assert recording['samples_by_topic']['/vel_raw'] > 0
            assert recording['samples_by_topic']['/mgx23/velocity_stamped'] > 0
            assert json.loads(data.read('analysis/status.json'))['state'] == 'SUCCEEDED'
            assert data.read('analysis/report.md').decode().startswith('# 本轮实验结果汇总')
            communication = json.loads(data.read('analysis/communication.json'))
            assert communication['available']
            assert communication['topics']['/mgx23/velocity_stamped']['received'] > 0
            assert data.read('communication/ros_timing.csv')
            assert data.read('communication/http_server.csv')
            assert len(data.read('analysis/communication_loss.png')) > 10000
            assert len(data.read('analysis/communication_latency.png')) > 10000
            assert '全覆盖作业几何质量评价报告' in data.read('analysis/coverage_quality_report.md').decode()
            assert 'analysis/coverage_quality.json' in data.namelist()
            assert 'analysis/coverage_quality_plan_cells.csv' in data.namelist()
            assert 'analysis/coverage_quality_plan.svg' in data.namelist()
            assert not any(name.endswith('.bib') for name in data.namelist())

        # This fixture is explicitly isolated (Domain 200..230, localhost only).
        # Place its synthetic vehicle at the reference start with a 90deg error
        # to exercise real start/pause/resume services during alignment.
        post(f'/api/missions/{coverage["id"]}/plan')
        until(lambda: (client.get('/api/workspace').json().get('active_run') or {}).get('state') == 'READY', 30)
        until(lambda: system.tracker.pending_path is not None)
        from agri_base.runtime import yaw_of
        first = system.tracker.pending_path.poses[0].pose
        system.state.update(x=first.position.x, y=first.position.y,
                            yaw=yaw_of(first.orientation) + math.pi/2)
        until(lambda: math.hypot(system.tracker.odom.pose.pose.position.x-first.position.x,
                                system.tracker.odom.pose.pose.position.y-first.position.y) < .05)
        post(f'/api/missions/{coverage["id"]}/start')
        until(lambda: system.tracker.state == 'RUNNING' and system.tracker.controller.phase == 'ALIGNING')
        until(lambda: abs(system.state['w']) > .01)
        assert system.state['v'] == 0 and abs(system.state['w']) <= .3
        assert system.tracker.controller.progress == 0
        post('/api/tasks/tracking/pause')
        until(lambda: system.state['v'] == system.state['w'] == 0)
        assert system.safety.gate.owner == 'tracking'
        time.sleep(.6)
        assert system.tracker.state == 'PAUSED' and system.state['v'] == 0
        post('/api/tasks/tracking/resume')
        assert system.tracker.controller.phase == 'ALIGNING'
        assert system.state['v'] == 0
        until(lambda: system.tracker.controller.phase == 'TRACKING' and system.state['v'] > 0, 20)
        assert system.tracker.controller.progress < 1
        # Record enough formal samples to exercise a live flushed segment,
        # rather than cancelling after only the first tracking pose.
        first_tracking_count = system.tracker.record_count
        until(lambda: system.tracker.record_count >= first_tracking_count + 25)
        live_run = client.get('/api/workspace').json()['active_run']
        map_url = f'/api/runs/{live_run["id"]}/coverage-map'
        until(lambda: client.get(map_url).json().get('sources', {}).get('actual', {}).get('status') == 'OBSERVED', 20)
        post('/api/tasks/tracking/cancel')
        until(lambda: system.tracker.state == 'CANCELED' and not system.safety.gate.owner)
        alignment_run = client.get('/api/workspace').json()['active_run']
        alignment_directory = system.workspace.store.run_path(alignment_run['id'])
        until(lambda: not system.backend.workspace_ros.recorder.busy(alignment_directory))
        import csv
        alignment_rows = list(csv.DictReader(next((alignment_directory/'tracks').glob('alignment_*.csv')).open()))
        assert alignment_rows and all(float(r['progress']) == 0 for r in alignment_rows)
        alignment_metrics = json.loads((alignment_directory/'analysis/metrics.json').read_text())
        assert alignment_metrics['phases']['alignment']['samples'] == len(alignment_rows)
        assert alignment_metrics['phases']['tracking']['samples'] > 0
        map_url = f'/api/runs/{alignment_run["id"]}/coverage-map'
        until(lambda: client.get(map_url).json().get('sources', {}).get('actual', {}).get('status') == 'FINAL', 20)
        final_layers = client.get(map_url).json()
        assert final_layers['sources']['actual']['samples'] > 0
        assert final_layers['sources']['actual']['track_lines']
        post('/api/system/mode', {'mode': 'base'})
        until(lambda: client.get('/api/devices').json()['safety']['mode'] == 'base')
        session = post('/api/control/arm')
        until(lambda: client.get('/api/devices').json()['safety']['owner'] == 'web')
        post('/api/control/command', {'token': session['token'], 'seq': 1, 'linear': 0.1,
            'angular': 0.0, 'expires_at': time.time() + 0.2})
        until(lambda: system.state['v'] > 0, 1)
        until(lambda: system.state['v'] == 0 and not system.safety.gate.owner, 2)
        assert client.post('/api/control/command', json={'token': session['token'], 'seq': 2, 'linear': 0.1,
            'angular': 0.0, 'expires_at': time.time() + 0.2}).status_code == 409

        # New ordered channel uses the same real ROS lease/safety chain.
        # All vehicle motion here is synthetic in an isolated localhost domain.
        with client.websocket_connect('/api/control/channel') as channel:
            session = post('/api/control/arm')
            assert session['heartbeat_protocol'] == 4
            until(lambda: client.get('/api/devices').json()['safety']['owner'] == 'web')
            for sequence in range(1, 9):
                channel.send_json({'token': session['token'], 'seq': sequence,
                    'linear': 0.0 if sequence == 1 else .15, 'angular': 0.0,
                    'expires_at': time.time() + .2})
                reply = channel.receive_json(); assert reply['success'] and reply['seq'] == sequence and reply['applied']
                time.sleep(.08)
            until(lambda: system.state['v'] > 0, 1)
            # A single stale frame cannot interrupt a still-valid command.
            channel.send_json({'token': session['token'], 'seq': 9, 'linear': -.15,
                'angular': 0.0, 'expires_at': time.time()-.0058})
            reply = channel.receive_json()
            assert not reply['applied'] and reply['reason'] == 'expired'
            assert system.state['v'] > 0
            channel.send_json({'token': session['token'], 'seq': 10, 'linear': .15,
                'angular': 0.0, 'expires_at': time.time()+.2})
            assert channel.receive_json()['applied']
            # A real >200ms gap still zeros output; only a new fresh held-input
            # frame can resume within the active session, without another arm.
            until(lambda: system.state['v'] == 0, .4)
            channel.send_json({'token': session['token'], 'seq': 11, 'linear': .15,
                'angular': 0.0, 'expires_at': time.time()+.2})
            assert channel.receive_json()['applied']
            until(lambda: system.state['v'] > 0, .3)
        until(lambda: not system.safety.gate.owner and system.state['v'] == 0, 2)

        # A completed run must finalize capture/plots without any browser polling
        # /api/workspace to drive the state transition.
        post('/api/system/mode', {'mode': 'waypoint'})
        until(lambda: system.waypoint.mode_matches)
        lon, lat = frame.transformer.transform(frame.e0 + system.state['x'] + .6,
            frame.n0 + system.state['y'], direction=TransformDirection.INVERSE)
        final_mission = post('/api/missions', {'name': '自动完成与独立归档验证', 'mode': 'waypoint',
            'waypoints': [{'latitude': lat, 'longitude': lon, 'dwell_time': 0}]})
        final_start = post(f'/api/missions/{final_mission["id"]}/start')
        final_directory = system.workspace.store.run_path(final_start['run_id'])
        until(lambda: system.waypoint.state == 'COMPLETED' and system.state['v'] == 0, 30)
        until(lambda: not system.backend.workspace_ros.recorder.busy(final_directory), 15)
        recording = json.loads((final_directory / 'telemetry/recording.json').read_text())
        assert recording['terminal']['state'] == 'COMPLETED'
        metrics = json.loads((final_directory / 'analysis/metrics.json').read_text())
        assert metrics['task_state'] == 'COMPLETED'
        assert (final_directory / 'analysis/trajectory_and_speed.png').is_file()
    finally:
        client.close()
        system.close()
