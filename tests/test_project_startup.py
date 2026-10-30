"""Exercise process ownership, rollback and stop without ROS/hardware/network."""
import importlib.util
import io
import os
from pathlib import Path
import signal
import sys
from time import monotonic, sleep

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('project_startup', ROOT / 'scripts/project.py')
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


class Reply(io.BytesIO):
    status = 200


class Http:
    def __init__(self):
        self.stops = 0

    def open(self, request, timeout=0):
        url = request if isinstance(request, str) else request.full_url
        if not isinstance(request, str) and request.method == 'POST':
            assert url.endswith('/robot/emergency-stop')
            self.stops += 1
            return Reply(b'{"success":true}')
        if url.endswith('/robot/status'):
            return Reply(b'{"ros_connected":true}')
        if url.endswith('/devices'):
            return Reply(b'{"safety":{"fresh":true,"emergency_stop":true,"owner":""}}')
        return Reply(b'<html>test only</html>')


@pytest.fixture
def project(tmp_path, monkeypatch):
    manager = startup.Project(tmp_path)
    manager.http = Http()
    monkeypatch.setattr(manager, 'preflight', lambda: None)
    monkeypatch.setattr(manager, 'show_urls', lambda: None)
    monkeypatch.setattr(manager, 'existing', lambda: (False, [], []))
    commands = {name: ([sys.executable, '-B', '-u', '-c', 'import time; print("running"); time.sleep(3600)'], tmp_path)
                for name in ('core', 'backend', 'frontend')}
    monkeypatch.setattr(manager, 'commands', lambda: commands)
    try:
        yield manager, commands
    finally:
        manager.shutdown(manager.load())


def test_three_services_detach_and_start_is_idempotent(project):
    manager, _ = project
    manager.start(timeout=2)
    before = manager.load()
    assert len(before['services']) == 3
    for entry in before['services'].values():
        assert manager.owns(entry)
        assert os.getpgid(entry['pid']) == entry['pid']
        assert manager.members(entry)
    manager.start(timeout=2)
    assert manager.load() == before
    assert manager.http.stops == 0  # Starting an existing project must not alter control.
    manager.stop()
    assert manager.http.stops == 1
    assert not any(manager.active(e) for e in before['services'].values())
    assert manager.load()['services'] == {}


def test_start_failure_rolls_back_only_created_services(project, monkeypatch):
    manager, commands = project
    commands['backend'] = ([sys.executable, '-B', '-c', 'raise SystemExit(7)'], manager.root)
    monkeypatch.setattr(manager, 'ready', lambda: False)
    with pytest.raises(RuntimeError):
        manager.start(timeout=1)
    assert manager.load()['services'] == {}
    assert (Path(manager.load()['logs']) / 'backend.log').exists()


def test_partial_existing_services_are_not_duplicated(project, monkeypatch):
    manager, _ = project
    monkeypatch.setattr(manager, 'existing', lambda: (False, [], [8000]))
    with pytest.raises(RuntimeError, match='占用'):
        manager.start()
    with pytest.raises(RuntimeError, match='不是本脚本启动'):
        manager.stop()
    assert not manager.state_file.exists()


def test_external_running_project_is_not_adopted_or_interrupted(project, monkeypatch):
    manager, _ = project
    monkeypatch.setattr(manager, 'existing', lambda: (True, ['joy_node'], [8000, 5173]))
    manager.start()
    assert not manager.state_file.exists()
    assert manager.http.stops == 0


def test_status_distinguishes_external_services_from_missing_frontend(project, monkeypatch, capsys):
    manager, _ = project
    monkeypatch.setattr(manager, 'existing', lambda: (True, ['joy_node'], [8000, 8080]))
    monkeypatch.setattr(manager, 'ready', lambda: False)
    assert not manager.status()
    output = capsys.readouterr().out
    assert 'core: 运行中（其他方式启动）' in output
    assert 'backend: 端口 8000 已占用（非本脚本管理）' in output
    assert 'frontend: 未运行' in output
    assert manager.http.stops == 0


def test_stale_pid_record_cannot_target_a_different_boot(project):
    manager, _ = project
    info = startup.process_info(os.getpid())
    entry = {**info, 'boot_id': 'previous-boot', 'token': 'old'}
    assert not manager.owns(entry) and not manager.active(entry)
    manager.terminate(entry)  # Must never signal this pytest process.


def test_orphaned_child_is_still_stopped_by_service_ownership(project):
    manager, commands = project
    program = ('import subprocess,sys,time; '
               'p=subprocess.Popen([sys.executable,"-B","-c","import time; time.sleep(3600)"]); '
               'print(p.pid,flush=True); time.sleep(3600)')
    commands['core'] = ([sys.executable, '-B', '-u', '-c', program], manager.root)
    manager.start(timeout=2)
    state = manager.load()
    log = Path(state['logs']) / 'core.log'
    deadline = monotonic() + 3
    while not log.read_text().strip() and monotonic() < deadline:
        sleep(.02)
    child_pid = int(log.read_text().strip())
    core = state['services']['core']
    os.kill(core['pid'], signal.SIGKILL)
    deadline = monotonic() + 2
    while manager.owns(core) and monotonic() < deadline:
        sleep(.02)
    assert manager.active(core) and any(p['pid'] == child_pid for p in manager.members(core))
    manager.stop()
    assert not startup.process_info(child_pid)


def test_default_profile_keeps_validated_limits_and_shared_physical_origin():
    profile = yaml.safe_load((ROOT / 'config/jetson_system.yaml').read_text())
    shared = yaml.safe_load((ROOT / 'agri_robot_ws/src/agri_bringup/config/system.yaml').read_text())
    assert profile['/**'] == shared['/**']
    assert profile['joy_adapter'] == shared['joy_adapter']
    assert profile['safety'] == shared['safety']
    assert profile['waypoint']['ros__parameters']['max_linear'] == .5
    assert profile['path_tracker']['ros__parameters']['speed'] == .1
    assert profile['mgx23_node']['ros__parameters']['raw_log_directory'] == '.runtime/mgx23_raw'


def test_start_uses_one_config_for_both_core_and_backend(project):
    manager, _ = project
    commands = startup.Project.commands(manager)
    core = commands['core'][0]
    assert f'params_file:={manager.config}' in core
    assert 'require_lidar:=false' in core
    assert not any('/data/experiments/' in arg for arg in core)
    assert '--strictPort' in commands['frontend'][0]
