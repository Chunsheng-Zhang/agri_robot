"""Real gateway expiry/route checks with fake ROS ports; no DDS or hardware."""
import math
from threading import Lock
from types import SimpleNamespace

import pytest
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient

from agri_backend import ros_workspace, workspace
from agri_backend.devices import DeviceCache


@pytest.fixture
def bench(monkeypatch, tmp_path):
    clock = [1000.0]
    cache = DeviceCache()
    def status(**extra):
        cache.update('safety', dict(mode='base', owner='', emergency_stop=False,
            mode_revision=1, motion_inhibited=False, limits={'linear': 1, 'angular': 2}, **extra))
    status()
    monkeypatch.setattr(ros_workspace, 'device_cache', cache)
    monkeypatch.setattr(ros_workspace, 'time', lambda: clock[0])
    monkeypatch.setattr(ros_workspace, 'monotonic', lambda: clock[0])
    gateway = object.__new__(ros_workspace.RosWorkspace)
    gateway.lock = Lock()
    gateway.token, gateway.seq = None, -1
    gateway.last_command = gateway.expiry = -math.inf
    gateway.v = gateway.w = 0.0
    gateway.lease_revision = gateway.lease_generation = 0
    published, releases = [], []
    gateway.publish = lambda v=0.0, w=0.0: published.append((v, w))
    gateway.types = {'lease': SimpleNamespace(Request=lambda **kw: SimpleNamespace(**kw))}
    def release(request):
        releases.append(request)
        value = cache.snapshot()['safety']; value['owner'] = ''; cache.update('safety', value)
    gateway.clients = {'/safety/acquire_control': SimpleNamespace(service_is_ready=lambda: True, call_async=release)}
    def acquire(_name, _request):
        value = cache.snapshot()['safety']; value['owner'] = 'web'; cache.update('safety', value)
        return SimpleNamespace(success=True)
    gateway.call = acquire
    app = FastAPI()
    workspace.install_routes(app, lambda: SimpleNamespace(node=SimpleNamespace(workspace_ros=gateway)), lambda: {})
    app.state.workspace.store = workspace.MissionStore(tmp_path)
    client = TestClient(app)  # No app lifespan or ROS node.
    return SimpleNamespace(gateway=gateway, cache=cache, clock=clock, published=published,
        releases=releases, client=client)


def frame(b, token, seq=1, linear=0.0, **extra):
    return dict(token=token, seq=seq, linear=linear, angular=0.0,
        expires_at=b.clock[0]+0.2, **extra)


def test_websocket_zero_probe_ordered_motion_and_disconnect_revoke(bench):
    b = bench
    with b.client.websocket_connect('/api/control/channel') as channel:
        session = b.client.post('/api/control/arm').json()
        assert session['heartbeat_protocol'] == 4
        channel.send_json(frame(b, session['token']))
        reply = channel.receive_json(); assert reply['success'] and reply['seq'] == 1 and reply['applied']
        assert b.gateway.v == 0
        channel.send_json(frame(b, session['token'], seq=2, linear=.15))
        assert channel.receive_json()['seq'] == 2
        b.gateway.tick(); assert b.published[-1] == (.15, 0.0)
    assert b.gateway.token is None
    assert b.published[-1] == (0.0, 0.0)
    assert b.releases


def test_acknowledgement_does_not_extend_200ms_command_expiry(bench):
    b = bench; session = b.gateway.arm()
    b.gateway.command(**frame(b, session['token'], linear=.15))
    b.clock[0] += .201
    b.gateway.tick()
    assert b.published[-1] == (0.0, 0.0)
    b.clock[0] += .301
    b.gateway.tick()
    assert b.gateway.token is None


@pytest.mark.parametrize('kind', ['future_timestamp', 'duplicate', 'wrong_token', 'nonzero_first', 'invalid', 'estop'])
def test_invalid_frames_close_without_continued_motion(bench, kind):
    b = bench; session = b.gateway.arm(); token = session['token']
    with b.client.websocket_connect('/api/control/channel') as channel:
        if kind != 'nonzero_first':
            channel.send_json(frame(b, token)); assert channel.receive_json()['success']
        value = frame(b, token, seq=2, linear=.1)
        if kind == 'future_timestamp': value['expires_at'] = b.clock[0] + 1
        if kind == 'duplicate': value['seq'] = 1
        if kind == 'wrong_token': value['token'] = 'f' * 32
        if kind == 'nonzero_first': value['seq'] = 1
        if kind == 'invalid': value['linear'] = float('inf')
        if kind == 'estop':
            safety = b.cache.snapshot()['safety']; safety['owner'] = ''; safety['emergency_stop'] = True
            b.cache.update('safety', safety)
        channel.send_json(value)
        assert channel.receive_json()['success'] is False
    assert b.published[-1] == (0.0, 0.0)
    if kind != 'nonzero_first': assert b.gateway.token is None


def test_an_old_socket_closing_does_not_revoke_a_new_session(bench):
    b = bench; first = b.gateway.arm()['token']
    with b.client.websocket_connect('/api/control/channel') as channel:
        channel.send_json(frame(b, first)); assert channel.receive_json()['success']
        b.gateway.stop(first)
        second = b.gateway.arm()['token']
    assert b.gateway.token == second
    b.gateway.stop(second)


def test_socket_without_token_cannot_revoke_an_existing_session(bench):
    b = bench; session = b.gateway.arm()
    with b.client.websocket_connect('/api/control/channel'):
        pass
    assert b.gateway.token == session['token']


def test_no_heartbeat_closes_and_releases_without_client_polling(bench):
    b = bench; token = b.gateway.arm()['token']
    with b.client.websocket_connect('/api/control/channel') as channel:
        channel.send_json(frame(b, token)); assert channel.receive_json()['success']
        assert channel.receive_json()['success'] is False
    assert b.gateway.token is None


def test_stale_frame_does_not_interrupt_a_live_command_or_require_repress(bench):
    b = bench; token = b.gateway.arm()['token']
    with b.client.websocket_connect('/api/control/channel') as channel:
        channel.send_json(frame(b, token)); assert channel.receive_json()['applied']
        channel.send_json(frame(b, token, seq=2, linear=.15)); assert channel.receive_json()['applied']
        b.gateway.tick(); assert b.published[-1] == (.15, 0.0)
        b.clock[0] += .08
        stale = frame(b, token, seq=3, linear=-.15); stale['expires_at'] = b.clock[0]-.0058
        old_expiry, old_fresh = b.gateway.expiry, b.gateway.last_command
        channel.send_json(stale); reply = channel.receive_json()
        assert reply['success'] and not reply['applied'] and reply['reason'] == 'expired'
        assert reply['overdue_ms'] == 5.8 and 'neutral_required' not in reply
        assert b.gateway.expiry == old_expiry and b.gateway.last_command == old_fresh
        assert b.gateway.token == token
        b.gateway.tick(); assert b.published[-1] == (.15, 0.0)
        b.clock[0] += .08
        channel.send_json(frame(b, token, seq=4, linear=.15)); assert channel.receive_json()['applied']
        b.gateway.tick(); assert b.published[-1] == (.15, 0.0)
        # A fresh release takes effect; stale motion cannot undo it.
        channel.send_json(frame(b, token, seq=5)); assert channel.receive_json()['applied']
        stale.update(seq=6)
        channel.send_json(stale); assert not channel.receive_json()['applied']
        b.gateway.tick(); assert b.published[-1] == (0.0, 0.0)


def test_expired_frames_cannot_extend_motion_or_keep_a_session_alive(bench):
    b = bench; token = b.gateway.arm()['token']
    with b.client.websocket_connect('/api/control/channel') as channel:
        channel.send_json(frame(b, token)); assert channel.receive_json()['applied']
        channel.send_json(frame(b, token, seq=2, linear=.15)); assert channel.receive_json()['applied']
        b.clock[0] += .201
        stale = frame(b, token, seq=3, linear=.15); stale['expires_at'] = b.clock[0]-.01
        channel.send_json(stale); assert not channel.receive_json()['applied']
        b.gateway.tick(); assert b.published[-1] == (0.0, 0.0)
        b.clock[0] += .300; b.gateway.tick()
        assert b.gateway.token is None
        channel.send_json(frame(b, token, seq=4)); assert not channel.receive_json()['success']


def test_gap_zeros_output_but_a_new_fresh_held_input_can_resume_within_session(bench):
    b = bench; token = b.gateway.arm()['token']
    b.gateway.command(**frame(b, token, linear=.15))
    b.clock[0] += .201; b.gateway.tick()
    assert b.published[-1] == (0.0, 0.0)
    assert b.gateway.v == b.gateway.w == 0
    # Another tick cannot replay old motion. A new fresh command is necessary.
    b.gateway.tick(); assert b.published[-1] == (0.0, 0.0)
    reply = b.gateway.command(**frame(b, token, seq=2, linear=.15), tolerate_expired=True)
    assert reply['applied']
    b.gateway.tick(); assert b.published[-1] == (.15, 0.0)


def test_continuous_half_second_gap_cannot_resume_even_before_next_tick(bench):
    b = bench; token = b.gateway.arm()['token']
    b.gateway.command(**frame(b, token, linear=.15))
    b.clock[0] += .501
    with pytest.raises(HTTPException, match='') as exc:
        b.gateway.command(**frame(b, token, seq=2, linear=.15), tolerate_expired=True)
    assert exc.value.status_code == 409 and '持续中断' in exc.value.detail
    b.gateway.tick(); assert b.gateway.token is None and b.published[-1] == (0.0, 0.0)


def test_initial_probe_expiry_remains_a_hard_failure(bench):
    b = bench; token = b.gateway.arm()['token']
    with b.client.websocket_connect('/api/control/channel') as channel:
        value = frame(b, token); value['expires_at'] = b.clock[0]-.01
        channel.send_json(value); reply = channel.receive_json()
        assert not reply['success'] and '已过期' in reply['detail']


def test_real_vite_wsproto_channel_with_delayed_ack_and_camera(bench, monkeypatch):
    """Real loopback Vite/WS/Uvicorn path; the gateway's ROS ports remain fake."""
    import os
    import socket
    import subprocess
    import time
    from pathlib import Path
    from threading import Thread, Event
    import uvicorn
    from fastapi.responses import Response

    b = bench
    monkeypatch.setattr(ros_workspace, 'time', time.time)
    monkeypatch.setattr(ros_workspace, 'monotonic', time.monotonic)
    root = Path(__file__).resolve().parents[1]
    image = (root/'tests/fixtures/camera_preview.jpg').read_bytes()
    b.client.app.add_api_route('/snapshot', lambda: Response(image, media_type='image/jpeg'))
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(b.client.app, log_level='error', lifespan='off', ws='wsproto'))
    thread = Thread(target=lambda: server.run(sockets=[sock]), daemon=True)
    finished = Event()
    def updates():
        while not finished.wait(.03):
            value = b.cache.snapshot()['safety']; b.cache.update('safety', value)
            b.gateway.tick()
    ticking = Thread(target=updates, daemon=True)
    thread.start(); ticking.start()
    try:
        deadline = time.monotonic() + 5
        while not server.started and time.monotonic() < deadline:
            time.sleep(.01)
        assert server.started
        result = subprocess.run(['node', str(root/'tests/manual_channel_wire.cjs'),
            f'http://127.0.0.1:{port}'], cwd=root, env=os.environ.copy(),
            capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stdout + result.stderr
        assert '"acknowledgement_delay_ms":250' in result.stdout
        assert (.15, 0.0) in b.published
        deadline = time.monotonic()+1
        while b.gateway.token is not None and time.monotonic() < deadline:
            time.sleep(.01)
        assert b.gateway.token is None and b.published[-1] == (0.0, 0.0)
    finally:
        finished.set(); ticking.join(1)
        server.should_exit = True; thread.join(3); sock.close()
