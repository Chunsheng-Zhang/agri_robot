"""Actual ROS diagnostic transport on localhost synthetic domain, no UART hardware."""
import os
import struct
import time
import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_WEB_ROS_TESTS')!='1',reason='Opt-in isolated synthetic ROS domain')


def test_ros_logs_and_observed_frames_reach_web_without_motion(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import diagnostics
    from agri_base.serial_observer import SerialCapture,attach_ros_observer
    from web_test_runtime import SyntheticSystem
    cache=diagnostics.RuntimeDiagnostics(tmp_path)
    monkeypatch.setattr(diagnostics,'runtime_diagnostics',cache);cache.start()
    system=SyntheticSystem();client=TestClient(system.app)
    def until(fn,timeout=10):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if fn():return
            time.sleep(.05)
        raise AssertionError('diagnostic feedback timeout')
    def get(kind):
        r=client.get('/api/diagnostics/'+kind);assert r.status_code==200,r.text
        return r.json()
    def packet(direction,function,payload):
        body=bytes([len(payload)+3,function])+payload
        return (b'\xff\xfc' if direction=='TX' else b'\xff\xfb')+body+bytes([sum(body)&255])
    try:
        time.sleep(.3)  # Synthetic /vel_raw exists, but it is not UART observation.
        assert get('serial')['entries']==[]
        assert not get('serial')['status']['observer_online']
        capture=SerialCapture();capture.port='synthetic-memory-no-hardware'
        observer=attach_ros_observer(system.bench,capture)
        def feed():
            capture.feed('TX',packet('TX',0x12,b'\1'+struct.pack('<hhh',250,0,100)))
            capture.feed('RX',packet('RX',0x0a,struct.pack('<hhhB',100,0,0,12)))
        producer=system.bench.create_timer(.1,feed)
        until(lambda:get('serial')['status']['rx_fresh'])
        entries=get('serial')['entries']
        assert {r['direction'] for r in entries}=={'TX','RX'}
        assert any(r['function']==0x12 and 'vx=+0.250' in r['summary'] for r in entries)
        assert all(r['checksum_valid'] for r in entries)
        system.bench.get_logger().error('isolated UART diagnostic test error')
        until(lambda:any('isolated UART diagnostic test error' in r['message'] for r in get('logs')['entries']))
        producer.cancel()
        until(lambda:not get('serial')['status']['rx_fresh'])
        assert get('serial')['status']['observer_online']
        observer.cancel();until(lambda:not get('serial')['status']['observer_online'])
        assert get('serial')['entries']  # Historical rows remain available, without a live indicator.
        assert system.state['v']==system.state['w']==0 and not system.safety.gate.owner
        assert system.backend.count_publishers('/cmd_vel')==1
    finally:
        client.close();system.close();cache.close()
