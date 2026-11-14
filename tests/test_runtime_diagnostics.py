"""UART observation and live diagnostic API; no hardware or motion authority."""
import importlib.util
import json
import logging
import struct
import sys
import types
from pathlib import Path
from types import SimpleNamespace
from time import monotonic,time_ns

import pytest
from agri_base.serial_observer import SerialCapture,SerialTap,FrameStream,decode_frame,install_tap
from agri_backend.diagnostics import RuntimeDiagnostics,clean

ROOT=Path(__file__).resolve().parents[1]


def frame(direction,function,payload=b''):
    body=bytes([len(payload)+3,function])+payload
    return (b'\xff\xfc' if direction=='TX' else b'\xff\xfb')+body+bytes([sum(body)&255])


def test_bytewise_stream_preserves_real_frames_and_decodes_car_type_before_velocity():
    raw=frame('TX',0x12,b'\x01'+struct.pack('<hhh',250,-125,700))
    stream=FrameStream('TX');got=[]
    for b in b'noise'+raw:got+=stream.feed(bytes([b]))
    assert got==[raw] and stream.discarded_bytes==5
    d=decode_frame('TX',got[0])
    assert d['checksum_valid'] and d['values']=={'car_type':1,'vx_m_s':.25,'vy_m_s':-.125,'wz_rad_s':.7}
    received=frame('RX',0x0a,struct.pack('<hhhB',100,-10,500,12))
    d=decode_frame('RX',received)
    assert d['values']=={'vx_m_s':.1,'vy_m_s':-.01,'wz_rad_s':.5,'voltage_v':12}


def test_bad_checksum_and_unknown_function_remain_visible_and_resync():
    raw=frame('RX',0x0a,b'\0'*7);bad=raw[:-1]+bytes([raw[-1]^1])
    good=frame('RX',0xe1,b'abc');frames=FrameStream('RX').feed(bad+good)
    assert frames==[bad,good]
    assert not decode_frame('RX',bad)['checksum_valid']
    assert decode_frame('RX',good)['function_name']=='未知功能'
    assert decode_frame('RX',frame('RX',0x0a,b''))['decode_error']


class FakeSerial:
    def __init__(self,*args,**kwargs):self.port='/test/serial';self.writes=[];self.input=bytearray();self.calls=0;self.limit=None
    def isOpen(self):return True
    def close(self):pass
    def write(self,data):
        b=bytes(data);n=min(len(b),self.limit) if self.limit is not None else len(b)
        self.writes.append(b[:n]);return n
    def read(self,size=1):
        self.calls+=1;result=bytes(self.input[:size]);del self.input[:size];return result


def test_tap_calls_the_same_port_once_observes_only_written_bytes_and_preserves_errors():
    serial=FakeSerial();capture=SerialCapture();tap=SerialTap(serial,capture)
    raw=frame('TX',0x12,b'\1'+struct.pack('<hhh',300,0,0))
    serial.limit=2;assert tap.write(raw)==2 and not capture.take()
    serial.limit=None;assert tap.write(raw[2:])==len(raw)-2
    assert capture.take()[0]['frame']==raw.hex(' ').upper()
    received=frame('RX',0xe1,b'abc');serial.input.extend(received)
    assert tap.read(len(received))==received and serial.calls==1
    assert capture.take()[0]['direction']=='RX'
    def fail(data):raise OSError('test UART failure')
    serial.write=fail
    with pytest.raises(OSError):tap.write(raw)
    assert capture.status()['error']=='串口写入失败'


def test_vendor_library_is_unchanged_and_motion_bytes_match_without_observer(monkeypatch):
    source=ROOT/'yahboomcar_ros2_ws/software/py_install_V3.3.1/Rosmaster_Lib/Rosmaster_Lib.py'
    before=source.read_bytes();fake_module=SimpleNamespace(Serial=FakeSerial)
    monkeypatch.setitem(sys.modules,'serial',fake_module)
    spec=importlib.util.spec_from_file_location('vendor_rosmaster_observation_test',source)
    library=importlib.util.module_from_spec(spec);spec.loader.exec_module(library)
    baseline=library.Rosmaster();baseline.set_car_motion(.25,-.1,.3)
    expected=baseline.ser.writes[-1]
    capture=SerialCapture();original=install_tap(library,capture)
    observed=library.Rosmaster();observed.set_car_motion(.25,-.1,.3)
    assert observed.ser._serial.writes[-1]==expected
    assert decode_frame('TX',bytes.fromhex(capture.take()[-1]['frame']))['values']['vx_m_s']==.25
    assert original is fake_module and fake_module.Serial is FakeSerial
    assert source.read_bytes()==before


def test_bounded_observation_reports_its_own_drops():
    c=SerialCapture(capacity=1);raw=frame('RX',0xe1,b'a')
    c.feed('RX',raw*3);s=c.status()
    assert s['rx_frames']==3 and s['observer_dropped']==2 and len(c.take())==1
    assert s['tx_age_seconds'] is None and s['rx_age_seconds']>=0


def batch(session,sequence=1):
    return json.dumps({'schema_version':1,'frames':[{'session':session,'sequence':sequence,
        'unix_ns':time_ns(),'direction':'RX','frame':frame('RX',0x0a,struct.pack('<hhhB',100,0,0,12)).hex()}]})


def test_buffers_cursors_gap_reset_and_no_ros_velocity_as_serial(tmp_path,monkeypatch):
    d=RuntimeDiagnostics(tmp_path,capacity=3)
    assert d.snapshot('serial')['entries']==[] and not d.snapshot('serial')['status']['observer_online']
    for i in range(5):d.log('INFO','test',str(i))
    page=d.snapshot('logs',after=1,limit=2)
    assert [r['id'] for r in page['entries']]==[4,5] and page['skipped']==2
    assert d.snapshot('logs',after=999,session='old')['reset']
    session='a'*32;d.serial_batch(batch(session));d.serial_batch(batch(session,3));d.serial_batch(batch(session,3))
    page=d.snapshot('serial');assert len(page['entries'])==2 and page['status']['sequence_gaps']==1
    assert not page['status']['rx_fresh'] and not page['status']['observer_online']
    d.serial_status(json.dumps({'source':'yahboom_rosmaster_uart','session':session,'unix_ns':time_ns(),'port':'/test',
        'tx_frames':0,'rx_frames':3,'observer_dropped':0,'discarded_bytes':0,'tx_age_seconds':None,'rx_age_seconds':0}))
    assert d.snapshot('serial')['status']['observer_online'] and d.snapshot('serial')['status']['rx_fresh']
    monkeypatch.setattr('agri_backend.diagnostics.monotonic',lambda:monotonic()+3)
    assert not d.snapshot('serial')['status']['observer_online']
    assert not d.snapshot('serial')['status']['rx_fresh']
    d.serial_batch('{bad json');assert d.snapshot('serial')['status']['malformed_batches']==1


def test_task_errors_and_rotated_session_recording_are_read_only_and_redact_secrets(tmp_path,monkeypatch):
    d=RuntimeDiagnostics(tmp_path);d.start()
    try:
        d.state('planning','{"state":"FAILED","reason":"边界净空不足"}')
        d.state('planning','{"state":"FAILED","reason":"边界净空不足"}')
        logger=logging.getLogger('diagnostic_unit')
        monkeypatch.setattr(logger,'propagate',True)  # launch.logging installs nonpropagating loggers in other tests.
        logger.warning('password="a b c" authorization: Bearer xyz token=abc')
        session='c'*32;d.serial_batch(batch(session))
    finally:d.close()
    messages=d.snapshot('logs')['entries']
    assert sum('边界净空不足' in v['message'] for v in messages)==1
    text=(d.archive_dir/'logs.jsonl').read_text()
    assert '[REDACTED]' in text and 'Bearer xyz' not in text and 'token=abc' not in text and 'a b c' not in text
    assert json.loads((d.archive_dir/'serial.jsonl').read_text())['direction']=='RX'


def test_http_endpoints_only_read_diagnostics_and_capture_validation_errors(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main,diagnostics
    d=RuntimeDiagnostics(tmp_path);monkeypatch.setattr(diagnostics,'runtime_diagnostics',d)
    client=TestClient(main.app)
    try:
        d.log('ERROR','path_tracker','test tracking fault')
        result=client.get('/api/diagnostics/logs').json()
        assert result['entries'][-1]['message']=='test tracking fault'
        assert not client.get('/api/diagnostics/serial').json()['status']['observer_online']
        assert client.get('/api/diagnostics/serial?after=-1').status_code==422
        assert any('参数校验失败' in r['message'] for r in d.snapshot('logs')['entries'])
        assert client.get('/api/diagnostics/arbitrary').status_code==404
        assert client.post('/api/diagnostics/serial').status_code==405
        assert not list(tmp_path.iterdir())
    finally:client.close()


def test_managed_console_reads_only_live_project_logs_and_never_arbitrary_files(tmp_path):
    import os
    folder=tmp_path/'.runtime/project/runs/test';folder.mkdir(parents=True)
    (folder/'core.log').write_text('[ERROR] [driver] test UART open failure\n')
    (folder/'backend.log').write_text('GET /api/diagnostics/logs HTTP/1.1" 200\n')
    (folder/'frontend.log').write_text('Vite Error: test missing module\n')
    fields=Path('/proc/self/stat').read_text().split(') ',1)[1].split()
    state={'logs':str(folder),'services':{'backend':{'pid':os.getpid(),'start_ticks':fields[19],
        'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}}}
    state_path=tmp_path/'.runtime/project/state.json';state_path.write_text(json.dumps(state))
    d=RuntimeDiagnostics(tmp_path);d._console_logs();first=d.snapshot('logs')['entries']
    assert len(first)==2 and all(v['level']=='ERROR' for v in first)
    d._console_logs();assert len(d.snapshot('logs')['entries'])==2
    state['logs']=str(tmp_path);state_path.write_text(json.dumps(state));d._console_logs()
    assert len(d.snapshot('logs')['entries'])==2


def test_delayed_or_bad_rx_data_never_appears_live(tmp_path):
    d=RuntimeDiagnostics(tmp_path);session='f'*32
    d.serial_status(json.dumps({'source':'yahboom_rosmaster_uart','session':session,'unix_ns':time_ns(),
        'port':'/test','tx_frames':0,'rx_frames':1,'observer_dropped':0,'discarded_bytes':0,
        'tx_age_seconds':None,'rx_age_seconds':0}))
    packet=json.loads(batch(session));packet['frames'][0]['unix_ns']=time_ns()-5*10**9
    d.serial_batch(json.dumps(packet));assert not d.snapshot('serial')['status']['rx_fresh']
    raw=bytes.fromhex(packet['frames'][0]['frame']);packet['frames'][0].update(
        sequence=2,unix_ns=time_ns(),frame=(raw[:-1]+bytes([raw[-1]^1])).hex())
    d.serial_batch(json.dumps(packet));status=d.snapshot('serial')['status']
    assert status['bad_frames']==1 and not status['rx_fresh']


def test_archive_failure_does_not_log_into_its_own_console_or_break_live_buffer(tmp_path,monkeypatch):
    from agri_backend import diagnostics
    class BrokenStream:
        def write(self,value):raise OSError('test disk failure')
        def flush(self):pass
        def close(self):pass
    monkeypatch.setattr(diagnostics.DiagnosticArchive,'_open',lambda self:BrokenStream())
    d=RuntimeDiagnostics(tmp_path);d.start()
    d.log('ERROR','test','observable even when archive fails')
    end=monotonic()+2
    while not d.archive_error and monotonic()<end:
        import time
        time.sleep(.01)
    d.close()
    assert '归档写入失败' in d.archive_error
    assert any('observable even' in v['message'] for v in d.snapshot('logs')['entries'])
