"""Synthetic files/ASGI only: no sockets, ROS nodes or real experiment directories."""
import asyncio
import csv
import json
import struct
from types import SimpleNamespace

import pytest

from agri_backend.experiment_communication import CommunicationMiddleware, analyze_communication, distribution, raw_sequence
from agri_backend.experiment_recording import CaptureSession
from agri_backend.experiment_analysis import statistics
from agri_backend.experiment_summary import tracking_assessment
from test_experiment_recording import packet, finish


def test_capture_writes_timings_and_distinguishes_missing_and_future_source(tmp_path):
    session=CaptureSession(tmp_path,'tracking')
    session.put('motion','/cmd_vel',[10000000000,10,100,'','/cmd_vel','base_link',0,0,0])
    session.put('mgx','/mgx23/position',[10000000000,10,100,9990000000,'/mgx23/position','gnss',36,117,0,0])
    session.put('mgx','/mgx23/position',[10100000000,11,100000100,10110000000,'/mgx23/position','gnss',36,117,0,0])
    session.put('raw','/mgx23/raw',([10000000000,10,100],packet(b'abc',10)))
    session.put('raw','/mgx23/raw',([10100000000,11,100000100],packet(b'def',12)))
    session.put('http','/api/devices',[10,100,'GET','/api/devices',503,2500,'http_error'])
    finish(session)
    capture=json.loads((tmp_path/'telemetry/recording.json').read_text())
    result=analyze_communication(tmp_path,capture)
    assert result['raw_sequence_by_session']['device-1']['missing']==1
    assert result['raw_sequence_by_session']['device-1']['missing_percent']==pytest.approx(100/3)
    assert result['topics']['/cmd_vel']['unstamped_samples']==1
    assert result['topics']['/cmd_vel']['source_to_recorder_ms']['mean'] is None
    position=result['topics']['/mgx23/position']
    assert position['source_to_recorder_ms']['mean']==10
    assert position['negative_age_samples']==1
    assert result['http_server']['/api/devices']['server_duration_ms']['over_threshold_percent']==100
    assert result['http_server']['/api/devices']['failure_percent']==100
    assert not result['navi']['available']
    assert '未测' in result['unmeasured']['ip_packet_loss']
    assert (tmp_path/'communication/summary.csv').is_file()


def test_shared_headers_use_inner_window_and_report_real_missing_sample(tmp_path):
    session=CaptureSession(tmp_path,'tracking')
    for second in range(5):
        stamp=(second+100)*10**9
        session.put('mgx','/mgx23/position',[stamp,stamp,stamp,stamp,'/mgx23/position','gnss',36,117,0,0])
        if second!=2:
            session.put('odom','/odom',[stamp+1000,stamp,stamp+1000,stamp,'map',0,0,0,0])
    finish(session)
    result=analyze_communication(tmp_path,{})
    match=result['matched_source_stamps']['/odom']
    assert match['position_reference_samples']==3
    assert match['unmatched']==1
    assert match['unmatched_percent']==pytest.approx(100/3)


def navi(time, bad=False):
    frame=bytearray(100);frame[:4]=b'fmin';frame[98:]=b'ed'
    struct.pack_into('<d',frame,4,time)
    struct.pack_into('<f',frame,60,.01)
    struct.pack_into('<H',frame,96,(sum(frame[:96])+(1 if bad else 0))&0xffff)
    return bytes(frame)


def test_navi_parser_handles_split_frames_other_packets_and_utc_rollover(tmp_path):
    session=CaptureSession(tmp_path,'tracking')
    data=b'fmig-other-packet'+navi(235959.9)+navi(0)+navi(.2)+navi(.3,True)
    for sequence,chunk in enumerate((data[:30],data[30:190],data[190:]),1):
        session.put('raw','/mgx23/raw',([sequence*10**9]*3,packet(chunk,sequence)))
    finish(session)
    result=analyze_communication(tmp_path,{})['navi']
    assert result['frames']==4
    assert result['checksum_failures']==1
    assert result['estimated_missing_at_10hz']==1
    assert result['device_interval_ms']['max']==pytest.approx(200)
    assert result['invalid_or_backward_device_times']==0


def test_sequence_restarts_are_not_loss_and_old_capture_is_not_fabricated(tmp_path):
    result=raw_sequence([{'session':'a','sequence':40},{'session':'a','sequence':41},
                         {'session':'b','sequence':1}])
    assert result['a']['missing']==result['b']['missing']==0
    assert analyze_communication(tmp_path,{})['available'] is False
    assert not (tmp_path/'communication').exists()
    assert distribution([])['mean'] is None


def test_long_device_clock_gap_is_not_reported_as_zero_loss(tmp_path):
    session=CaptureSession(tmp_path,'tracking')
    session.put('raw','/mgx23/raw',([10**9]*3,packet(navi(0)+navi(30),1)))
    finish(session)
    result=analyze_communication(tmp_path,{})['navi']
    assert result['unclassified_long_intervals']==1
    assert result['estimated_missing_at_10hz'] is None


def test_http_observer_binds_starting_session_and_preserves_exception_and_privacy():
    old,new=[],[]
    active=[SimpleNamespace(put=lambda *args:old.append(args))]
    async def application(scope, receive, send):
        active[0]=SimpleNamespace(put=lambda *args:new.append(args))
        await send({'type':'http.response.start','status':200})
        await send({'type':'http.response.body','body':b'ok'})
    middleware=CommunicationMiddleware(application,lambda:active[0])
    async def send(message):pass
    asyncio.run(middleware({'type':'http','method':'GET','path':'/api/devices','query_string':b'token=SECRET'},None,send))
    assert len(old)==1 and not new
    assert old[0][2][-1]=='ok'
    assert 'SECRET' not in str(old)
    async def failed(scope,receive,send):raise RuntimeError('expected')
    middleware.app=failed
    with pytest.raises(RuntimeError,match='expected'):
        asyncio.run(middleware({'type':'http','method':'GET','path':'/api/devices'},None,send))
    assert new[-1][2][-1]=='incomplete'
    before=len(new)
    with pytest.raises(RuntimeError):
        asyncio.run(middleware({'type':'http','method':'POST','path':'/api/control/arm'},None,send))
    assert len(new)==before


def test_tracking_assessment_separates_startup_turns_and_measured_speed(tmp_path):
    (tmp_path/'analysis').mkdir()
    rows=[dict(stamp=100,x=2,y=.01,yaw=0,cross_track=.01,heading_error=1,progress=2),
          dict(stamp=106,x=5,y=.1,yaw=0,cross_track=.1,heading_error=.1,progress=5),
          dict(stamp=107,x=9.5,y=.2,yaw=0,cross_track=.2,heading_error=.2,progress=9.5)]
    motion=[{'topic':'/control/tracking/cmd_vel','source_stamp_ns':106*10**9,'linear_x':1,'linear_y':0,'angular_z':1}]
    mgx=[{'topic':'/mgx23/velocity_stamped','source_stamp_ns':106*10**9,'v1':.7,'v2':0}]
    result=tracking_assessment(tmp_path,{'parameters':{'speed':1,'max_angular':1}}, {'mode':'coverage'},
        [(0,0),(10,0),(10,10)], {'tracking':rows},motion,mgx,statistics)
    assert result['subsets']['all_tracking']['samples']==3
    assert result['subsets']['after_startup']['samples']==2
    assert result['subsets']['straight_core_after_startup']['samples']==1
    assert result['subsets']['turn_transition_after_startup']['samples']==1
    assert result['subsets']['all_tracking']['within_cross_track_percent']['0.1']==pytest.approx(200/3)
    assert result['mgx23_minus_command_speed_m_s']['mean']==pytest.approx(-.3)
    assert result['angular_command']['near_limit_percent']==100
    assert result['final_progress_percent']==47.5
    assert (tmp_path/'analysis/tracking_samples.csv').is_file()


@pytest.mark.parametrize('state',['FAULT','CANCELED','START_REJECTED'])
def test_summary_keeps_aborted_state_and_marks_absent_tracking_unmeasured(tmp_path,monkeypatch,state):
    from test_experiment_recording import prepared_run
    from agri_backend import experiment_analysis
    prepared_run(tmp_path)
    (tmp_path/'tracks/track_test.csv').write_text('stamp,x,y,yaw,cross_track,heading_error,progress\n')
    session=CaptureSession(tmp_path,'tracking')
    session.request_stop(state,'synthetic stop reason',tail=0)
    finish(session)
    monkeypatch.setattr(experiment_analysis,'plot_report',lambda *args:None)
    result=experiment_analysis.analyze_run(tmp_path)
    assert result['task_state']==state
    assert not result['tracking_assessment']['available']
    report=(tmp_path/'analysis/report.md').read_text()
    assert f'`{state}`' in report and 'synthetic stop reason' in report
    assert '没有正式覆盖跟踪样本' in report
