"""Deterministic plot inputs, no real task files, network or ROS nodes."""
import csv
import json

import pytest

from agri_backend.communication_plots import build_plot_data, phase_intervals, plot_communication
from agri_backend.missions import atomic_json


def csv_file(path, columns, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as file:
        writer=csv.writer(file);writer.writerow(columns);writer.writerows(rows)


def observations(path):
    atomic_json(path/'communication/metadata.json', {'latency_reference_ms':100,'http_reference_ms':2000,'notes':[]})
    columns=['received_ros_ns','received_unix_ns','received_monotonic_ns','topic','source_stamp_ns','source_age_ms']
    rows=[]
    for second in range(5):
        for topic in ('/mgx23/position','/mgx23/attitude_stamped','/mgx23/velocity_stamped','/odom'):
            if second==2 and topic=='/odom':continue
            stamp=(100+second)*10**9
            rows.append([stamp,999999999999999999,(1000+second)*10**9,topic,stamp,10])
    rows.append([103*10**9,999999999999999999,1003*10**9,'/scan',102*10**9,-1])
    csv_file(path/'communication/ros_timing.csv',columns,rows)
    csv_file(path/'communication/http_server.csv',
        ['received_unix_ns','received_monotonic_ns','method','path','status','duration_ms','outcome'],
        [[999999999999999999,1002*10**9,'GET','/api/devices',200,2500,'ok']])
    csv_file(path/'raw/chunks.csv',['session','sequence','received_ros_ns'],
        [['a',10,109900000000],['a',12,110100000000],['b',1,110200000000]])
    return {'queue_drops_by_topic':{'/odom':1},
            'queue_drop_events':[{'received_monotonic_ns':1002500000000,'topic':'/odom'}]}


def test_window_denominators_carry_sequence_gap_and_session_restart(tmp_path):
    capture=observations(tmp_path)
    result=build_plot_data(tmp_path,{'available':True},capture,100,'coverage','tracking')
    assert result['time_origin_ros_s']==100
    assert result['raw_missing'][0]['expected']==1 and result['raw_missing'][0]['missing']==0
    second=result['raw_missing'][1]
    assert second['start_s']==10 and second['expected']==3 and second['missing']==1
    assert second['percent']==pytest.approx(100/3)
    match=result['matched_missing']['/odom'][0]
    assert match['expected']==3 and match['missing']==1
    assert result['matched_missing']['/mgx23/attitude_stamped'][0]['missing']==0


def test_http_and_drop_timestamps_use_monotonic_ros_anchor_not_unix(tmp_path):
    capture=observations(tmp_path)
    result=build_plot_data(tmp_path,{'available':True},capture,100,'coverage','tracking')
    http=result['http_latency']['/api/devices'][0]
    assert http['start_s']==2
    assert http['over_threshold_percent']==100
    assert result['queue_drop_windows']==[{'start_s':0,'end_s':10,'count':1}]
    assert '/scan' not in result['ros_latency']  # Negative source age is not a timed sample.
    assert result['http_samples_unplaced']==0


def test_empty_windows_remain_missing_and_far_clock_anchor_is_rejected(tmp_path):
    capture=observations(tmp_path)
    csv_file(tmp_path/'raw/chunks.csv',['session','sequence','received_ros_ns'],
             [['a',10,100000000000],['a',11,130000000000]])
    csv_file(tmp_path/'communication/http_server.csv',
             ['received_unix_ns','received_monotonic_ns','method','path','status','duration_ms','outcome'],
             [[0,2000*10**9,'GET','/api/devices',200,10,'ok']])
    result=build_plot_data(tmp_path,{'available':True},capture,100,'coverage','tracking')
    assert [r['start_s'] for r in result['raw_missing']]==[0,30]
    assert result['http_latency']=={} and result['http_samples_unplaced']==1


def test_phase_shading_uses_matching_task_and_stops_across_status_outage(tmp_path):
    path=tmp_path/'telemetry/status.jsonl';path.parent.mkdir()
    values=[(100,'RUNNING','APPROACHING'),(100.1,'RUNNING','ALIGNING'),
            (100.2,'PAUSED','TRACKING'),(105,'RUNNING','TRACKING'),(105.1,'COMPLETED','TRACKING')]
    rows=[{'topic':'/tracking/status','received_ros_ns':round(t*1e9),
           'data':{'run_directory':str(tmp_path/'tracks'),'state':state,'phase':phase}} for t,state,phase in values]
    rows.append({'topic':'/tracking/status','received_ros_ns':106*10**9,
                 'data':{'run_directory':'/another/run/tracks','state':'RUNNING','phase':'TRACKING'}})
    path.write_text('\n'.join(json.dumps(row) for row in rows))
    result=phase_intervals(tmp_path,'coverage',100)
    assert [r['phase'] for r in result]==['APPROACHING','ALIGNING','PAUSED','TRACKING']
    assert result[2]['start_s']==pytest.approx(.2) and result[2]['end_s']==pytest.approx(1.2)
    assert result[-1]['end_s']==pytest.approx(5.1)


def test_missing_capture_plots_are_explicit_and_never_report_zero_queue_loss(tmp_path,monkeypatch):
    monkeypatch.setenv('MPLCONFIGDIR',str(tmp_path/'mpl'))
    import matplotlib
    matplotlib.use('Agg')
    (tmp_path/'analysis').mkdir()
    data=build_plot_data(tmp_path,{'available':False},{},None,'coverage',None)
    assert data['time_origin_ros_s'] is None and data['raw_missing']==[]
    assert data['ros_latency']=={} and data['queue_drop_total'] is None
    plot_communication(tmp_path/'analysis',data,{},'SYNTHETIC_EMPTY')
    for name in ('communication_loss.png','communication_latency.png'):
        assert (tmp_path/'analysis'/name).stat().st_size>10000
