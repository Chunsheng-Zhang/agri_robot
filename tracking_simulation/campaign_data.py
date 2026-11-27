"""Read-only ground campaign inputs; no ROS, ports or service calls.

Use observed UART TX timestamps when available, rather than the arrival of
headerless driver Twist in another process. TX is not an MCU receipt ACK.
"""
import csv
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

import numpy as np

from ground_model import GroundRun, ROOT, csv_rows
sys.path.insert(0, str(ROOT/'agri_robot_ws/src/agri_base'))
from agri_base.coordinates import LocalFrame

SUPPLEMENT = ROOT/'data/experiments/20261127_094400_outdoor_supplement'
TRAIN_NAMES = ('20261127_094417_334349_web_ede7e84a', '20261127_094846_691259_web_3bd25a03')
HOLDOUT_NAMES = ('20261127_095200_244499_web_64ef7e41',
                 '20261127_083324_157825_web_1e6d040f', '20261127_091746_560120_web_d31cfad4')
BASIC = ROOT/'data/experiments/20261127_094249_ground_response'


def fingerprint(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@lru_cache(maxsize=2)
def serial_commands(filename, digest):
    """Cache frozen observer input only; reject modified or malformed input."""
    filename=Path(filename)
    if fingerprint(filename)!=digest:raise ValueError('串口观察文件已改变')
    records=[]
    with filename.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['topic']!='/base/serial_frames':continue
            for frame in event['data']['frames']:
                if frame['direction']!='TX':continue
                b=bytes.fromhex(frame['frame'])
                if len(b)<4 or b[3]!=0x12:continue
                if len(b)!=12 or b[:2]!=b'\xff\xfc' or b[2]+2!=len(b) or sum(b[2:-1])%256!=b[-1]:
                    raise ValueError('运动TX观察帧结构/校验错误')
                vx,vy,w=struct.unpack('<hhh',b[5:11])
                records.append((int(frame['unix_ns']),vx/1000,vy/1000,w/1000))
    records.sort(key=lambda r:r[0])
    return records


@dataclass
class CampaignRun(GroundRun):
    lateral: np.ndarray
    delivery_age: np.ndarray
    source_origin_ns: int
    tracking_window: tuple | None


def read_campaign_run(directory, *, supplement=SUPPLEMENT, allow_basic=False):
    directory=Path(directory).resolve()
    basic=(directory/'config/protocol.json').is_file()
    if basic:
        if not allow_basic:raise ValueError('响应试验需要显式allow_basic')
        recording=json.loads((directory/'telemetry/recording.json').read_text())
        if recording['terminal']['state']!='COMPLETED' or (directory/'config/intervention_annotation.json').exists():
            raise ValueError('拒绝故障或人工干预响应试验；不静默截取后冒充完整标定')
        # Frozen experiment YAML, never today's mutable runtime settings.
        import yaml
        cfg=directory/'config/system.yaml'
        system=yaml.safe_load(cfg.read_text())
        shared=system['/**']['ros__parameters']
        proj=dict(latitude=shared['origin_latitude'],longitude=shared['origin_longitude'])
        proj['epsg']=LocalFrame(proj['latitude'],proj['longitude']).epsg
    else:
        doc=json.loads((directory/'run.json').read_text())
        if doc['mode']!='coverage' or doc['state']!='COMPLETED':raise ValueError('只接受已完成的地面覆盖试验')
        cfg=directory/'config/system.json';system=json.loads(cfg.read_text());proj=doc['projection']
    loc=system['localization']['ros__parameters']
    frame=LocalFrame(proj['latitude'],proj['longitude'])
    if frame.epsg!=proj['epsg']:raise ValueError('固定原点/投影不一致')
    files=[cfg,directory/'telemetry/motion.csv',directory/'telemetry/mgx23.csv',directory/'telemetry/odometry.csv']
    hashes={str(p.relative_to(ROOT)):fingerprint(p) for p in files}
    for p in (directory/'run.json',directory/'config/mission.json',directory/'telemetry/recording.json',
              directory/'config/operator_annotation.json',directory/'config/protocol.json'):
        if p.exists():hashes[str(p.relative_to(ROOT))]=fingerprint(p)
    if not basic:
        timing_inputs=[directory/'config/tracking_controller.json',*sorted((directory/'paths').glob('coverage_*.json'))]
        for p in timing_inputs:hashes[str(p.relative_to(ROOT))]=fingerprint(p)
    mgx=csv_rows(files[2]);odom=csv_rows(files[3])
    fixes={r['source_stamp_ns']:r for r in mgx if r['topic']=='/mgx23/position'}
    poses={r['source_stamp_ns']:r for r in odom}
    records=[]
    for r in mgx:
        stamp=r['source_stamp_ns']
        if r['topic']!='/mgx23/velocity_stamped' or stamp not in poses or stamp not in fixes:continue
        pose,fix=poses[stamp],fixes[stamp]
        north=frame.yaw(float(fix['v1']),float(fix['v2']),0.)
        vn,ve=float(r['v1']),float(r['v2'])
        vx=math.cos(north)*vn+math.sin(north)*ve;vy=math.sin(north)*vn-math.cos(north)*ve
        yaw,w=float(pose['yaw']),float(pose['angular_z'])
        v=math.cos(yaw)*vx+math.sin(yaw)*vy+w*float(loc['antenna_y'])
        lateral=-math.sin(yaw)*vx+math.cos(yaw)*vy-w*float(loc['antenna_x'])
        age=(int(pose['received_ros_ns'])-int(stamp))*1e-9 if 'received_ros_ns' in pose else 0.
        records.append([int(stamp),v,w,float(pose['x']),float(pose['y']),yaw,lateral,age])
    records.sort(key=lambda r:r[0])
    if len(records)<100:raise ValueError('配对导航观测不足')
    if any(abs(int(r['received_ros_ns'])-int(r['received_unix_ns']))>5_000_000 for r in odom):
        raise ValueError('ROS与Unix时钟偏差超过5ms，不能直接配对串口时间')
    first,last=records[0][0],records[-1][0]
    serial_file=directory/'telemetry/status.jsonl' if basic else Path(supplement)/'telemetry/status.jsonl'
    serial=[]
    if serial_file.exists():
        digest=fingerprint(serial_file)
        serial=serial_commands(str(serial_file),digest)
        # Serial observer may belong to another, nonoverlapping run.
        if serial and any(first<=r[0]<=last for r in serial):
            hashes[str(serial_file.relative_to(ROOT))]=digest
        else:serial=[]
    if serial:
        times=np.array([r[0] for r in serial],dtype=np.int64)
        lo=max(0,np.searchsorted(times,first)-1);hi=np.searchsorted(times,last,side='right')
        command=np.array([[r[0],r[1],r[2],r[3]] for r in serial[lo:hi]])
        command_ns=np.array([r[0] for r in serial[lo:hi]],dtype=np.int64)
        commands=command[:,[1,3]];lateral_command=command[:,2]
        source='observed UART TX unix_ns (after host write); not MCU ACK'
        if command_ns[0]>first:
            # Healthy idle intentionally sends no repeated zero UART frames.
            # Bootstrap only the known stationary initial condition, not a
            # missing moving input. No held-out measurements used in fitting.
            if abs(records[0][1])>.035 or abs(records[0][2])>.1:
                raise ValueError('串口观察前缺少已知停车初值')
            command_ns=np.r_[first-1,command_ns]
            commands=np.vstack(([0.,0.],commands));lateral_command=np.r_[0.,lateral_command]
            source+='; initial stationary zero before first TX'
    else:
        motion=[r for r in csv_rows(files[1]) if r['topic']=='/base/driver_cmd_vel']
        command_ns=np.array([int(r['received_ros_ns']) for r in motion],dtype=np.int64)
        commands=np.array([[float(r['linear_x']),float(r['angular_z'])] for r in motion])
        lateral_command=np.array([float(r['linear_y']) for r in motion])
        source='headerless driver Twist received_ros_ns; observer delivery uncertainty remains'
    if len(commands)<10 or np.any(np.diff(command_ns)<0) or np.any(abs(lateral_command)>.001):
        raise ValueError('运动输入不足、倒序或含横移')
    unique=np.r_[True,np.diff(command_ns)>0];command_ns=command_ns[unique];commands=commands[unique]
    origin=min(int(command_ns[0]),first)
    times=(np.array([r[0] for r in records],dtype=np.int64)-origin)*1e-9
    values=np.array([r[1:] for r in records]);ct=(command_ns-origin)*1e-9
    if np.any(np.diff(times)<=0) or not np.isfinite(values).all():raise ValueError('导航时钟倒序/重复或无效数值')
    indices=np.searchsorted(ct,times,side='right')-1
    held=commands[np.maximum(indices,0)]
    valid=(indices>=0)&((times-ct[np.maximum(indices,0)]<=.25)|np.all(held==0,axis=1))
    valid&=times>=max(ct[0],times[0])+3
    for end in times[1:][np.diff(times)>.3]:valid&=~((times>=end-.3)&(times<end+3))
    track_window=None
    if not basic:
        track=list((directory/'tracks').glob('track_*.csv'))
        if len(track)!=1:raise ValueError('需要唯一正式跟踪轨迹')
        r=csv_rows(track[0]);track_window=(float(r[0]['stamp'])-origin*1e-9,float(r[-1]['stamp'])-origin*1e-9)
        hashes[str(track[0].relative_to(ROOT))]=fingerprint(track[0])
    metadata=dict(source_sha256=hashes,projection=proj,command_source=source,
        feedback_period_median_s=float(np.median(np.diff(times))),samples=len(times),scoring_samples=int(valid.sum()),
        lateral_velocity_rms_m_s=float(np.sqrt(np.mean(values[valid,5]**2))),
        velocity_source='MGX NED -> UTM -> body; no encoder speed used as truth',
        antenna_xy_m=[loc['antenna_x'],loc['antenna_y']])
    return CampaignRun(directory.name,ct,commands,times,values[:,:2],values[:,2:5],valid,metadata,
                       values[:,5],values[:,6],origin,track_window)


def controller_timing(directory):
    """Infer first-use intervals from unique LQR outputs, not observer arrival.

Recorded pose/error and the frozen, stateless LQR law identify an output value;
the command's header gives its creation time. Ambiguous/repeated/saturated
outputs are excluded. Delays are interval censored between control ticks.
"""
    from simulate import load_path
    from agri_path_tracking.controller import Polyline
    from four_controler.four_controler import build_controller
    from collections import Counter,defaultdict
    directory=Path(directory)
    p=json.loads((directory/'config/mission.json').read_text())['parameters']
    ctrl=json.loads((directory/'config/tracking_controller.json').read_text())
    if ctrl['algorithm']!='lqr':raise ValueError('当前时序还原仅支持已冻结的LQR')
    poses,_=load_path(next((directory/'paths').glob('coverage_*.json')))
    track=csv_rows(next((directory/'tracks').glob('track_*.csv')))
    tracker=build_controller('lqr',dt=.05,tuning={'lqr':ctrl['tuning']})(Polyline([s[:2] for s in poses]),
        **{k:p[k] for k in ('lookahead','speed','max_angular','completion_tolerance','search_window')})
    expected=[]
    key=lambda v,w:(round(v,10),round(w,10))
    for row in track:
        tracker.progress=float(row['progress'])
        out=tracker.command(*[float(row[k]) for k in ('x','y','yaw')])
        expected.append(key(*out[:2]))
    frequency=Counter(expected)
    commands=[r for r in csv_rows(directory/'telemetry/motion.csv') if r['topic']=='/control/tracking/cmd_vel']
    lookup=defaultdict(list)
    for r in commands:lookup[key(float(r['linear_x']),float(r['angular_z']))].append(int(r['source_stamp_ns']))
    ticks=np.array(sorted({int(r['source_stamp_ns']) for r in commands}),dtype=np.int64)
    ages=[];missed=[];phases=[];bounds=[]
    for row,k in zip(track,expected):
        if frequency[k]!=1 or abs(k[1])>=p['max_angular']-1e-8:continue
        stamp=int(round(float(row['stamp'])*1e9))
        candidates=[x for x in lookup[k] if stamp<=x<stamp+500_000_000]
        if not candidates:continue
        used=min(candidates);first=np.searchsorted(ticks,stamp);j=np.searchsorted(ticks,used)
        if first>=len(ticks) or j<first:continue
        age=(used-stamp)*1e-9
        lower=max(0.,(int(ticks[j-1])-stamp)*1e-9) if j>0 else 0.
        ages.append(age);missed.append(int(j-first));phases.append((int(ticks[first])-stamp)*1e-9)
        bounds.append([lower,age])
    if len(ages)<100 or len(ages)<len(track)*.5:raise ValueError('控制输入时序还原匹配不足，不能冒充精确控制器延迟')
    formal=(ticks>=float(track[0]['stamp'])*1e9)&(ticks<=float(track[-1]['stamp'])*1e9)
    intervals=np.diff(ticks[formal])*1e-9
    intervals=intervals[(intervals>=.005)&(intervals<=.5)]
    return dict(matched=len(ages),total=len(track),first_use_age_s=ages,missed_control_ticks=missed,
                sample_to_next_tick_s=phases,delivery_intervals_s=bounds,control_intervals_s=intervals.tolist(),
                method='Unique frozen-LQR output matching -> command source header; excludes ambiguous/saturated outputs; physical transport delay is interval censored')
