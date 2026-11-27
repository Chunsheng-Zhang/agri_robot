"""Offline X3 equivalent response model; never opens ROS or a device.

The fitted delay includes archived host timestamp alignment and navigation
measurement response. It is not a measurement of MCU/serial one-way delay.
"""
import csv
from dataclasses import dataclass
import hashlib
import heapq
import json
import math
from pathlib import Path

import numpy as np

from disturbances import FirstOrderActuator

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CHANNEL_KEYS = {'gain', 'limit', 'tau_s', 'delay_s'}


def load_profile(filename):
    path = Path(filename).resolve()
    profile = json.loads(path.read_text(encoding='utf-8'))
    validate_profile(profile)
    return profile, {'file': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def validate_profile(profile):
    if isinstance(profile,dict) and profile.get('schema_version')==2:
        from campaign_model import validate_campaign_profile
        return validate_campaign_profile(profile)
    if not isinstance(profile, dict) or type(profile.get('schema_version')) is not int or profile.get('schema_version') != 1 or profile.get('kind') != 'X3_GROUND_EQUIVALENT_RESPONSE':
        raise ValueError('不支持的地面响应模型格式')
    for name in ('linear', 'angular'):
        channel = profile[name]
        if not isinstance(channel, dict) or set(channel) != CHANNEL_KEYS:
            raise ValueError('响应通道必须包含gain/limit/tau_s/delay_s')
        for key, value in channel.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('响应模型参数必须为有限数值')
            if value < 0 or (key in ('gain', 'limit') and value == 0):
                raise ValueError('响应增益/限幅必须为正，时间参数不能为负')
        if channel['tau_s'] > 5 or channel['delay_s'] > 1:
            raise ValueError('响应时间常数不能超过5s，等效延迟不能超过1s')
        if channel['gain'] > 5 or channel['limit'] > 5:
            raise ValueError('本低速等效模型的gain/limit不能超过5')
    period = profile['feedback_period_s']
    if isinstance(period, bool) or not isinstance(period, (int, float)) or not math.isfinite(period) or not .005 <= period <= 1:
        raise ValueError('反馈周期需在0.005～1s之间')


class GroundActuator:
    """gain/clip -> causal per-channel delay -> exact first order response.

    reset() purges pending commands as well as state. Like the original planar
    simulator this is an explicit fault abstraction, not mechanical braking.
    """
    def __init__(self, profile):
        validate_profile(profile)
        if profile['schema_version']!=1:raise ValueError('schema2请使用make_ground_actuator')
        self.channels = tuple(dict(profile[k]) for k in ('linear', 'angular'))
        self.taus = tuple(c['tau_s'] for c in self.channels)
        self.delays = tuple(c['delay_s'] for c in self.channels)
        self.base = FirstOrderActuator(*self.taus)
        self.pending = []
        self.sequence = 0

    @property
    def output(self):
        return self.base.output

    def advance(self, stamp):
        if not math.isfinite(stamp) or (self.base.stamp is not None and stamp < self.base.stamp-1e-9):
            raise ValueError('执行器时间必须有限且不能倒退')
        while self.pending and self.pending[0][0] <= stamp:
            when, _, axis, value = heapq.heappop(self.pending)
            target = list(self.base.target)
            target[axis] = value
            self.base.command(*target, when)
        return self.base.advance(stamp)

    def command(self, linear, angular, stamp):
        if not all(math.isfinite(v) for v in (linear, angular)):
            raise ValueError('执行器输入必须有限')
        self.advance(stamp)
        for axis, (value, channel) in enumerate(zip((linear, angular), self.channels)):
            target = max(-channel['limit'], min(channel['limit'], channel['gain']*value))
            self.sequence += 1
            heapq.heappush(self.pending, (stamp+channel['delay_s'], self.sequence, axis, target))
        return self.advance(stamp)

    def reset(self, stamp):
        self.advance(stamp)
        self.pending.clear()
        return self.base.reset(stamp)


def make_ground_actuator(profile):
    validate_profile(profile)
    if profile['schema_version']==2:
        from campaign_model import CampaignActuator
        return CampaignActuator(profile)
    return GroundActuator(profile)


@dataclass
class GroundRun:
    name: str
    command_time: np.ndarray
    commands: np.ndarray
    time: np.ndarray
    measured: np.ndarray
    pose: np.ndarray
    valid: np.ndarray
    metadata: dict


def csv_rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def read_ground_run(directory):
    """Pair MGX velocity and archived odometry by EXACT source stamp.

    Use the same NED -> UTM grid -> base_link projection and lever arm as the
    project. The input is the archived driver command, not /vel_raw. All times
    share host ROS clock: command reception vs MGX host source stamp.
    """
    directory = Path(directory).resolve()
    files = [directory/'run.json', directory/'config/system.json',
             directory/'telemetry/motion.csv', directory/'telemetry/mgx23.csv',
             directory/'telemetry/odometry.csv']
    fingerprints = {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):
                    hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    run = json.loads(files[0].read_text())
    if run['mode'] != 'coverage' or run['state'] != 'COMPLETED':
        raise ValueError('地面辨识只接受已完成的coverage记录；架空/故障记录不能混用')
    projection = run['projection']
    system = json.loads(files[1].read_text())
    loc = system['localization']['ros__parameters']
    ax, ay = float(loc['antenna_x']), float(loc['antenna_y'])
    # Import only coordinate maths, no ROS runtime.
    import sys
    sys.path.insert(0, str(ROOT/'agri_robot_ws/src/agri_base'))
    from agri_base.coordinates import LocalFrame
    frame = LocalFrame(projection['latitude'], projection['longitude'])
    if frame.epsg != projection['epsg']:
        raise ValueError('投影快照不一致')
    motion = [r for r in csv_rows(files[2]) if r['topic'] == '/base/driver_cmd_vel']
    if not motion:
        raise ValueError('缺少实际驱动指令归档')
    command_ns = np.array([int(r['received_ros_ns']) for r in motion], dtype=np.int64)
    command = np.array([[float(r['linear_x']), float(r['angular_z'])] for r in motion])
    if any(abs(float(r['linear_y'])) > .001 for r in motion):
        raise ValueError('本等效模型不支持含横移指令的实验')
    mgx = csv_rows(files[3])
    fixes = {r['source_stamp_ns']: r for r in mgx if r['topic'] == '/mgx23/position'}
    odom = {r['source_stamp_ns']: r for r in csv_rows(files[4])}
    records = []
    missing = 0
    for velocity in mgx:
        if velocity['topic'] != '/mgx23/velocity_stamped':
            continue
        stamp = velocity['source_stamp_ns']
        if stamp not in odom or stamp not in fixes:
            missing += 1
            continue
        pose, fix = odom[stamp], fixes[stamp]
        north = frame.yaw(float(fix['v1']), float(fix['v2']), 0.)
        vn, ve = float(velocity['v1']), float(velocity['v2'])
        vx = math.cos(north)*vn + math.sin(north)*ve
        vy = math.sin(north)*vn - math.cos(north)*ve
        yaw, w = float(pose['yaw']), float(pose['angular_z'])
        v = math.cos(yaw)*vx + math.sin(yaw)*vy + w*ay
        lateral = -math.sin(yaw)*vx + math.cos(yaw)*vy - w*ax
        records.append((int(stamp), v, w, float(pose['x']), float(pose['y']), yaw,
                        lateral, (int(velocity['received_ros_ns'])-int(stamp))*1e-9))
    records.sort(key=lambda row: row[0])
    if len(records) < 100:
        raise ValueError('匹配的MGX/里程计数据不足100帧')
    # Keep first reception for duplicated timestamps; reject clock reversal.
    commands_unique = np.r_[True, np.diff(command_ns) > 0]
    if np.any(np.diff(command_ns) < 0):
        raise ValueError('指令ROS时钟发生倒退，不能辨识')
    command_ns, command = command_ns[commands_unique], command[commands_unique]
    origin = min(int(command_ns[0]), records[0][0])
    command_t = (command_ns-origin)*1e-9
    observed_t = np.array([(r[0]-origin)*1e-9 for r in records])
    values = np.array([r[1:] for r in records])
    if np.any(np.diff(observed_t) <= 0) or not np.isfinite(values).all() or not np.isfinite(command).all():
        raise ValueError('反馈时间重复/倒退或含非有限数据')
    index = np.searchsorted(command_t, observed_t, side='right')-1
    valid = (index >= 0) & (observed_t-command_t[np.maximum(index, 0)] <= .25)
    # Unknown input over a gap must not be treated as known zero or held forever.
    # Exclude 3s after command or observation gaps and 3s initial warmup.
    valid &= observed_t >= max(command_t[0], observed_t[0])+3
    for end in command_t[1:][np.diff(command_t) > .25]:
        valid &= ~((observed_t >= end-.25) & (observed_t < end+3))
    for end in observed_t[1:][np.diff(observed_t) > .3]:
        valid &= ~((observed_t >= end-.3) & (observed_t < end+3))
    if valid.sum() < 100:
        raise ValueError('剔除时钟/数据间断后的反馈不足100帧')
    metadata = dict(source_sha256=fingerprints, projection=projection, antenna_xy_m=[ax, ay],
                    velocity_source='MGX NED projected to base_link; NOT /vel_raw',
                    time_basis='driver received_ros_ns vs MGX source_stamp_ns (host clock)',
                    unmatched_velocity_frames=missing, samples=len(records), scoring_samples=int(valid.sum()),
                    feedback_period_median_s=float(np.median(np.diff(observed_t))),
                    velocity_receive_age_p95_s=float(np.quantile(values[:, 6], .95)),
                    lateral_velocity_rms_m_s=float(np.sqrt(np.mean(values[valid, 5]**2))),
                    max_abs_command=np.max(np.abs(command), axis=0).tolist())
    return GroundRun(directory.name, command_t, command, observed_t, values[:, :2],
                     values[:, 2:5], valid, metadata)


def replay_channel(run, channel, params, dt=.01):
    """10ms sampled exact-ZOH first-order replay; no measured-state resets."""
    from scipy.signal import lfilter
    grid = np.arange(min(0., run.command_time[0]), run.time[-1]+dt, dt)
    indices = np.searchsorted(run.command_time, grid-params['delay_s'], side='right')-1
    target = params['gain']*run.commands[np.maximum(indices, 0), channel]
    target[indices < 0] = 0
    target = np.clip(target, -params['limit'], params['limit'])
    a = math.exp(-dt/max(params['tau_s'], 1e-12))
    # y[n+1] = a*y[n] + (1-a)*u[n]; y[0]=0.
    response = np.r_[0., lfilter([1-a], [1., -a], target[:-1])]
    return np.interp(run.time, grid, response)


def fit_channel(runs, channel):
    from scipy.optimize import least_squares
    best = None
    # Delay resolution is explicit. Joint gain/clip/tau optimization for each
    # candidate delay; validation records are never passed into this function.
    for delay in np.arange(0., .3001, .025):
        def residual(parameters):
            p = dict(gain=parameters[0], limit=parameters[1], tau_s=parameters[2], delay_s=float(delay))
            return np.concatenate([(replay_channel(r, channel, p)[r.valid]-r.measured[r.valid, channel])/
                                    math.sqrt(int(r.valid.sum())) for r in runs])
        limits = (.15, 1.5) if channel == 0 else (.1, 2.)
        candidate = least_squares(residual, [1., .7 if channel == 0 else 1., .15],
                                  bounds=([.1, limits[0], .005], [2., limits[1], 2.]),
                                  max_nfev=100)
        score = float(np.dot(candidate.fun, candidate.fun))
        if best is None or score < best[0]:
            best = (score, candidate, delay)
    _, result, delay = best
    if not result.success:
        raise ValueError('响应模型辨识未收敛')
    return dict(gain=float(result.x[0]), limit=float(result.x[1]), tau_s=float(result.x[2]), delay_s=float(delay))


def evaluate_run(run, profile):
    predictions = np.column_stack([replay_channel(run, i, profile[name]) for i, name in enumerate(('linear', 'angular'))])
    indices = np.searchsorted(run.command_time, run.time, side='right')-1
    baseline = run.commands[np.maximum(indices, 0)].copy()
    baseline[indices < 0] = 0
    metrics = {}
    for i, name in enumerate(('linear', 'angular')):
        residual = predictions[run.valid, i]-run.measured[run.valid, i]
        ideal = baseline[run.valid, i]-run.measured[run.valid, i]
        rmse, ideal_rmse = float(np.sqrt(np.mean(residual**2))), float(np.sqrt(np.mean(ideal**2)))
        metrics[name] = dict(rmse=rmse, mae=float(np.mean(np.abs(residual))),
                             p95_absolute_error=float(np.quantile(np.abs(residual), .95)),
                             ideal_rmse=ideal_rmse,
                             rmse_improvement_percent=100*(1-rmse/ideal_rmse) if ideal_rmse else None)
        moving = run.valid & ((np.abs(baseline[:, 0]) > .05) | (np.abs(baseline[:, 1]) > .05) |
                               (np.abs(run.measured[:, 0]) > .05) | (np.abs(run.measured[:, 1]) > .05))
        active = predictions[moving, i]-run.measured[moving, i]
        metrics[name]['moving_samples'] = int(moving.sum())
        metrics[name]['moving_rmse'] = float(np.sqrt(np.mean(active**2))) if len(active) else None
        metrics[name]['moving_ideal_rmse'] = float(np.sqrt(np.mean((baseline[moving, i]-run.measured[moving, i])**2))) if len(active) else None
    return predictions, baseline, metrics


def pose_window_check(run, predicted, baseline, horizon_s=5., lateral_prediction=None):
    """Independent 5s windows, initial pose measured once, never path-snapped.

    This checks velocity/pose consistency against MGX, not absolute position
    accuracy. Windows with missing inputs/feedback are excluded.
    """
    windows = []
    start = 0
    while start < len(run.time)-1:
        end = int(np.searchsorted(run.time, run.time[start]+horizon_s))
        if end >= len(run.time):
            break
        if np.all(run.valid[start:end+1]) and np.max(np.diff(run.time[start:end+1])) <= .3:
            result = dict(start_s=float(run.time[start]), end_s=float(run.time[end]))
            for label, speeds in (('model', predicted), ('ideal', baseline)):
                x, y, yaw = run.pose[start].copy()
                for i in range(start, end):
                    dt = run.time[i+1]-run.time[i]
                    v, w = (speeds[i]+speeds[i+1])/2
                    half = w*dt/2
                    distance = v*dt*(math.sin(half)/half if abs(half)>1e-10 else 1.)
                    lateral=0. if label=='ideal' or lateral_prediction is None else (lateral_prediction[i]+lateral_prediction[i+1])/2
                    side=lateral*dt*(math.sin(half)/half if abs(half)>1e-10 else 1.)
                    x += distance*math.cos(yaw+half)-side*math.sin(yaw+half)
                    y += distance*math.sin(yaw+half)+side*math.cos(yaw+half)
                    yaw += w*dt
                result[label+'_endpoint_error_m'] = float(math.hypot(x-run.pose[end, 0], y-run.pose[end, 1]))
                delta = yaw-run.pose[end, 2]
                result[label+'_heading_error_deg'] = math.degrees(math.atan2(math.sin(delta), math.cos(delta)))
            windows.append(result)
        start = end
    summary = dict(horizon_s=horizon_s, windows=len(windows),
                   note='5s局部开环运动学一致性；每窗仅初始位姿取MGX，不是独立真值精度')
    for label in ('model', 'ideal'):
        errors = [r[label+'_endpoint_error_m'] for r in windows]
        summary[label+'_endpoint_rmse_m'] = float(np.sqrt(np.mean(np.square(errors)))) if errors else None
    return summary, windows
