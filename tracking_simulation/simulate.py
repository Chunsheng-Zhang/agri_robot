#!/usr/bin/env python3
"""Offline tracking algorithm only: no ROS nodes, devices, Web or task services."""
import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
for folder in ('agri_base', 'agri_waypoint', 'agri_path_tracking'):
    sys.path.insert(0, str(ROOT/'agri_robot_ws/src'/folder))
sys.path.insert(0, str(ROOT))

from agri_base.coordinates import wrap_angle
from agri_base.path_geometry import directed_poses, point_turn_indices
from agri_path_tracking.controller import Polyline
from agri_path_tracking.coverage import AlignmentTimeout, CoverageController
from four_controler.four_controler import PurePursuit
from disturbances import GaussianPoseNoise, FirstOrderActuator
from agri_backend.experiment_analysis import statistics


@dataclass(frozen=True)
class SimulationConfig:
    lookahead: float = .8
    speed: float = .2
    max_angular: float = 1.
    completion_tolerance: float = .1
    search_window: float = 1.
    dt: float = .05
    duration: float | None = None
    initial_lateral: float = .2
    initial_heading_deg: float = 10.
    position_noise: float = 0.
    heading_noise_deg: float = 0.
    feedback_period: float = 0.
    actuator_linear_tau: float = 0.
    actuator_angular_tau: float = 0.
    seed: int = 0


def forward_poses(points):
    if len(points) < 2:
        raise ValueError('参考路径至少需要两个不同位置')
    result = []
    for index, (x, y) in enumerate(points):
        a, b = points[index:index+2] if index+1 < len(points) else points[-2:]
        result.append((float(x), float(y), math.atan2(b[1]-a[1], b[0]-a[0])))
    return result


def scenario(name):
    if name == 'straight':
        return forward_poses([(i*.1, 0.) for i in range(201)])
    if name == 'sine':
        return forward_poses([(i*.1, 1.5*math.sin(i*.1*2*math.pi/15)) for i in range(301)])
    if name == 'circle':
        return [(5*math.cos(i*2*math.pi/360), 5*math.sin(i*2*math.pi/360),
                 wrap_angle(i*2*math.pi/360+math.pi/2)) for i in range(361)]
    if name == 'point_turn':
        return [(0.,0.,0.),(10.,0.,0.),(10.,0.,math.pi/2),(10.,2.,math.pi/2),
                (10.,2.,math.pi),(0.,2.,math.pi),(0.,2.,math.pi/2),(0.,4.,math.pi/2),
                (0.,4.,0.),(10.,4.,0.)]
    raise ValueError('未知示例路径')


def load_path(filename):
    source = Path(filename).resolve()
    document = json.loads(source.read_text(encoding='utf-8'))
    points = document['points'] if isinstance(document, dict) else document
    if not isinstance(points, list) or not 2 <= len(points) <= 150000:
        raise ValueError('JSON需要2～150000个米制平面points')
    if not all(isinstance(p, dict) and 'x' in p and 'y' in p for p in points):
        raise ValueError('每个参考点需要米制x/y字段；经纬度文件应先转换或使用已规划路径')
    if any('yaw' in p for p in points):
        if not all('yaw' in p for p in points):raise ValueError('yaw必须全部提供或全部省略')
        poses = [(float(p['x']),float(p['y']),float(p['yaw'])) for p in points]
    else:
        xy = []
        for p in points:
            point = (float(p['x']),float(p['y']))
            if not xy or math.dist(xy[-1],point)>1e-6:xy.append(point)
        poses = forward_poses(xy)
    directed_poses(poses)
    return poses, {'file':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                   'frame_id':document.get('frame_id') if isinstance(document,dict) else None,
                   'projection':document.get('projection') if isinstance(document,dict) else None}


def integrate(x, y, yaw, v, w, dt):
    """Exact constant-body-twist integration; no reference-path snapping."""
    half = w*dt/2
    distance = v*dt*(math.sin(half)/half if abs(half)>1e-10 else 1.)
    return x+distance*math.cos(yaw+half), y+distance*math.sin(yaw+half), wrap_angle(yaw+w*dt)


def create_control(path, start, settings, controller_factory=None, start_tolerance=.8):
    """Use local control laws; reuse only the production phase/stop state machine."""
    factory = controller_factory or PurePursuit
    segmented = len(path.legs) > 1 or any(d < 0 for d in path.directions)
    if segmented and not getattr(factory, 'supports_segmented', False):
        raise ValueError('点转/倒车路径请使用本地PP跟踪器')
    tracker = factory(path, **settings)
    if not segmented:
        return tracker
    control = CoverageController(path, start, start_tolerance=start_tolerance, **settings)
    control.tracker = tracker
    return control


def simulate(poses, config=SimulationConfig(), *, initial_pose=None, initial_progress=0., pause_intervals=(), controller_factory=None):
    values = asdict(config)
    if any(not math.isfinite(v) for k,v in values.items() if v is not None):
        raise ValueError('仿真参数必须是有限数值')
    if (not .005 <= config.dt <= .2 or config.position_noise < 0 or
            config.heading_noise_deg < 0 or config.feedback_period < 0):
        raise ValueError('dt需在0.005～0.2秒内，噪声标准差及反馈周期不能为负')
    if initial_pose is not None and (len(initial_pose)!=3 or not all(math.isfinite(v) for v in initial_pose)):
        raise ValueError('初始状态需要有限的x/y/yaw')
    pauses = list(pause_intervals)
    previous_end = 0.
    for start,end in pauses:
        if not all(math.isfinite(v) for v in (start,end)) or start<previous_end or end<=start:
            raise ValueError('暂停区间必须有序、不重叠，且满足0<=start<end')
        previous_end = end
    clean, directions = directed_poses(poses)
    stops = point_turn_indices(poses)
    path = Polyline([p[:2] for p in clean], directions, stops)
    settings = {k:values[k] for k in ('lookahead','speed','max_angular','completion_tolerance','search_window')}
    initial_yaw = clean[0][2]
    x = clean[0][0]-config.initial_lateral*math.sin(initial_yaw)
    y = clean[0][1]+config.initial_lateral*math.cos(initial_yaw)
    yaw = wrap_angle(initial_yaw+math.radians(config.initial_heading_deg))
    if initial_pose is not None:
        x,y,yaw = initial_pose
        yaw = wrap_angle(yaw)
    segmented = bool(stops or -1 in directions)
    control = create_control(path, (x,y), settings, controller_factory,
                             start_tolerance=max(.8, abs(config.initial_lateral)+.1))
    tracker = control.tracker if segmented else control
    if not math.isfinite(initial_progress) or not 0<=initial_progress<=tracker.leg[1]:
        raise ValueError('初始进度必须位于首个行驶段内')
    tracker.progress=initial_progress
    duration = config.duration if config.duration is not None else max(60.,5*path.total/config.speed+120)
    actuator = FirstOrderActuator(config.actuator_linear_tau, config.actuator_angular_tau)
    brake_duration = max(1., 5*max(actuator.taus)) if any(actuator.taus) else 0.
    if duration <= 0 or math.ceil((duration+brake_duration)/config.dt)>500000:
        raise ValueError('仿真时长必须为正，且不能超过500000个积分步')
    noise = GaussianPoseNoise(config.position_noise, config.heading_noise_deg, config.seed)
    rows = []; state = 'TIME_LIMIT'; reason = '达到仿真时限，尚未完成路径'
    t = 0.
    last_feedback = -math.inf
    actual_speed = 0.
    feedback_stamp = None
    terminal_time = None
    for step in range(math.ceil((duration+brake_duration)/config.dt)+2):
        feedback_fresh = t-last_feedback >= config.feedback_period-1e-9
        if feedback_fresh:
            measured_x, measured_y, measured_yaw = noise.sample(x, y, yaw)
            last_feedback,feedback_stamp = t,step
            measured_speed = actual_speed
        if hasattr(tracker, 'set_speed_feedback'):
            tracker.set_speed_feedback(measured_speed)
        # Evaluate true pose against the active controller leg, not a nearby row.
        distance, _, lateral, body_yaw = tracker.error(x,y)
        heading = wrap_angle(body_yaw-yaw)
        phase = control.phase if segmented else 'TRACKING'
        paused = any(start<=t<end for start,end in pauses)
        if paused:phase = 'PAUSED'
        if terminal_time is not None:phase = 'BRAKING'
        leg = tracker.leg_index
        begin = perf_counter()
        try:
            if paused or terminal_time is not None:
                _,_,observed_lateral,observed_yaw = tracker.error(measured_x,measured_y)
                command = (0.,0.,distance,observed_lateral,wrap_angle(observed_yaw-measured_yaw),False)
            else:
                command = (control.command(measured_x,measured_y,measured_yaw,now=t,sample_stamp=feedback_stamp)
                           if segmented else control.command(measured_x,measured_y,measured_yaw))
        except AlignmentTimeout as exc:
            state,reason='ALIGNMENT_TIMEOUT',str(exc)
            break
        v,w,_,observed_lateral,observed_heading,done = command
        if not all(math.isfinite(value) for value in command[:5]):
            raise ValueError('控制器输出包含非有限数值')
        compute_ms = (perf_counter()-begin)*1000
        if t >= duration:
            v,w = 0.,0.
        if paused:
            applied_v, applied_w = actuator.reset(t)
        else:
            applied_v, applied_w = actuator.command(v, w, t)
        rows.append({'time_s':t,'x_true_m':x,'y_true_m':y,'yaw_true_rad':yaw,
            'compute_ms':compute_ms,
            'feedback_time_s':last_feedback,'feedback_fresh':feedback_fresh,
            'x_observed_m':measured_x,'y_observed_m':measured_y,'yaw_observed_rad':measured_yaw,
            'v_observed_m_s':measured_speed,
            'cross_track_true_m':lateral,'heading_error_true_rad':heading,'distance_true_m':distance,
            'cross_track_observed_m':observed_lateral,'heading_error_observed_rad':observed_heading,
            'v_command_m_s':v,'w_command_rad_s':w,'progress_m':tracker.progress,'leg':leg,'phase':phase})
        rows[-1].update(v_actuator_m_s=applied_v, w_actuator_rad_s=applied_w,
                       v_actual_body_m_s=applied_v, w_actual_rad_s=applied_w)
        if done and terminal_time is None:
            state,reason='COMPLETED','控制器报告路径跟踪完成'
            if brake_duration == 0:break
            terminal_time = t
        if t >= duration and terminal_time is None:
            if brake_duration == 0:break
            terminal_time = t
        if terminal_time is not None and t-terminal_time >= brake_duration-1e-9:break
        next_t = t+config.dt if terminal_time is not None else min((step+1)*config.dt, duration)
        if config.actuator_linear_tau == config.actuator_angular_tau == 0:
            x,y,yaw = integrate(x,y,yaw,applied_v,applied_w,next_t-t)
        else:
            # Resolve the changing actuator response with <=5ms midpoint steps;
            # physics is driven by the filtered output, not the controller request.
            count = max(1, math.ceil((next_t-t)/.005))
            step_dt = (next_t-t)/count
            for i in range(count):
                middle = t+(i+.5)*step_dt
                mv,mw = actuator.advance(middle)
                x,y,yaw = integrate(x,y,yaw,mv,mw,step_dt)
            actuator.advance(next_t)
        actual_speed = actuator.output[0]
        t = next_t
    tracking = [row for row in rows if row['phase']=='TRACKING']
    metrics = {'kind':'SIMULATION','controller':getattr(tracker,'name',type(tracker).__name__),
        'controller_implementation':type(tracker).__module__+'.'+type(tracker).__name__,
        'wrapper':'CoverageController' if segmented else None,'state':state,'reason':reason,
        'model':('planar unicycle with first-order body-velocity actuator; <=5ms midpoint integration'
                 if any(actuator.taus) else 'ideal planar unicycle; exact zero-order-hold integration'),
        'parameters':values,'duration_limit_s':duration,'elapsed_simulation_s':t,
        'reference_length_m':path.total,'final_progress_m':tracker.progress,
        'final_endpoint_distance_m':math.dist((x,y),path.points[-1]),
        'initial_pose':list(initial_pose) if initial_pose is not None else None,
        'initial_progress_m':initial_progress,
        'pause_intervals_s':pauses,
        'tracking_samples':len(tracking),'alignment_samples':sum(r['phase']=='ALIGNING' for r in rows),
        'paused_samples':sum(r['phase']=='PAUSED' for r in rows),
        'compute_ms':statistics([r['compute_ms'] for r in rows]),
        'optimizer':{'calls':getattr(control,'optimization_calls',0),
                     'unconverged':getattr(control,'optimization_unconverged',0),
                     'iterations':getattr(control,'optimization_iterations',0)},
        'cross_track_true_m':statistics([r['cross_track_true_m'] for r in tracking]),
        'heading_error_true_deg':statistics([math.degrees(r['heading_error_true_rad']) for r in tracking]),
        'cross_track_observed_m':statistics([r['cross_track_observed_m'] for r in tracking]),
        'heading_error_observed_deg':statistics([math.degrees(r['heading_error_observed_rad']) for r in tracking]),
        'actuator_linear_request_error_m_s':statistics([r['v_actuator_m_s']-r['v_command_m_s'] for r in tracking]),
        'actuator_angular_request_error_rad_s':statistics([r['w_actuator_rad_s']-r['w_command_rad_s'] for r in tracking]),
        'notes':['纯算法仿真，不启动ROS/定位/规划/安全节点或硬件；不接入当前Web任务。',
                 '真值是运动学模型积分结果，带噪观测只输入控制器；未将车辆位置吸附到参考路径。',
                 '四种控制律均来自four_controler/four_controler.py；PP为所给参考脚本的速度相关前视/转角公式适配，不调用实车PP.command。',
                 'PP点转/倒车复用CoverageController状态机，其内部跟踪器替换为本地PP；匹配/进度几何工具共用。',
                 '统计只含TRACKING阶段；原地对齐样本另外保存在CSV，不混入正式跟踪误差。',
                 '噪声只进入周期观测；执行器为独立v/ω一阶滞后，tau=0关闭，暂停直接清零。',
                 '平面模型未包含刚体惯性、侧滑或土壤；附加一阶响应未实车标定，不将该误差当作实车精度。']}
    return rows,metrics


def write_outputs(output, poses, rows, metrics):
    output = Path(output)
    if output.exists() and any(output.iterdir()):raise ValueError('输出目录非空，拒绝覆盖已有结果')
    output.mkdir(parents=True,exist_ok=True)
    origin = metrics.get('source') or {}
    (output/'reference.json').write_text(json.dumps({'kind':'SIMULATION','source':origin,
        'frame_id':origin.get('frame_id'),'projection':origin.get('projection'),
        'points':[dict(zip(('x','y','yaw'),pose)) for pose in poses]},ensure_ascii=False,indent=2),encoding='utf-8')
    with (output/'tracking.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        columns=list(rows[0]) if rows else ['time_s','phase']
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader();writer.writerows(rows)
    (output/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    os.environ.setdefault('MPLCONFIGDIR',str(HERE/'.cache/matplotlib'))
    Path(os.environ['MPLCONFIGDIR']).mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
    try:
        fig.suptitle('SIMULATION | '+metrics['controller']+' | '+metrics['state'])
        ax=axes[0,0];ax.plot([p[0] for p in poses],[p[1] for p in poses],'--',label='Reference')
        if rows:
            ax.plot([r['x_true_m'] for r in rows],[r['y_true_m'] for r in rows],label='Model truth')
            if metrics['parameters']['position_noise']:
                ax.scatter([r['x_observed_m'] for r in rows],[r['y_observed_m'] for r in rows],s=1,alpha=.25,label='Noisy observation')
        ax.set(title='Reference and model trajectory',xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal',adjustable='datalim');ax.legend(fontsize=8)
        # NaNs preserve phase gaps rather than joining across stationary alignment.
        t=[r['time_s'] for r in rows]
        tracking=[r['phase']=='TRACKING' for r in rows]
        axes[0,1].plot(t,[100*r['cross_track_true_m'] if active else math.nan for r,active in zip(rows,tracking)],label='Truth')
        axes[0,1].plot(t,[100*r['cross_track_observed_m'] if active else math.nan for r,active in zip(rows,tracking)],alpha=.5,label='Controller observation')
        axes[0,1].set(title='Lateral error (tracking only)',xlabel='Simulation time (s)',ylabel='Error (cm)');axes[0,1].legend(fontsize=8)
        axes[1,0].plot(t,[math.degrees(r['heading_error_true_rad']) if active else math.nan for r,active in zip(rows,tracking)],label='Truth')
        axes[1,0].plot(t,[math.degrees(r['heading_error_observed_rad']) if active else math.nan for r,active in zip(rows,tracking)],alpha=.5,label='Controller observation')
        axes[1,0].legend(fontsize=8)
        axes[1,0].set(title='Body-heading error (tracking only)',xlabel='Simulation time (s)',ylabel='Error (deg)')
        ax=axes[1,1];ax.plot(t,[r['v_command_m_s'] for r in rows],label='Linear',color='tab:blue');ax.set_ylabel('v command (m/s)',color='tab:blue')
        angular=ax.twinx();angular.plot(t,[r['w_command_rad_s'] for r in rows],label='Angular',color='tab:orange',alpha=.7);angular.set_ylabel('w command (rad/s)',color='tab:orange')
        ax.set(title='Controller requests before actuator response',xlabel='Simulation time (s)')
        for ax in axes.flat:ax.grid(True,alpha=.25)
        fig.supxlabel('Model truth is not a real-vehicle measurement. Alignment samples remain in tracking.csv.',fontsize=9)
        fig.savefig(output/'tracking_simulation.png',dpi=160)
    finally:plt.close(fig)
    from reports import save_actuator_response
    save_actuator_response(output, rows)


def main():
    temporary = HERE/'.cache/tmp'
    temporary.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('TMPDIR',str(temporary))
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group()
    source.add_argument('--scenario',choices=('straight','sine','circle','point_turn'),default=None)
    source.add_argument('--path-file',type=Path,help='Saved JSON with planar points x/y and optional body yaw in radians')
    defaults=asdict(SimulationConfig())
    for key,value in defaults.items():
        parser.add_argument('--'+key.replace('_','-'),type=int if key=='seed' else float,default=value)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    try:
        if args.path_file:poses,origin=load_path(args.path_file)
        else:
            name=args.scenario or 'sine';poses=scenario(name);origin={'scenario':name,'frame_id':'synthetic_local_m','projection':None}
        config=SimulationConfig(**{key:getattr(args,key) for key in defaults})
        rows,metrics=simulate(poses,config)
        metrics['source']=origin
        sources=[Path(__file__),ROOT/'agri_backend/experiment_analysis.py']
        for package,module in (('agri_base','coordinates.py'),('agri_base','path_geometry.py'),
                               ('agri_waypoint','controller.py'),('agri_path_tracking','controller.py'),
                               ('agri_path_tracking','coverage.py')):
            sources.append(ROOT/'agri_robot_ws/src'/package/package/module)
        metrics['implementation_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
        output=args.output or HERE/'results'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        write_outputs(output,poses,rows,metrics)
        print(json.dumps({'kind':'SIMULATION','state':metrics['state'],'output':str(output.resolve()),
            'lateral_rmse_cm':None if metrics['cross_track_true_m']['rmse'] is None else 100*metrics['cross_track_true_m']['rmse'],
            'final_endpoint_distance_m':metrics['final_endpoint_distance_m']},ensure_ascii=False,indent=2))
        return 0 if metrics['state']=='COMPLETED' else 2
    except (ValueError,TypeError,KeyError,OSError) as exc:
        parser.exit(1,str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
