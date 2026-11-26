#!/usr/bin/env python3
"""Run the existing tracking controllers against Gazebo ODE feedback only."""
import argparse
from collections import deque
from dataclasses import asdict, replace
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
sys.path.insert(0, str(SIM))
from run_tracking import read_config
from simulate import (Polyline, create_control, AlignmentTimeout,
                      directed_poses, point_turn_indices, load_path, statistics, wrap_angle, write_outputs)
from four_controler.four_controler import NAMES, build_controller
from disturbances import GaussianPoseNoise, FirstOrderActuator
from model import build_world, DESCRIPTION
from report import physical_report
from reports import save_comparison
import yaml


def physical_config(filename):
    data = yaml.safe_load(Path(filename).read_text(encoding='utf-8'))
    required = {
        'physics': {'step_size', 'real_time_factor', 'settle_time', 'feedback_delay', 'feedback_timeout', 'wall_timeout'},
        'vehicle': {'body_mass', 'body_length', 'body_width', 'body_height', 'base_height',
                    'wheel_mass', 'wheel_radius', 'wheel_width', 'wheelbase', 'wheel_separation', 'wheel_z',
                    'max_wheel_torque', 'max_wheel_acceleration', 'friction_longitudinal', 'friction_lateral'}}
    if not isinstance(data, dict) or set(data) != set(required):
        raise ValueError('物理配置必须含physics和vehicle')
    for group, defaults in required.items():
        if not isinstance(data[group], dict) or set(data[group]) != set(defaults):
            raise ValueError('物理配置字段不完整或有未知字段：'+group)
        for key, value in data[group].items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(group+'.'+key+'必须为有限数值')
            if key != 'wheel_z' and (value < 0 or (value == 0 and key != 'feedback_delay')):
                raise ValueError(group+'.'+key+'必须为正数（仅反馈延迟可为0）')
    p, v = data['physics'], data['vehicle']
    if not .0001 <= p['step_size'] <= .01 or not .1 <= p['real_time_factor'] <= 10:
        raise ValueError('step_size需为0.0001～0.01，real_time_factor需为0.1～10')
    if p['feedback_delay'] >= p['feedback_timeout']:
        raise ValueError('反馈延迟必须小于反馈超时')
    if v['base_height'] + v['wheel_z'] < v['wheel_radius']-.01:
        raise ValueError('初始轮子明显穿过地面')
    return data


def reserve_domain(requested):
    """Never inherit a real-vehicle Domain. Lock local simulator runs apart."""
    occupied = {0, 225}
    if os.environ.get('ROS_DOMAIN_ID', '').isdigit():
        occupied.add(int(os.environ['ROS_DOMAIN_ID']))
    for entry in Path('/proc').glob('[0-9]*/environ'):
        try:
            for variable in entry.read_bytes().split(b'\0'):
                if variable.startswith(b'ROS_DOMAIN_ID='):
                    occupied.add(int(variable.split(b'=', 1)[1]))
        except (OSError, ValueError):
            pass
    options = [requested] if requested is not None else list(range(201, 231))
    for domain in options:
        if domain is None or not 200 <= domain <= 230 or domain in occupied:
            continue
        handle = (SIM/f'.cache/gazebo_domain_{domain}.lock').open('w')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return domain, handle
        except BlockingIOError:
            handle.close()
    raise ValueError('没有可用隔离Domain（200～230，排除225及已用Domain）')


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Localization:
    """Hold a periodic noisy observation, with separate acquisition and delivery times."""
    def __init__(self, settings, delay):
        self.settings, self.delay = settings, delay
        self.noise = GaussianPoseNoise(settings.position_noise, settings.heading_noise_deg, settings.seed)
        self.queue = deque()
        self.last_capture = -math.inf
        self.delivered = None
        self.delivered_speed = 0.

    def update(self, sample):
        stamp, x, y, yaw, *_ = sample
        if stamp-self.last_capture >= max(self.settings.feedback_period, .001)-1e-8:
            self.last_capture = stamp
            observation = (stamp, *self.noise.sample(x, y, yaw))
            speed = sample[4]*math.cos(yaw)+sample[5]*math.sin(yaw) if len(sample) >= 6 else 0.
            self.queue.append((observation, speed))
        fresh = False
        while self.queue and self.queue[0][0][0]+self.delay <= stamp+1e-8:
            self.delivered, self.delivered_speed = self.queue.popleft()
            fresh = True
        return self.delivered, fresh


def run_one(name, poses, source, settings, tuning, physics, output, gui, domain):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry

    clean, directions = directed_poses(poses)
    stops = point_turn_indices(poses)
    path = Polyline([p[:2] for p in clean], directions, stops)
    segmented = bool(stops or -1 in directions)
    yaw = clean[0][2]
    initial = (clean[0][0]-settings.initial_lateral*math.sin(yaw),
               clean[0][1]+settings.initial_lateral*math.cos(yaw),
               wrap_angle(yaw+math.radians(settings.initial_heading_deg)))
    kwargs = {key: getattr(settings, key) for key in
              ('lookahead', 'speed', 'max_angular', 'completion_tolerance', 'search_window')}
    factory = build_controller(name, dt=settings.dt, tuning=tuning)
    control = create_control(path, initial[:2], kwargs, factory,
                             start_tolerance=max(.8, abs(settings.initial_lateral)+.1))
    tracker = control.tracker if segmented else control
    duration = settings.duration or max(60., 5*path.total/settings.speed+120)
    output.mkdir(parents=True)
    world = output/'world.sdf'
    build_world(world, poses, initial, physics)
    env = os.environ.copy()
    (output/'xdg_runtime').mkdir(mode=0o700)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        master_port = probe.getsockname()[1]
    env.update(GAZEBO_MASTER_URI=f'http://127.0.0.1:{master_port}', GAZEBO_IP='127.0.0.1',
        ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
        GAZEBO_MODEL_DATABASE_URI='', GAZEBO_LOG_PATH=str(output/'state_logs'),
        AGRI_GAZEBO_CONSOLE_DIR=str(output/'console'),
        GAZEBO_GUI_INI_FILE=str(output/'gui.ini'),
        XDG_CACHE_HOME=str(SIM/'.cache/xdg'),
        XDG_RUNTIME_DIR=str(output/'xdg_runtime'),
        LD_PRELOAD=str(SIM/'.cache/gazebo_build/libagri_gazebo_console.so'))
    (output/'gui.ini').write_text('[geometry]\nx=0\ny=0\nwidth=1200\nheight=800\n', encoding='utf-8')
    node = Node('tracking_gazebo_'+name, namespace='/tracking_sim')
    command_pub = node.create_publisher(Twist, '/tracking_sim/cmd_vel', 1)
    observation_pub = node.create_publisher(PoseStamped, '/tracking_sim/localization', 1)
    samples = deque(maxlen=500)
    raw_count = 0
    last_received_stamp = -math.inf
    raw_stream = (output/'ground_truth.csv').open('w', encoding='utf-8', buffering=1)
    raw_stream.write('gazebo_time_s,x_m,y_m,z_m,yaw_rad,vx_world_m_s,vy_world_m_s,wz_rad_s\n')
    actuator = FirstOrderActuator(settings.actuator_linear_tau, settings.actuator_angular_tau)
    actuator_stream = (output/'actuator.csv').open('w', encoding='utf-8', buffering=1)
    actuator_stream.write('gazebo_time_s,v_command_m_s,w_command_rad_s,v_actuator_m_s,w_actuator_rad_s,v_actual_body_m_s,w_actual_rad_s,fault_stop\n')

    def publish_drive(sample, request=None, fault_stop=False):
        stamp, _, _, yaw, vx, vy, wz, _ = sample
        if fault_stop:
            av, aw = actuator.reset(stamp)
        elif request is not None:
            av, aw = actuator.command(*request, stamp)
        else:
            av, aw = actuator.advance(stamp)
        msg = Twist()
        msg.linear.x, msg.angular.z = av, aw
        command_pub.publish(msg)
        actuator_stream.write(','.join(map(str, (stamp, *actuator.target, av, aw,
                              vx*math.cos(yaw)+vy*math.sin(yaw), wz, int(fault_stop))))+'\n')
        return av, aw

    def received(msg):
        nonlocal raw_count, last_received_stamp
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        pose, twist = msg.pose.pose, msg.twist.twist
        sample = (stamp, pose.position.x, pose.position.y, yaw_of(pose.orientation),
                  twist.linear.x, twist.linear.y, twist.angular.z, pose.position.z)
        if not all(math.isfinite(x) for x in sample):
            raise ValueError('Gazebo真值含非有限值')
        if stamp <= last_received_stamp:
            raise ValueError('Gazebo时间倒退或重复，停止本次仿真')
        last_received_stamp = stamp
        if len(samples) == samples.maxlen:
            raise RuntimeError('Gazebo真值处理积压，拒绝静默丢帧')
        samples.append(sample)
        raw_count += 1
        raw_stream.write(','.join(map(str, (stamp, sample[1], sample[2], sample[7], *sample[3:7])))+'\n')

    subscription = node.create_subscription(Odometry, '/tracking_sim/ground_truth', received, qos_profile_sensor_data)
    server = client = None
    rows = []
    state, reason = 'TIME_LIMIT', '达到仿真时限，尚未完成'
    start_stamp = next_control = None
    latest_observation = None
    localization = Localization(settings, physics['physics']['feedback_delay'])
    last_wall_feedback = time.monotonic()
    gui_live_at_end = False
    launched = last_progress_wall = time.monotonic()
    terminal_stamp = None
    log = (output/'gzserver.log').open('w', encoding='utf-8')
    client_log = (output/'gzclient.log').open('w', encoding='utf-8')
    try:
        server = subprocess.Popen(['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', str(world)],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        if gui:
            client = subprocess.Popen(['gzclient'], env=env, stdout=client_log,
                stderr=subprocess.STDOUT, start_new_session=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            wall = time.monotonic()
            if server.poll() is not None:
                state, reason = 'GAZEBO_EXITED', 'gzserver已退出，见gzserver.log'
                break
            if wall-launched > physics['physics']['wall_timeout']:
                state, reason = 'WALL_TIMEOUT', '达到独立墙钟超时，未把中途结果当完成'
                break
            if not samples:
                if wall-last_wall_feedback > (30 if start_stamp is None else 2):
                    state, reason = 'FEEDBACK_LOST', 'Gazebo真值/进程通信中断'
                    break
                continue
            while samples:
                sample = samples.popleft()
                stamp, x, y, yaw, vx, vy, wz, z = sample
                last_wall_feedback = wall
                observation, fresh = localization.update(sample)
                if fresh and observation:
                    latest_observation = observation
                    msg = PoseStamped()
                    msg.header.frame_id = 'map'
                    msg.header.stamp.sec = int(observation[0])
                    msg.header.stamp.nanosec = int((observation[0] % 1)*1e9)
                    msg.pose.position.x, msg.pose.position.y = observation[1:3]
                    msg.pose.orientation.z = math.sin(observation[3]/2)
                    msg.pose.orientation.w = math.cos(observation[3]/2)
                    observation_pub.publish(msg)
                subscribers = command_pub.get_subscription_count()
                if subscribers > 1 or (start_stamp is not None and subscribers != 1):
                    raise RuntimeError('仿真速度接收者不再唯一，停止本次仿真')
                if stamp < physics['physics']['settle_time'] or subscribers != 1:
                    publish_drive(sample, fault_stop=True)
                    if wall-launched > 30:
                        raise RuntimeError('仿真驱动在30秒内未订阅指令')
                    continue
                if start_stamp is None:
                    start_stamp = stamp
                    next_control = stamp
                t = stamp-start_stamp
                if terminal_stamp is not None:
                    publish_drive(sample, request=(0., 0.), fault_stop=state not in ('COMPLETED', 'TIME_LIMIT'))
                    if stamp-terminal_stamp >= max(1., 5*max(actuator.taus)):
                        break
                    continue
                if observation is None or stamp-observation[0] > physics['physics']['feedback_timeout']:
                    state, reason = 'LOCALIZATION_TIMEOUT', '模拟定位超时，停止仿真'
                    terminal_stamp = stamp
                    publish_drive(sample, fault_stop=True)
                    continue
                if stamp+1e-8 < next_control:
                    publish_drive(sample)
                    continue
                distance, _, lateral, reference_yaw = tracker.error(x, y)
                begin = time.perf_counter()
                phase = control.phase if segmented else 'TRACKING'
                if hasattr(tracker, 'set_speed_feedback'):
                    tracker.set_speed_feedback(localization.delivered_speed)
                try:
                    command = (control.command(*observation[1:], now=t, sample_stamp=observation[0])
                               if segmented else control.command(*observation[1:]))
                except AlignmentTimeout as exc:
                    state, reason = 'ALIGNMENT_TIMEOUT', str(exc)
                    terminal_stamp = stamp
                    publish_drive(sample, fault_stop=True)
                    continue
                compute_ms = (time.perf_counter()-begin)*1000
                v, w, _, observed_lateral, observed_heading, done = command
                if not all(math.isfinite(value) for value in command[:5]):
                    raise ValueError('控制输出非有限值')
                if t >= duration:
                    v, w = 0., 0.
                applied_v, applied_w = publish_drive(sample, request=(v, w))
                rows.append(dict(time_s=t, gazebo_time_s=stamp, x_true_m=x, y_true_m=y,
                    yaw_true_rad=yaw, compute_ms=compute_ms, feedback_time_s=observation[0]-start_stamp,
                    feedback_age_s=stamp-observation[0], feedback_fresh=latest_observation != getattr(node, '_last_observation', None),
                    x_observed_m=observation[1], y_observed_m=observation[2], yaw_observed_rad=observation[3],
                    v_observed_m_s=localization.delivered_speed,
                    cross_track_true_m=lateral, heading_error_true_rad=wrap_angle(reference_yaw-yaw),
                    distance_true_m=distance, cross_track_observed_m=observed_lateral,
                    heading_error_observed_rad=observed_heading, v_command_m_s=v, w_command_rad_s=w,
                    v_actuator_m_s=applied_v, w_actuator_rad_s=applied_w,
                    v_actual_body_m_s=vx*math.cos(yaw)+vy*math.sin(yaw),
                    lateral_actual_body_m_s=-vx*math.sin(yaw)+vy*math.cos(yaw),
                    w_actual_rad_s=wz, progress_m=tracker.progress, leg=tracker.leg_index, phase=phase,
                    control_interval_s=0 if not rows else t-rows[-1]['time_s']))
                node._last_observation = latest_observation
                next_control = stamp+settings.dt
                if done:
                    state, reason, terminal_stamp = 'COMPLETED', '控制器报告完成，随后零请求记录执行器与物理制动响应', stamp
                elif t >= duration:
                    terminal_stamp = stamp
                if wall-last_progress_wall > 10:
                    print(f'{name}: t={t:.1f}s, progress={tracker.progress:.2f}/{path.total:.2f}m', flush=True)
                    last_progress_wall = wall
            if terminal_stamp is not None and stamp-terminal_stamp >= max(1., 5*max(actuator.taus)):
                break
    except KeyboardInterrupt:
        state, reason = 'CANCELLED', '用户中断，停止本次仿真进程'
    except Exception as exc:
        state, reason = 'ERROR', str(exc)
        (output/'error.log').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        if rclpy.ok():
            if actuator.stamp is not None:
                actuator.reset(actuator.stamp)
            command_pub.publish(Twist())
        gui_live_at_end = client is not None and client.poll() is None
        stop_process(client)
        stop_process(server)
        raw_stream.close()
        actuator_stream.close()
        node.destroy_node()
        log.close()
        client_log.close()
    tracking = [r for r in rows if r['phase'] == 'TRACKING']
    final = sample if start_stamp is not None else None
    metrics = dict(kind='GAZEBO_ODE_TRACKING', algorithm=name, controller=name, state=state, reason=reason,
        controller_implementation=type(tracker).__module__+'.'+type(tracker).__name__,
        controller_settings=tuning.get(name, {}),
        model='Gazebo Classic ODE; four-wheel differential equivalent, not calibrated mecanum rollers',
        source=source, parameters=asdict(settings), physics=physics, ros_domain_id=domain,
        gui_requested=gui, gui_process_alive_at_end=gui_live_at_end,
        elapsed_simulation_s=rows[-1]['time_s'] if rows else 0, raw_ground_truth_samples=raw_count,
        reference_length_m=path.total, final_progress_m=tracker.progress,
        final_endpoint_distance_m=math.dist(final[1:3], path.points[-1]) if final else None,
        initial_pose=list(initial), settled_initial_pose=[rows[0][k] for k in ('x_true_m','y_true_m','yaw_true_rad')] if rows else None,
        post_zero_body_speed_m_s=math.hypot(final[4], final[5]) if final else None,
        post_zero_angular_speed_rad_s=final[6] if final else None,
        tracking_samples=len(tracking),
        optimizer=dict(calls=getattr(control, 'optimization_calls', 0),
            unconverged=getattr(control, 'optimization_unconverged', 0), iterations=getattr(control, 'optimization_iterations', 0)),
        cross_track_true_m=statistics([r['cross_track_true_m'] for r in tracking]),
        cross_track_observed_m=statistics([r['cross_track_observed_m'] for r in tracking]),
        heading_error_true_deg=statistics([math.degrees(r['heading_error_true_rad']) for r in tracking]),
        heading_error_observed_deg=statistics([math.degrees(r['heading_error_observed_rad']) for r in tracking]),
        actuator_linear_request_error_m_s=statistics([r['v_actuator_m_s']-r['v_command_m_s'] for r in tracking]),
        actuator_angular_request_error_rad_s=statistics([r['w_actuator_rad_s']-r['w_command_rad_s'] for r in tracking]),
        linear_execution_error_m_s=statistics([r['v_actual_body_m_s']-r['v_command_m_s'] for r in tracking]),
        angular_execution_error_rad_s=statistics([r['w_actual_rad_s']-r['w_command_rad_s'] for r in tracking]),
        feedback_age_s=statistics([r['feedback_age_s'] for r in rows]),
        control_interval_s=statistics([r['control_interval_s'] for r in rows[1:]]),
        control_deadline_misses=sum(r['control_interval_s'] > 1.5*settings.dt for r in rows[1:]),
        notes=['Gazebo物理真值与带噪/延迟观测分别记录，真值不是实车测量。',
               '四种控制律均来自four_controler/four_controler.py；PP迁移所给脚本，采用速度相关前视和虚拟转角到v/ω的适配，不调用实车PP.command。',
               '控制请求、独立v/ω一阶滞后输出和物理实际速度分别保存；实际速度由Gazebo世界速度投影至base_link。',
               '附加一阶响应位于Gazebo驱动之前，不代替或重复标定Gazebo固有惯性/轮地接触；参数未实车标定。故障/取消直接清零并清除滞后状态。',
               '保留原地图坐标，不归零路径；不发布实车话题或TF。',
               '质量/摩擦/电机参数未实车标定，不承诺2cm精度或实车响应一致。'])
    # Existing exporter supplies the shared CSV/metrics; physical plot replaces its ideal-model labels.
    export = output/'tracking'
    write_outputs(export, poses, rows, metrics)
    physical_report(export, poses, rows, metrics)
    return rows, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=SIM/'config.yaml')
    parser.add_argument('--physics-config', type=Path, default=HERE/'config.yaml')
    parser.add_argument('--path-file', type=Path)
    parser.add_argument('--controller', choices=('all',)+NAMES, default='pp')
    parser.add_argument('--speed', type=float)
    parser.add_argument('--lookahead', type=float)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--domain', type=int)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--no-animation', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.gui and not os.environ.get('DISPLAY'):
        raise ValueError('没有图形桌面DISPLAY；请省略--gui使用无窗口仿真')
    if args.gui and shutil.which('xdpyinfo'):
        try:
            display = subprocess.run(['xdpyinfo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        except subprocess.TimeoutExpired:
            raise ValueError('连接图形桌面超时；请在桌面终端运行，或省略--gui') from None
        if display.returncode:
            raise ValueError('无法连接图形桌面DISPLAY；请在桌面终端运行，或省略--gui使用无窗口仿真')
    config, settings, names, default_path = read_config(args.config)
    overrides = {key: getattr(args, key) for key in ('speed', 'lookahead', 'duration') if getattr(args, key) is not None}
    settings = replace(settings, **overrides)
    values = asdict(settings)
    if any(isinstance(value, bool) or not math.isfinite(value) for value in values.values() if value is not None):
        raise ValueError('仿真配置必须为有限数值')
    if not (.005 <= settings.dt <= .2 and 0 < settings.speed <= 1 and 0 < settings.max_angular <= 1
            and settings.position_noise >= 0 and settings.heading_noise_deg >= 0 and settings.feedback_period >= 0
            and settings.actuator_linear_tau >= 0 and settings.actuator_angular_tau >= 0
            and (settings.duration is None or settings.duration > 0)):
        raise ValueError('控制周期/速度/噪声/时长超出有效范围')
    physics = physical_config(args.physics_config)
    filename = (args.path_file or default_path).resolve()
    poses, source = load_path(filename)
    if source['frame_id'] not in ('map', 'synthetic_local_m', None):
        raise ValueError('Gazebo路径应使用map米制坐标')
    names = names if args.controller == 'all' else [args.controller]
    _, directions = directed_poses(poses)
    if any(n != 'pp' for n in names) and (-1 in directions or point_turn_indices(poses)):
        raise ValueError('点转/倒车路径仅支持PP，请使用--controller pp')
    output = (args.output or SIM/'results'/('gazebo_'+filename.stem+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).resolve()
    if SIM/'results' not in output.parents:
        raise ValueError('Gazebo结果必须保存在tracking_simulation/results/下')
    if output.exists():
        raise ValueError('输出目录已存在，拒绝覆盖')
    output.mkdir(parents=True)
    domain, domain_lock = reserve_domain(args.domain)
    os.environ['ROS_DOMAIN_ID'], os.environ['ROS_LOCALHOST_ONLY'] = str(domain), '1'
    os.environ['ROS_LOG_DIR'] = str(output/'ros_log')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    probe = rclpy.create_node('tracking_sim_isolation_probe')
    try:
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.1)
        others = [(name, ns) for name, ns in probe.get_node_names_and_namespaces() if name != probe.get_name()]
        if others:
            raise RuntimeError('所选Domain存在其他节点，拒绝运行：'+str(others))
        if not probe.context.ok():
            raise RuntimeError('隔离ROS Context不可用')
    finally:
        probe.destroy_node()
    sources = [*HERE.glob('*.py'), HERE/'native/console_path.cpp', HERE/'run.bash', HERE/'CMakeLists.txt',
               SIM/'simulate.py', SIM/'disturbances.py', SIM/'reports.py', SIM/'run_tracking.py', SIM/'four_controler/four_controler.py',
               SIM.parents[0]/'agri_robot_ws/src/agri_path_tracking/agri_path_tracking/controller.py',
               SIM.parents[0]/'agri_robot_ws/src/agri_path_tracking/agri_path_tracking/coverage.py',
               SIM.parents[0]/'agri_robot_ws/src/agri_base/agri_base/coordinates.py',
               SIM.parents[0]/'agri_robot_ws/src/agri_base/agri_base/path_geometry.py',
               SIM.parents[0]/'agri_robot_ws/src/agri_waypoint/agri_waypoint/controller.py',
               SIM.parents[0]/'agri_backend/experiment_analysis.py',
               DESCRIPTION/'urdf/yahboomcar_X3.urdf']
    sources.extend(DESCRIPTION/'meshes'/name for name in
                   ('base_link_X3.STL', 'front_left_wheel_X3.STL', 'front_right_wheel_X3.STL',
                    'back_left_wheel_X3.STL', 'back_right_wheel_X3.STL'))
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (output/'config.yaml').write_text(yaml.safe_dump(dict(controller_config=config, actual_simulation=asdict(settings),
        physics=physics, controllers=names, source=source, domain=domain), allow_unicode=True), encoding='utf-8')
    results, comparisons = {}, {}
    try:
        print(f'Gazebo隔离Domain={domain}，唯一指令话题=/tracking_sim/cmd_vel\n结果：{output}', flush=True)
        for name in names:
            rows, result = run_one(name, poses, source, settings, config['tuning'], physics, output/name, args.gui, domain)
            results[name] = result
            if rows:
                comparisons[name] = (rows, result)
            summary = dict(kind='GAZEBO_ODE_TRACKING', source=source, implementation_sha256=hashes, algorithms=results)
            (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            print(f'{name}: {result["state"]}, 横向RMSE={result["cross_track_true_m"]["rmse"]} m', flush=True)
            if result['state'] in ('CANCELLED', 'GAZEBO_EXITED', 'FEEDBACK_LOST'):
                break
        if hashlib.sha256(filename.read_bytes()).hexdigest() != source['sha256']:
            raise ValueError('运行期间参考路径被修改')
        if comparisons:
            animation = dict(config['animation'])
            if args.no_animation:
                animation['enabled'] = False
            save_comparison(output, poses, comparisons, animation)
        lines = ['# Gazebo 路径跟踪对照', '',
                 '| 控制器 | 状态 | 横向真值RMSE cm | 航向真值RMSE ° | MPC未收敛次数 |',
                 '| --- | --- | ---: | ---: | ---: |']
        for name, result in results.items():
            lateral, heading = result['cross_track_true_m']['rmse'], result['heading_error_true_deg']['rmse']
            lines.append(f'| {name} | {result["state"]} | {None if lateral is None else round(100*lateral, 4)} | {heading} | {result["optimizer"]["unconverged"]} |')
        lines += ['', '![四算法对照](controller_comparison.png)', '',
                  '每个控制器的`tracking/`包含CSV、指标、物理速度与指令对照图及报告。',
                  '这是未标定的Gazebo差速等效物理模型结果，不是实车精度或算法最佳性能排名。', '']
        (output/'report.md').write_text('\n'.join(lines), encoding='utf-8')
    finally:
        rclpy.shutdown()
        domain_lock.close()
    return 0 if len(results) == len(names) and all(r['state'] == 'COMPLETED' and r['optimizer']['unconverged'] == 0 for r in results.values()) else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
