"""Standalone action client for the existing OpenNav coverage server."""

import json
import math
import multiprocessing
import os
import hashlib
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
from pathlib import Path as FilePath
from queue import Empty, Full
from time import monotonic

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates

from agri_base.coordinates import frame_from_node, finite_positive
from agri_base.runtime import LATCHED, run, yaw_of
from .field import load_field, validate_path, validate_native_work_area
from .contour import plan_contour
from .optimizer import optimize as optimize_path
from .saved_path import read_saved_path
from .curves import CURVES, curve_name
from .entry import parse_entry
from .entry_planner import manual_entry_path, contour_entry_path
from agri_base.path_geometry import backward_flags, directed_poses, validate_point_turn_path, motion_signature, rotation_envelope

_WORKER_PROGRESS = None


def optimize_worker(field_wkb, settings, token):
    def progress(value):
        try:
            _WORKER_PROGRESS.put_nowait((token, value))
        except Full:
            pass  # Progress must never block cancellation or planning.
    return optimize_path(field_wkb, settings, progress=progress)


class CoverageAdapter(Node):
    def __init__(self):
        super().__init__('coverage_adapter')
        self.frame = frame_from_node(self)
        defaults = {'boundary_file': '', 'output_directory': '', 'robot_width': 0.2,
                    'robot_length':0.45,'rotation_safety_margin':0.05,'rotation_center_x':0.,'rotation_center_y':0.,
                    'headland_width': 2.0, 'swath_angle': 0.0,
                    'sampling_distance': 0.1, 'planning_timeout': 120.0,
                    'generate_decomp': False, 'route_mode': 'BOUSTROPHEDON',
                    'angle_mode': 'SET_ANGLE', 'path_continuity': 'DUBINS_CC',
                    'operation_width': 0.8, 'min_turning_radius': 1.0,
                    'linear_curv_change': 2.0, 'optimization_config': '{}', 'saved_path_file': '',
                    'allow_reverse': False, 'entry_config': 'null'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('robot_width', 'sampling_distance', 'planning_timeout',
                     'operation_width', 'min_turning_radius', 'linear_curv_change'):
            finite_positive(self.get_parameter(name).value, name)
        finite_positive(self.get_parameter('headland_width').value, 'headland_width', True)
        if not math.isfinite(self.get_parameter('swath_angle').value):
            raise ValueError('Invalid swath_angle')
        self.client = ActionClient(self, ComputeCoveragePath, '/compute_coverage_path')
        self.path_pub = self.create_publisher(Path, '/reference_path', LATCHED)
        self.status = self.create_publisher(String, '/planning/status', LATCHED)
        self.create_subscription(String, '/localization/origin', self.on_origin, LATCHED)
        self.create_subscription(String, '/safety/status', self.on_safety, LATCHED)
        self.create_service(Trigger, '/planning/plan', self.plan)
        self.create_service(Trigger, '/planning/optimize', self.optimize)
        self.create_service(Trigger, '/planning/load', self.load_saved)
        self.create_service(Trigger, '/planning/cancel', self.cancel)
        self.state, self.reason = 'IDLE', ''
        self.origin_matches = False
        self.token = 0
        self.goal_handle = None
        self.deadline = math.inf
        self.contour_executor = None
        self.contour_cancel = None
        self.contour_future = None
        self.contour_token = None
        self.progress_queue = None
        self.planning_progress = None
        self.planning_started = None
        self.plan_metadata = None
        self.entry_reference = None
        self.mode, self.mode_revision, self.safety_time, self.owner = None, None, -math.inf, ''
        self.add_on_set_parameters_callback(self.configure_idle)
        self.create_timer(0.2, self.tick)

    def on_safety(self, msg):
        data = json.loads(msg.data)
        mode, revision = data.get('mode'), data.get('mode_revision')
        if self.mode_revision is not None and revision != self.mode_revision:
            if self.state == 'PLANNING':
                self.cancel(None, Trigger.Response())
            empty = Path()
            empty.header.frame_id = 'map'
            self.path_pub.publish(empty)
        self.mode, self.mode_revision, self.safety_time, self.owner = mode, revision, monotonic(), data.get('owner', '')

    def mode_ready(self):
        return (self.mode == 'coverage' and monotonic() - self.safety_time < 1.0
                and not self.owner and self.count_publishers('/safety/status') == 1)

    def configure_idle(self, parameters):
        if self.state == 'PLANNING' or self.owner:
            return SetParametersResult(successful=False, reason='Cancel active task before configuring planning')
        choices = {'route_mode': ('BOUSTROPHEDON', 'SNAKE', 'SPIRAL', 'CONTOUR'),
                   'angle_mode': ('SET_ANGLE', 'BRUTE_FORCE'),
                   'path_continuity': (*CURVES, 'CONTINUOUS', 'DISCONTINUOUS')}
        for p in parameters:
            if p.name in choices and p.value not in choices[p.name]:
                return SetParametersResult(successful=False, reason='Unsupported planning mode')
            if isinstance(p.value, (int, float)) and not isinstance(p.value, bool):
                if not math.isfinite(p.value) or (p.name not in ('swath_angle','rotation_center_x','rotation_center_y') and (
                        p.value < 0 or (p.value == 0 and p.name not in ('headland_width','rotation_safety_margin')))):
                    return SetParametersResult(successful=False, reason='Invalid planning parameter')
        return SetParametersResult(successful=True)

    def ensure_worker(self):
        if self.contour_executor is None:
            context = multiprocessing.get_context('spawn')
            self.contour_cancel = context.Event()
            self.progress_queue = context.Queue(maxsize=64)
            self.contour_executor = ProcessPoolExecutor(max_workers=1,
                mp_context=context, initializer=contour_worker_setup,
                initargs=(self.contour_cancel,self.progress_queue))

    def entry_settings(self):
        names=('operation_width','headland_width','min_turning_radius','sampling_distance','robot_width',
               'robot_length','rotation_safety_margin','rotation_center_x','rotation_center_y',
               'path_continuity','linear_curv_change','allow_reverse','route_mode')
        return {**{k:self.get_parameter(k).value for k in names},'entry':self.entry_reference,
                'planning_timeout':max(.001,self.deadline-monotonic())}

    def on_origin(self, msg):
        self.origin_matches = json.loads(msg.data) == self.frame.identity

    def plan(self, request, response):
        return self.request_plan(request,response,False)

    def optimize(self, request, response):
        return self.request_plan(request,response,True)

    def load_saved(self, request, response):
        response.success=False
        if self.state=='PLANNING' or not self.origin_matches or not self.mode_ready() or (
                self.contour_future is not None and not self.contour_future.done()):
            response.message='请先选择覆盖模式并等待所有控制/规划空闲，确认固定原点一致'
            return response
        try:
            polygon=load_field(self.get_parameter('boundary_file').value,self.frame)
            data,points=read_saved_path(self.get_parameter('saved_path_file').value,self.frame,polygon,
                self.get_parameter('robot_width').value,self.get_parameter('allow_reverse').value,
                expected_envelope=rotation_envelope(*[self.get_parameter(k).value for k in (
                    'robot_length','robot_width','rotation_safety_margin','rotation_center_x','rotation_center_y')]),
                expected_entry=parse_entry(json.loads(self.get_parameter('entry_config').value),self.frame))
            if not self.origin_matches or not self.mode_ready():raise ValueError('加载期间原点或控制状态改变')
            self.token+=1
            self.planning_progress = None
            path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
            for x,y,yaw in points:
                pose=PoseStamped();pose.header=path.header
                pose.pose.position.x=float(x);pose.pose.position.y=float(y)
                pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
                path.poses.append(pose)
            self.plan_metadata=data.get('planning')
            self.save(path,data.get('velocities',[]),backward_flags(points))
            self.path_pub.publish(path)
            self.state,self.reason='SUCCEEDED',f'已加载保存路径：{len(points)}点；尚未开始行驶'
            response.success,response.message=True,self.reason
        except (ValueError,OSError,KeyError,TypeError) as exc:
            self.state,self.reason='FAILED',str(exc);response.message=str(exc)
        return response

    def request_plan(self, request, response, smart):
        response.success = False
        if self.contour_future is not None and not self.contour_future.done():
            response.message = '上一轮回形计算仍在退出，请稍后重试'
            return response
        contour = self.get_parameter('route_mode').value == 'CONTOUR'
        if self.state == 'PLANNING' or not self.origin_matches or (not (contour or smart) and not self.client.server_is_ready()) or not self.mode_ready():
            response.message = 'Select coverage mode; require idle controls, matching origin and available planner'
            return response
        try:
            self.polygon = load_field(self.get_parameter('boundary_file').value, self.frame)
            self.plan_metadata = None
            self.entry_reference = parse_entry(json.loads(self.get_parameter('entry_config').value),self.frame)
            self.planning_progress = None
            self.planning_started = monotonic()
            if contour or smart:
                self.token += 1
                self.contour_token = self.token
                self.state, self.reason = 'PLANNING', '正在搜索往复式作业角度和行顺序（固定原地转向）' if smart else '正在由内向外生成连续回形路径'
                self.deadline = monotonic() + self.get_parameter('planning_timeout').value
                empty = Path(); empty.header.frame_id = 'map'; self.path_pub.publish(empty)
                self.ensure_worker()
                self.contour_cancel.clear()
                if smart:
                    settings=json.loads(self.get_parameter('optimization_config').value)
                    if not isinstance(settings,dict):raise ValueError('智能规划配置必须为对象')
                    settings['entry']=self.entry_reference
                    self.planning_progress={'phase':'INITIALIZING','evaluated_count':0,'feasible_count':0,
                        'stage_completed':0,'stage_total':0,'elapsed_seconds':0.}
                    self.contour_future=self.contour_executor.submit(optimize_worker,self.polygon.wkb,settings,self.token)
                elif self.entry_reference is not None:
                    self.contour_future = self.contour_executor.submit(contour_entry_path,self.polygon.wkb,self.entry_settings())
                else:
                    self.contour_future = self.contour_executor.submit(plan_contour,self.polygon.wkb,
                        width=self.get_parameter('operation_width').value,
                        headland=self.get_parameter('headland_width').value,
                        radius=self.get_parameter('min_turning_radius').value,
                        spacing=self.get_parameter('sampling_distance').value,
                        robot_width=self.get_parameter('robot_width').value,
                        continuity=self.get_parameter('path_continuity').value,
                        curvature_change=self.get_parameter('linear_curv_change').value,
                        allow_reverse=self.get_parameter('allow_reverse').value,
                        timeout=self.get_parameter('planning_timeout').value)
                response.success, response.message = True, 'PLANNING; inspect /planning/status'
                return response
            # Invalidate the preceding preview even if the new geometry is
            # rejected before sending an action to OpenNav.
            empty = Path(); empty.header.frame_id = 'map'; self.path_pub.publish(empty)
            validate_native_work_area(self.polygon, self.get_parameter('headland_width').value)
            goal = ComputeCoveragePath.Goal()
            goal.frame_id = 'map'
            goal.generate_decomp = self.get_parameter('generate_decomp').value
            goal.generate_headland = self.get_parameter('headland_width').value > 0
            goal.generate_route = self.entry_reference is None
            goal.generate_path = self.entry_reference is None
            goal.headland_mode.mode = 'CONSTANT'
            goal.headland_mode.width = float(self.get_parameter('headland_width').value)
            goal.swath_mode.mode, goal.swath_mode.objective = self.get_parameter('angle_mode').value, 'LENGTH'
            goal.swath_mode.best_angle = float(self.get_parameter('swath_angle').value)
            goal.route_mode.mode = 'TSP' if goal.generate_decomp else self.get_parameter('route_mode').value
            selected_curve=curve_name(self.get_parameter('path_continuity').value)
            _,goal.path_mode.mode,goal.path_mode.continuity_mode=CURVES[selected_curve]
            self.plan_metadata={'curve_type':selected_curve,'allow_reverse':self.get_parameter('allow_reverse').value}
            goal.path_mode.turn_point_distance = float(self.get_parameter('sampling_distance').value)
            rings = [self.polygon.exterior, *self.polygon.interiors]
            for ring in rings:
                coords = Coordinates()
                coords.coordinates = [Coordinate(axis1=float(x), axis2=float(y)) for x, y in ring.coords]
                goal.polygons.append(coords)
            self.token += 1
            token = self.token
            self.state, self.reason = 'PLANNING', ''
            self.deadline = monotonic() + self.get_parameter('planning_timeout').value
            future = self.client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.on_goal(f, token))
            response.success, response.message = True, 'PLANNING; inspect /planning/status'
        except (ValueError, OSError, KeyError, TypeError, RuntimeError) as exc:
            self.state, self.reason = 'FAILED', str(exc)
            response.message = str(exc)
        return response

    def on_goal(self, future, token):
        try:
            handle = future.result()
            if token != self.token:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                raise ValueError('Coverage goal rejected')
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.on_result(f, token))
        except Exception as exc:
            if token == self.token:
                self.state, self.reason = 'FAILED', str(exc)

    def on_result(self, future, token):
        if token != self.token:
            return
        self.goal_handle = None
        try:
            wrapped = future.result()
            result = wrapped.result
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
                raise ValueError(f'Coverage action failed: {result.error_code}')
            if self.entry_reference is not None:
                if result.coverage_path.header.frame_id != 'map':
                    raise ValueError('OpenNav作业行坐标系不是map')
                rows=[((float(s.start.x),float(s.start.y)),(float(s.end.x),float(s.end.y))) for s in result.coverage_path.swaths]
                if not rows or len(rows)>1500 or any(not all(math.isfinite(v) and abs(v)<=self.frame.max_radius for pt in row for v in pt) for row in rows):
                    raise ValueError('OpenNav入口作业行为空、超限或坐标异常')
                self.ensure_worker();self.contour_cancel.clear();self.contour_token=token
                self.contour_future=self.contour_executor.submit(manual_entry_path,self.polygon.wkb,rows,self.entry_settings())
                return
            path = result.nav_path
            if path.header.frame_id != 'map':
                raise ValueError('Planner returned a different coordinate frame')
            backward = getattr(result.coverage_path, 'is_backward', [])
            velocities = getattr(result.coverage_path, 'velocities', [])
            allow_reverse=self.get_parameter('allow_reverse').value
            if (any(backward) or any(v < 0 for v in velocities)) and not allow_reverse:
                raise ValueError('规划含倒车段，请勾选允许倒车后重新规划')
            points = []
            for pose in path.poses:
                # Local OpenNav toMsg(PathState) leaves individual headers empty.
                # Inherit the validated Path header; reject explicit contradictions.
                if pose.header.frame_id not in ('', 'map'):
                    raise ValueError('Mixed-frame path')
                pose.header = path.header
                yaw_of(pose.pose.orientation)
                x, y = pose.pose.position.x, pose.pose.position.y
                if not math.isfinite(x + y) or math.hypot(x, y) > self.frame.max_radius:
                    raise ValueError('Path outside local operating radius')
                points.append((x, y))
            validate_path(points, self.polygon, self.get_parameter('robot_width').value)
            poses=[(p.pose.position.x,p.pose.position.y,yaw_of(p.pose.orientation)) for p in path.poses]
            _,directions=directed_poses(poses)
            if any(d<0 for d in directions) and not allow_reverse:
                raise ValueError('参考路径包含反向运动，尚未允许倒车')
            if not self.origin_matches or not self.mode_ready():
                raise ValueError('Coordinate origin changed during planning')
            self.save(path, velocities, backward_flags(poses))
            self.path_pub.publish(path)
            self.state, self.reason = 'SUCCEEDED', f'{len(points)} path points'
        except Exception as exc:
            self.state, self.reason = 'FAILED', str(exc)

    def save(self, path, velocities, backward):
        folder = self.get_parameter('output_directory').value
        if not folder:
            return
        target = FilePath(folder).expanduser()
        target.mkdir(parents=True, exist_ok=True)
        filename = datetime.now().strftime('coverage_%Y%m%d_%H%M%S_%f.json')
        data = {'frame_id': 'map', 'projection': self.frame.identity,
                'velocities': list(velocities), 'is_backward': list(backward),
                'points': [{'x': p.pose.position.x, 'y': p.pose.position.y,
                            'yaw': yaw_of(p.pose.orientation)} for p in path.poses]}
        if self.plan_metadata is not None:
            data['planning'] = self.plan_metadata
            if self.plan_metadata.get('turn_mode')=='POINT_TURN':
                data['motion_signature']=motion_signature([(p['x'],p['y'],p['yaw']) for p in data['points']])
        with (target / filename).open('x', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)

    def cancel(self, request, response):
        self.token += 1
        if self.contour_future is not None:
            if self.contour_cancel is not None:self.contour_cancel.set()
            self.contour_future.cancel()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.state, self.reason = 'CANCELED', ''
        if self.planning_progress is not None:self.planning_progress['phase']='CANCELED'
        response.success, response.message = True, self.state
        return response

    def tick(self):
        if self.progress_queue is not None:
            for _ in range(64):
                try:token,progress=self.progress_queue.get_nowait()
                except Empty:break
                if token==self.token and self.state=='PLANNING':self.planning_progress=progress
        if self.state == 'PLANNING' and (monotonic() > self.deadline or not self.mode_ready()):
            self.cancel(None, Trigger.Response())
            self.state, self.reason = 'FAILED', 'Planning timeout or work mode/safety heartbeat changed'
        if self.contour_future is not None and self.contour_future.done():
            future, token = self.contour_future, self.contour_token
            self.contour_future = None
            if token == self.token and self.state == 'PLANNING':
                try:
                    value = future.result()
                    report=value.get('optimization')
                    if report is not None:
                        self.planning_progress={**(self.planning_progress or {}),
                            'evaluated_count':report['evaluated_count'],
                            'feasible_count':sum(c.get('executable',False) for c in report['candidates']),
                            'phase':'FINALIZING'}
                    points = value['points']
                    if not self.origin_matches or not self.mode_ready():
                        raise ValueError('规划期间原点/控制状态改变')
                    if value.get('optimization') is not None and not points:
                        self.save_optimization(value)
                        raise ValueError(value['optimization']['reason'])
                    if not all(math.isfinite(sum(p[:3])) and math.hypot(*p[:2])<=self.frame.max_radius for p in points):
                        raise ValueError('回形路径超出统一局部坐标范围')
                    validate_path([p[:2] for p in points],self.polygon,self.get_parameter('robot_width').value)
                    if value['metadata'].get('turn_mode')=='POINT_TURN':
                        validate_point_turn_path(points,self.polygon,value['metadata']['rotation_envelope'])
                    _,directions=directed_poses(points)
                    if any(d<0 for d in directions) and not self.get_parameter('allow_reverse').value:
                        raise ValueError('规划包含倒车段，请勾选允许倒车后重新规划')
                    if not self.origin_matches or not self.mode_ready():
                        raise ValueError('回形规划期间原点/控制状态改变')
                    path = Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
                    for point in points:
                        x,y,yaw=point[:3]
                        pose=PoseStamped();pose.header=path.header
                        pose.pose.position.x=float(x);pose.pose.position.y=float(y)
                        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
                        path.poses.append(pose)
                    self.plan_metadata=value['metadata']
                    self.save(path,[],backward_flags(points))
                    if value.get('optimization') is not None:self.save_optimization(value)
                    self.path_pub.publish(path)
                    self.state='SUCCEEDED'
                    self.reason=(f'智能规划完成：{value["metadata"]["mode"]}，{len(points)}点' if value.get('optimization') is not None
                        else f'入口覆盖规划完成：{len(points)}点' if value['metadata'].get('entry') and value['metadata']['mode']!='CONTOUR'
                        else f'连续回形由内向外：{value["metadata"]["ring_count"]}层，{len(points)}点')
                except Exception as exc:
                    self.state,self.reason='FAILED',str(exc)
        if self.planning_progress is not None:
            if self.state=='PLANNING':
                self.planning_progress['elapsed_seconds']=monotonic()-self.planning_started
            elif self.state in ('FAILED','CANCELED','SUCCEEDED'):
                self.planning_progress['phase']='COMPLETE' if self.state=='SUCCEEDED' else self.state
                if self.planning_started is not None:
                    self.planning_progress['elapsed_seconds']=monotonic()-self.planning_started
                self.planning_started=None
        self.status.publish(String(data=json.dumps({'state': self.state, 'reason': self.reason,
            'search_progress': self.planning_progress,
            'output_directory': self.get_parameter('output_directory').value})))

    def save_optimization(self,value):
        folder=self.get_parameter('output_directory').value
        if not folder:return
        target=FilePath(folder);target.mkdir(parents=True,exist_ok=True)
        report={**value['optimization'],'frame_id':'map','projection':self.frame.identity}
        if value['points']:
            xy=[{'x':float(p[0]),'y':float(p[1])} for p in value['points']]
            report['reference_fingerprint']=hashlib.sha256(json.dumps(['map',xy],sort_keys=True).encode()).hexdigest()
        temp=target/f'.optimization_{self.token}.tmp'
        try:
            temp.write_text(json.dumps(report,ensure_ascii=False,allow_nan=False),encoding='utf-8')
            temp.replace(target/'optimization.json')
        finally:temp.unlink(missing_ok=True)

    def destroy_node(self):
        if self.contour_executor is not None:
            self.contour_cancel.set()
            self.contour_executor.shutdown(wait=False,cancel_futures=True)
        return super().destroy_node()


def contour_worker_setup(cancel, progress_queue):
    global _WORKER_PROGRESS
    _WORKER_PROGRESS=progress_queue
    from .contour import set_cancellation
    set_cancellation(cancel)
    os.environ.setdefault('MPLCONFIGDIR',str(FilePath(os.environ.get('TMPDIR',str(FilePath.cwd()/'.runtime/tmp')))/'matplotlib'))
    if hasattr(os,'nice'):
        os.nice(10)


def main(args=None):
    run(CoverageAdapter, args)
