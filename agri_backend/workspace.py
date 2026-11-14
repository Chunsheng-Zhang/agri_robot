"""Web mission operations: validated files -> ROS services -> observed status."""
import hashlib
import asyncio
import json
import tempfile
import zipfile
import logging
from contextlib import contextmanager
from pathlib import Path
from threading import Lock
from time import monotonic
from typing import Literal

import yaml
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from pydantic import Field, ValidationError
from starlette.background import BackgroundTask

from .devices import device_cache, service_call
from .missions import Mission, MissionStore, StrictModel, WaypointParameters, CoverageParameters, atomic_json, now, DELETABLE_RUN_STATES
from .coverage_quality_jobs import PlanQualityJobs, snapshot as quality_snapshot
from .coverage_map import CoverageMaps
from .saved_plans import SavedPlans, read_plan


class ManualCommand(StrictModel):
    token: str = Field(pattern='^[0-9a-f]{32}$')
    seq: int = Field(ge=0)
    linear: float = Field(ge=-1, le=1)
    angular: float = Field(ge=-2, le=2)
    expires_at: float


class StopCommand(StrictModel):
    token: str = Field(pattern='^[0-9a-f]{32}$')


class ModeCommand(StrictModel):
    mode: Literal['base', 'waypoint', 'coverage']


class Workspace:
    def __init__(self, require_bridge, robot_snapshot, store=None):
        self.require_bridge, self.robot_snapshot = require_bridge, robot_snapshot
        self.store = store or MissionStore()
        self.lock = Lock()
        self.active = None
        self.plan_sequence = None
        self.plan_requested = None
        self.quality_jobs = PlanQualityJobs()
        self.coverage_maps = CoverageMaps()
        self.saved_plans = SavedPlans()

    def gateway(self):
        node = self.require_bridge().node
        if not hasattr(node, 'workspace_ros'):
            raise HTTPException(503, '请重启更新后的 FastAPI 后端')
        return node.workspace_ros

    @contextmanager
    def operation(self):
        if not self.lock.acquire(blocking=False):
            raise HTTPException(409, '另一个任务操作正在执行，请等待状态确认')
        try:
            yield
        except FileNotFoundError as exc:
            raise HTTPException(404, '任务或数据文件不存在') from exc
        except (ValueError, TypeError, KeyError) as exc:
            raise HTTPException(422, str(exc)) from exc
        finally:
            self.lock.release()

    def origin(self):
        cached = device_cache.snapshot().get('origin', {})
        value = cached.get('projection')
        if value:
            return value, 'active_ros'
        return self.store.configured_origin(), 'configured_yaml'

    def require_idle(self, mode=None):
        snapshot = device_cache.snapshot()
        safety = snapshot['safety']
        if not safety['fresh'] or safety.get('mode') not in ('base', 'waypoint', 'coverage'):
            raise HTTPException(503, '安全状态未更新，请启动新版 ROS 核心')
        if mode and safety['mode'] != mode:
            raise HTTPException(409, '请先选择与任务对应的工作模式')
        if safety.get('owner'):
            raise HTTPException(409, '已有控制任务，请先取消任务或释放手柄/遥控')
        for key in ('waypoint', 'tracking', 'planning'):
            if snapshot[key]['fresh'] and snapshot[key].get('state') in ('ACQUIRING', 'RUNNING', 'PAUSED', 'PLANNING'):
                raise HTTPException(409, '请先取消现有任务并等待停止确认')
        return safety

    def event(self, kind, **details):
        from .diagnostics import runtime_diagnostics
        runtime_diagnostics.log('INFO', 'web_task', kind + ' ' + json.dumps(details, ensure_ascii=False), channel='web_event')
        path = self.store.root / 'data/web/events.jsonl'
        path.parent.mkdir(parents=True, exist_ok=True)
        value = {'time': now(), 'event': kind, 'channel': 'web', **details}
        device_cache.add_event(value)
        try:
            with path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
        except OSError as exc:
            logging.getLogger(__name__).warning('Web event log unavailable: %s', exc)

    def sync(self):
        snapshot, data = device_cache.snapshot(), self.robot_snapshot()
        if not self.active:
            # Reconnect observations after backend restart; never restart a controller here.
            for candidate in self.store.list_runs():
                source = 'waypoint' if candidate['mode'] == 'waypoint' else 'tracking'
                status = snapshot[source] if candidate['stage'] == 'execution' else snapshot['planning']
                expected = self.store.run_path(candidate['id']) / ('tracks' if candidate['stage'] == 'execution' else 'paths')
                actual = status.get('run_directory' if candidate['stage'] == 'execution' else 'output_directory')
                if not status['fresh'] or actual != str(expected):
                    continue
                if candidate['stage'] == 'planning':
                    files = list(expected.glob('coverage_*.json'))
                    if files:
                        saved = json.loads(files[-1].read_text())
                        points = [{'x': p['x'], 'y': p['y']} for p in saved['points']]
                        signature = hashlib.sha256(json.dumps(['map', points], sort_keys=True).encode()).hexdigest()
                        if saved.get('projection') != candidate['projection'] or signature != data.get('reference_fingerprint'):
                            continue
                        candidate['reference_fingerprint'] = signature
                    else:
                        continue
                self.active = candidate
                candidate['_requested'] = -float('inf')
                self.plan_requested, self.plan_sequence = -float('inf'), -1
                break
            if not self.active:
                return
        safety = snapshot['safety']
        run = self.active
        if run['state'] in ('COMPLETED', 'CANCELED', 'FAILED', 'FAULT'):
            return  # Later mode changes must not rewrite a completed experiment's outcome.
        if safety['fresh'] and safety.get('mode_revision') != run['mode_revision']:
            state, reason = 'CANCELED', '工作模式已切换，需明确重新规划/开始'
        elif run['stage'] == 'planning':
            status = snapshot['planning']
            received = monotonic() - (status.get('age_seconds') or 0)
            if not status['fresh'] or received < (self.plan_requested if self.plan_requested is not None else monotonic()):
                return
            state, reason = status.get('state', 'UNKNOWN'), status.get('reason', '')
            if state == 'SUCCEEDED':
                if data.get('reference_sequence', 0) <= (self.plan_sequence or 0) or not data.get('reference_path'):
                    if run['state'] == 'READY':
                        run['state'], run['reason'] = 'WAITING_PATH', '参考路径已失效，等待本次规划路径'
                        self.store.save_run(run)
                    return
                files = list((self.store.run_path(run['id']) / 'paths').glob('coverage_*.json'))
                if not files:
                    return
                saved = json.loads(files[-1].read_text())
                points = [{'x': p['x'], 'y': p['y']} for p in saved['points']]
                signature = hashlib.sha256(json.dumps(['map', points], sort_keys=True).encode()).hexdigest()
                if saved.get('projection') != run['projection'] or signature != data.get('reference_fingerprint'):
                    state, reason = 'WAITING_PATH', '等待本次规划的参考路径，尚未允许开始'
                else:
                    run['reference_fingerprint'] = signature
                    state = 'READY'
                    if run.get('coverage_quality_version') == 1:
                        try:
                            self.quality_jobs.submit(self.store.run_path(run['id']))
                        except (OSError, ValueError, RuntimeError) as exc:
                            # Diagnostics must not change the validated path or
                            # motion permission. Report the failure separately.
                            run['coverage_quality_error'] = str(exc)
        else:
            status = snapshot['waypoint' if run['mode'] == 'waypoint' else 'tracking']
            if not status['fresh']:
                return
            received = monotonic() - (status.get('age_seconds') or 0)
            if received < run.get('_requested', monotonic()):
                return
            state, reason = status.get('state', 'UNKNOWN'), status.get('reason', '')
        if state != run['state'] or reason != run['reason']:
            run['state'], run['reason'] = state, reason
            if run['stage'] == 'execution':
                run['elapsed_seconds'] = snapshot['waypoint' if run['mode'] == 'waypoint' else 'tracking'].get('elapsed_seconds')
            self.store.save_run(run)

    def context(self):
        # Polling must not wait behind a long lifecycle transition.
        if self.lock.acquire(blocking=False):
            try:
                self.sync()
            finally:
                self.lock.release()
        origin, source = self.origin()
        active = {k: v for k, v in (self.active or {}).items() if not k.startswith('_')}
        if active:
            active['coverage_quality'] = quality_snapshot(self.store.run_path(active['id']),active)
            if active.get('planning_strategy') in ('SMART','REUSED'):
                from .optimization_results import summary
                active['optimization']=summary(self.store.run_path(active['id']))
        missions=self.store.list()
        plans=self.saved_plans.summaries(self.store,missions)
        for mission in missions:
            if mission['mode']=='coverage':mission['coverage_plans']=plans.get(mission['id'],[])
        return {'origin': origin, 'origin_source': source, 'defaults': self.store.defaults(),
                'synthetic': getattr(self, 'synthetic', False),
                'missions': missions, 'runs': self.store.list_runs(), 'active_run': active or None,
                'config_file': str(self.store.config_path),
                'events': device_cache.recent_events(),
                'mode': device_cache.snapshot()['safety'].get('mode')}

    def select_mode(self, mode):
        # Mode changes and estop remain available even if a mission operation is waiting on ROS.
        gateway = self.gateway()
        gateway.stop()
        result = gateway.trigger(f'/system/mode/{mode}')
        self.event('MODE_SELECTED', mode=mode)
        return result

    def validate_mission(self, identity, mode):
        mission = self.store.get(identity)
        origin, source = self.origin()
        if source != 'active_ros' or origin != mission['projection']:
            raise HTTPException(409, '任务与 ROS 当前固定原点不一致，请核对并重新保存任务')
        if mission['mode'] != mode:
            raise HTTPException(409, '任务类型不匹配')
        mission['parameters'] = (WaypointParameters if mode == 'waypoint' else CoverageParameters)(
            **mission['parameters']).model_dump()
        self.store.preview(Mission(**{k: mission[k] for k in ('name', 'mode', 'parameters', 'waypoints', 'boundary')}), origin)
        return mission

    def plan(self, identity, optimized=False):
        with self.operation():
            safety = self.require_idle('coverage')
            mission = self.validate_mission(identity, 'coverage')
            gateway = self.gateway()
            run = self.store.create_run(mission, safety['mode_revision'], 'planning')
            # New planning rounds opt in; never backfill archived experiments.
            run['coverage_quality_version'] = 1
            run['plan_save_required'] = True
            run['planning_strategy'] = 'SMART' if optimized else 'MANUAL'
            if optimized:run['planning_policy']='POINT_TURN_ANGLE'
            self.store.save_run(run)
            directory = self.store.run_path(run['id'])
            boundary = directory / 'config/field.json'
            atomic_json(boundary, mission['boundary'])
            p = mission['parameters']
            self.active = run
            try:
                if not optimized:gateway.configure_server({
                    'robot_width': float(p['robot_width']),
                    'operation_width': float(p['operation_width']), 'min_turning_radius': float(p['min_turning_radius']),
                    'default_turn_point_distance': float(p['sampling_distance'])})
                if p['route_mode'] == 'CONTOUR' and not optimized:
                    curve = gateway.get_parameters('coverage_server', ['linear_curv_change'])
                    gateway.parameters('coverage_adapter', {'operation_width':float(p['operation_width']),
                        'min_turning_radius':float(p['min_turning_radius']),
                        'linear_curv_change':float(curve['linear_curv_change'])})
                if optimized:
                    settings={k:p[k] for k in ('operation_width','robot_width','robot_length',
                        'rotation_safety_margin','rotation_center_x','rotation_center_y',
                        'sampling_distance','quality_resolution')}
                    gateway.parameters('coverage_adapter',{'optimization_config':json.dumps(settings,allow_nan=False)})
                gateway.parameters('coverage_adapter', {**{k: p[k] for k in (
                    'robot_width','robot_length','rotation_safety_margin','rotation_center_x','rotation_center_y',
                    'headland_width', 'swath_angle', 'sampling_distance', 'route_mode', 'angle_mode', 'path_continuity')},
                    'allow_reverse':False if optimized else bool(p.get('allow_reverse',False)),
                    'boundary_file': str(boundary), 'output_directory': str(directory / 'paths'), 'generate_decomp': False})
                self.plan_sequence = self.robot_snapshot().get('reference_sequence', 0)
                self.plan_requested = monotonic()
                result = gateway.trigger('/planning/optimize' if optimized else '/planning/plan')
                run['state'] = 'PLANNING'
                self.store.save_run(run)
                self.event('PLAN_REQUESTED', run_id=run['id'], mission_id=identity)
                return {**result, 'run': run}
            except Exception as exc:
                run['state'], run['reason'] = 'FAILED', str(getattr(exc, 'detail', exc))
                self.store.save_run(run)
                raise

    def save_plan(self, identity, plan_id):
        # Persist a validated plan in the reusable list. No ROS calls or control
        # ownership are needed: saving is separate from loading and starting.
        with self.operation():
            mission=self.store.get(identity)
            if mission.get('mode')!='coverage':raise HTTPException(422,'只有覆盖任务可以保存规划路径')
            if mission['projection']!=self.origin()[0]:raise HTTPException(409,'当前固定原点与任务不一致')
            run,_,_,fingerprint=read_plan(self.store,mission,plan_id,allow_pending_save=True)
            if run.get('saved_plan_version')==1:
                return {'success':True,'already_saved':True,'run':run}
            if run.get('stage')!='planning' or run.get('state')!='READY':
                raise HTTPException(409,'请等待当前规划成功就绪，再保存规划路径')
            run.update(saved_plan_version=1,plan_save_required=False,plan_saved_at=now(),reference_fingerprint=fingerprint)
            self.store.save_run(run)
            if self.active and self.active.get('id')==plan_id:self.active.update(run)
            self.saved_plans.updated=-float('inf')
            self.event('PLANNED_PATH_SAVED',run_id=plan_id,mission_id=identity)
            return {'success':True,'already_saved':False,'run':run}

    def load_plan(self, identity, plan_id):
        with self.operation():
            safety=self.require_idle('coverage')
            mission=self.validate_mission(identity,'coverage')
            source,saved,points,fingerprint=read_plan(self.store,mission,plan_id)
            run=self.store.create_run(mission,safety['mode_revision'],'planning')
            run.update(coverage_quality_version=1,saved_plan_version=1,source_plan_id=plan_id,planning_strategy='REUSED')
            if source.get('planning_policy'):run['planning_policy']=source['planning_policy']
            self.store.save_run(run)
            directory=self.store.run_path(run['id'])
            self.active=run
            try:
                atomic_json(directory/'config/field.json',mission['boundary'])
                atomic_json(directory/'config/saved_path.json',saved)
                # Keep the original decision/model as provenance, not a new
                # optimization. All new telemetry goes to this new round.
                if source.get('planning_strategy')=='SMART':
                    from .optimization_results import read as read_optimization
                    try:report=read_optimization(self.store.run_path(plan_id))
                    except (OSError,ValueError,TypeError):report=None
                    if report and report.get('reference_fingerprint')==fingerprint:
                        atomic_json(directory/'paths/optimization.json',{**report,'reused_from':plan_id})
                gateway=self.gateway()
                gateway.parameters('coverage_adapter',{'boundary_file':str(directory/'config/field.json'),
                    'output_directory':str(directory/'paths'),'saved_path_file':str(directory/'config/saved_path.json'),
                    'robot_width':float(mission['parameters']['robot_width']),
                    **{k:float(mission['parameters'][k]) for k in ('robot_length','rotation_safety_margin','rotation_center_x','rotation_center_y')},
                    'allow_reverse':bool(mission['parameters'].get('allow_reverse',False))})
                self.plan_sequence=self.robot_snapshot().get('reference_sequence',0)
                self.plan_requested=monotonic()
                result=gateway.trigger('/planning/load')
                run['state']='PLANNING';self.store.save_run(run)
                self.event('SAVED_PATH_LOADED',run_id=run['id'],source_plan_id=plan_id)
                return {**result,'run':run}
            except Exception as exc:
                run['state'],run['reason']='FAILED',str(getattr(exc,'detail',exc))
                self.store.save_run(run)
                raise

    def start(self, identity):
        with self.operation():
            mission = self.store.get(identity)
            safety = self.require_idle(mission['mode'])
            mission = self.validate_mission(identity, mission['mode'])
            if safety.get('motion_inhibited'):
                raise HTTPException(409, '急停或障碍停车尚未解除，任务不会开始')
            gateway, p = self.gateway(), mission['parameters']
            if p.get('max_linear', p.get('speed', 0)) > safety.get('limits', {}).get('linear', 0):
                raise HTTPException(409, '任务速度超过 ROS 安全限速，请降低任务速度')
            if p['max_angular'] > safety.get('limits', {}).get('angular', 0):
                raise HTTPException(409, '任务角速度超过 ROS 安全限速')
            if p.get('max_path_error', 0) > safety.get('limits', {}).get('path_error', 0):
                raise HTTPException(409, '路径偏离阈值超过安全模块上限')
            if mission['mode'] == 'coverage':
                self.sync()
                run = self.active
                data = self.robot_snapshot()
                if not run or run['stage'] != 'planning' or run['state'] != 'READY' or any((
                    run['mission_id'] != identity, run['mission_revision'] != mission['revision'],
                    run['mode_revision'] != safety['mode_revision'],
                    run.get('reference_fingerprint') != data.get('reference_fingerprint'))):
                    raise HTTPException(409, '请先为当前任务成功规划，修改任务/模式/参考路径后须重新规划')
                directory = self.store.run_path(run['id'])
                if p.get('allow_reverse',False) or run.get('planning_policy')=='POINT_TURN_ANGLE':
                    from agri_base.path_geometry import motion_signature
                    files=list((directory/'paths').glob('coverage_*.json'))
                    if len(files)!=1:raise HTTPException(409,'缺少唯一的带方向覆盖路径，请重新规划')
                    saved=json.loads(files[0].read_text())
                    expected=motion_signature([(p['x'],p['y'],p['yaw']) for p in saved['points']])
                    if expected!=data.get('reference_motion_fingerprint'):
                        raise HTTPException(409,'参考路径航向或方向不一致，请重新加载/规划后开始')
                tracker = {k: p[k] for k in ('lookahead', 'speed', 'max_angular', 'completion_tolerance',
                                            'start_tolerance', 'search_window', 'max_path_error')}
                gateway.parameters('path_tracker', {**tracker, 'allow_reverse':bool(p.get('allow_reverse',False)),
                    'trajectory_directory': str(directory / 'tracks')})
                source = 'tracking'
            else:
                run = self.store.create_run(mission, safety['mode_revision'], 'execution')
                directory = self.store.run_path(run['id'])
                filename = directory / 'config/waypoints.yaml'
                filename.write_text(yaml.safe_dump({'waypoints': mission['waypoints']}, allow_unicode=True), encoding='utf-8')
                self.active = run
                try:
                    gateway.parameters('waypoint', {**p, 'waypoints_file': str(filename),
                                                    'trajectory_directory': str(directory / 'tracks')})
                except Exception as exc:
                    run['state'], run['reason'] = 'FAILED', str(getattr(exc, 'detail', exc))
                    self.store.save_run(run)
                    raise
                source = 'waypoint'
            # Safety checks the mode again atomically when granting the lease.
            requested = monotonic()
            capture_started = False
            try:
                # Open task-scoped capture before any controller can acquire motion.
                try:
                    gateway.recorder.begin(directory, source)
                    capture_started = True
                except (OSError, RuntimeError) as exc:
                    raise HTTPException(503, f'实验记录器尚未就绪，任务未开始：{exc}') from exc
                requested = monotonic()
                result = service_call(self.require_bridge().node, f'/{source}/start')
            except Exception as exc:
                # Only explicit rejection proves the task was not accepted.
                # Transport errors/timeouts may occur after acceptance; keep
                # capture until observed status closes it, never assume a stop.
                if capture_started and isinstance(exc, HTTPException) and exc.status_code == 409:
                    gateway.recorder.abort(str(getattr(exc, 'detail', exc)))
                run['reason'] = str(getattr(exc, 'detail', exc))
                if mission['mode'] == 'waypoint':
                    run['state'] = 'FAILED'
                self.store.save_run(run)
                self.event('START_REJECTED', run_id=run['id'], reason=run['reason'])
                raise
            run['stage'], run['state'], run['_requested'] = 'execution', 'ACQUIRING', requested
            self.active = run
            self.store.save_run(run)
            self.event('START_REQUESTED', run_id=run['id'], mission_id=identity)
            return {**result, 'run_id': run['id']}


def install_routes(app, require_bridge, robot_snapshot):
    workspace = Workspace(require_bridge, robot_snapshot)
    app.state.workspace = workspace
    router = APIRouter()

    @router.get('/api/workspace')
    def context():
        return workspace.context()

    @router.post('/api/system/mode')
    def select_mode(data: ModeCommand):
        return workspace.select_mode(data.mode)

    @router.post('/api/missions/preview')
    def preview(data: Mission):
        try:
            origin, source = workspace.origin()
            return {**workspace.store.preview(data, origin), 'projection': origin, 'origin_source': source}
        except (ValueError, TypeError, KeyError) as exc:
            raise HTTPException(422, str(exc)) from exc

    @router.post('/api/missions')
    def create(data: Mission):
        with workspace.operation():
            return workspace.store.save(data, workspace.origin()[0])

    @router.put('/api/missions/{identity}')
    def update(identity: str, data: Mission):
        with workspace.operation():
            workspace.sync()
            run = workspace.active
            if run and run['mission_id'] == identity and run['state'] in ('ACQUIRING', 'RUNNING', 'PAUSED', 'PLANNING'):
                raise HTTPException(409, '请先取消任务再修改')
            return workspace.store.save(data, workspace.origin()[0], identity)

    @router.delete('/api/missions/{identity}')
    def delete(identity: str):
        with workspace.operation():
            workspace.sync()
            run = workspace.active
            if run and run['mission_id'] == identity and run['state'] in ('ACQUIRING', 'RUNNING', 'PAUSED', 'PLANNING'):
                raise HTTPException(409, '请先取消任务再删除')
            workspace.store.path(identity).unlink()
            return {'success': True}

    @router.post('/api/missions/{identity}/plan')
    def plan(identity: str):
        return workspace.plan(identity)

    @router.post('/api/missions/{identity}/optimize')
    def optimize(identity: str):
        return workspace.plan(identity,optimized=True)

    @router.post('/api/missions/{identity}/start')
    def start(identity: str):
        return workspace.start(identity)

    @router.get('/api/missions/{identity}/plans/{plan_id}/path')
    def saved_path(identity: str, plan_id: str):
        try:
            mission=workspace.store.get(identity)
            if mission['projection']!=workspace.origin()[0]:raise ValueError('当前固定原点与任务不一致')
            run,data,points,fingerprint=read_plan(workspace.store,mission,plan_id)
            original=json.loads((workspace.store.run_path(plan_id)/'config/mission.json').read_text(encoding='utf-8'))
            small={k:v for k,v in run.items() if not k.startswith('_')}
            small['coverage_quality']=quality_snapshot(workspace.store.run_path(plan_id),run)
            if run.get('planning_strategy')=='SMART':
                from .optimization_results import summary
                small['optimization']=summary(workspace.store.run_path(plan_id))
            return {'run':small,'preview':original['preview'],'count':len(points),'frame_id':'map','projection':run['projection'],
                    'fingerprint':fingerprint,'points':[{'x':x,'y':y} for x,y,_ in points]}
        except FileNotFoundError as exc:raise HTTPException(404,'已保存路径不存在') from exc
        except (ValueError,OSError,KeyError,TypeError) as exc:raise HTTPException(409,str(exc)) from exc

    @router.post('/api/missions/{identity}/plans/{plan_id}/load')
    def load_plan(identity: str, plan_id: str):
        return workspace.load_plan(identity,plan_id)

    @router.post('/api/missions/{identity}/plans/{plan_id}/save')
    def save_plan(identity: str, plan_id: str):
        return workspace.save_plan(identity,plan_id)

    @router.post('/api/planning/cancel')
    def cancel_plan():
        result = workspace.gateway().trigger('/planning/cancel')
        workspace.event('PLANNING_CANCEL_REQUESTED')
        return result

    @router.post('/api/control/arm')
    def arm():
        result = workspace.gateway().arm()
        workspace.event('WEB_CONTROL_ARMED')
        return result

    @router.post('/api/control/command')
    def command(data: ManualCommand):
        return workspace.gateway().command(**data.model_dump())

    @router.websocket('/api/control/channel')
    async def control_channel(socket: WebSocket):
        # This connection never acquires a lease. The first frame must prove an
        # existing token with a zero command before any motion can be accepted.
        # Tokens stay in frames, never query strings or access logs.
        await socket.accept()
        token, gateway = None, None
        detail = None
        try:
            while True:
                payload = await asyncio.wait_for(socket.receive_json(), 1.0 if token is None else 0.5)
                frame = ManualCommand.model_validate(payload)
                if token is None:
                    if frame.seq != 1 or frame.linear != 0 or frame.angular != 0:
                        raise HTTPException(409, '遥控连接必须先确认零速度心跳')
                    gateway = workspace.gateway()
                elif frame.token != token:
                    raise HTTPException(409, '遥控连接不能切换会话，请重新启用')
                # The existing validator retains sequence, owner, mode, speed,
                # clock/expiry checks. Each frame's lifetime is still <=200ms.
                result = gateway.command(**frame.model_dump(), tolerate_expired=token is not None)
                token = frame.token
                await socket.send_json(result)
        except WebSocketDisconnect:
            pass
        except asyncio.TimeoutError:
            detail = '遥控心跳中断，已释放控制权'
        except HTTPException as exc:
            detail = str(exc.detail)
        except (ValidationError, ValueError, TypeError, KeyError):
            detail = '遥控数据格式无效，已退出遥控'
        finally:
            # An old socket closing must not revoke a newer token's lease.
            if token is not None and gateway is not None:
                gateway.stop(token)
            try:
                if detail is not None:
                    await socket.send_json({'success': False, 'detail': detail})
                await socket.close()
            except (RuntimeError, OSError, WebSocketDisconnect):
                pass

    @router.post('/api/control/stop')
    def stop(data: StopCommand):
        result = workspace.gateway().stop(data.token)
        workspace.event('WEB_CONTROL_STOPPED')
        return result

    @router.delete('/api/runs/{identity}')
    def delete_run(identity: str):
        with workspace.operation():
            directory = workspace.store.run_path(identity)
            workspace.sync()
            run = json.loads((directory / 'run.json').read_text(encoding='utf-8'))
            active = workspace.active
            if run['state'] not in DELETABLE_RUN_STATES or (
                    active and active['id'] == identity and active['state'] not in DELETABLE_RUN_STATES):
                raise HTTPException(409, '任务尚未结束，请先取消任务并等待停止确认')
            snapshot = device_cache.snapshot()
            for source in ('waypoint', 'tracking', 'planning'):
                live = snapshot[source]
                location = live.get('output_directory' if source == 'planning' else 'run_directory')
                expected = directory / ('paths' if source == 'planning' else 'tracks')
                if live['fresh'] and location == str(expected) and live.get('state') in (
                        'ACQUIRING', 'RUNNING', 'PAUSED', 'PLANNING'):
                    raise HTTPException(409, '控制器仍在使用这条记录，请等待任务停止确认')
            workspace.store.delete_run(identity)
            if active and active['id'] == identity:
                workspace.active = None
                workspace.plan_requested = workspace.plan_sequence = None
            workspace.event('RUN_RECORD_DELETED', run_id=identity, data_preserved=True)
            return {'success': True, 'data_preserved': True}

    @router.get('/api/runs/{identity}/download')
    def download(identity: str):
        try:
            directory = workspace.store.run_path(identity)
            if not directory.is_dir():
                raise HTTPException(404, '任务记录不存在')
            if workspace.lock.acquire(blocking=False):
                try:
                    workspace.sync()
                finally:
                    workspace.lock.release()
            run = json.loads((directory / 'run.json').read_text())
            if run['state'] in ('PREPARING', 'PLANNING', 'ACQUIRING', 'RUNNING', 'PAUSED'):
                raise HTTPException(409, '任务尚未结束，请先停车并取消，再下载完整数据')
            source = 'waypoint' if run['mode'] == 'waypoint' else 'tracking'
            live = device_cache.snapshot()[source]
            if live['fresh'] and live.get('run_directory') == str(directory / 'tracks') and live.get('state') in ('RUNNING', 'PAUSED', 'ACQUIRING'):
                raise HTTPException(409, '正在等待控制器停止并关闭记录文件，请稍后重试')
            bridge = getattr(app.state, 'ros_bridge', None)
            recorder = getattr(getattr(getattr(bridge, 'node', None), 'workspace_ros', None), 'recorder', None)
            if recorder and recorder.busy(directory):
                raise HTTPException(409, '任务采集或指标分析正在收尾，请稍后再下载归档')
            if workspace.quality_jobs.busy(directory):
                raise HTTPException(409, '覆盖质量评价正在生成，请稍后再下载归档')
        except ValueError as exc:
            raise HTTPException(404, '任务记录不存在') from exc
        files = [p for p in directory.rglob('*') if p.is_file() and not p.is_symlink()
                 and p.suffix in ('.json', '.yaml', '.csv', '.jsonl', '.bin', '.png', '.svg', '.md') and directory in p.resolve().parents]
        if sum(p.stat().st_size for p in files) > 200 * 1024 * 1024:
            raise HTTPException(413, '任务数据超过 200 MB，请从实验目录直接复制')
        archive_cache = workspace.store.root / '.runtime/archives'
        archive_cache.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(suffix='.zip', delete=False, dir=archive_cache) as temporary:
            filename = Path(temporary.name)
        try:
            with zipfile.ZipFile(filename, 'w', zipfile.ZIP_DEFLATED) as archive:
                for path in files:
                    archive.write(path, path.relative_to(directory))
        except Exception:
            filename.unlink(missing_ok=True)
            raise
        return FileResponse(filename, filename=f'{identity}.zip', background=BackgroundTask(filename.unlink, missing_ok=True))

    @router.get('/api/runs/{identity}/optimization')
    def optimization_report(identity: str):
        from .optimization_results import read
        try:
            value=read(workspace.store.run_path(identity))
        except (ValueError,OSError) as exc:
            raise HTTPException(404,'优化报告不可读取') from exc
        if value is None:raise HTTPException(404,'本轮没有优化报告')
        return value

    @router.get('/api/runs/{identity}/coverage-map')
    def coverage_map(identity: str):
        try:
            directory=workspace.store.run_path(identity)
            run=json.loads((directory/'run.json').read_text())
        except (ValueError,OSError) as exc:
            raise HTTPException(404,'任务记录不存在') from exc
        if run.get('mode')!='coverage':
            raise HTTPException(422,'只有覆盖任务具有作业区域图层')
        return workspace.coverage_maps.request(directory,run)

    @router.get('/api/runs/{identity}/coverage-map/{source}/{layer}.png')
    def coverage_map_image(identity: str,source: str,layer: str,v: str):
        try:
            workspace.store.run_path(identity)
        except ValueError as exc:
            raise HTTPException(404,'任务记录不存在') from exc
        if source not in ('planned','actual') or layer not in ('completed','remaining','overlap'):
            raise HTTPException(404,'作业图层不存在')
        value=workspace.coverage_maps.image(identity,source,layer,v)
        if value is None:raise HTTPException(404,'作业图层已过期，请刷新地图')
        return Response(value,media_type='image/png',headers={'Cache-Control':'private, max-age=31536000, immutable'})

    @router.get('/api/runs/{identity}/coverage-quality/{artifact}')
    def quality_artifact(identity: str, artifact: str):
        names = {'plan.png': 'coverage_quality_plan.png', 'actual.png': 'coverage_quality_actual.png',
                 'report.md': 'coverage_quality_report.md', 'metrics.json': 'coverage_quality.json'}
        try:
            directory = workspace.store.run_path(identity)
        except ValueError as exc:
            raise HTTPException(404, '任务记录不存在') from exc
        if artifact not in names:
            raise HTTPException(404, '评价文件不存在')
        target = directory/'analysis'/names[artifact]
        if not target.is_file() or target.is_symlink() or directory not in target.resolve().parents:
            raise HTTPException(404, '评价尚未生成')
        return FileResponse(target,media_type='image/png' if artifact.endswith('.png') else None)

    app.include_router(router)
