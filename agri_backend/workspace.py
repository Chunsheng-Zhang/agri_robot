"""Web mission operations: validated files -> ROS services -> observed status."""
import hashlib
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
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from pydantic import Field
from starlette.background import BackgroundTask

from .devices import device_cache, service_call
from .missions import Mission, MissionStore, StrictModel, WaypointParameters, CoverageParameters, atomic_json, now


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
        return {'origin': origin, 'origin_source': source, 'defaults': self.store.defaults(),
                'synthetic': getattr(self, 'synthetic', False),
                'missions': self.store.list(), 'runs': self.store.list_runs(), 'active_run': active or None,
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

    def plan(self, identity):
        with self.operation():
            safety = self.require_idle('coverage')
            mission = self.validate_mission(identity, 'coverage')
            gateway = self.gateway()
            run = self.store.create_run(mission, safety['mode_revision'], 'planning')
            directory = self.store.run_path(run['id'])
            boundary = directory / 'config/field.json'
            atomic_json(boundary, mission['boundary'])
            p = mission['parameters']
            self.active = run
            try:
                gateway.configure_server({
                    'operation_width': float(p['operation_width']), 'min_turning_radius': float(p['min_turning_radius']),
                    'default_turn_point_distance': float(p['sampling_distance'])})
                gateway.parameters('coverage_adapter', {**{k: p[k] for k in (
                    'headland_width', 'swath_angle', 'sampling_distance', 'route_mode', 'angle_mode', 'path_continuity')},
                    'boundary_file': str(boundary), 'output_directory': str(directory / 'paths'), 'generate_decomp': False})
                self.plan_sequence = self.robot_snapshot().get('reference_sequence', 0)
                self.plan_requested = monotonic()
                result = gateway.trigger('/planning/plan')
                run['state'] = 'PLANNING'
                self.store.save_run(run)
                self.event('PLAN_REQUESTED', run_id=run['id'], mission_id=identity)
                return {**result, 'run': run}
            except Exception as exc:
                run['state'], run['reason'] = 'FAILED', str(getattr(exc, 'detail', exc))
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
                tracker = {k: p[k] for k in ('lookahead', 'speed', 'max_angular', 'completion_tolerance',
                                            'start_tolerance', 'search_window', 'max_path_error')}
                gateway.parameters('path_tracker', {**tracker, 'trajectory_directory': str(directory / 'tracks')})
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
            try:
                result = service_call(self.require_bridge().node, f'/{source}/start')
            except Exception as exc:
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

    @router.post('/api/missions/{identity}/start')
    def start(identity: str):
        return workspace.start(identity)

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

    @router.post('/api/control/stop')
    def stop(data: StopCommand):
        result = workspace.gateway().stop(data.token)
        workspace.event('WEB_CONTROL_STOPPED')
        return result

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
        except ValueError as exc:
            raise HTTPException(404, '任务记录不存在') from exc
        files = [p for p in directory.rglob('*') if p.is_file() and not p.is_symlink()
                 and p.suffix in ('.json', '.yaml', '.csv', '.jsonl') and directory in p.resolve().parents]
        if sum(p.stat().st_size for p in files) > 200 * 1024 * 1024:
            raise HTTPException(413, '任务数据超过 200 MB，请从实验目录直接复制')
        with tempfile.NamedTemporaryFile(suffix='.zip', delete=False) as temporary:
            filename = Path(temporary.name)
        try:
            with zipfile.ZipFile(filename, 'w', zipfile.ZIP_DEFLATED) as archive:
                for path in files:
                    archive.write(path, path.relative_to(directory))
        except Exception:
            filename.unlink(missing_ok=True)
            raise
        return FileResponse(filename, filename=f'{identity}.zip', background=BackgroundTask(filename.unlink, missing_ok=True))

    app.include_router(router)
