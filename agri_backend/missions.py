"""Validated task documents and per-run files. No ROS or chassis control here."""
import json
import math
import os
import re
import tempfile
import uuid
import sys
from datetime import datetime
from pathlib import Path
from typing import Literal
from zoneinfo import ZoneInfo

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

ROOT = Path(__file__).resolve().parents[1]
DELETABLE_RUN_STATES = frozenset(('COMPLETED', 'CANCELED', 'FAILED', 'FAULT', 'READY'))
try:
    from agri_base.coordinates import LocalFrame
    from agri_path_planning.field import field_from_data
except ModuleNotFoundError as exc:
    if exc.name not in ('agri_base', 'agri_path_planning'):
        raise
    # Offline document editing reuses exactly the core conversion implementation.
    for package in ('agri_base', 'agri_path_planning'):
        sys.path.insert(0, str(ROOT / 'agri_robot_ws/src' / package))
    from agri_base.coordinates import LocalFrame
    from agri_path_planning.field import field_from_data


def now():
    return datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds')


def atomic_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.write-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class StrictModel(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False, validate_default=True)


class GeoPoint(StrictModel):
    longitude: float = Field(ge=-180, le=180)
    latitude: float = Field(ge=-80, le=84)
    dwell_time: float = Field(default=2.0, ge=0, le=600)


class WaypointParameters(StrictModel):
    max_linear: float = Field(default=0.2, gt=0, le=1)
    min_linear: float = Field(default=0.03, gt=0, le=1)
    max_angular: float = Field(default=0.5, gt=0, le=2)
    arrival_tolerance: float = Field(default=0.1, ge=0.02, le=5)
    dwell_time: float = Field(default=2, ge=0, le=600)
    heading_kp: float = Field(default=1, gt=0, le=10)
    distance_kp: float = Field(default=0.5, gt=0, le=10)
    rotate_threshold: float = Field(default=1, gt=0, le=math.pi)
    align_threshold: float = Field(default=0.2, gt=0, le=math.pi)

    @model_validator(mode='after')
    def relationships(self):
        if self.min_linear > self.max_linear or self.align_threshold >= self.rotate_threshold:
            raise ValueError('最小速度不能大于最大速度；转向结束阈值须小于开始阈值')
        return self


class SmartPlanningParameters(StrictModel):
    @model_validator(mode='before')
    @classmethod
    def remove_legacy_coverage_constraint(cls, value):
        # Old saved/imported tasks remain usable; the removed field has no effect
        # and is not emitted into new task parameters or execution snapshots.
        if isinstance(value, dict):
            from agri_path_planning.planning_options import LEGACY
            return {k:v for k,v in value.items() if k not in LEGACY}
        return value



class CoverageParameters(SmartPlanningParameters):
    robot_width: float = Field(default=0.2, ge=0.05, le=20)
    robot_length: float = Field(default=0.45, ge=0.05, le=20)
    rotation_safety_margin: float = Field(default=0.05, ge=0, le=5)
    rotation_center_x: float = Field(default=0, ge=-10, le=10)
    rotation_center_y: float = Field(default=0, ge=-10, le=10)
    operation_width: float = Field(default=0.8, ge=0.1, le=20)
    evaluation_width: float = Field(default=0, ge=0, le=20)
    quality_resolution: float = Field(default=0.05, ge=0.01, le=0.5)
    min_turning_radius: float = Field(default=1.0, ge=0.05, le=20)
    headland_width: float = Field(default=3, ge=0, le=50)
    swath_angle: float = Field(default=0, ge=-math.pi, le=math.pi)
    sampling_distance: float = Field(default=0.1, ge=0.02, le=1)
    route_mode: Literal['BOUSTROPHEDON', 'SNAKE', 'SPIRAL', 'CONTOUR'] = 'BOUSTROPHEDON'
    angle_mode: Literal['SET_ANGLE', 'BRUTE_FORCE'] = 'SET_ANGLE'
    path_continuity: Literal['DUBINS', 'DUBINS_CC', 'REEDS_SHEPP', 'REEDS_SHEPP_HC'] = 'DUBINS_CC'
    allow_reverse: bool = Field(default=False, strict=True)
    lookahead: float = Field(default=0.8, ge=0.1, le=10)
    speed: float = Field(default=0.2, gt=0, le=1)
    max_angular: float = Field(default=1.0, gt=0, le=1)
    completion_tolerance: float = Field(default=0.2, ge=0.02, le=2)
    start_tolerance: float = Field(default=0.8, ge=0.05, le=5)
    search_window: float = Field(default=1, ge=0.1, le=10)
    max_path_error: float = Field(default=2, ge=0.05, le=10)

    @model_validator(mode='before')
    @classmethod
    def native_curve_name(cls, value):
        if isinstance(value, dict) and 'path_continuity' in value:
            from agri_path_planning.curves import curve_name
            return {**value, 'path_continuity': curve_name(value['path_continuity'])}
        return value

    @model_validator(mode='after')
    def evaluation_parameters(self):
        if 0 < self.evaluation_width < .1:
            raise ValueError('评价幅宽须为0（沿用作业幅宽）或至少0.1米')
        return self


class Mission(StrictModel):
    name: str = Field(min_length=1, max_length=80)
    mode: Literal['waypoint', 'coverage']
    waypoints: list[GeoPoint] = Field(default_factory=list, max_length=500)
    boundary: dict | None = None
    parameters: dict = Field(default_factory=dict)
    revision: int | None = Field(default=None, ge=1)
    import_origin: dict | None = None


class MissionStore:
    def __init__(self, root=None):
        self.root = Path(root) if root else ROOT
        self.config_path = Path(os.getenv('AGRI_SYSTEM_CONFIG', str(
            self.root / 'agri_robot_ws/src/agri_bringup/config/system.yaml')))
        self.documents = self.root / 'data/web/missions'
        self.runs = self.root / 'data/experiments'
        self.deleted_runs = self.root / 'data/web/deleted_runs'

    def config(self):
        with self.config_path.open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    def configured_origin(self):
        p = self.config()['/**']['ros__parameters']
        return LocalFrame(p['origin_latitude'], p['origin_longitude'], p.get('max_local_radius', 5000)).identity

    def defaults(self):
        config = self.config()
        def values(node):
            return config.get(node, {}).get('ros__parameters', {})
        waypoint = WaypointParameters(**{k: v for k, v in values('waypoint').items()
                                        if k in WaypointParameters.model_fields}).model_dump()
        merged = {**values('/**'), **values('coverage_server'), **values('coverage_adapter'), **values('path_tracker')}
        coverage = CoverageParameters(**{k: v for k, v in merged.items()
                                        if k in CoverageParameters.model_fields}).model_dump()
        return {'waypoint': waypoint, 'coverage': coverage}

    def preview(self, mission, origin):
        if mission.import_origin is not None and mission.import_origin != origin:
            raise ValueError('导入任务的固定原点不同，请先核对场地和坐标基准')
        (WaypointParameters if mission.mode == 'waypoint' else CoverageParameters)(**mission.parameters)
        frame = LocalFrame(origin['latitude'], origin['longitude'], self.config()['/**']['ros__parameters'].get('max_local_radius', 5000))
        if frame.identity != origin:
            raise ValueError('投影与固定原点不一致')
        if mission.mode == 'waypoint':
            if not mission.waypoints:
                raise ValueError('至少输入一个目标点')
            points = [dict(zip(('x', 'y'), frame.xy(p.latitude, p.longitude))) for p in mission.waypoints]
            return {'frame_id': 'map', 'points': points, 'rings': [], 'area': None,
                    'length': sum(math.dist((a['x'], a['y']), (b['x'], b['y'])) for a, b in zip(points, points[1:]))}
        polygon = field_from_data(mission.boundary, frame)
        rings = [[{'x': x, 'y': y} for x, y in ring.coords]
                 for ring in [polygon.exterior, *polygon.interiors]]
        return {'frame_id': 'map', 'points': [], 'rings': rings,
                'area': polygon.area, 'length': polygon.length}

    def path(self, identity):
        if not re.fullmatch('[0-9a-f]{32}', identity):
            raise ValueError('Invalid mission ID')
        return self.documents / f'{identity}.json'

    def get(self, identity):
        return json.loads(self.path(identity).read_text(encoding='utf-8'))

    def list(self):
        return sorted([json.loads(p.read_text(encoding='utf-8')) for p in self.documents.glob('*.json')],
                      key=lambda p: p['updated_at'], reverse=True)

    def save(self, mission, origin, identity=None):
        mission.name = mission.name.strip()
        if not mission.name:
            raise ValueError('任务名称不能为空')
        if mission.import_origin is not None and mission.import_origin != origin:
            raise ValueError('导入任务的固定原点不同，请先核对场地和坐标基准')
        model = WaypointParameters if mission.mode == 'waypoint' else CoverageParameters
        parameters = model(**mission.parameters).model_dump()
        preview = self.preview(mission, origin)
        existing = self.get(identity) if identity else None
        if existing and mission.revision != existing['revision']:
            raise ValueError('任务已被修改，请重新加载后再保存')
        identity = identity or uuid.uuid4().hex
        document = {'id': identity, 'revision': existing['revision'] + 1 if existing else 1,
                    'name': mission.name, 'mode': mission.mode,
                    'waypoints': [p.model_dump() for p in mission.waypoints], 'boundary': mission.boundary,
                    'parameters': parameters, 'projection': origin, 'preview': preview,
                    'created_at': existing['created_at'] if existing else now(), 'updated_at': now()}
        atomic_json(self.path(identity), document)
        return document

    def create_run(self, mission, mode_revision, stage):
        identity = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f_web_') + uuid.uuid4().hex[:8]
        directory = self.runs / identity
        (directory / 'config').mkdir(parents=True)
        atomic_json(directory / 'config/mission.json', mission)
        atomic_json(directory / 'config/system.json', self.config())
        run = {'id': identity, 'mission_id': mission['id'], 'mission_revision': mission['revision'],
               'name': mission['name'], 'mode': mission['mode'], 'mode_revision': mode_revision,
               'projection': mission['projection'], 'stage': stage, 'state': 'PREPARING',
               'created_at': now(), 'updated_at': now(), 'reason': ''}
        self.save_run(run)
        return run

    def run_path(self, identity):
        if not re.fullmatch(r'\d{8}_\d{6}_\d{6}_web_[0-9a-f]{8}', identity):
            raise ValueError('Invalid run ID')
        return self.runs / identity

    def save_run(self, run):
        run['updated_at'] = now()
        atomic_json(self.run_path(run['id']) / 'run.json', {k: v for k, v in run.items() if not k.startswith('_')})

    def list_runs(self, limit=50):
        records = []
        for path in sorted(self.runs.glob('*_web_*/run.json'), reverse=True):
            if (self.deleted_runs / f'{path.parent.name}.json').exists():
                continue
            records.append(json.loads(path.read_text(encoding='utf-8')))
            if limit is not None and len(records) >= limit:
                break
        return records

    def deleted_run_path(self, identity):
        self.run_path(identity)  # Validate before constructing any writable path.
        return self.deleted_runs / f'{identity}.json'

    def delete_run(self, identity):
        """Remove a list entry persistently; keep all original experiment files."""
        run = json.loads((self.run_path(identity) / 'run.json').read_text(encoding='utf-8'))
        if run['state'] not in DELETABLE_RUN_STATES:
            raise ValueError('任务尚未结束，请先取消任务并等待停止确认')
        atomic_json(self.deleted_run_path(identity), {'id': identity, 'deleted_at': now()})
