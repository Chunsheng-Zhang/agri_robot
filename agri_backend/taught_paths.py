"""Ordered RTK/INS teaching samples and a versioned file library (no ROS IO)."""
import json
import math
import re
import uuid
from pathlib import Path
from typing import Literal
from statistics import median

from pydantic import BaseModel, ConfigDict, Field

from .missions import atomic_json, now
from agri_base.path_geometry import directed_poses, motion_signature, point_turn_indices


class ProcessingParameters(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    spacing: float = Field(default=0.1, ge=0.03, le=1.)
    min_spacing: float = Field(default=0.03, ge=0.01, le=0.1)
    max_speed: float = Field(default=1.5, ge=0.2, le=3.)
    max_gap: float = Field(default=0.5, ge=0.1, le=0.5)
    smoothing: float = Field(default=0.03, ge=0., le=0.1)
    allow_reverse: bool = False


class FollowingParameters(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    controller_algorithm: Literal['pp', 'stanley', 'lqr', 'mpc'] = 'pp'
    lookahead: float = Field(default=0.5, ge=0.1, le=3.)
    speed: float = Field(default=0.1, ge=0.02, le=1.)
    max_angular: float = Field(default=0.5, ge=0.05, le=1.)
    completion_tolerance: float = Field(default=0.15, ge=0.03, le=0.5)
    start_tolerance: float = Field(default=0.3, ge=0.05, le=0.5)
    search_window: float = Field(default=1., ge=0.1, le=5.)
    max_path_error: float = Field(default=0.5, ge=0.1, le=2.)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def ros_signature(poses):
    # ROS Path carries quaternion, not yaw; trig round trips can differ by an
    # ulp. Keep the immutable file fingerprint and transport fingerprint separate.
    converted = []
    for x,y,yaw in poses:
        z,w = math.sin(yaw/2),math.cos(yaw/2)
        converted.append((float(x),float(y),math.atan2(2*(w*z),1-2*(z*z))))
    return motion_signature(converted)


def process_samples(samples, parameters):
    """Preserve chronology; never bridge a localization outage or reorder loops.

    Median/weighted local smoothing is bounded about the measured polyline.
    Sharp corners and direction cusps remain anchors and explicit stopped turns.
    This is not proof of traversable road width or independent RTK accuracy.
    """
    p = ProcessingParameters(**parameters).model_dump()
    clean, rejected = [], {'nonfinite': 0, 'isolated_spike': 0, 'stationary': 0}
    for row in samples:
        if not all(isinstance(row.get(k), (int, float)) and math.isfinite(row[k]) for k in ('stamp', 'x', 'y', 'yaw')):
            rejected['nonfinite'] += 1
            continue
        point = {k: float(row[k]) for k in ('stamp', 'x', 'y', 'yaw')}
        if clean and point['stamp'] <= clean[-1]['stamp']:
            raise ValueError('定位时间戳不递增，不能生成示教路径')
        clean.append(point)
    def dist(a, b):
        return math.hypot(a['x']-b['x'], a['y']-b['y'])
    def fast(a, b):
        return dist(a, b) > p['max_speed'] * (b['stamp']-a['stamp']) + 0.05
    filtered = []
    for i, row in enumerate(clean):
        if filtered and i+1 < len(clean) and fast(filtered[-1], row) and fast(row, clean[i+1]) and not fast(filtered[-1], clean[i+1]):
            rejected['isolated_spike'] += 1
            continue
        filtered.append(row)
    if len(filtered) < 3:
        raise ValueError('有效定位样本不足，至少需要三个样本')
    for a, b in zip(filtered, filtered[1:]):
        if b['stamp']-a['stamp'] > p['max_gap'] + 1e-6:
            raise ValueError('定位记录存在空档，请重新示教，不能跨空档连接道路')
        if fast(a, b):
            raise ValueError('轨迹存在持续跳变或超过示教速度门槛，请重新录制')
    return process_geometry(filtered, p, len(samples), rejected)


def process_geometry(filtered, parameters, raw_count, rejected=None):
    """Shared ordered geometry processing, independent of sampling time.

    File coordinates have no measured timing: do not invent samples or apply
    velocity/dropout tests to their spatial vertex spacing.
    """
    p = ProcessingParameters(**parameters).model_dump()
    rejected = dict(rejected or {'nonfinite': 0, 'isolated_spike': 0, 'stationary': 0})
    def dist(a, b):
        return math.hypot(a['x']-b['x'], a['y']-b['y'])
    # At a stationary rotation retain the outgoing body yaw at this anchor.
    sparse = [filtered[0].copy()]
    for row in filtered[1:]:
        if dist(sparse[-1], row) < p['min_spacing']:
            sparse[-1]['yaw'] = row['yaw']
            rejected['stationary'] += 1
        else:
            sparse.append(row.copy())
    if len(sparse)>1:
        sparse[-1] = filtered[-1].copy()  # Always retain the actual measured endpoint.
    if len(sparse) < 3 or sum(dist(a,b) for a,b in zip(sparse,sparse[1:])) < 0.5:
        raise ValueError('轨迹过短，至少需要三个有效点及 0.5 m 长度，不能保存')
    directions, tangents = [], []
    for a,b in zip(sparse,sparse[1:]):
        tangent = math.atan2(b['y']-a['y'],b['x']-a['x'])
        dot = math.cos(tangent-a['yaw'])
        if abs(dot) < .5:
            raise ValueError('轨迹含横向平移或航向与行驶方向不一致，不适用于差速路径跟随')
        direction = 1 if dot >= 0 else -1
        if direction < 0 and not p['allow_reverse']:
            raise ValueError('检测到倒车段；请勾选允许倒车后重新录制或导入，或只使用前进轨迹')
        directions.append(direction)
        tangents.append(tangent)
    anchors = {0, len(sparse)-1}
    for i in range(1,len(sparse)-1):
        if directions[i] != directions[i-1] or abs(angle(tangents[i]-tangents[i-1])) > math.radians(40):
            anchors.add(i)
    smoothed = [(r['x'],r['y']) for r in sparse]
    # Smooth each span independently; do not round a stopped turn across its apex.
    ordered = sorted(anchors)
    for first,last in zip(ordered,ordered[1:]):
        med = {i: tuple(median(sparse[j][k] for j in range(max(first,i-1),min(last,i+1)+1))
                         for k in ('x','y')) for i in range(first,last+1)}
        for i in range(first+1,last):
            target = tuple((med[i-1][k]+2*med[i][k]+med[i+1][k])/4 for k in (0,1))
            original = (sparse[i]['x'],sparse[i]['y'])
            delta = math.dist(target,original)
            fraction = min(1.,p['smoothing']/delta) if delta else 0.
            smoothed[i] = tuple(original[k]+fraction*(target[k]-original[k]) for k in (0,1))
    poses = []
    length = 0.
    # Split at protected corners/cusps before arc-length resampling.
    for first,last in zip(ordered,ordered[1:]):
        vertices = smoothed[first:last+1]
        distances = [0.]
        for a,b in zip(vertices,vertices[1:]):
            distances.append(distances[-1]+math.dist(a,b))
        total = distances[-1]
        if total < 1e-6:
            raise ValueError('平滑后产生退化路径段')
        length += total
        if len(poses) + math.ceil(total/p['spacing']) + 1 > 100000:
            raise ValueError('处理后超过 10 万路径点，请缩短路径或增大重采样间距')
        targets = [i*p['spacing'] for i in range(int(total/p['spacing'])+1)]
        if total-targets[-1] < p['spacing']*.2 and len(targets)>1:
            targets.pop()
        if total-targets[-1] > 1e-6:
            targets.append(total)
        points, segment = [], 0
        for s in targets:
            while segment+1 < len(distances)-1 and distances[segment+1] < s:
                segment += 1
            a,b = vertices[segment:segment+2]
            span = distances[segment+1]-distances[segment]
            if span <= 1e-9:
                raise ValueError('平滑后产生重复路径点')
            f = (s-distances[segment])/span
            points.append((a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])))
        direction = directions[first]
        for i,(x,y) in enumerate(points):
            a,b = (points[i],points[i+1]) if i+1<len(points) else (points[i-1],points[i])
            yaw = angle(math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if direction<0 else 0.))
            poses.append((x,y,yaw))
        if len(poses)>100000:
            raise ValueError('处理后超过 10 万路径点，请缩短示教或增大重采样间距')
    # Shared tracker validation and signature include repeated-XY turn markers.
    _, checked = directed_poses(poses)
    stops = point_turn_indices(poses)
    return {'points':[dict(zip(('x','y','yaw'),v)) for v in poses],
            'fingerprint':motion_signature(poses), 'reference_fingerprint':ros_signature(poses),
            'frame_id':'map', 'processing':p,
            'statistics':{'raw_samples':raw_count,'retained_samples':len(sparse),
                'path_points':len(poses),'length_m':length,'point_turn_count':len(stops),
                'reverse_segments':sum(d<0 for d in checked),'rejected':rejected}}


class TaughtPathStore:
    def __init__(self, root):
        self.root = Path(root)/'data/taught_paths'

    def directory(self, identity):
        if not re.fullmatch('[0-9a-f]{32}',identity):
            raise ValueError('无效的示教路径 ID')
        return self.root/identity

    def create(self, name, projection, processing, *, source_kind='recording'):
        name = name.strip()
        if not name or len(name)>80:
            raise ValueError('路径名称需要 1–80 个字符')
        identity = uuid.uuid4().hex
        directory = self.directory(identity)
        directory.mkdir(parents=True)
        document = {'schema_version':1,'id':identity,'revision':1,'name':name,
                    'projection':projection,'processing':processing,
                    'state':'PROCESSING' if source_kind=='import' else 'RECORDING',
                    'source_kind':source_kind,
                    'sample_source':'imported coordinate file' if source_kind=='import' else '/odom: MGX23-derived base_link pose, map and odom coincident',
                    'units':{'stamp':'ROS seconds','x':'m east','y':'m north','yaw':'rad east-zero counterclockwise'},
                    'created_at':now(),'updated_at':now(),'reason':''}
        if source_kind=='import':
            document['units'].pop('stamp')
        atomic_json(directory/'metadata.json',document)
        return document

    def get(self, identity, ready=True):
        directory = self.directory(identity)
        document = json.loads((directory/'metadata.json').read_text())
        if document.get('deleted_at') or ready and document.get('state')!='READY':
            raise ValueError('示教路径已删除或未处理成功')
        if ready:
            path = json.loads((directory/'path.json').read_text())
            if path.get('schema_version')!=1 or path.get('frame_id')!='map' or path.get('projection')!=document['projection']:
                raise ValueError('示教路径版本、坐标系或原点不一致')
            poses = [(p['x'],p['y'],p['yaw']) for p in path['points']]
            directed_poses(poses)
            point_turn_indices(poses)
            if motion_signature(poses)!=path.get('fingerprint'):
                raise ValueError('示教路径文件指纹不一致')
            if ros_signature(poses)!=path.get('reference_fingerprint'):
                raise ValueError('示教路径传输指纹不一致')
            return {**document, **path, 'name':document['name']}
        return document

    def list(self):
        if not self.root.exists():
            return []
        result = []
        for file in sorted(self.root.glob('*/metadata.json'),reverse=True):
            try:
                item = json.loads(file.read_text())
                if not item.get('deleted_at'):
                    result.append(item)
            except (ValueError,OSError):
                continue
        return sorted(result,key=lambda d:d['created_at'],reverse=True)

    def update(self, document, **values):
        document.update(values,updated_at=now())
        atomic_json(self.directory(document['id'])/'metadata.json',document)

    def recover_interrupted(self):
        for document in self.list():
            if document.get('state') in ('RECORDING','PROCESSING'):
                action = '导入' if document.get('source_kind')=='import' else '示教'
                self.update(document,state='FAULT',reason=f'上次后端运行中断，原始记录保留；请重新{action}')

    def finish(self, document, samples):
        result = process_samples(samples,document['processing'])
        path = {**result,'schema_version':1,'projection':document['projection']}
        atomic_json(self.directory(document['id'])/'path.json',path)
        self.update(document,state='READY',statistics=result['statistics'],fingerprint=result['fingerprint'])
        return path
