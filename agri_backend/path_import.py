"""Coordinate-only following imports. No ROS operations or fabricated timing."""
import csv
import hashlib
import io
import json
import math
from pathlib import Path

from agri_base.coordinates import LocalFrame
from agri_base.path_geometry import directed_poses, motion_signature, point_turn_indices

from .missions import atomic_json
from .taught_paths import ProcessingParameters, ros_signature

MAX_BYTES = 2 * 1024 * 1024
MAX_POINTS = 20000


def parse_coordinates(filename, content, projection):
    if len(content.encode('utf-8')) > MAX_BYTES:
        raise ValueError('文件超过 2 MiB，请减少输入点数')
    extension = Path(filename).suffix.lower()
    text = content.lstrip('\ufeff')
    local = False
    if extension == '.csv':
        reader = csv.DictReader(io.StringIO(text))
        columns = reader.fieldnames or []
        if len(columns) != len(set(columns)) or not {'longitude', 'latitude'} <= set(columns):
            raise ValueError('CSV 表头需要 longitude,latitude，可选 yaw（ROS 弧度）')
        rows = []
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError('CSV 列数不一致，请检查缺失或多余的列')
            rows.append(row)
            if len(rows) > MAX_POINTS:
                raise ValueError('输入超过 20000 个坐标点，请分段导入')
    elif extension in ('.json', '.geojson'):
        try:
            data = json.loads(text)
        except (ValueError, RecursionError) as exc:
            raise ValueError('JSON 文件无法解析') from exc
        if not isinstance(data, dict):
            raise ValueError('JSON 顶层需要为对象')
        geometry = data.get('geometry') if data.get('type') == 'Feature' else data
        if isinstance(geometry, dict) and geometry.get('type'):
            if geometry['type'] != 'LineString':
                raise ValueError('轨迹 GeoJSON 只支持单条 LineString；Polygon 是地块边界')
            coordinates = geometry.get('coordinates')
            if not isinstance(coordinates, list) or any(not isinstance(p, list) or len(p) not in (2, 3) for p in coordinates):
                raise ValueError('LineString 坐标需要为 [经度,纬度]，可选第三维高程')
            rows = [dict(zip(('longitude', 'latitude'), p[:2])) for p in coordinates]
        else:
            rows = data.get('points', data.get('waypoints'))
            local = data.get('frame_id') == 'map'
            if data.get('frame_id') not in (None, 'map', 'wgs84'):
                raise ValueError('仅支持 WGS84 经纬度或固定原点 map 坐标')
        if data.get('projection') is not None and data['projection'] != projection:
            raise ValueError('导入文件固定原点与当前系统不一致')
        if local and data.get('projection') != projection:
            raise ValueError('map 坐标必须提供与当前系统一致的 projection 固定原点')
    else:
        raise ValueError('只支持 .csv、.json 和 .geojson 轨迹文件')
    if not isinstance(rows, list) or not 3 <= len(rows) <= MAX_POINTS:
        raise ValueError('轨迹需要 3–20000 个有序坐标点')
    if any(not isinstance(row, dict) for row in rows):
        raise ValueError('每个轨迹点必须为坐标对象')
    has_yaw = ['yaw' in row for row in rows]
    if any(has_yaw) and not all(has_yaw):
        raise ValueError('yaw 必须全部提供或全部省略')
    supplied_yaw = all(has_yaw)
    frame = LocalFrame(projection['latitude'], projection['longitude'])
    if frame.epsg != projection['epsg']:
        raise ValueError('系统固定原点与 UTM 投影不一致')
    def number(value):
        try:
            if isinstance(value, bool):
                raise ValueError()
            result = float(value)
            if not math.isfinite(result):
                raise ValueError()
            return result
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError('轨迹坐标和 yaw 必须是有限数值，不能缺失或包含 NaN/Inf') from exc
    points = []
    for index, row in enumerate(rows, 1):
        keys = ('x', 'y') if local else ('longitude', 'latitude')
        if not all(k in row for k in keys):
            raise ValueError(f'第 {index} 个点缺少 {keys[0]}/{keys[1]} 坐标')
        a, b = (number(row[k]) for k in keys)
        x, y = (a, b) if local else frame.xy(b, a)
        if math.hypot(x, y) > frame.max_radius:
            raise ValueError('轨迹超出固定原点 5000 m 范围')
        point = {'x': x, 'y': y}
        if supplied_yaw:
            yaw = number(row['yaw'])
            if not -math.pi <= yaw <= math.pi:
                raise ValueError('yaw 为 ROS map 航向，单位弧度，范围 [-π,π]；不可直接填写 MGX23 北向顺时针角')
            point['yaw'] = yaw
        points.append(point)
    if not supplied_yaw:
        # Infer forward travel from the file order; duplicate vertices do not
        # invent a body heading or imply a reverse segment.
        next_distinct = None
        for i in range(len(points)-1, -1, -1):
            row = points[i]
            if i+1 < len(points) and math.dist((row['x'], row['y']), (points[i+1]['x'], points[i+1]['y'])) > 1e-9:
                next_distinct = points[i+1]
            row['yaw'] = math.atan2(next_distinct['y']-row['y'], next_distinct['x']-row['x']) if next_distinct else None
        last_yaw = next((p['yaw'] for p in reversed(points) if p['yaw'] is not None), 0.)
        for row in points:
            if row['yaw'] is None:
                row['yaw'] = last_yaw
    return points, extension, 'map' if local else 'WGS84', supplied_yaw


def preserve_geometry(points, parameters):
    """Validate imported references without filtering, moving or inserting points.

    Explicit turn markers remain part of the input. Unlike the legacy teaching
    processor, a coordinate-only import cannot invent stopped-turn poses.
    """
    allow_reverse = ProcessingParameters(**parameters).allow_reverse
    poses = [(p['x'], p['y'], p['yaw']) for p in points]
    length = sum(math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:]))
    if length < .5:
        raise ValueError('轨迹过短，至少需要 0.5 m 长度，不能保存')
    clean, directions = directed_poses(poses)
    if any(d < 0 for d in directions) and not allow_reverse:
        raise ValueError('检测到倒车段；请勾选允许倒车后重新导入，或只使用前进轨迹')
    stops = point_turn_indices(poses)
    for i in range(1, len(clean)-1):
        if directions[i-1] != directions[i] or i in stops:
            continue
        a, b, c = clean[i-1:i+2]
        before = math.atan2(b[1]-a[1], b[0]-a[0])
        after = math.atan2(c[1]-b[1], c[0]-b[0])
        change = abs(math.atan2(math.sin(after-before), math.cos(after-before)))
        if change > math.radians(40):
            raise ValueError('轨迹急角缺少原地转向标记；请在文件中提供同 XY、不同入/出 yaw 的相邻点，或提供连续曲线路径；导入不会自动插点')
    return {'points': [p.copy() for p in points], 'frame_id': 'map',
            'fingerprint': motion_signature(poses), 'reference_fingerprint': ros_signature(poses),
            'processing': {'mode': 'original_points', 'allow_reverse': allow_reverse},
            'statistics': {'raw_samples': len(points), 'retained_samples': len(points),
                           'path_points': len(points), 'length_m': length,
                           'point_turn_count': len(stops),
                           'reverse_segments': sum(d < 0 for d in directions),
                           'rejected': {'nonfinite': 0, 'isolated_spike': 0, 'stationary': 0}}}


def import_path(store, name, projection, filename, content, processing):
    points, extension, coordinates, supplied_yaw = parse_coordinates(filename, content, projection)
    result = preserve_geometry(points, processing)
    document = store.create(name, projection, result['processing'], source_kind='import')
    directory = store.directory(document['id'])
    try:
        original = 'import' + extension
        (directory/original).write_text(content, encoding='utf-8')
        provenance = {'filename': Path(filename).name, 'archive': original,
                      'sha256': hashlib.sha256(content.encode('utf-8')).hexdigest(),
                      'coordinate_system': coordinates, 'yaw_supplied': supplied_yaw,
                      'timing_available': False,
                      'note': '保留原始点序、坐标及已提供航向，不重采样、不平滑、不按距离删点；未执行采样时间、速度跳点或定位空档检查；无航向时按点序推导前进航向'}
        path = {**result, 'schema_version': 1, 'projection': projection}
        atomic_json(directory/'path.json', path)
        store.update(document, state='READY', source_kind='import', sample_source='imported coordinate file',
                     units={'x': 'm east', 'y': 'm north', 'yaw': 'rad east-zero counterclockwise'},
                     import_source=provenance, statistics=result['statistics'], fingerprint=result['fingerprint'])
    except Exception:
        store.update(document, state='FAULT', source_kind='import', reason='导入文件保存失败，请检查磁盘后重新导入')
        raise
    return store.get(document['id'])
