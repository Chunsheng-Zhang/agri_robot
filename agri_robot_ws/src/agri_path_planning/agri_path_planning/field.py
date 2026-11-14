"""Boundary I/O and geometric validation, not a coverage planner."""

import json
import math
from pathlib import Path

import yaml
from shapely.geometry import LineString, Polygon, shape
from shapely.geometry.polygon import orient


def load_field(filename, frame):
    with Path(filename).expanduser().open(encoding='utf-8') as stream:
        data = json.load(stream) if str(filename).endswith(('.json', '.geojson')) else yaml.safe_load(stream)
    return field_from_data(data, frame)


def field_from_data(data, frame):
    if not isinstance(data, dict):
        raise ValueError('Field boundary must be a JSON object')
    if data.get('type') in ('Feature', 'Polygon'):
        geom = shape(data['geometry'] if data['type'] == 'Feature' else data)
        if not isinstance(geom, Polygon):
            raise ValueError('Only a single GeoJSON Polygon is supported')
        rings = [list(geom.exterior.coords)] + [list(r.coords) for r in geom.interiors]
        rings = [[frame.xy(float(lat), float(lon)) for lon, lat, *_ in ring] for ring in rings]
    else:
        raw = data.get('rings', [data.get('points', [])])
        rings = [[frame.xy(float(p['latitude']), float(p['longitude'])) for p in ring] for ring in raw]
    if not rings or any(len(ring) < 3 for ring in rings):
        raise ValueError('Every boundary ring needs at least 3 points')
    polygon = Polygon(rings[0], rings[1:])
    if not polygon.is_valid or polygon.is_empty or polygon.area < 0.1:
        raise ValueError('Invalid, self-intersecting, or empty field polygon')
    polygon = orient(polygon, sign=1.0)
    return polygon


def validate_path(points, polygon, robot_width):
    if len(points) < 2:
        raise ValueError('Coverage server returned an empty path')
    line = LineString(points)
    if not line.is_valid or line.length <= 0:
        raise ValueError('Invalid path geometry')
    # Conservative circular footprint, including travel through connecting turns.
    clearance = polygon.buffer(-robot_width / 2)
    if clearance.is_empty or not clearance.buffer(1e-4).covers(line):
        outside = line.difference(clearance)
        raise ValueError('Planned path crosses field boundary/hole or lacks footprint clearance: '
                         f'outside_length={outside.length:.3f}m, outside_bounds={outside.bounds}')


def validate_native_work_area(polygon, headland_width):
    """The current native single-field API takes only buffer geometry zero.

    Reject split work areas instead of silently dropping every other polygon.
    This is an interface completeness check, not a minimum coverage constraint
    or proof that the robot cannot travel between the work areas.
    """
    if not math.isfinite(headland_width) or headland_width < 0:
        raise ValueError('地头预留宽度必须是有限非负数')
    working = polygon.buffer(-headland_width) if headland_width else polygon
    if working.is_empty:
        raise ValueError('地头内缩后没有可用作业区域，请减小地头预留宽度')
    if working.geom_type != 'Polygon':
        count = len(getattr(working, 'geoms', []))
        raise ValueError(f'地头内缩后分裂为 {count} 个作业区域；当前普通条带接口不能完整保留多个分区，'
                         '已拒绝仅规划其中一块。请减小人工地头宽度或使用一键优化规划')
    return working
