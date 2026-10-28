"""Boundary I/O and geometric validation, not a coverage planner."""

import json
from pathlib import Path

import yaml
from shapely.geometry import LineString, Polygon, shape
from shapely.geometry.polygon import orient


def load_field(filename, frame):
    with Path(filename).expanduser().open(encoding='utf-8') as stream:
        data = json.load(stream) if str(filename).endswith(('.json', '.geojson')) else yaml.safe_load(stream)
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
