#!/usr/bin/env python3
"""Render a small, reproducibly selected diagnostic set from a frozen report.

Reconstruct the connected-visit disk regions, and verify their areas against the
independent evaluator before plotting. This does not plan or change any route.
"""
import argparse
import gc
import gzip
import hashlib
import json
import math
from pathlib import Path

from shapely import wkt
from shapely.geometry import GeometryCollection, LineString, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

from benchmark_coverage_metrics import compact_path
from benchmark_path_topology import route_segments

ROOT = Path(__file__).resolve().parents[1]


def regions(vertices, field, width):
    points = compact_path(vertices)
    segments = [LineString([a, b]) for a, b in zip(points, points[1:]) if math.dist(a, b) > 1e-9]
    buffers = [s.buffer(width / 2, quad_segs=32) for s in segments]
    tree = STRtree(buffers)
    repeats = []
    for i, footprint in enumerate(buffers):
        prior = [int(j) for j in tree.query(footprint, predicate='intersects') if int(j) < i]
        if prior:
            hit = footprint.intersection(unary_union([buffers[j] for j in prior]))
            visit_continues = Point(segments[i].coords[0]).buffer(width / 2, quad_segs=32)
            repeats.append(hit.difference(visit_continues))
    covered = unary_union(buffers).intersection(field)
    repeat = unary_union(repeats).intersection(field) if repeats else GeometryCollection()
    return covered, repeat, field.difference(covered)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT / '.diagnostics/coverage50_10m_20261115')
    args = parser.parse_args()
    out = args.out
    rows = [r for r in json.loads((out / 'results_with_topology.json').read_text()) if r['state'] == 'SUCCEEDED']
    selected = {}
    for metric, container, reverse, n, pool in (
        ('coverage_percent', 'independent', False, 2, rows),
        ('overlap_percent', 'independent', True, 2, rows),
        ('extra_retraced_fraction', 'topology', True, 2, rows),
        ('overlap_percent', 'independent', True, 2, [r for r in rows if r['source']['holes']]),
    ):
        for row in sorted(pool, key=lambda r: r[container][metric], reverse=reverse)[:n]:
            selected.setdefault(row['field'], {'row': row, 'selected_for': []})['selected_for'].append(metric)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.collections import LineCollection
    from matplotlib.path import Path as PlotPath
    from matplotlib.patches import PathPatch
    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams['axes.unicode_minus'] = False

    def paint(ax, geometry, color):
        from shapely.geometry.polygon import orient
        if geometry.is_empty:
            return
        if geometry.geom_type != 'Polygon':
            for part in getattr(geometry, 'geoms', []):
                paint(ax, part, color)
            return
        polygon = orient(geometry, sign=1.)
        points, codes = [], []
        for ring in (polygon.exterior, *polygon.interiors):
            coords = list(ring.coords)
            points.extend(coords)
            codes.extend([PlotPath.MOVETO] + [PlotPath.LINETO] * (len(coords) - 2) + [PlotPath.CLOSEPOLY])
        ax.add_patch(PathPatch(PlotPath(points, codes), facecolor=color, edgecolor='none'))

    destination = out / 'quality_cases'
    destination.mkdir(exist_ok=True)
    records = []
    for field_name, item in selected.items():
        row = item['row']
        with gzip.open(out / 'jobs' / f'{row["key"]}_path.json.gz', 'rt') as f:
            data = json.load(f)
        field = wkt.loads(data['field_wkt'])
        covered, repeat, missed = regions(data['vertices'], field, row['width_m'])
        for actual, expected in ((covered.area, row['independent']['covered_m2']), (repeat.area, row['independent']['overlap_m2'])):
            assert math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-5)
        fig, axes = plt.subplots(1, 2, figsize=(13, 7), sharex=True, sharey=True)
        for ax in axes:
            paint(ax, field, '#9dcfa6')
            paint(ax, missed, '#f5bf45')
            paint(ax, repeat, '#d76b7a')
            ax.plot(*field.exterior.xy, color='#333', lw=1)
            for ring in field.interiors:
                ax.fill(*ring.xy, color='#bcbcbc')
                ax.plot(*ring.xy, color='#333', lw=.7)
            ax.set_aspect('equal')
            ax.set_xlabel('局部 UTM X / m')
            ax.autoscale_view()
        segments, roles = route_segments(data['vertices'], data.get('path_details'))
        for role, color in [('work', '#15527c'), ('transition', '#87611c')]:
            paths = [[a, b] for a, b, r in segments if r == role]
            if paths:
                axes[0].add_collection(LineCollection(paths, colors=color, linewidths=.65))
        axes[0].set_title('蓝：作业行 / 棕：连接；叠加评价区域')
        axes[1].set_title('绿：覆盖一次 / 黄：漏覆盖 / 粉：重复覆盖 / 灰：孔洞')
        axes[0].set_ylabel('Y / m')
        m = row['independent']
        fig.suptitle(f'{row["index"]+1}/50 · {field_name} · 覆盖 {m["coverage_percent"]:.3f}% / 重复 {m["overlap_percent"]:.3f}%\n'
                     '10m等效圆盘；包含连接作业足迹，非真实机具外形；按连续经过次数统计', fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, .9))
        filename = destination / f'{row["index"]+1:02d}_{field_name}.png'
        fig.savefig(filename, dpi=160, bbox_inches='tight', pad_inches=.15)
        plt.close(fig)
        records.append({'field': field_name, 'index': row['index'] + 1, 'selected_for': item['selected_for'],
                        'covered_m2': covered.area, 'repeated_m2': repeat.area, 'missed_m2': missed.area,
                        'areas_match_independent_metrics': True, 'figure': str(filename.relative_to(out))})
        del data, covered, repeat, missed, fig, axes
        gc.collect()
    payload = {'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'cases': records}
    (destination / 'index.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2))
    print(json.dumps(payload, ensure_ascii=False))


if __name__ == '__main__':
    main()
