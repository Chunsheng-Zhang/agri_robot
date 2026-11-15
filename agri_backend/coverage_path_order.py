"""Read-only work-row visit labels from the saved native Route metadata.

Never infer work rows from every straight section: connections may be straight
too. Missing or inconsistent metadata only disables labels, not the map itself.
"""
import math


def compact(points):
    result = []
    for point in points:
        p = tuple(float(v) for v in point[:2])
        if not all(math.isfinite(v) for v in p):
            raise ValueError('作业行坐标无效')
        if result and math.dist(result[-1], p) < 1e-9:
            continue
        while len(result) >= 2:
            a, b = result[-2:]
            u, v = (b[0]-a[0], b[1]-a[1]), (p[0]-b[0], p[1]-b[1])
            if u[0]*v[0]+u[1]*v[1] > 0 and abs(u[0]*v[1]-u[1]*v[0]) <= 1e-7*math.dist(a, p):
                result.pop()
            else:
                break
        result.append(p)
    return result


def same_traversal(a, b, tolerance=1e-5):
    """Compare chronological polylines despite different collinear sampling."""
    if not a or not b or math.dist(a[0], b[0]) > tolerance:
        return False
    i = j = 0
    pa, pb = a[0], b[0]

    def on_segment(point, start, end):
        dx, dy = end[0]-start[0], end[1]-start[1]
        length2 = dx*dx+dy*dy
        if not length2:
            return math.dist(point, start) <= tolerance
        t = max(0., min(1., ((point[0]-start[0])*dx+(point[1]-start[1])*dy)/length2))
        return math.dist(point, (start[0]+t*dx, start[1]+t*dy)) <= tolerance

    while i+1 < len(a) and j+1 < len(b):
        na, nb = a[i+1], b[j+1]
        if math.dist(na, nb) <= tolerance:
            i += 1; j += 1; pa, pb = na, nb
        elif on_segment(na, pb, nb):
            i += 1; pa = pb = na
        elif on_segment(nb, pa, na):
            j += 1; pa = pb = nb
        else:
            return False
    return (sum(math.dist(p, q) for p, q in zip([pa, *a[i+1:]], a[i+1:])) <= tolerance
            and sum(math.dist(p, q) for p, q in zip([pb, *b[j+1:]], b[j+1:])) <= tolerance)


def work_row_order(saved):
    unavailable = {'state': 'UNAVAILABLE', 'rows': [], 'reason': '当前路径没有保存可核对的作业行访问信息'}
    planning = saved.get('planning')
    details = planning.get('path_details') if isinstance(planning, dict) else None
    if not isinstance(details, dict):
        return unavailable
    visits, connections = details.get('swath_visits'), details.get('native_route_connections')
    if not visits or not connections:
        return unavailable
    try:
        if not 1 <= len(visits) <= 1500 or len(visits) != len(connections):
            raise ValueError('作业行访问元数据数量不一致')
        points = compact([(p['x'], p['y']) for p in saved['points']])
        if len(points) < 2:
            raise ValueError('作业路径为空')
        cursor = points[0]
        reconstructed, rows, seen = [cursor], [], set()
        for i, (visit, connection) in enumerate(zip(visits, connections)):
            connection = compact(connection)
            if connection:
                if math.dist(cursor, connection[0]) > 1e-5:
                    raise ValueError('作业行连接不连续')
                reconstructed.extend(connection[1:])
                cursor = connection[-1]
            if i+1 < len(connections):
                if not connections[i+1]:
                    raise ValueError('作业行缺少可核对的分界')
                end = tuple(float(v) for v in connections[i+1][0][:2])
            else:
                end = points[-1]
            identity = int(visit['swath_id'])
            if identity in seen or math.dist(cursor, end) <= 1e-8:
                raise ValueError('作业行重复或长度为零')
            seen.add(identity)
            rows.append({'visit': i+1, 'start': dict(zip(('x', 'y'), cursor)),
                         'end': dict(zip(('x', 'y'), end))})
            reconstructed.append(end)
            cursor = end
        rebuilt = compact(reconstructed)
        if not same_traversal(rebuilt, points):
            raise ValueError('作业行元数据与参考路径不一致')
        return {'state': 'READY', 'rows': rows}
    except (ValueError, KeyError, TypeError, IndexError, OverflowError):
        return {**unavailable, 'reason': '作业行访问信息无法与当前参考路径核对，未显示序号'}


def path_display(saved):
    result = work_row_order(saved)
    result.update(retracing_state='UNAVAILABLE', retraced_lines=[])
    try:
        from shapely.geometry import LineString
        from shapely.ops import unary_union
        from shapely.strtree import STRtree
        vertices = compact([(p['x'], p['y']) for p in saved['points']])
        segments = list(zip(vertices, vertices[1:]))
        if result['state'] == 'READY':
            # Use the verified native vertices, just as the PDF evaluator does.
            # Re-compacting/snap-rounding sampled poses can change collinearity
            # by a grid unit at nearly collinear channel junctions.
            segments = []
            connections = saved['planning']['path_details']['native_route_connections']
            for row, connection in zip(result['rows'], connections):
                segments.extend(zip(connection, connection[1:]))
                segments.append(((row['start']['x'], row['start']['y']), (row['end']['x'], row['end']['y'])))
        if len(segments) > 12000:
            raise ValueError('重走识别超出地图显示规模')
        tolerance = 1e-5  # Same numerical precision as the offline topology evaluation.
        snap = lambda p: tuple(round(v/tolerance)*tolerance for v in p)
        lines = [LineString([snap(a), snap(b)]) for a, b in segments if snap(a) != snap(b)]
        tree, repeated, comparisons = STRtree(lines), [], 0

        def line_parts(geometry):
            if geometry.geom_type in ('LineString', 'LinearRing'):
                return [geometry] if geometry.length > tolerance else []
            return [part for item in getattr(geometry, 'geoms', []) for part in line_parts(item)]

        for i, line in enumerate(lines):
            for j in tree.query(line):
                if j >= i:
                    continue
                comparisons += 1
                if comparisons > 200000:
                    raise ValueError('重走识别超出地图显示规模')
                repeated.extend(line_parts(line.intersection(lines[int(j)])))
        merged = unary_union(repeated)
        result.update(retracing_state='READY', retraced_lines=[
            [{'x': x, 'y': y} for x, y in part.coords] for part in line_parts(merged)])
    except (ValueError, KeyError, TypeError, IndexError, OverflowError):
        result['retracing_reason'] = '当前路径的重走中心线暂无法完整识别'
    return result
