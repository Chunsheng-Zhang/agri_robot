"""Consume F2C TSP Route swaths AND connections, then mark stopped turns."""
import math
from .cell_routing import to_map


def route_vertices(route, prepared, check):
    expected = prepared['expected']
    groups = route.sizeVectorSwaths()
    if not groups or route.sizeConnections() not in (groups, groups+1):
        raise ValueError('原生TSP返回空路径或连接数量不一致')
    vertices, visits, connections, seen = [], [], [], set()
    work_length = connection_length = 0.

    def attach(points):
        if not points:
            return
        if vertices and math.dist(vertices[-1], points[0]) > 1e-5:
            raise ValueError('原生Route连接存在空档，禁止用直线替代')
        vertices.extend(points[1:] if vertices else points)

    def connection(index):
        nonlocal connection_length
        value = route.getConnection(index)
        points = []
        for j in range(value.size()):
            p = value.getGeometry(j)
            points.append((p.getX(), p.getY()))
        connection_length += sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        connections.append([to_map(p, prepared['angle']) for p in points])
        attach(points)

    for index in range(groups):
        check()
        connection(index)
        rows = route.getSwaths(index)
        for j in range(rows.size()):
            check()
            if j:
                connections.append([])
            row = rows[j]
            identity = int(row.getId())
            if identity not in expected or identity in seen:
                raise ValueError('原生TSP遗漏或重复访问作业行')
            source = expected[identity]
            a, b = row.startPoint(), row.endPoint()
            a, b = (a.getX(), a.getY()), (b.getX(), b.getY())
            forward = math.dist(a, source['start']) < 1e-5 and math.dist(b, source['end']) < 1e-5
            reverse = math.dist(a, source['end']) < 1e-5 and math.dist(b, source['start']) < 1e-5
            if not forward and not reverse:
                raise ValueError('原生TSP改变了作业行端点')
            if not vertices:
                vertices.append(a)
            elif math.dist(vertices[-1], a) > 1e-5:
                raise ValueError('原生Route未提供作业行之间的完整连接')
            vertices.append(b)
            work_length += math.dist(a, b)
            visits.append({'swath_id': identity, 'cell_id': source['cell_id'],
                           'row_orientation': 'FORWARD' if forward else 'REVERSED'})
            seen.add(identity)
    if route.sizeConnections() == groups+1:
        connection(groups)
        if connections[-1]:
            raise ValueError('原生开放路径意外包含返程连接')
        connections.pop()
    if connections[0]:
        raise ValueError('原生开放路径意外包含固定入口连接')
    if seen != set(expected):
        raise ValueError('原生TSP未完整访问全部作业行')
    if not all(math.isfinite(v) for p in vertices for v in p):
        raise ValueError('原生Route包含非有限坐标')
    length = sum(math.dist(a, b) for a, b in zip(vertices, vertices[1:]))
    if abs(length-work_length-connection_length) > 1e-4:
        raise ValueError('原生Route连接/作业长度不一致')
    runs = []
    for i, visit in enumerate(visits):
        if not runs or runs[-1]['cell_id'] != visit['cell_id']:
            runs.append({'cell_id': visit['cell_id'], 'first_swath': i, 'swath_count': 0})
        runs[-1]['swath_count'] += 1
    details = {'routing': 'F2C_TSP', 'connection_policy': 'F2C_ROUTE_CONNECTIONS',
        'decomposition': prepared['decomposition'], 'cell_count': len(prepared['polygons']),
        'continuous_cells_required': False, 'cell_visit_order': [r['cell_id'] for r in runs],
        'cell_reentries': len(runs)-len({r['cell_id'] for r in runs}), 'cell_swath_ranges': runs,
        'swath_count': len(visits), 'swath_length_m': work_length, 'transition_length_m': connection_length,
        'precision_m': prepared['precision_m'], 'decomposition_fallback_reasons': prepared['fallback_reasons'],
        'native_tsp': {'library': 'Fields2Cover', 'solver': 'OR-Tools RoutingModel',
            'local_api_version': '2.x', 'time_limit_seconds': 1, 'search_for_optimum': False,
            'start_end_policy': 'FREE_OPEN_ROUTE', 'global_optimality_proven': False},
        'safe_offset_join': 'MITRE',
        'cell_polygons': [[to_map(p, prepared['angle']) for p in polygon.exterior.coords]
                          for polygon in prepared['polygons']],
        'native_route_connections': connections, 'swath_visits': visits,
        'row_layout':prepared.get('row_layout')}
    details.update({key: prepared[key] for key in (
        'layout_family', 'direction_mode', 'split_angle_rad', 'swath_objective',
        'cell_directions', 'dominant_angle_rad', 'angle_search_step_deg') if key in prepared})
    return [to_map(p, prepared['angle']) for p in vertices], details


def reversed_route(vertices, details):
    """Traverse the same native open route forwards, starting at its other end."""
    visits = [{**v, 'row_orientation': 'REVERSED' if v['row_orientation']=='FORWARD' else 'FORWARD'}
              for v in reversed(details['swath_visits'])]
    runs = []
    for i, visit in enumerate(visits):
        if not runs or runs[-1]['cell_id'] != visit['cell_id']:
            runs.append({'cell_id': visit['cell_id'], 'first_swath': i, 'swath_count': 0})
        runs[-1]['swath_count'] += 1
    connections = [[]] + [list(reversed(c)) for c in reversed(details['native_route_connections'][1:])]
    return list(reversed(vertices)), {**details, 'swath_visits': visits, 'cell_swath_ranges': runs,
        'cell_visit_order': [r['cell_id'] for r in runs], 'native_route_connections': connections}


def plan_native_route(prepared, check, entry_anchor=None, guided_local_search=False):
    import fields2cover as f
    check()
    planner = f.RP_RoutePlannerBase()
    if entry_anchor is not None:
        # 2.x only exposes a shared start/end depot, not a fixed-start open
        # route. Use its native depot-aware ordering as an additional seed;
        # exclude ONLY the approach and return connections from execution.
        local_anchor = to_map(entry_anchor, -prepared['angle'])
        # Native 2.x assumes its depot inserts a NEW graph node. Reusing an
        # endpoint or midpoint violates that assumption; retain free-route
        # candidates rather than entering the native solver with that graph.
        for row in prepared['expected'].values():
            a,b = row['start'],row['end']
            midpoint = ((a[0]+b[0])/2,(a[1]+b[1])/2)
            if min(math.dist(local_anchor,p) for p in (a,b,midpoint)) < 1e-5:
                raise ValueError('入口引导种子与作业图节点重合，拒绝该种子；仍比较自由开放路线')
        planner.setStartAndEndPoint(f.Point(*local_anchor))
    route = planner.genRoute(prepared['cells'], prepared['swaths'], False, 1e-4, True, 1, guided_local_search)
    check()
    omitted = []
    if entry_anchor is not None:
        groups = route.sizeVectorSwaths()
        if not groups or route.sizeConnections()!=groups+1:
            raise ValueError('入口引导原生Route缺少首尾连接')
        opened = f.Route()
        for i in range(groups):
            check()
            if i:
                opened.addConnection(route.getConnection(i))
            opened.addSwaths(route.getSwaths(i))
        for i in (0,groups):
            connection = route.getConnection(i)
            points = []
            for j in range(connection.size()):
                p = connection.getGeometry(j)
                points.append(to_map((p.getX(),p.getY()),prepared['angle']))
            omitted.append(points)
        vertices, details = route_vertices(opened, prepared, check)
    else:
        vertices, details = route_vertices(route, prepared, check)
    details['native_seed'] = 'ENTRY_ANCHORED_CYCLE_OPENED' if entry_anchor is not None else 'FREE_OPEN_ROUTE'
    details['native_tsp'] = {**details['native_tsp'], 'start_end_policy': details['native_seed'],
                             'guided_local_search': guided_local_search,
                             'return_connection_executed': False}
    details['omitted_depot_connections'] = omitted
    return vertices, details
