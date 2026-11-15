"""Native F2C decomposition/swaths in a numerically stable sweep frame.

Cells are geometric inputs, not indivisible scheduling blocks. RoutePlannerBase
may visit their rows in any order. No custom cell-order/2-opt/DP solver remains.
"""
import math
from shapely import wkt
from shapely.affinity import rotate
from shapely.geometry import LineString
from shapely.ops import transform, unary_union
from .point_turn_planner import field_cell

PRECISION_M = 1e-6
SEAM_TOLERANCE_M = 1e-5
MAX_NATIVE_GRAPH_NODES = 768
# Keep native routing endpoints distinct from decomposition corner nodes.
ROW_VERTEX_CLEARANCE_M = 2e-4


def to_map(point, angle):
    x, y = point[:2]
    c, s = math.cos(angle), math.sin(angle)
    return x*c-y*s, x*s+y*c


def snapped(geometry):
    # Worker geometry only: <=0.71 micrometres; stored boundaries never change.
    # ops.transform supports Shapely 1.8 as well as 2.x.
    return transform(lambda x, y, z=None: (round(x, 6), round(y, 6)), geometry)


def sweep_monotone(polygon, check):
    if polygon.interiors:
        return False
    levels = sorted(set(y for x, y, *z in polygon.exterior.coords))
    x0, _, x1, _ = polygon.bounds
    for lo, hi in zip(levels, levels[1:]):
        check()
        if hi-lo < PRECISION_M:
            continue
        cut = polygon.intersection(LineString([(x0-1, (lo+hi)/2), (x1+1, (lo+hi)/2)]))
        lines = [cut] if cut.geom_type == 'LineString' else list(getattr(cut, 'geoms', []))
        if sum(g.geom_type == 'LineString' and g.length > PRECISION_M for g in lines) != 1:
            return False
    return True


def edge_rows(cells, polygons, original, safe, width, check):
    """One fixed-pitch layout in the sweep frame; F2C clips every row.

    Only the first/last transverse placement changes. Longitudinal endpoints
    and holes still use the SAME safe cells, including full turn clearance.
    """
    import fields2cover as f
    x0, y0, x1, y1 = original.bounds
    epsilon = ROW_VERTEX_CLEARANCE_M
    first = max(y0+width/2, safe.bounds[1]+epsilon)
    last = min(y1-width/2, safe.bounds[3]-epsilon)
    if first >= last:
        levels = [(safe.bounds[1]+safe.bounds[3])/2]
        extra = False
    else:
        count = math.floor((last-first)/width)+1
        if count > 1500:
            raise ValueError('作业行数量超过1500条几何预算')
        levels = [first+i*width for i in range(count)]
        extra = last-levels[-1] > epsilon
        if extra:
            levels.append(last)
        elif len(levels)>1:
            levels[-1] = last
    # A scan exactly along a shared cell seam would create duplicate swaths.
    # Move only such numerical coincidences, not the field or its origin.
    vertices = {y for p in polygons for x,y,*_ in p.exterior.coords}
    for i, y in enumerate(levels):
        check()
        if any(abs(y-v)<epsilon for v in vertices):
            for candidate in (y+2*epsilon, y-2*epsilon):
                if (safe.bounds[1]+epsilon/2<candidate<safe.bounds[3]-epsilon/2
                        and all(abs(candidate-v)>=epsilon for v in vertices)):
                    levels[i]=candidate
                    break
            else:
                raise ValueError('靠边作业线与分区边界重合，无法稳定生成原生路由')
    levels=sorted(set(levels))
    def clip(level, cell):
        line=f.LineString();line.addPoint(x0-1.,float(level));line.addPoint(x1+1.,float(level))
        result=f.Swaths();result.append(line,cell,float(width))
        return result
    generated=f.SwathsByCells();fallback=0
    for cell_id, polygon in enumerate(polygons):
        check();cell=cells.getGeometry(cell_id);rows=f.Swaths()
        for level in levels:
            check()
            if polygon.bounds[1]<level<polygon.bounds[3]:
                rows.append(clip(level,cell))
        if not rows.size():
            # A narrow decomposition cell can lie between global scan levels.
            # Retain it with a native-clipped middle row, never omit the cell.
            rows.append(clip((polygon.bounds[1]+polygon.bounds[3])/2,cell));fallback+=1
        generated.emplace_back(rows)
    return generated, {'policy':'EDGE_ALIGNED_FIXED_SPACING','nominal_spacing_m':width,
        'implement_width_m':width,'target_edge_offset_m':width/2,
        'first_center_y_local_m':levels[0],'last_center_y_local_m':levels[-1],
        'scan_level_count':len(levels),'last_side_row_added':extra,
        'last_gap_m':levels[-1]-levels[-2] if len(levels)>1 else None,
        'fallback_cell_count':fallback,'turn_clearance_unchanged':True}


def decompose_cells(field, angle, check):
    import fields2cover as f
    check()
    if field.is_empty or field.geom_type != 'Polygon':
        raise ValueError('安全内缩作业区不连通或为空，不能遗漏分区完成规划')
    local = rotate(field, -angle, origin=(0, 0), use_radians=True)
    stable = snapped(local)
    if not stable.is_valid or stable.is_empty:
        raise ValueError('安全工作区无法在微米数值精度下保持有效')
    errors = []
    for name, factory in [('F2C_BOUSTROPHEDON', f.DECOMP_Boustrophedon),
                          ('F2C_TRAPEZOIDAL_FALLBACK', f.DECOMP_TrapezoidalDecomp)]:
        check()
        decomp = factory()
        decomp.setSplitAngle(0.)
        native = decomp.decompose(f.Cells(field_cell(stable, f)))
        polygons = [snapped(wkt.loads(native.getGeometry(i).exportToWkt())) for i in range(native.size())]
        if not polygons or any(not p.is_valid or p.is_empty or p.area <= 0 or not sweep_monotone(p, check)
                               for p in polygons):
            errors.append(name+': 分区仍非扫描单调或几何无效')
            continue
        union = unary_union(polygons)
        if (union.geom_type != 'Polygon' or not union.buffer(SEAM_TOLERANCE_M).covers(local)
                or not local.buffer(SEAM_TOLERANCE_M).covers(union)):
            errors.append(name+': 分区缝不连通或未完整保留工作区')
            continue
        cells = f.Cells()
        for polygon in polygons:
            cells.addGeometry(field_cell(polygon, f))
        return {'cells':cells,'polygons':polygons,'safe':stable,'angle':angle,
                'decomposition':name,'fallback_reasons':errors,'precision_m':PRECISION_M}
    raise ValueError('原生分解未通过完整性/扫描单调检查：'+'; '.join(errors))


def rows_from_cells(prepared, width, check, coverage_field=None, row_layout=None):
    import fields2cover as f
    cells,polygons,angle=prepared['cells'],prepared['polygons'],prepared['angle']
    if coverage_field is None:
        generated=f.SG_BruteForce().generateSwaths(0.,float(width),cells);layout=None
    else:
        original=rotate(coverage_field,-angle,origin=(0,0),use_radians=True)
        if row_layout is None:
            generated,layout=edge_rows(cells,polygons,original,prepared['safe'],width,check)
        else:
            from .edge_candidates import native_rows
            generated,layout=native_rows(cells,polygons,original,width,row_layout,check)
    return identify_swaths(prepared, generated, check, layout)


def identify_swaths(prepared, generated, check, layout=None):
    """Assign stable row IDs and enforce the common native routing budgets."""
    import fields2cover as f
    cells, polygons, angle = prepared['cells'], prepared['polygons'], prepared['angle']
    if generated.size() != cells.size():
        raise ValueError('原生条带分区数量不一致')
    swaths = f.SwathsByCells()
    expected = {}
    endpoints = set()
    for cell_id in range(generated.size()):
        part = generated[cell_id]
        identified = f.Swaths()
        if not part.size():
            raise ValueError('分解后存在无作业行的分区，拒绝遗漏该分区')
        for i in range(part.size()):
            check()
            row = part[i]
            row.setId(len(expected))
            a, b = row.startPoint(), row.endPoint()
            a, b = (a.getX(), a.getY()), (b.getX(), b.getY())
            keys = [tuple(round(v, 7) for v in p) for p in (a, b)]
            if math.dist(a, b) < SEAM_TOLERANCE_M or any(k in endpoints for k in keys):
                raise ValueError('作业行过短或共享端点，无法安全编码原生TSP访问图')
            endpoints.update(keys)
            expected[row.getId()] = {'cell_id': cell_id, 'start': a, 'end': b}
            # SWIG indexing may return a value copy: explicitly store the
            # identified swath instead of relying on setId mutating it.
            identified.push_back(row)
        swaths.emplace_back(identified)
    if not expected:
        raise ValueError('安全地头内缩后没有可用作业行')
    if len(expected) > 1500:
        raise ValueError('作业行数量超过1500条几何预算')
    junctions = {tuple(p[:2]) for polygon in polygons
                 for ring in (polygon.exterior, *polygon.interiors) for p in ring.coords}
    # Native Graph2D materializes all-pairs paths. Guard Jetson memory
    # independently of candidate/angle counts; no partial result is used.
    if len(endpoints)+len(junctions) > MAX_NATIVE_GRAPH_NODES:
        raise ValueError('原生TSP端点/通道图超过内存保护规模，请增大幅宽或拆分任务')
    return {'cells': cells, 'swaths': swaths, 'expected': expected, 'angle': angle,
            'polygons': polygons, 'decomposition': prepared['decomposition'], 'precision_m': PRECISION_M,
            'fallback_reasons': prepared['fallback_reasons'], 'row_layout':layout}


def decompose_rows(field, width, angle, check, coverage_field=None):
    return rows_from_cells(decompose_cells(field,angle,check),width,check,coverage_field)
