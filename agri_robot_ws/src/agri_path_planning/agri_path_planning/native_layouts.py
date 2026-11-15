"""Native whole-field and per-cell direction portfolios in the common map frame.

Partition seams are virtual: apply the physical clearance ONCE before this
module. No per-cell headland, straight-line connection, or narrow-cell repair.
"""
import math
from shapely import wkt
from shapely.ops import unary_union
from .cell_routing import snapped, identify_swaths, PRECISION_M, SEAM_TOLERANCE_M
from .point_turn_planner import field_cell

OBJECTIVES = ('NSwath', 'NSwathModified', 'SwathLength')
PARTITIONS = (('NONE', None), ('BCD', 0.), ('BCD', math.pi/2),
              ('TRAPEZOIDAL', 0.), ('TRAPEZOIDAL', math.pi/2))


def partition_cells(safe, kind, split_angle, check):
    import fields2cover as f
    check()
    if safe.is_empty or safe.geom_type != 'Polygon' or not safe.is_valid:
        raise ValueError('安全内缩作业区不连通或无效')
    stable = snapped(safe)
    source = f.Cells(field_cell(stable, f))
    if kind == 'NONE':
        native = source
    else:
        factory = {'BCD': f.DECOMP_Boustrophedon, 'TRAPEZOIDAL': f.DECOMP_TrapezoidalDecomp}[kind]
        decomp = factory(); decomp.setSplitAngle(split_angle)
        native = decomp.decompose(source)
    check()
    polygons = [snapped(wkt.loads(native.getGeometry(i).exportToWkt())) for i in range(native.size())]
    if not polygons or any(p.is_empty or not p.is_valid or p.area <= 0 for p in polygons):
        raise ValueError('原生独立方向分区几何无效')
    union = unary_union(polygons)
    if (not union.buffer(SEAM_TOLERANCE_M).covers(safe)
            or not safe.buffer(SEAM_TOLERANCE_M).covers(union)
            or abs(sum(p.area for p in polygons)-union.area) > max(1e-6, safe.area*1e-10)):
        raise ValueError('原生独立方向分区未完整保留工作区或分区重叠')
    cells = f.Cells()
    for p in polygons:
        cells.addGeometry(field_cell(p, f))
    return {'cells': cells, 'polygons': polygons, 'angle': 0., 'safe': stable,
            'decomposition': 'F2C_NONE' if kind == 'NONE' else 'F2C_'+kind,
            'precision_m': PRECISION_M, 'fallback_reasons': [],
            'layout_family': 'NATIVE_WHOLE_FIELD' if kind == 'NONE' else 'NATIVE_PER_CELL',
            'direction_mode': 'GLOBAL' if kind == 'NONE' else 'PER_CELL',
            'split_angle_rad': split_angle}


def best_rows(prepared, width, objective, check):
    """Let F2C optimize each cell angle; score the complete route elsewhere."""
    import fields2cover as f
    check()
    generator = f.SG_BruteForce(); generator.setStepAngle(math.pi/180)
    generated = generator.generateBestSwaths(getattr(f, 'OBJ_'+objective)(), width, prepared['cells'])
    check()
    result = identify_swaths(prepared, generated, check)
    directions = []
    for cell_id in range(generated.size()):
        rows = generated[cell_id]
        angles, length = [], 0.
        for i in range(rows.size()):
            check(); a, b = rows[i].startPoint(), rows[i].endPoint()
            angle = math.atan2(b.getY()-a.getY(), b.getX()-a.getX()) % math.pi
            angles.append(angle); length += math.hypot(b.getX()-a.getX(), b.getY()-a.getY())
        directions.append({'cell_id': cell_id, 'angle_rad': angles[0],
                           'swath_count': rows.size(), 'swath_length_m': length})
    # Compatibility angle for path metadata; PER_CELL is explicitly displayed.
    dominant = max(directions, key=lambda d: d['swath_length_m'])['angle_rad']
    result.update({k: prepared[k] for k in ('layout_family', 'direction_mode', 'split_angle_rad')})
    result.update(swath_objective=objective, cell_directions=directions, dominant_angle_rad=dominant,
                  angle_search_step_deg=1.)
    return result


def layouts(safe, width, check, progress):
    """Complete five partition variants × three native swath objectives."""
    for index, (kind, split_angle) in enumerate(PARTITIONS):
        label = {'layout_family': 'NATIVE_WHOLE_FIELD' if kind == 'NONE' else 'NATIVE_PER_CELL',
                 'direction_mode': 'GLOBAL' if kind == 'NONE' else 'PER_CELL',
                 'decomposition': kind, 'split_angle_rad': split_angle}
        progress(index*3, 15, **label)
        try:
            prepared = partition_cells(safe, kind, split_angle, check)
        except (ValueError, RuntimeError) as exc:
            check(); yield label, None, str(exc); continue
        for j, objective in enumerate(OBJECTIVES):
            current = {**label, 'swath_objective': objective}
            progress(index*3+j, 15, **current)
            try:
                result = best_rows(prepared, width, objective, check)
            except (ValueError, RuntimeError) as exc:
                check(); yield current, None, str(exc); continue
            yield current, result, None
        progress((index+1)*3, 15, **label)
