"""Boundary-anchored fixed-pitch layouts; no independently searched offset.

An edge fixes both the direction and the lattice origin. Extend the lattice in
both transverse directions so a concave reference edge cannot omit other arms.
All geometry here is worker-local; stored field boundaries are never changed.
"""
import math
from shapely.geometry import LineString
from shapely.ops import unary_union

EPS = 2e-4


def boundary_bases(field, width, clearance, check):
    result=[]; seen=set()
    for ring_index,ring in enumerate([field.exterior,*field.interiors]):
        coords=list(ring.coords)
        interior_left=1 if ring.is_ccw != bool(ring_index) else -1
        for edge_index,(a,b) in enumerate(zip(coords,coords[1:])):
            check();dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
            if length<1e-6:continue
            angle=math.atan2(dy,dx)%math.pi;c,s=math.cos(angle),math.sin(angle)
            sign=1 if interior_left*(s*dy+c*dx)>0 else -1
            edge_y=-s*(a[0]+b[0])/2+c*(a[1]+b[1])/2
            offset=max(width/2,clearance+EPS)
            anchor=edge_y+sign*offset
            phase=anchor%width
            if min(phase,width-phase)<1e-6:phase=0.
            key=(round(angle,8),round(phase,6))
            if key in seen:continue
            seen.add(key)
            result.append({'kind':'EDGE','ring':'OUTER' if ring_index==0 else 'HOLE',
                'ring_index':ring_index,'edge_index':edge_index,'edge_length_m':length,
                'edge_points':[list(a[:2]),list(b[:2])], 'angle_rad':angle,
                'anchor_y_local_m':anchor,'inward_sign':sign,
                'target_edge_offset_m':width/2,'actual_anchor_offset_m':offset})
    return result


def stable_levels(values, safe, polygons, check):
    vertices={y for p in polygons for x,y,*_ in p.exterior.coords}
    levels=[]
    for y in sorted(values):
        check()
        if not safe.bounds[1]+EPS/2<y<safe.bounds[3]-EPS/2:continue
        if any(abs(y-v)<EPS for v in vertices):
            for z in (y+2*EPS,y-2*EPS):
                if (safe.bounds[1]+EPS/2<z<safe.bounds[3]-EPS/2
                        and all(abs(z-v)>=EPS for v in vertices)):
                    y=z;break
            else:raise ValueError('沿边作业线与分区边界重合，无法稳定生成原生路由')
        if not levels or abs(levels[-1]-y)>1e-6:levels.append(y)
    return sorted(levels)


def layout_choices(original, safe, polygons, width, basis, check):
    """Compare the base lattice with a finite, positive-gain supplement set.

    Supplements are full safe-clipped rows, not unsafe shortcuts. The complete
    route score decides whether their coverage benefit justifies overlap/turns.
    This does not enumerate every subset of local supplements.
    """
    lo=max(original.bounds[1]+width/2,safe.bounds[1]+EPS)
    hi=min(original.bounds[3]-width/2,safe.bounds[3]-EPS)
    anchor=basis.get('anchor_y_local_m',lo)
    k0=math.ceil((safe.bounds[1]+EPS-anchor)/width)
    k1=math.floor((safe.bounds[3]-EPS-anchor)/width)
    if k1-k0+1>1500:raise ValueError('作业行数量超过1500条几何预算')
    levels=stable_levels([anchor+k*width for k in range(k0,k1+1)],safe,polygons,check)
    if not levels:levels=stable_levels([(safe.bounds[1]+safe.bounds[3])/2],safe,polygons,check)
    if not levels:raise ValueError('安全工作区过窄，无法稳定放置作业行')
    basis={**basis,'anchor_y_local_m':anchor}
    base={'levels':levels,'basis':basis,'supplement':'NONE','added_levels':[],'added_area_m2':0.}
    result=[base]
    x0,_,x1,_=original.bounds
    def line(y):return LineString([(x0-1,y),(x1+1,y)])
    # Include required narrow-cell fallback rows in the gain calculation.
    cuts=[]
    for poly in polygons:
        check();part=[poly.intersection(line(y)) for y in levels if poly.bounds[1]<y<poly.bounds[3]]
        if not part:part=[poly.intersection(line((poly.bounds[1]+poly.bounds[3])/2))]
        cuts.extend(p for p in part if not p.is_empty)
    covered=unary_union(cuts).buffer(width/2)
    targets=[lo,hi]
    for ring_index,ring in enumerate([original.exterior,*original.interiors]):
        interior_left=1 if ring.is_ccw != bool(ring_index) else -1
        for a,b in zip(ring.coords,list(ring.coords)[1:]):
            check();dx,dy=b[0]-a[0],b[1]-a[1]
            if abs(dy)<=1e-6 and abs(dx)>1e-6:
                sign=1 if interior_left*dx>0 else -1
                targets.append((a[1]+b[1])/2+sign*width/2)
    added=[];gain=0.
    for y in stable_levels(targets,safe,polygons,check):
        check()
        if any(abs(y-z)<1e-6 for z in levels+added):continue
        cut=safe.intersection(line(y))
        if cut.is_empty or cut.length<1e-5:continue
        footprint=cut.buffer(width/2)
        improvement=footprint.difference(covered).intersection(original).area
        if improvement<=1e-6:continue
        added.append(y);gain+=improvement;covered=covered.union(footprint)
    if added:
        if len(levels)+len(added)>1500:raise ValueError('作业行数量超过1500条几何预算')
        result.append({**base,'levels':sorted(levels+added),'supplement':'POSITIVE_GAIN_BOUNDARY_ROWS',
                       'added_levels':added,'added_area_m2':gain})
    return result


def native_rows(cells, polygons, original, width, layout, check):
    import fields2cover as f
    x0,_,x1,_=original.bounds;levels=layout['levels'];fallback=0
    def clip(level,cell):
        line=f.LineString();line.addPoint(x0-1.,float(level));line.addPoint(x1+1.,float(level))
        rows=f.Swaths();rows.append(line,cell,float(width));return rows
    generated=f.SwathsByCells()
    for i,poly in enumerate(polygons):
        check();cell=cells.getGeometry(i);rows=f.Swaths()
        for level in levels:
            check()
            if poly.bounds[1]<level<poly.bounds[3]:rows.append(clip(level,cell))
        if not rows.size():
            rows.append(clip((poly.bounds[1]+poly.bounds[3])/2,cell));fallback+=1
        generated.emplace_back(rows)
    return generated,{'policy':'EDGE_ANCHORED_FIXED_SPACING','nominal_spacing_m':width,
        'implement_width_m':width,'target_edge_offset_m':width/2,'basis':layout['basis'],
        'first_center_y_local_m':levels[0],'last_center_y_local_m':levels[-1],
        'scan_level_count':len(levels),'last_side_row_added':bool(layout['added_levels']),
        'last_gap_m':levels[-1]-levels[-2] if len(levels)>1 else None,
        'supplement':layout['supplement'],'supplement_level_count':len(layout['added_levels']),
        'supplement_gain_m2':layout['added_area_m2'],'fallback_cell_count':fallback,
        'lattice_extended_both_sides':True,'turn_clearance_unchanged':True}
