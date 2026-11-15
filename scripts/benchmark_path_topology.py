"""Geometry diagnostics, not a declaration that every crossing is a bad route."""
import math
from shapely.geometry import LineString,Point,GeometryCollection,mapping
from shapely.ops import unary_union
from shapely.strtree import STRtree
from benchmark_coverage_metrics import compact_path


def route_segments(vertices,details=None):
    """Recover chronological work/connection segments, with an honest fallback."""
    points=compact_path(vertices)
    unknown=[(a,b,'unknown') for a,b in zip(points,points[1:]) if math.dist(a,b)>1e-9]
    if not details:return unknown,'unavailable'
    connections=details.get('native_route_connections');visits=details.get('swath_visits')
    if not connections or not visits or len(connections)!=len(visits):return unknown,'unavailable'
    result=[];cursor=points[0]
    for i,c in enumerate(connections):
        if c:
            if math.dist(cursor,c[0])>1e-5:return unknown,'unavailable'
            result.extend((a,b,'transition') for a,b in zip(c,c[1:]) if math.dist(a,b)>1e-9)
            cursor=c[-1]
        if i+1<len(connections):
            if not connections[i+1]:return unknown,'unavailable'
            end=connections[i+1][0]
        else:end=points[-1]
        if math.dist(cursor,end)>1e-9:result.append((cursor,end,'work'))
        cursor=end
    if abs(sum(math.dist(a,b) for a,b,_ in result)-sum(math.dist(a,b) for a,b,_ in unknown))>1e-4:
        return unknown,'unavailable'
    return result,'reconstructed'


def line_parts(geometry):
    if geometry.geom_type in ('LineString','LinearRing'):return [geometry]
    return [part for g in getattr(geometry,'geoms',[]) for part in line_parts(g)]


def point_parts(geometry):
    if geometry.geom_type=='Point':return [geometry]
    return [part for g in getattr(geometry,'geoms',[]) for part in point_parts(g)]


def topology_metrics(vertices,details=None,tolerance=1e-5):
    segments,role_state=route_segments(vertices,details)
    snap=lambda p:(round(p[0]/tolerance)*tolerance,round(p[1]/tolerance)*tolerance)
    lines=[];roles=[]
    for a,b,role in segments:
        a,b=snap(a),snap(b)
        if math.dist(a,b)>tolerance/10:lines.append(LineString([a,b]));roles.append(role)
    if not lines:raise ValueError('No nonzero path segments')
    tree=STRtree(lines);crossings={};pairs=0;classes={};repeated=[];extra_by_role={}
    for i,line in enumerate(lines):
        prior_overlap=[]
        for j in tree.query(line,predicate='intersects'):
            j=int(j)
            if j>=i:continue
            other=lines[j];intersection=line.intersection(other)
            overlaps=line_parts(intersection)
            repeated.extend(g for g in overlaps if g.length>tolerance)
            prior_overlap.extend(g for g in overlaps if g.length>tolerance)
            if abs(i-j)<=1:continue
            for point in point_parts(intersection):
                # Exclude shared endpoints and T-junctions from proper crossings.
                ends=[Point(p) for p in (*line.coords,*other.coords)]
                if any(point.distance(p)<=tolerance for p in ends):continue
                key=snap(point.coords[0]);crossings[key]=point
                pairs+=1;kind='/'.join(sorted((roles[i],roles[j])))
                classes[kind]=classes.get(kind,0)+1
        extra_by_role[roles[i]]=extra_by_role.get(roles[i],0.)+(unary_union(prior_overlap).length if prior_overlap else 0.)
    length=sum(g.length for g in lines);unique=unary_union(lines).length
    repeat=unary_union(repeated) if repeated else GeometryCollection()
    reversals=[]
    for i,(a,b) in enumerate(zip(lines,lines[1:])):
        u=(a.coords[-1][0]-a.coords[0][0],a.coords[-1][1]-a.coords[0][1])
        v=(b.coords[-1][0]-b.coords[0][0],b.coords[-1][1]-b.coords[0][1])
        if math.dist(a.coords[-1],b.coords[0])<=tolerance and (u[0]*v[0]+u[1]*v[1])/(a.length*b.length)<-math.cos(math.radians(.1)):
            reversals.append(list(a.coords[-1]))
    return {'tolerance_m':tolerance,'role_reconstruction':role_state,'segment_count':len(lines),
        'proper_crossing_locations':len(crossings),'proper_crossing_segment_pairs':pairs,
        'crossing_pair_roles':classes,'centerline_length_m':length,'unique_centerline_length_m':unique,
        'extra_retraced_by_current_role_m':extra_by_role,
        'proper_crossing_locations_per_km':len(crossings)/(length/1000),
        'extra_retraced_length_m':max(0.,length-unique),'extra_retraced_fraction':max(0.,length-unique)/length,
        'repeated_centerline_unique_m':repeat.length,'immediate_collinear_backtracks':len(reversals),
        'crossing_points':[list(p.coords[0]) for p in crossings.values()],
        'repeated_centerline_geometry':mapping(repeat),'backtrack_points':reversals,
        'interpretation':'Proper nonadjacent interior crossings and exact retracing at 10 micrometre precision; planned shared corridors may be legitimate. Not field safety or agronomic truth.'}
