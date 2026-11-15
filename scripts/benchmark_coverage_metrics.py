"""Independent continuous swept-disk checks for offline planning benchmarks.

For segment i the footprint is convex, so a point can enter it only once.
Overlap with earlier segment footprints is a second visit unless the point is
inside the disk at i's start (then the immediately preceding visit continues).
An earlier second visit is already in the accumulated repeat region. This gives
the same connected-visit definition as AreaGrid, without a large-field raster.
"""
import math
import numpy as np
from shapely.geometry import LineString,Point,GeometryCollection
from shapely.ops import unary_union
from shapely.strtree import STRtree


def compact_path(points):
    vertices=[]
    for p in points:
        p=tuple(p[:2])
        if vertices and math.dist(vertices[-1],p)<1e-9:continue
        while len(vertices)>=2:
            a,b=vertices[-2:];u=(b[0]-a[0],b[1]-a[1]);v=(p[0]-b[0],p[1]-b[1])
            if u[0]*v[0]+u[1]*v[1]>0 and abs(u[0]*v[1]-u[1]*v[0])<=1e-7*math.dist(a,p):vertices.pop()
            else:break
        vertices.append(p)
    return vertices


def swept_metrics(points, field, width, check=lambda:None, quad_segs=32):
    if not points or len(points)<2:raise ValueError('Need a complete path')
    # RDP simplification can erase collinear retracing; keep every reversal.
    vertices=compact_path(points);line=LineString(vertices)
    segments=[LineString([a,b]) for a,b in zip(vertices,vertices[1:]) if math.dist(a,b)>1e-9]
    radius=width/2
    buffers=[s.buffer(radius,quad_segs=quad_segs) for s in segments]
    tree=STRtree(buffers);repeats=[]
    for i,footprint in enumerate(buffers):
        if i%32==0:check()
        prior=[int(j) for j in tree.query(footprint,predicate='intersects') if int(j)<i]
        if not prior:continue
        intersection=footprint.intersection(unary_union([buffers[j] for j in prior]))
        joint=Point(segments[i].coords[0]).buffer(radius,quad_segs=quad_segs)
        reentry=intersection.difference(joint)
        if not reentry.is_empty:repeats.append(reentry)
    covered=unary_union(buffers).intersection(field)
    repeated=(unary_union(repeats).intersection(field) if repeats else GeometryCollection())
    area=field.area
    return {'coverage_percent':100*covered.area/area,'overlap_percent':100*repeated.area/area,
        'covered_m2':covered.area,'overlap_m2':repeated.area,'missed_m2':max(0.,area-covered.area),
        'field_area_m2':area,'length_m':line.length,'segments':len(segments),
        'method':'continuous connected-visit disk footprints','buffer_quadrant_segments':quad_segs,
        'path_simplification_tolerance_m':1e-7}
