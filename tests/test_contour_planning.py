"""Inside-out contour geometry and bounded execution; never connects to ROS."""
import math
from threading import Event

import pytest
from shapely.geometry import LineString, Point, Polygon, box
from shapely import affinity

from agri_path_planning import contour
from agri_path_planning.contour import Builder, plan_contour
from agri_backend.missions import CoverageParameters


def make(field=None, **kw):
    params=dict(width=1.,headland=1.,radius=.6,spacing=.1,robot_width=.35,continuity='DISCONTINUOUS')
    params.update(kw)
    return plan_contour((field if field is not None else box(0,0,20,14)).wkb,**params)


@pytest.mark.parametrize('continuity',['DISCONTINUOUS','CONTINUOUS'])
def test_inside_out_connected_curves_respect_spacing_radius_and_chassis(continuity):
    value=make(continuity=continuity)
    points=value['points'];meta=value['metadata'];layers=meta['layers']
    assert meta['direction']=='INSIDE_OUT' and meta['ring_count']>=3
    assert layers[0]['level']==max(layer['level'] for layer in layers)
    assert layers[-1]['level']==0
    assert all(a['offset_m']>=b['offset_m'] for a,b in zip(layers,layers[1:]))
    assert box(0,0,20,14).buffer(-.35/2+1e-5).covers(LineString([p[:2] for p in points]))
    assert max(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))<=.1+1e-4
    # Each section is open. Inter-layer joins share exact indices with the
    # preceding exit and following entry, rather than closing another full lap.
    assert meta['topology']=='OPEN_SPIRAL' and meta['closed_laps'] is False
    assert LineString([p[:2] for p in points]).is_simple
    for layer in layers:
        a,b=points[layer['start_index']],points[layer['end_index']]
        assert not layer['closed'] and math.dist(a[:2],b[:2])>1e-6
    assert len(meta['connections'])==len(layers)-1
    for i,join in enumerate(meta['connections']):
        assert join['start_index']==layers[i]['end_index']
        assert join['end_index']==layers[i+1]['start_index']
        assert join['end_index']>join['start_index']
    # A repeated start XY must not terminate the forward controller early.
    from agri_path_tracking.controller import Polyline,PurePursuit
    controller=PurePursuit(Polyline([p[:2] for p in points]))
    assert not controller.command(*points[0])[-1]
    curvature=[]
    for a,b in zip(points,points[1:]):
        ds=math.dist(a[:2],b[:2])
        if ds>1e-5:curvature.append(abs(contour.wrap(b[2]-a[2]))/ds)
    assert max(curvature)<=1/.6*1.03


def test_skew_quadrilateral_and_translated_map_preserve_global_coordinates():
    field=affinity.translate(Polygon([(0,0),(20,0),(18,14),(2,12)]),xoff=347.2,yoff=-221.8)
    value=make(field,continuity='CONTINUOUS')
    line=LineString([p[:2] for p in value['points']])
    assert field.buffer(-.175+1e-5).covers(line)
    assert line.bounds[0]>347 and line.bounds[1]<-210
    assert value['metadata']['layers'][0]['level']>0


def test_forbidden_hole_is_never_bridged_by_a_straight_jump():
    field=Polygon([(0,0),(24,0),(24,20),(0,20)],holes=[[(9,8),(15,8),(15,12),(9,12)]])
    value=make(field,continuity='CONTINUOUS')
    assert any(layer['kind']=='hole' for layer in value['metadata']['layers'])
    line=LineString([p[:2] for p in value['points']])
    assert field.buffer(-.175+1e-5).covers(line)
    assert not line.intersects(box(9,8,15,12))


def test_infeasible_corner_space_reports_failure_instead_of_partial_path():
    with pytest.raises(ValueError,match='无可行|最外层'):
        make(box(0,0,2,2),headland=0,radius=2)
    with pytest.raises(ValueError,match='无可行'):
        make(box(0,0,2,2),headland=3)
    for kw in [dict(width=0),dict(radius=float('nan')),dict(headland=-1),dict(continuity='bad')]:
        with pytest.raises(ValueError):make(**kw)


def test_cancellation_deadline_and_layer_budgets_never_return_partial_geometry(monkeypatch):
    event=Event();event.set();monkeypatch.setattr(contour,'_CANCEL',event)
    with pytest.raises(ValueError,match='取消'):make()
    event.clear()
    with pytest.raises(ValueError,match='超时'):make(timeout=1e-9)
    monkeypatch.setattr(contour,'MAX_LAYERS',1)
    with pytest.raises(ValueError,match='层数超限'):make()


def test_new_mode_is_additive_and_saved_task_validation_accepts_it():
    for mode in ['BOUSTROPHEDON','SNAKE','SPIRAL','CONTOUR']:
        assert CoverageParameters(route_mode=mode).route_mode==mode
    assert CoverageParameters().route_mode=='BOUSTROPHEDON'


def test_raw_order_matches_reference_o_rote_coordinates_then_reverses_inside_out():
    # Fixed output from the user's unmodified Coordinateself.o_rote fixture.
    # No runtime dependency on the reference checkout or its plotting scripts.
    field=Polygon([(6436.9,26841.3),(6524.3,26832.9),(6515.1,26926.4),(6450.7,26903.9)])
    expected=[(6440.802824961023,26844.139645507366),(6453.440671902017,26901.467849746947),
        (6512.313857194715,26922.03689740356),(6520.734511268124,26836.457423940094),
        (6444.011671459138,26843.831243784436),(6444.705649922046,26846.979291014733),
        (6456.1813438040335,26899.035699493892),(6509.527714389428,26917.673794807113),
        (6517.169022536249,26840.01484788019),(6447.9144964201605,26846.670889291803)]
    result=make(field,width=3.2,headland=1.6,radius=1.,continuity='DUBINS_CC')
    meta=result['metadata'];raw=meta['raw_spiral_vertices']
    assert meta['ordering']=='REFERENCE_O_REVERSED'
    assert len(raw)>len(expected)
    assert max(math.dist(a,b) for a,b in zip(reversed(raw),expected))<1e-8
    assert field.boundary.distance(Point(raw[0]))>field.boundary.distance(Point(raw[-1]))
    assert LineString([p[:2] for p in result['points']]).is_simple


@pytest.mark.parametrize('curve',['DUBINS','DUBINS_CC'])
def test_projected_boundary_roundoff_does_not_collapse_spiral_to_outer_edge(curve):
    from agri_backend.missions import LocalFrame
    from pyproj.enums import TransformDirection
    f=LocalFrame(36.6468527633281,117.021433066391);vertices=[]
    for x,y in [(0,0),(20,0),(20,14),(0,14)]:
        lon,lat=f.transformer.transform(f.e0+x,f.n0+y,direction=TransformDirection.INVERSE)
        e,n=f.transformer.transform(lon,lat);vertices.append((e-f.e0,n-f.n0))
    result=make(Polygon(vertices),continuity=curve)
    assert result['metadata']['ring_count']>=3
    assert LineString([p[:2] for p in result['points']]).is_simple


@pytest.mark.parametrize('width,headland,radius',[(.8,0.,1.),(.8,0.,.5),(.5,0.,.2),(.5,1.,.2)])
def test_actual_five_point_concave_field_uses_reference_o_not_old_seam_branch(width,headland,radius):
    # Geometry from the user's 2026-11-10 planning attempts. Keep the concave
    # fifth corner; convexifying it would silently change the working area.
    field=Polygon([(0.,0.),(-.0903990528313443,-16.356757320463657),
        (12.755411364603788,-16.490246892441064),(10.91983889346011,-5.4276303220540285),
        (10.95955598494038,-.14654403179883957)])
    assert field.area/field.convex_hull.area<.98
    value=make(field,width=width,headland=headland,radius=radius,continuity='DUBINS')
    p=value['points'];m=value['metadata']
    assert m['ordering']=='REFERENCE_O_REVERSED' and m['ring_count']>=3
    assert m['layers'][-1]['level']==0 and all(not r['closed'] for r in m['layers'])
    line=LineString([q[:2] for q in p])
    assert line.is_simple and field.buffer(-.175+1e-5).covers(line)
    assert max(abs(contour.wrap(b[2]-a[2]))/math.dist(a[:2],b[:2])
        for a,b in zip(p,p[1:]) if math.dist(a[:2],b[:2])>1e-5)<=1/radius*1.03


@pytest.mark.parametrize('curve',['DUBINS_CC','REEDS_SHEPP_HC'])
def test_shallow_concave_native_cc_corner_gets_local_entry_space(curve):
    field=Polygon([(0.,0.),(-.0903990528313443,-16.356757320463657),
        (12.755411364603788,-16.490246892441064),(10.91983889346011,-5.4276303220540285),
        (10.95955598494038,-.14654403179883957)])
    value=make(field,width=1.,headland=1.,radius=.6,continuity=curve)
    assert value['metadata']['ring_count']>=2
    assert value['metadata']['ordering']=='REFERENCE_O_REVERSED'
    assert LineString([p[:2] for p in value['points']]).is_simple
    # Even if a native corner has no simple cross-layer connection, do not
    # silently publish a single outside pass in place of the requested spiral.
    with pytest.raises(ValueError,match='O形连续转角无可行路径'):
        make(field,width=.5,headland=0.,radius=.2,continuity=curve)


@pytest.mark.parametrize('curve',['DUBINS','DUBINS_CC','REEDS_SHEPP','REEDS_SHEPP_HC'])
def test_vehicle_one_metre_radius_yields_one_open_winding_in_skew_field(curve):
    field=Polygon([(0,0),(28,-2),(25,20),(3,17)])
    value=make(field,headland=1.,width=1.2,radius=1.,continuity=curve)
    points=value['points'];meta=value['metadata']
    assert meta['ring_count']>=3 and meta['topology']=='OPEN_SPIRAL'
    assert LineString([p[:2] for p in points]).is_simple
    assert field.buffer(-.175+1e-5).covers(LineString([p[:2] for p in points]))
    assert math.dist(points[0][:2],points[-1][:2])>1
    curvature=[abs(contour.wrap(b[2]-a[2]))/math.dist(a[:2],b[:2])
               for a,b in zip(points,points[1:]) if math.dist(a[:2],b[:2])>1e-5]
    assert max(curvature)<=1.03


def test_external_headland_approach_still_has_explicit_continuous_joins():
    value=Builder(box(0,0,24,18),1.,0.,1.,.1,.35,'DUBINS_CC').generate(
        max_offset=3.,start_pose=(12.,9.,0.))
    meta=value['metadata']
    assert meta['connections'][0]['from_layer']==-1
    assert meta['connections'][0]['start_index']==0
    assert all(not layer['closed'] for layer in meta['layers'])
    assert box(0,0,24,18).buffer(-.175+1e-5).covers(LineString([p[:2] for p in value['points']]))


def test_pursuit_traverses_the_open_spiral_and_every_seam_in_synthetic_kinematics():
    from agri_path_tracking.controller import Polyline,PurePursuit
    value=make(box(0,0,16,12),headland=.5,width=1.2,radius=1.,continuity='DUBINS_CC')
    points=value['points'];path=Polyline([p[:2] for p in points])
    c=PurePursuit(path,lookahead=.5,speed=.3,completion_tolerance=.12)
    x,y,yaw=points[0];largest_error=0.
    for _ in range(20000):
        old=c.progress;v,w,distance,*_,done=c.command(x,y,yaw)
        assert c.progress>=old;largest_error=max(largest_error,distance)
        if done:break
        x+=v*math.cos(yaw)*.05;y+=v*math.sin(yaw)*.05;yaw+=w*.05
    assert done and largest_error<.4
    assert c.progress>=path.total-.12
