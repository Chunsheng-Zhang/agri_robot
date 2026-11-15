"""Native TSP integration, stable decomposition and faithful Route consumption."""
import math
import pytest
from shapely.geometry import Polygon, box, LineString, mapping
from shapely.ops import unary_union, transform
from pyproj.enums import TransformDirection
from agri_base.coordinates import LocalFrame
from agri_base.path_geometry import rotation_envelope, validate_point_turn_path
from agri_path_planning.cell_routing import decompose_rows, sweep_monotone, MAX_NATIVE_GRAPH_NODES
from agri_path_planning.native_route import plan_native_route, route_vertices, reversed_route
from agri_path_planning.point_turn_planner import sample_path, field_cell
from agri_path_planning.optimizer import optimize
from agri_path_planning.field import field_from_data


def field():
    return Polygon([(0,0),(28,0),(28,22),(0,22)], holes=[[(11,8),(17,8),(17,14),(11,14)]])


def check():
    pass


def projected_field():
    frame=LocalFrame(36.6468527633281,117.021433066391)
    def geo(x,y,z=None):
        return frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
    return field_from_data(mapping(transform(geo,field())),frame)


@pytest.mark.parametrize('angle',[0.,math.pi/4,math.pi/2])
@pytest.mark.parametrize('geometry',[field,projected_field])
def test_native_cells_preserve_all_area_and_are_sweep_monotone_even_after_gnss_roundtrip(angle,geometry):
    safe=geometry().buffer(-.3,join_style=2)
    prepared=decompose_rows(safe,.8,angle,check)
    assert len(prepared['polygons'])==4
    assert all(sweep_monotone(p,check) for p in prepared['polygons'])
    assert not any(p.interiors for p in prepared['polygons'])
    assert abs(unary_union(prepared['polygons']).area-safe.area)<.001
    ids=[int(prepared['swaths'].getSwath(i).getId()) for i in range(prepared['swaths'].sizeTotal())]
    assert set(ids)==set(prepared['expected']) and len(ids)==len(set(ids))


def test_decomposition_uses_native_trapezoidal_fallback_instead_of_accepting_a_remaining_hole(monkeypatch):
    import fields2cover as f
    class Unsplit:
        def setSplitAngle(self,a):pass
        def decompose(self,cells):return cells
    monkeypatch.setattr(f,'DECOMP_Boustrophedon',Unsplit)
    prepared=decompose_rows(field().buffer(-.3,join_style=2),.8,0.,check)
    assert prepared['decomposition']=='F2C_TRAPEZOIDAL_FALLBACK'
    assert prepared['fallback_reasons']
    assert all(sweep_monotone(p,check) for p in prepared['polygons'])


def test_native_solver_is_called_and_every_row_and_connection_is_consumed(monkeypatch):
    import fields2cover as f
    native=f.RP_RoutePlannerBase
    calls=[]
    class Observed:
        def genRoute(self,*args):
            calls.append(args[2:])
            return native().genRoute(*args)
        def setStartAndEndPoint(self,p):
            raise AssertionError('The task must remain an open route')
    monkeypatch.setattr(f,'RP_RoutePlannerBase',Observed)
    env=rotation_envelope(.45,.2,.05)
    prepared=decompose_rows(field().buffer(-env['headland_width_m'],join_style=2),.8,0.,check)
    vertices,details=plan_native_route(prepared,check)
    assert calls==[(False,1e-4,True,1,False)]
    assert {v['swath_id'] for v in details['swath_visits']}==set(prepared['expected'])
    assert details['swath_count']==len(prepared['expected'])
    assert details['routing']=='F2C_TSP' and not details['continuous_cells_required']
    assert len(set(details['cell_visit_order']))==details['cell_count']
    assert details['cell_reentries']==len(details['cell_visit_order'])-details['cell_count']
    assert len(details['native_route_connections'])==details['swath_count']
    points,_,_=sample_path(vertices,.1,check)
    validate_point_turn_path(points,field(),env)
    assert LineString(vertices).length==pytest.approx(details['swath_length_m']+details['transition_length_m'])
    rev,meta=reversed_route(vertices,details)
    assert rev==list(reversed(vertices))
    assert [v['swath_id'] for v in meta['swath_visits']]==list(reversed([v['swath_id'] for v in details['swath_visits']]))
    assert meta['native_route_connections'][0]==[]
    assert details['native_tsp']['global_optimality_proven'] is False


def test_native_entry_seed_keeps_only_work_rows_and_internal_connections():
    safe=field().buffer(-.3,join_style=2)
    prepared=decompose_rows(safe,.8,0.,check)
    anchor=(.3,.3)
    vertices,details=plan_native_route(prepared,check,anchor)
    assert details['native_seed']=='ENTRY_ANCHORED_CYCLE_OPENED'
    assert details['native_tsp']['return_connection_executed'] is False
    assert len(details['omitted_depot_connections'])==2
    first,last=details['omitted_depot_connections']
    assert math.dist(first[0],anchor)<1e-5 and math.dist(last[-1],anchor)<1e-5
    assert vertices[0]!=anchor and vertices[-1]!=anchor
    assert len(details['native_route_connections'])==details['swath_count']
    assert details['native_route_connections'][0]==[]
    assert {v['swath_id'] for v in details['swath_visits']}==set(prepared['expected'])
    assert LineString(vertices).length==pytest.approx(details['swath_length_m']+details['transition_length_m'])
    assert safe.buffer(1e-5).covers(LineString(vertices))


def test_entry_seed_never_reuses_a_native_coverage_graph_node_as_depot():
    prepared=decompose_rows(field().buffer(-.3,join_style=2),.8,0.,check)
    anchor=prepared['expected'][0]['start']
    with pytest.raises(ValueError,match='节点重合'):
        plan_native_route(prepared,check,anchor)
    vertices,details=plan_native_route(prepared,check)
    assert vertices and details['native_seed']=='FREE_OPEN_ROUTE'


def synthetic_route(order=(0,1,2),connect=True):
    import fields2cover as f
    rows=[((0.,0.),(1.,0.)),((3.,0.),(4.,0.)),((0.,1.),(1.,1.))]
    route=f.Route()
    for n,identity in enumerate(order):
        a,b=rows[identity]
        if n and connect:
            previous=rows[order[n-1]][1]
            for p in [previous,(previous[0],2.),(a[0],2.),a]:
                route.addConnection(f.Point(*p))
        line=f.LineString();line.addPoint(*a);line.addPoint(*b)
        row=f.Swath(line,.8);row.setId(identity)
        part=f.Swaths();part.push_back(row);route.addSwaths(part)
    prepared={'expected':{i:{'start':a,'end':b,'cell_id':0 if i!=1 else 1} for i,(a,b) in enumerate(rows)},
        'polygons':[box(0,0,1,1),box(3,0,4,1)],'angle':0.,'precision_m':1e-6,
        'decomposition':'F2C_BOUSTROPHEDON','fallback_reasons':[]}
    return route,prepared


def test_interleaved_cell_visits_are_allowed_and_detours_are_not_replaced_by_straight_lines():
    route,prepared=synthetic_route()
    vertices,details=route_vertices(route,prepared,check)
    assert details['cell_visit_order']==[0,1,0] and details['cell_reentries']==1
    assert not details['continuous_cells_required']
    assert (1.,2.) in vertices and (3.,2.) in vertices
    assert details['transition_length_m']>math.dist((1,0),(3,0))+math.dist((4,0),(0,1))
    reverse,meta=reversed_route(vertices,details)
    assert meta['cell_reentries']==1 and len(meta['native_route_connections'])==3
    assert reverse[0]==vertices[-1]


@pytest.mark.parametrize('order,connect,reason',[((0,1),True,'完整访问'),((0,0,2),True,'重复访问'),((0,1,2),False,'完整连接')])
def test_invalid_native_routes_never_get_an_implicit_connection_or_missing_row(order,connect,reason):
    route,prepared=synthetic_route(order,connect)
    with pytest.raises(ValueError,match=reason):
        route_vertices(route,prepared,check)


def test_disconnected_work_area_is_rejected_before_entering_native_solver():
    with pytest.raises(ValueError,match='不连通'):
        decompose_rows(unary_union([box(0,0,4,4),box(6,0,10,4)]),1.,0.,check)


def test_native_graph_memory_guard_is_not_an_angle_candidate_limit():
    with pytest.raises(ValueError,match='内存保护'):
        decompose_rows(box(0,0,100,80),.1,0.,check)


def test_full_native_optimization_preserves_policy_and_entry_distance_without_closed_return():
    p=dict(operation_width=1.,robot_width=.2,robot_length=.45,rotation_safety_margin=.05,
           sampling_distance=.1,quality_resolution=.15,
           entry={'x':-2.,'y':-2.,'latitude':36.,'longitude':117.,'source':'coordinates'})
    result=optimize(field().wkb,p)
    assert result['optimization']['state']=='SUCCEEDED'
    assert result['metadata']['policy_version']==7
    assert {c['variant'] for c in result['optimization']['candidates'] if c['executable']}=={0,1,2,3}
    selected=result['optimization']['selected'];entry=result['metadata']['entry']
    assert entry['access_verified'] is False
    assert selected['entry_length_m']==pytest.approx(entry['distance_m'])
    assert selected['length_m']==pytest.approx(selected['working_length_m']+entry['distance_m'])
    assert selected['swath_length_m']+selected['transition_length_m']==pytest.approx(selected['working_length_m'])
    assert selected['native_tsp']['start_end_policy'] in ('FREE_OPEN_ROUTE','ENTRY_ANCHORED_CYCLE_OPENED')
    assert selected['native_tsp']['return_connection_executed'] is False
    validate_point_turn_path(result['points'],field(),result['metadata']['rotation_envelope'])


@pytest.mark.parametrize('angle',[0.,math.pi/2,.37])
def test_edge_rows_use_original_half_width_and_fixed_pitch_without_changing_turn_clearance(angle):
    from shapely.affinity import rotate
    original=rotate(box(0,0,24,18),angle,origin=(0,0),use_radians=True)
    env=rotation_envelope(.45,.2,.05)
    safe=original.buffer(-env['headland_width_m'],join_style=2)
    before=original.wkb
    prepared=decompose_rows(safe,.8,angle,check,coverage_field=original)
    levels=sorted({row['start'][1] for row in prepared['expected'].values()})
    assert len(levels)==23
    assert levels[0]==pytest.approx(.4) and levels[-1]==pytest.approx(17.6)
    assert all(b-a==pytest.approx(.8) for a,b in zip(levels[:-2],levels[1:-1]))
    assert levels[-1]-levels[-2]==pytest.approx(.4)
    for row in prepared['expected'].values():
        assert sorted((row['start'][0],row['end'][0]))==pytest.approx(
            [env['headland_width_m'],24-env['headland_width_m']],abs=1e-6)
    assert original.wkb==before
    meta=prepared['row_layout']
    assert meta['last_side_row_added'] and meta['turn_clearance_unchanged']
    vertices,_=plan_native_route(prepared,check)
    points,_,_=sample_path(vertices,.1,check)
    validate_point_turn_path(points,original,env)
    assert safe.buffer(1e-5).covers(LineString(vertices))


def test_edge_rows_never_approach_closer_than_the_full_rotation_envelope_when_width_is_small():
    original=box(0,0,6,4);env=rotation_envelope(.45,.2,.05)
    safe=original.buffer(-env['headland_width_m'],join_style=2)
    prepared=decompose_rows(safe,.2,0.,check,coverage_field=original)
    levels=sorted({row['start'][1] for row in prepared['expected'].values()})
    assert levels[0]>env['headland_width_m']>.1
    assert levels[-1]<4-env['headland_width_m']
    vertices,_=plan_native_route(prepared,check)
    points,_,_=sample_path(vertices,.1,check)
    validate_point_turn_path(points,original,env)


def test_edge_rows_keep_small_cells_and_use_native_clipping_around_holes():
    original=Polygon([(0,0),(28,0),(28,22),(0,22)],holes=[[(11,1),(17,1),(17,14),(11,14)]])
    prepared=decompose_rows(original.buffer(-.3,join_style=2),3.,0.,check,coverage_field=original)
    assert prepared['row_layout']['fallback_cell_count']>0
    assert {row['cell_id'] for row in prepared['expected'].values()}==set(range(len(prepared['polygons'])))
    original=field();env=rotation_envelope(.45,.2,.05)
    prepared=decompose_rows(original.buffer(-env['headland_width_m'],join_style=2),.8,0.,check,coverage_field=original)
    vertices,details=plan_native_route(prepared,check)
    points,_,_=sample_path(vertices,.1,check)
    validate_point_turn_path(points,original,env)
    assert not LineString(vertices).intersects(box(11,8,17,14))
    assert details['row_layout']['policy']=='EDGE_ALIGNED_FIXED_SPACING'


def test_edge_rows_avoid_coincident_scan_levels_on_native_cell_seams():
    original=Polygon([(0,0),(20,0),(20,14),(0,14)],holes=[[(7,6.8),(12,6.8),(12,10),(7,10)]])
    safe=original.buffer(-.3,join_style=2)
    prepared=decompose_rows(safe,1.,0.,check,coverage_field=original)
    assert all(abs(row['start'][1]-6.5)>1e-4 for row in prepared['expected'].values())
    vertices,_=plan_native_route(prepared,check)
    assert safe.buffer(1e-5).covers(LineString(vertices))
