"""Real boundary anchors, full-field concave layouts and four-objective ranking."""
import math
import pytest
from shapely.affinity import rotate
from shapely.geometry import Polygon,box
from agri_base.path_geometry import rotation_envelope,point_turn_indices,validate_point_turn_path
from agri_path_planning.edge_candidates import boundary_bases,layout_choices
from agri_path_planning.cell_routing import decompose_cells,rows_from_cells
from agri_path_planning.native_route import plan_native_route
from agri_path_planning.point_turn_planner import sample_path
from agri_path_planning.optimizer import optimize,score


def check():pass


FIVE_POINT=Polygon([(0,0),(-.0903990528313443,-16.356757320463657),
    (12.755411364603788,-16.490246892441064),(10.91983889346011,-5.4276303220540285),
    (10.95955598494038,-.14654403179883957)])


def settings():
    return dict(operation_width=.8,robot_width=.2,robot_length=.45,rotation_safety_margin=.05,
                sampling_distance=.1,quality_resolution=.05)


def test_parallel_edges_retain_distinct_layouts_and_ring_reversal_preserves_physical_candidates():
    field=box(0,0,12.7,9.3)
    normal=boundary_bases(field,.8,.3,check)
    reversed_=boundary_bases(Polygon(list(field.exterior.coords)[::-1]),.8,.3,check)
    assert len(normal)==4
    def signature(bases):
        return {(round(b['angle_rad'],7),round(b['anchor_y_local_m']%.8,6)) for b in bases}
    assert signature(normal)==signature(reversed_)
    assert len([b for b in normal if abs(b['angle_rad'])<1e-7])==2


def test_hole_edge_anchors_are_on_the_field_side_and_clearance_overrides_small_half_width():
    field=Polygon([(0,0),(20,0),(20,16),(0,16)],holes=[[(7.13,5.17),(12.19,5.17),(12.19,10.23),(7.13,10.23)]])
    bases=boundary_bases(field,.2,.3,check)
    assert any(b['ring']=='HOLE' for b in bases)
    for b in bases:
        assert b['actual_anchor_offset_m']==pytest.approx(.3002)
        if b['ring']=='HOLE':
            a,c=b['edge_points'];x=(a[0]+c[0])/2;y=(a[1]+c[1])/2
            dx=-math.sin(b['angle_rad'])*b['inward_sign']*.3002
            dy=math.cos(b['angle_rad'])*b['inward_sign']*.3002
            from shapely.geometry import Point
            assert field.contains(Point(x+dx,y+dy))


def test_concave_internal_reference_edge_extends_rows_to_both_sides_and_keeps_all_cells():
    field=Polygon([(0,0),(24,0),(24,10),(12.3,10),(12.3,22),(0,22)])
    env=rotation_envelope(.45,.2,.05)
    basis=next(b for b in boundary_bases(field,.8,env['headland_width_m'],check) if b['edge_index']==3)
    angle=basis['angle_rad'];cells=decompose_cells(field.buffer(-env['headland_width_m'],join_style=2),angle,check)
    original=rotate(field,-angle,origin=(0,0),use_radians=True)
    layout=layout_choices(original,cells['safe'],cells['polygons'],.8,basis,check)[0]
    anchor=basis['anchor_y_local_m']
    assert min(layout['levels'])<anchor<max(layout['levels'])
    prep=rows_from_cells(cells,.8,check,field,layout)
    assert {r['cell_id'] for r in prep['expected'].values()}==set(range(len(cells['polygons'])))
    vertices,meta=plan_native_route(prep,check)
    poses,_,_=sample_path(vertices,.1,check)
    validate_point_turn_path(poses,field,env)
    assert meta['swath_count']==len(prep['expected'])


def test_actual_left_edge_base_has_fixed_pitch_without_the_nearly_coincident_long_row():
    env=rotation_envelope(.45,.2,.05)
    basis=next(b for b in boundary_bases(FIVE_POINT,.8,env['headland_width_m'],check) if b['edge_index']==0)
    angle=basis['angle_rad'];safe=FIVE_POINT.buffer(-env['headland_width_m'],join_style=2)
    cells=decompose_cells(safe,angle,check);original=rotate(FIVE_POINT,-angle,origin=(0,0),use_radians=True)
    layouts=layout_choices(original,cells['safe'],cells['polygons'],.8,basis,check)
    assert layouts[0]['supplement']=='NONE' and len(layouts)==2
    assert layouts[0]['levels'][-1]==pytest.approx(-.4)
    assert all(b-a==pytest.approx(.8) for a,b in zip(layouts[0]['levels'],layouts[0]['levels'][1:]))
    assert layouts[1]['added_area_m2']>0
    prep=rows_from_cells(cells,.8,check,FIVE_POINT,layouts[0])
    assert len(prep['expected'])==17 and prep['row_layout']['fallback_cell_count']==1
    vertices,_=plan_native_route(prep,check)
    poses,_,_=sample_path(vertices,.1,check);validate_point_turn_path(poses,FIVE_POINT,env)


def test_actual_edges_are_evaluated_and_the_final_marker_count_is_used_in_the_score():
    p=settings();before=FIVE_POINT.wkb
    v=optimize(FIVE_POINT.wkb,p);r=v['optimization'];chosen=r['selected']
    assert v['metadata']['policy_version']==7 and r['schema_version']==3
    expected=boundary_bases(FIVE_POINT,.8,v['metadata']['rotation_envelope']['headland_width_m'],check)
    actual_angles={round(c['angle_rad'],8) for c in r['candidates']}
    assert {round(b['angle_rad'],8) for b in expected}<=actual_angles
    assert any((c.get('row_layout') or {}).get('supplement')=='NONE' for c in r['candidates'])
    assert any((c.get('row_layout') or {}).get('last_side_row_added') for c in r['candidates'])
    assert chosen['point_turn_count']==len(point_turn_indices(v['points']))
    normal=r['normalization'];turns=chosen['point_turn_count']
    assert chosen['objective_terms']['turns']==pytest.approx(turns/(turns+normal['turn_reference_count']))
    assert chosen['score']==pytest.approx(sum(chosen['weighted_terms'].values()))
    assert chosen['score']==min(c['score'] for c in r['candidates'] if c['executable'])
    validate_point_turn_path(v['points'],FIVE_POINT,v['metadata']['rotation_envelope'])
    assert FIVE_POINT.wkb==before and p==settings()


def test_turn_objective_penalizes_real_turns_with_one_fixed_reference_for_all_candidates():
    assert score(99,2,200,180,20,30)<score(99,2,200,180,40,30)
    assert score(99,2,200,180,0,30)==pytest.approx(.4*.01+.25*.02+.2*200/380)
    # Identical turn locations counted once despite denser straight sampling.
    vertices=[(0,0),(2,0),(2,1),(0,1)]
    coarse,_,a=sample_path(vertices,.2,check);fine,_,b=sample_path(vertices,.05,check)
    assert a['point_turn_count']==b['point_turn_count']==2
    assert len(point_turn_indices(coarse))==len(point_turn_indices(fine))==2
