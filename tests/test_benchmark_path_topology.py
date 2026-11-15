import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from benchmark_path_topology import topology_metrics,route_segments


def test_parallel_sweep_has_no_proper_crossing_or_centerline_retrace():
    r=topology_metrics([(0,0),(10,0),(10,2),(0,2),(0,4),(10,4)])
    assert r['proper_crossing_locations']==0 and r['extra_retraced_length_m']==pytest.approx(0)


def test_bow_tie_has_one_proper_crossing_but_no_retraced_length():
    r=topology_metrics([(0,0),(2,2),(0,2),(2,0)])
    assert r['proper_crossing_locations']==1 and r['crossing_points'][0]==pytest.approx([1,1])
    assert r['extra_retraced_length_m']==pytest.approx(0)


def test_three_passes_retain_reversals_and_count_extra_travel_not_just_repeated_geometry():
    r=topology_metrics([(0,0),(6,0),(0,0),(6,0)])
    assert r['extra_retraced_length_m']==pytest.approx(12)
    assert r['repeated_centerline_unique_m']==pytest.approx(6)
    assert r['immediate_collinear_backtracks']==2 and r['proper_crossing_locations']==0


def test_shared_endpoint_and_t_junction_are_not_proper_crossings():
    r=topology_metrics([(0,0),(2,0),(2,2),(1,0),(1,-1)])
    assert r['proper_crossing_locations']==0


def test_roles_reconstruct_the_complete_chronological_route():
    points=[(0,0),(10,0),(10,3),(0,3)]
    details={'native_route_connections':[[],[(10,0),(10,3)]],'swath_visits':[{},{}]}
    segments,state=route_segments(points,details)
    assert state=='reconstructed' and [r for a,b,r in segments]==['work','transition','work']
    assert topology_metrics(points,details)['role_reconstruction']=='reconstructed'
    details['native_route_connections'][1]=[(11,0),(11,3)]
    assert route_segments(points,details)[1]=='unavailable'


def test_crossings_are_separated_by_work_and_transition_role():
    details={'native_route_connections':[[],[(2,2),(0,2)]],'swath_visits':[{},{}]}
    r=topology_metrics([(0,0),(2,2),(0,2),(2,0)],details)
    assert r['crossing_pair_roles']=={'work/work':1}


def test_connector_retrace_is_not_falsely_labelled_a_second_work_row():
    details={'native_route_connections':[[],[(4,0),(0,0)]],'swath_visits':[{},{}]}
    r=topology_metrics([(0,0),(4,0),(0,0),(0,2)],details)
    assert r['extra_retraced_by_current_role_m']['transition']==pytest.approx(4)
    assert r['extra_retraced_by_current_role_m']['work']==pytest.approx(0)
    assert sum(r['extra_retraced_by_current_role_m'].values())==pytest.approx(r['extra_retraced_length_m'])
