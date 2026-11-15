"""Validate the independent vector visit counter against known cases and grid."""
import sys
from pathlib import Path
import pytest
from shapely.geometry import box,Polygon
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from benchmark_coverage_metrics import swept_metrics
from agri_path_planning.coverage_geometry import AreaGrid


@pytest.mark.parametrize('points',[
    [(0,1),(6,1)],
    [(0,1),(2,1),(4,1),(6,1)],
    [(0,1),(6,1),(6,1.5),(0,1.5)],
    [(0,1),(6,1),(0,1),(6,1)],
    [(1,1),(5,1),(1.1,1.1),(5,1.1)],
    [(0,0),(5,0),(5,5),(0,5),(0,0),(5,0)],
])
def test_vector_repeat_is_connected_visit_area_not_adjacent_sampling_overlap(points):
    field=box(-1,-1,7,7);grid=AreaGrid(field,0,.02)
    counts,_=grid.visits([points],1.)
    reference=grid.metrics(counts)['field'];actual=swept_metrics(points,field,1.)
    assert actual['coverage_percent']==pytest.approx(reference['coverage_percent'],abs=.03)
    assert actual['overlap_percent']==pytest.approx(reference['overlap_percent'],abs=.05)
    if all(a[1]==b[1] and b[0]>=a[0] for a,b in zip(points,points[1:])):
        assert actual['overlap_m2']<1e-8


def test_vector_uses_the_original_field_minus_holes_and_keeps_empty_missed_regions():
    field=Polygon([(0,0),(6,0),(6,4),(0,4)],holes=[[(2,1),(4,1),(4,3),(2,3)]])
    r=swept_metrics([(0,2),(6,2)],field,1.)
    assert r['covered_m2']==pytest.approx(4.,abs=1e-5)
    assert r['missed_m2']==pytest.approx(16.,abs=1e-5)
    assert r['overlap_m2']<1e-8
