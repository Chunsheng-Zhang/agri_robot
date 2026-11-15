"""No input repair/scaling and no unpaired-success bias in benchmark summaries."""
import hashlib
import sys
from pathlib import Path
import pytest
from pyproj import Transformer
from shapely.geometry import Polygon,box
from shapely.ops import transform
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import benchmark_fields
from report_fields_benchmark import summarize


def test_input_projection_preserves_metres_holes_and_common_reference(tmp_path,monkeypatch):
    e,n=500000.,4000000.
    original=Polygon([(e,n),(e+20,n),(e+20,n+10),(e,n+10)],holes=[[(e+5,n+3),(e+8,n+3),(e+8,n+7),(e+5,n+7)]])
    raw=transform(Transformer.from_crs(32650,4326,always_xy=True).transform,original)
    file=tmp_path/'field.wkt';file.write_text(raw.wkt);before=file.read_bytes()
    monkeypatch.setattr(benchmark_fields,'ROOT',tmp_path)
    record={'file':'field.wkt','sha256':hashlib.sha256(before).hexdigest(),'epsg':32650,'utm_anchor':[e,n]}
    a=benchmark_fields.load_field(record);b=benchmark_fields.load_field(record)
    assert a.area==pytest.approx(188.,abs=1e-6) and len(a.interiors)==1
    assert a.bounds==pytest.approx((0,0,20,10),abs=1e-7)
    assert a.equals_exact(b,1e-12) and file.read_bytes()==before


def test_invalid_source_is_rejected_without_buffer_zero_repair(tmp_path,monkeypatch):
    file=tmp_path/'invalid.wkt';file.write_text('POLYGON ((117 36,117.01 36.01,117 36.01,117.01 36,117 36))')
    before=file.read_bytes();monkeypatch.setattr(benchmark_fields,'ROOT',tmp_path)
    record={'file':'invalid.wkt','sha256':hashlib.sha256(before).hexdigest(),'epsg':32650,'utm_anchor':[0,0]}
    with pytest.raises(ValueError,match='INVALID_INPUT'):benchmark_fields.load_field(record)
    assert file.read_bytes()==before


def test_quality_comparison_uses_only_same_field_pairs_and_missing_metrics_stay_unknown():
    def row(i,method,state,c=90):
        return {'index':i,'method':method,'width_m':.8,'state':state,'planning_seconds':2.,
            'independent':{'coverage_percent':c,'overlap_percent':1.,'length_m':10.,'point_turn_count':2,'score':.2,'field_area_m2':100.},
            'grid_vector_coverage_difference_pp':0.,'grid_vector_overlap_difference_pp':0.}
    data=[row(0,'current','SUCCEEDED',99),row(0,'native','FAILED'),
          row(1,'current','SUCCEEDED',95),row(1,'native','SUCCEEDED',93)]
    r=summarize(data)['profiles']['0.8']
    assert r['paired_successes']==1 and r['metrics']['coverage_percent']['current_mean']==95
    assert r['metrics']['coverage_percent']['mean']==2 and r['metrics']['coverage_percent']['ci95_mean'] is None
    assert r['status_counts']['current']['SUCCEEDED']==2 and r['status_counts']['current']['MISSING']==48
    assert summarize([])['profiles']['6.0']['metrics']['score']['mean'] is None
