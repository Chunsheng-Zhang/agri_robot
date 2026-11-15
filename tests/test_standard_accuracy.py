"""Independent numerical oracles and synthetic post-task reports, no hardware."""
import copy
import csv
import json
import math
import statistics

import numpy as np
import pytest
from shapely.geometry import Polygon, box

from agri_backend.standard_accuracy import (
    N, adjacent_pairs, assess, crossing_samples, formula_1_cm, formula_2_cm,
    observed_chunks, reference_units, section_precision, write_artifacts,
)

ORIGIN={'latitude':36.6468527633281,'longitude':117.021433066391,'epsg':32650}


def native_document():
    # TSP execution order is y=0, y=4, y=2; spatial neighbours are NOT visits 1/2.
    points=[(0,0),(10,0),(10,4),(0,4),(0,2),(10,2)]
    return {'frame_id':'map','projection':ORIGIN,'points':[{'x':x,'y':y} for x,y in points],
            'planning':{'turn_mode':'POINT_TURN','path_details':{
                'swath_visits':[{'swath_id':i} for i in (9,1,7)],
                'native_route_connections':[[],[(10,0),(10,4)],[(0,4),(0,2)]]}}}


def trace_for_units(units, count=201):
    result=[]
    for i,u in enumerate(units):
        a,b=u['start'],u['end'];length=u['length_m']
        for j,t in enumerate(np.linspace(0,1,count)):
            offset=.01*i+.01*math.sin(t*2*math.pi+i)
            result.append({'stamp':100+i*100+j*.1,'x':a[0]+t*(b[0]-a[0]),
                           'y':a[1]+t*(b[1]-a[1])+offset,'progress':u['s0']+length*t,'yaw':0})
    return result


def test_formula_1_is_sample_standard_deviation_not_rmse_or_population_std():
    values=[.1+i*.001 for i in range(N)]
    assert formula_1_cm(values)==pytest.approx(statistics.stdev(values)*100)
    assert formula_1_cm(values)!=pytest.approx(statistics.pstdev(values)*100)
    assert formula_1_cm([.1]*N)==pytest.approx(0,abs=1e-12)
    assert math.sqrt(sum(x*x for x in [.1]*N)/N)*100==pytest.approx(10)


def test_formula_2_uses_absolute_spacing_errors_before_centering():
    spacings=[2-.01,2+.01]*(N//2)
    assert formula_2_cm(spacings,2)==pytest.approx(0,abs=1e-12)
    signed=[h-2 for h in spacings]
    assert statistics.stdev(signed)*100>.9
    spacings=[2+.001*i for i in range(N)]
    assert formula_2_cm(spacings,2)==pytest.approx(statistics.stdev([abs(h-2) for h in spacings])*100)


@pytest.mark.parametrize('values',[[0.]*49,[0.]*51,[float('nan')]+[0.]*49])
def test_formula_rejects_wrong_sample_count_and_nonfinite(values):
    with pytest.raises(ValueError):formula_1_cm(values)


def test_straight_uses_fifty_equal_reference_stations_and_independent_interpolation():
    doc=native_document();units,_,_=reference_units(doc,'coverage')
    chunk=trace_for_units(units[:1],count=101);samples=[]
    record=section_precision(units[0],chunk,1,samples)
    assert record['state']=='AVAILABLE' and record['samples']==50
    stations=np.linspace(0,10,50)
    expected=np.interp(stations,[p['x'] for p in chunk],[p['y'] for p in chunk])
    assert [s['station_m'] for s in samples]==pytest.approx(stations)
    assert record['precision_cm']==pytest.approx(statistics.stdev(expected)*100)
    assert [s['error_cm'] for s in samples]==pytest.approx(expected*100)


def test_handover_pairs_spatial_neighbours_even_with_opposite_execution_direction():
    doc=native_document();units,_,_=reference_units(doc,'coverage');rows=trace_for_units(units)
    chunks,_=observed_chunks(units,rows,[])
    from agri_backend.standard_accuracy import handover_sections
    samples=[];sections=handover_sections(units,chunks,box(-1,-1,11,5),samples)
    assert {frozenset((s['first_reference_id'],s['second_reference_id'])) for s in sections}=={
        frozenset(('row_1','row_3')),frozenset(('row_3','row_2'))}
    assert all(s['state']=='AVAILABLE' and s['samples']==50 for s in sections)
    assert all(s['preset_spacing_cm']==pytest.approx(200) for s in sections)
    for section in sections:
        selected=[s for s in samples if s['section_id']==section['id']]
        expected=statistics.stdev([abs(s['spacing_cm']-200) for s in selected])
        assert section['precision_cm']==pytest.approx(expected)
        assert [p['station_m'] for p in selected]==pytest.approx(np.linspace(0,10,N))


def test_unobserved_middle_row_is_not_silently_skipped_to_pair_outer_rows():
    doc=native_document();units,_,_=reference_units(doc,'coverage')
    chunks,_=observed_chunks(units,trace_for_units(units[:2]),[])
    from agri_backend.standard_accuracy import handover_sections
    sections=handover_sections(units,chunks,None,[])
    assert sections==[]


def test_hole_is_not_bridged_as_a_valid_interrow_strip():
    doc=native_document();units,_,_=reference_units(doc,'coverage')
    field=Polygon([(-1,-1),(11,-1),(11,5),(-1,5)],holes=[[(4,.5),(6,.5),(6,1.5),(4,1.5)]])
    intervals=adjacent_pairs(units,field)
    first_pair=[p for p in intervals if {p[0]['id'],p[1]['id']}=={'row_1','row_3'}]
    assert [(p[3],p[4]) for p in first_pair]==pytest.approx([(0,4),(6,10)])


def test_hole_split_handover_windows_have_unique_ids_and_fifty_reproducible_samples_each():
    from agri_backend.standard_accuracy import handover_sections
    units,_,_=reference_units(native_document(),'coverage')
    chunks,_=observed_chunks(units,trace_for_units(units,count=601),[])
    field=Polygon([(-1,-1),(11,-1),(11,5),(-1,5)],holes=[[(4,.5),(6,.5),(6,1.5),(4,1.5)]])
    samples=[];sections=handover_sections(units,chunks,field,samples)
    assert len(sections)==len({section['id'] for section in sections})==3
    for section in sections:
        selected=[sample for sample in samples if sample['section_id']==section['id']]
        assert section['state']=='AVAILABLE' and len(selected)==N
        assert statistics.stdev(sample['error_cm'] for sample in selected)==pytest.approx(section['precision_cm'])


def test_gap_pause_and_progress_rollback_are_never_interpolated_across():
    units,_,_=reference_units(native_document(),'coverage')
    rows=trace_for_units(units[:1],count=101)
    chunks,_=observed_chunks(units,rows,[105.05])
    assert len(chunks['row_1'])==2
    first,second=chunks['row_1'];assert first[-1]['stamp']<105.05<second[0]['stamp']
    rows[50]['progress']=0
    chunks,_=observed_chunks(units,rows,[])
    assert len(chunks['row_1'])>=2


def test_few_points_or_outside_window_points_do_not_become_fifty_measurements():
    units,_,_=reference_units(native_document(),'coverage');samples=[]
    record=section_precision(units[0],trace_for_units(units[:1],count=20),1,samples)
    assert record['state']=='UNAVAILABLE' and record['precision_cm'] is None and not samples
    chunk=[{'stamp':100+i*.1,'x':float(i),'y':0.} for i in range(101)]
    with pytest.raises(ValueError,match='跨度内'):
        crossing_samples(chunk,(1,0),np.linspace(0,10,N))


def test_curve_uses_equal_actual_arc_distance_and_minimum_curve_distance():
    theta=np.linspace(0,math.pi/2,201)
    reference=[(10*math.cos(t),10*math.sin(t)) for t in theta]
    length=sum(math.dist(a,b) for a,b in zip(reference,reference[1:]))
    unit={'id':'curve','kind':'curve','source':'TEST_CURVE','length_m':length,'points':reference}
    chunk=[{'stamp':100+i*.1,'x':10.1*math.cos(t),'y':10.1*math.sin(t)} for i,t in enumerate(theta)]
    samples=[];result=section_precision(unit,chunk,1,samples)
    assert result['state']=='AVAILABLE' and result['samples']==50
    assert np.diff([s['station_m'] for s in samples])==pytest.approx([result['evaluated_length_m']/49]*49)
    assert result['mean_input_cm']==pytest.approx(10,abs=.001)
    assert result['precision_cm']<.001  # Fixed offset is not RMSE.


def test_point_turn_connectors_are_never_curved_work_passes():
    doc=native_document();units,_,_=reference_units(doc,'coverage')
    assert all(u['kind']=='straight' for u in units)
    doc['planning']['path_details']['swath_visits'].pop()
    units,_,_=reference_units(doc,'coverage')
    assert not units


def test_following_curve_has_curve_formula_but_no_guessed_handover_rows():
    theta=np.linspace(0,math.pi/2,101)
    doc={'points':[{'x':10*math.cos(t),'y':10*math.sin(t)} for t in theta]}
    units,_,_=reference_units(doc,'following')
    assert [u['kind'] for u in units]==['curve']
    assert adjacent_pairs(units,None)==[]


def test_artifacts_and_empty_results_are_explicit_and_do_not_mutate_inputs(tmp_path):
    doc=native_document();units,_,_=reference_units(doc,'coverage');rows=trace_for_units(units)
    mission={'projection':ORIGIN,'parameters':{'operation_width':2.},'boundary':None}
    run={'mode':'following','projection':ORIGIN}
    before=copy.deepcopy((doc,mission,run,rows))
    result,samples=assess(tmp_path,mission,run,doc,rows)
    assert result['straight']['available'] and result['handover']['available']
    assert not result['curve']['available'] and result['curve']['value_cm'] is None
    assert result['standard_test_conformance']=='NOT_VERIFIED'
    (tmp_path/'analysis').mkdir();write_artifacts(tmp_path,result,samples)
    assert (doc,mission,run,rows)==before
    for name in ('standard_accuracy.json','standard_accuracy.csv','standard_accuracy_samples.csv','standard_accuracy_report.md'):
        assert (tmp_path/'analysis'/name).is_file()
    with (tmp_path/'analysis/standard_accuracy_samples.csv').open(encoding='utf-8-sig') as f:exported=list(csv.DictReader(f))
    assert len(exported)==5*50
    saved=json.loads((tmp_path/'analysis/standard_accuracy.json').read_text())
    assert saved['curve']['value_cm'] is None
    empty,_=assess(tmp_path,mission,run,doc,[])
    assert not any(empty[k]['available'] for k in ('straight','handover','curve'))
    bad={**doc,'projection':{**ORIGIN,'latitude':0}}
    bad_result,_=assess(tmp_path,mission,run,bad,rows)
    assert not bad_result['straight']['available']


def saved_run(tmp_path, mode, document, rows):
    """Finished synthetic recorder files, using the real shared projection."""
    from agri_backend.missions import LocalFrame, atomic_json
    (tmp_path/'analysis').mkdir()
    frame=LocalFrame(ORIGIN['latitude'],ORIGIN['longitude'])
    coordinates=[frame.transformer.transform(frame.e0+x,frame.n0+y,direction='INVERSE')
                 for x,y in [(-1,-1),(11,-1),(11,5),(-1,5),(-1,-1)]]
    mission={'name':'synthetic accuracy trial','projection':ORIGIN,'parameters':{'operation_width':2.,'speed':.2},
             'boundary':{'type':'Polygon','coordinates':[coordinates]}}
    atomic_json(tmp_path/'run.json',{'id':'synthetic-accuracy','mode':mode,'state':'COMPLETED','projection':ORIGIN})
    atomic_json(tmp_path/'config/mission.json',mission)
    atomic_json(tmp_path/f'paths/{mode}_test.json',document)
    atomic_json(tmp_path/'tracks/track_test.json',{'frame_id':'map','projection':ORIGIN,
                'reference_path':{'frame_id':'map','points':document['points']}})
    atomic_json(tmp_path/'telemetry/recording.json',{'state':'CLOSED','samples_by_topic':{},
                'terminal':{'state':'COMPLETED'}})
    with (tmp_path/'tracks/track_test.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['stamp','x','y','yaw','cross_track','heading_error','progress'])
        writer.writeheader()
        writer.writerows({**row,'cross_track':.01,'heading_error':0} for row in rows)


def analyze_synthetic_run(tmp_path, monkeypatch):
    from agri_backend import experiment_analysis, communication_plots
    # Real plotting is covered by the existing capture/report integration test.
    monkeypatch.setattr(experiment_analysis,'plot_report',lambda *args:None)
    monkeypatch.setattr(communication_plots,'plot_communication',lambda *args:None)
    before={str(path.relative_to(tmp_path)):path.read_bytes() for folder in ('config','paths','tracks','telemetry')
            for path in (tmp_path/folder).glob('*') if path.is_file()}
    result=experiment_analysis.analyze_run(tmp_path)
    assert all((tmp_path/name).read_bytes()==data for name,data in before.items())
    assert json.loads((tmp_path/'analysis/status.json').read_text())['state']=='SUCCEEDED'
    saved=json.loads((tmp_path/'analysis/metrics.json').read_text())
    assert saved['standard_accuracy']==result['standard_accuracy']
    report=(tmp_path/'analysis/report.md').read_text()
    assert all(name in report for name in ('直线作业精度','交接行精度','曲线作业精度','横向均值 cm','速度'))
    return result


def test_finished_coverage_analysis_automatically_exports_three_metrics_and_fifty_point_inputs(tmp_path,monkeypatch):
    document=native_document();units,_,_=reference_units(document,'coverage')
    saved_run(tmp_path,'coverage',document,trace_for_units(units))
    result=analyze_synthetic_run(tmp_path,monkeypatch)
    accuracy=result['standard_accuracy']
    assert accuracy['straight']['valid_sections']==3
    assert accuracy['handover']['valid_sections']==2
    assert accuracy['curve']['state']=='NOT_APPLICABLE'
    with (tmp_path/'analysis/standard_accuracy_samples.csv').open(encoding='utf-8-sig') as stream:
        samples=list(csv.DictReader(stream))
    assert len(samples)==250
    for kind in ('straight','handover'):
        expected=[]
        for section in accuracy[kind]['sections']:
            errors=[float(row['error_cm']) for row in samples if row['section_id']==section['id']]
            assert len(errors)==50
            expected.append(statistics.stdev(errors))
            assert section['precision_cm']==pytest.approx(expected[-1])
        assert accuracy[kind]['value_cm']==pytest.approx(max(expected))


def test_finished_curved_following_analysis_uses_curve_formula(tmp_path,monkeypatch):
    theta=np.linspace(0,math.pi/2,201)
    points=[{'x':10*math.cos(t),'y':10*math.sin(t)} for t in theta]
    document={'frame_id':'map','projection':ORIGIN,'points':points}
    units,_,_=reference_units(document,'following')
    rows=[{'stamp':100+i*.1,'x':10.1*math.cos(t),'y':10.1*math.sin(t),'yaw':t+math.pi/2,
           'progress':units[0]['length_m']*i/(len(theta)-1)} for i,t in enumerate(theta)]
    saved_run(tmp_path,'following',document,rows)
    result=analyze_synthetic_run(tmp_path,monkeypatch)
    accuracy=result['standard_accuracy']
    assert accuracy['straight']['state']=='NOT_APPLICABLE'
    assert accuracy['handover']['state']=='NOT_APPLICABLE'
    assert accuracy['curve']['available']
    assert accuracy['curve']['sections'][0]['mean_input_cm']==pytest.approx(10,abs=.001)
    assert accuracy['curve']['value_cm']<.001


def test_analysis_does_not_bridge_an_invalid_position_or_report_missing_data_as_zero(tmp_path,monkeypatch):
    document={'frame_id':'map','projection':ORIGIN,'points':[{'x':0,'y':0},{'x':10,'y':0}]}
    rows=[{'stamp':100+i*.1,'x':i/8,'y':.01*math.sin(i),'yaw':0,'progress':i/8} for i in range(81)]
    rows[40]['x']='nan'
    saved_run(tmp_path,'following',document,rows)
    result=analyze_synthetic_run(tmp_path,monkeypatch)
    accuracy=result['standard_accuracy']
    assert accuracy['recording']['invalid_samples']==1
    assert len(accuracy['straight']['sections'])==2
    assert accuracy['straight']['value_cm'] is None
    assert all(section['precision_cm'] is None for section in accuracy['straight']['sections'])
    assert result['phases']['tracking']['samples']==80  # Legacy descriptive metrics preserved.


def test_fresh_background_worker_generates_new_reports_with_existing_plots(tmp_path):
    from agri_backend.experiment_recording import ExperimentRecorder
    document=native_document();units,_,_=reference_units(document,'coverage')
    saved_run(tmp_path,'coverage',document,trace_for_units(units,count=101))
    ExperimentRecorder.analyze(tmp_path)
    status=json.loads((tmp_path/'analysis/status.json').read_text())
    assert status['state']=='SUCCEEDED', (tmp_path/'analysis/analysis.log').read_text()
    accuracy=json.loads((tmp_path/'analysis/standard_accuracy.json').read_text())
    assert accuracy['straight']['valid_sections']==3 and accuracy['handover']['valid_sections']==2
    assert (tmp_path/'analysis/standard_accuracy_samples.csv').is_file()
    assert (tmp_path/'analysis/trajectory_and_speed.png').stat().st_size>10000
    report = (tmp_path/'analysis/report.md').read_text()
    assert '按GB/T 46270—2025公式计算的作业精度' in report
    from agri_backend.standard_accuracy import PLOT_FILES
    for filename in PLOT_FILES.values():
        assert (tmp_path/'analysis'/filename).stat().st_size>10000
        assert f']({filename})' in report
    assert accuracy['plots']==PLOT_FILES
