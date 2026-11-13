"""Known-area sweeps and completed reports; no ROS context, hardware or old data."""
import csv
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import Point, Polygon, box

from agri_backend.coverage_geometry import AreaGrid
from agri_backend.coverage_quality import completed_quality, planned_quality, tracking_chunks
from agri_backend.missions import Mission, MissionStore, LocalFrame, atomic_json
from pyproj.enums import TransformDirection

ROOT=Path(__file__).resolve().parents[1]


def test_straight_strip_known_area_and_sampling_invariance():
    grid=AreaGrid(box(0,0,10,10),1,.1)
    counts,_=grid.visits([[(-2,5),(12,5)]],2)
    dense,_=grid.visits([[(float(x),5) for x in np.linspace(-2,12,501)]],2)
    assert np.array_equal(counts,dense)
    metrics=grid.metrics(counts)['field']
    assert metrics['covered_m2']==pytest.approx(20)
    assert metrics['coverage_percent']==pytest.approx(20)
    assert metrics['overlap_m2']==0
    assert metrics['coverage_percent']+metrics['miss_percent']==pytest.approx(100)


def test_two_overlapping_parallel_passes_and_three_pass_multiplicity():
    grid=AreaGrid(box(0,0,10,4),0,.1)
    chunks=[[(-2,1),(12,1)],[(-2,2),(12,2)]]
    counts,_=grid.visits(chunks,2)
    metrics=grid.metrics(counts)['field']
    assert metrics['covered_m2']==pytest.approx(30)
    assert metrics['overlap_m2']==pytest.approx(10)
    assert metrics['overlap_percent']==pytest.approx(25)
    assert metrics['overlap_of_covered_percent']==pytest.approx(100/3)
    triple,_=grid.visits(chunks+[chunks[0]],2)
    metrics=grid.metrics(triple)['field']
    assert metrics['overlap_m2']==pytest.approx(20)
    assert metrics['extra_pass_area_m2']==pytest.approx(30)


def test_hole_area_headland_partition_and_empty_interior():
    field=Polygon([(0,0),(10,0),(10,10),(0,10)],holes=[[(4,4),(6,4),(6,6),(4,6)]])
    grid=AreaGrid(field,1,.17)
    assert grid.weights.sum()==pytest.approx(96)
    assert grid.interior_weights.sum()==pytest.approx(field.buffer(-1).area)
    counts,_=grid.visits([[(-2,5),(12,5)]],2)
    metric=grid.metrics(counts)
    assert metric['field']['area_m2']==pytest.approx(metric['interior']['area_m2']+metric['headland']['area_m2'])
    assert all(field.covers(Point(x,y))
               for x,y,w in zip(grid.x,grid.y,grid.weights) if w>0)
    empty=AreaGrid(box(0,0,1,1),2,.1)
    assert not empty.metrics(np.zeros(len(empty.weights),dtype=int))['interior']['available']


def test_stationary_and_small_continuous_turns_do_not_count_each_sample():
    points=[(0,0),(1,0),(1,1)]
    dense=[(x,0) for x in np.linspace(0,1,51)]+[(1,y) for y in np.linspace(0,1,51)]
    grid=AreaGrid(box(-1,-1,2,2),0,.1)
    counts,_=grid.visits([points],.5)
    dense_counts,_=grid.visits([dense],.5)
    assert np.array_equal(counts,dense_counts)
    # An inner corner can physically leave and re-enter the disk. That is a
    # genuine second visit, unlike adjacent-sample overlap. The joint itself
    # remains one connected visit under either sampling density.
    near_joint=(grid.x-.95)**2+(grid.y-.05)**2<1e-6
    assert np.all(counts[near_joint]==1)
    assert counts.max()<=2


def test_large_fields_coarsen_explicitly_to_a_bounded_grid(monkeypatch):
    from agri_backend import coverage_geometry
    monkeypatch.setattr(coverage_geometry,'MAX_CELLS',64)
    grid=AreaGrid(box(0,0,10,10),0,.05)
    assert grid.nx*grid.ny<=64
    assert grid.resolution>.05
    assert grid.weights.sum()==pytest.approx(100)


def make_run(tmp_path):
    store=MissionStore(tmp_path)
    store.config_path=ROOT/'config/jetson_system.yaml'
    origin=store.configured_origin()
    frame=LocalFrame(origin['latitude'],origin['longitude'])
    def geo(x,y):
        lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
        return {'longitude':lon,'latitude':lat}
    mission=store.save(Mission(name='quality synthetic',mode='coverage',
        boundary={'points':[geo(x,y) for x,y in ((0,0),(6,0),(6,4),(0,4))]},
        parameters={'operation_width':1.,'headland_width':.5,'quality_resolution':.1}),origin)
    run=store.create_run(mission,1,'planning')
    run.update(coverage_quality_version=1,state='COMPLETED')
    store.save_run(run)
    directory=store.run_path(run['id'])
    points=[{'x':0.,'y':1.},{'x':6.,'y':1.}]
    atomic_json(directory/'paths/coverage_test.json',{'projection':origin,'frame_id':'map','points':points})
    return store,run,directory,origin,points


def write_track(directory,origin,points,rows):
    atomic_json(directory/'tracks/track_test.json',{'projection':origin,'frame_id':'map',
        'reference_path':{'frame_id':'map','points':points}})
    with (directory/'tracks/track_test.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['stamp','x','y','yaw','cross_track','heading_error','progress'])
        writer.writerows(rows)


def test_plan_artifacts_then_gap_aware_actual_report(tmp_path):
    _,run,directory,origin,points=make_run(tmp_path)
    plan=planned_quality(directory)
    assert plan['metrics']['field']['coverage_percent']==pytest.approx(25,abs=.1)
    assert plan['metrics']['field']['overlap_percent']==0
    assert plan['vector_check']['coverage_percent']==pytest.approx(25,abs=.1)
    write_track(directory,origin,points,[[100,0,1,0,0,0,0],[100.1,.1,1,0,0,0,.1],
        [105,4,1,0,0,0,4],[105.1,4.1,1,0,0,0,4.1]])
    reference=[(p['x'],p['y']) for p in points]
    chunks,diagnostics,_=tracking_chunks(directory,reference)
    assert len(chunks)==2 and diagnostics['split_segments']==1
    metrics={'task_state':'COMPLETED','missing_topics':['/vel_raw'],'capture':{}}
    result=completed_quality(directory,metrics)
    assert result['actual']['available']
    assert result['actual']['metrics']['field']['covered_m2'] < 2
    assert result['actual']['path']['observed_length_m']==pytest.approx(.2)
    report=(directory/'analysis/coverage_quality_report.md').read_text()
    assert '假设完整规划路径' in report
    assert 'doi.org' not in report and '参考文献' not in report
    assert result['sources_sha256']['config/mission.json']
    for name in ('coverage_quality.csv','coverage_quality_plan_cells.csv','coverage_quality_actual_cells.csv',
                 'coverage_quality_plan.png','coverage_quality_actual.png','coverage_quality_plan.svg'):
        assert (directory/'analysis'/name).stat().st_size>0
    assert not list((directory/'analysis').glob('*.bib'))


def test_unknown_actual_is_not_zero_coverage_and_frame_mismatch_rejected(tmp_path):
    _,_,directory,origin,points=make_run(tmp_path)
    result=completed_quality(directory,{'task_state':'CANCELED'})
    assert result['actual']['available'] is False
    assert 'metrics' not in result['actual']
    write_track(directory,{**origin,'epsg':32649},points,[[100,0,1,0,0,0,0],[100.1,.1,1,0,0,0,.1]])
    with pytest.raises(ValueError,match='坐标'):
        completed_quality(directory,{'task_state':'COMPLETED'})


def test_legacy_run_is_not_backfilled(tmp_path):
    _,run,directory,_,_=make_run(tmp_path)
    run.pop('coverage_quality_version')
    atomic_json(directory/'run.json',run)
    with pytest.raises(ValueError,match='历史任务'):
        planned_quality(directory)
    assert not (directory/'analysis/coverage_quality_plan.json').exists()


def test_existing_completed_analysis_generates_quality_report(tmp_path):
    from agri_backend.experiment_analysis import analyze_run
    _,_,directory,origin,points=make_run(tmp_path)
    write_track(directory,origin,points,[[100+i*.1,i*.02,1,0,0,0,i*.02] for i in range(10)])
    result=analyze_run(directory)
    assert result['coverage_quality']['actual']['available']
    assert 'coverage_quality_report.md' in (directory/'analysis/report.md').read_text()
    assert (directory/'analysis/status.json').is_file()


def test_http_artifact_whitelist_and_workspace_small_snapshot(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.coverage_quality_jobs import snapshot
    store,run,directory,_,_=make_run(tmp_path)
    planned_quality(directory)
    atomic_json(directory/'analysis/coverage_quality_plan_status.json',{'state':'SUCCEEDED'})
    small=snapshot(directory,run)
    assert small['planned']['metrics']['field']['coverage_percent']>0
    assert 'sources_sha256' not in small['planned']
    monkeypatch.setattr(main.app.state.workspace,'store',store)
    client=TestClient(main.app)  # No lifespan: no ROS/hardware.
    response=client.get(f'/api/runs/{run["id"]}/coverage-quality/plan.png')
    assert response.status_code==200 and response.headers['content-type']=='image/png'
    assert client.get(f'/api/runs/{run["id"]}/coverage-quality/mission.json').status_code==404
    assert client.get(f'/api/runs/{run["id"]}/coverage-quality/actual.png').status_code==404


def test_verified_planning_result_schedules_quality_but_mismatched_path_does_not(tmp_path,monkeypatch):
    import hashlib
    from agri_backend.workspace import Workspace
    from agri_backend.devices import device_cache
    store,run,directory,_,points=make_run(tmp_path)
    run['state']='PLANNING'
    store.save_run(run)
    data={'reference_sequence':1,'reference_path':points,'reference_fingerprint':'wrong'}
    workspace=Workspace(lambda:None,lambda:data,store)
    scheduled=[]
    workspace.quality_jobs.close()
    workspace.quality_jobs=SimpleNamespace(submit=lambda path:scheduled.append(path))
    workspace.active=run
    workspace.plan_requested=0
    workspace.plan_sequence=0
    monkeypatch.setattr(device_cache,'values',{})
    device_cache.update('safety',{'mode':'coverage','mode_revision':1})
    device_cache.update('planning',{'state':'SUCCEEDED','output_directory':str(directory/'paths')})
    workspace.sync()
    assert run['state']=='WAITING_PATH' and not scheduled
    data['reference_fingerprint']=hashlib.sha256(json.dumps(['map',points],sort_keys=True).encode()).hexdigest()
    workspace.sync()
    assert run['state']=='READY' and scheduled==[directory]


def test_background_plan_worker_runs_real_offline_subprocess(tmp_path):
    from agri_backend.coverage_quality_jobs import run_plan_quality
    _,_,directory,_,_=make_run(tmp_path)
    run_plan_quality(directory)
    status=json.loads((directory/'analysis/coverage_quality_plan_status.json').read_text())
    assert status['state']=='SUCCEEDED',status
    assert (directory/'analysis/coverage_quality_plan.png').is_file()
