"""Live map uses this run's formal track, without ROS or hardware."""
import csv
import hashlib
import io
import json
import time

import numpy as np
import pytest
from PIL import Image
from shapely.geometry import Polygon, box

from agri_backend.coverage_geometry import AreaGrid, SweepState
from agri_backend.coverage_map import CoverageMaps, MapState, Tail
from agri_backend.coverage_quality import completed_quality
from agri_backend.missions import Mission, atomic_json
from test_coverage_quality import make_run, write_track


def fixture_run(tmp_path):
    store, run, directory, origin, points = make_run(tmp_path)
    run['reference_fingerprint'] = hashlib.sha256(json.dumps(['map', points], sort_keys=True).encode()).hexdigest()
    store.save_run(run)
    return store, run, directory, origin, points


def test_incremental_sweep_equals_one_shot_without_counting_each_poll():
    grid = AreaGrid(box(0, 0, 6, 4), .5, .1)
    points = [(float(x), 1.) for x in np.linspace(0, 6, 61)]
    points += [(float(x), 1.5) for x in np.linspace(6, 0, 61)]
    expected, _ = grid.visits([points], 1.)
    state = SweepState(len(grid.weights))
    for start in range(0, len(points)-1, 7):
        grid.visits([points[start:start+8]], 1., state)
    assert np.array_equal(state.counts, expected)
    before = state.counts.copy()
    grid.visits([[points[-1], points[-1]]], 1., state)
    assert np.array_equal(before, state.counts)
    assert state.counts.max() >= 2


def test_preview_inset_expands_holes_and_handles_multiple_pieces(tmp_path):
    store, run, _, origin, _ = fixture_run(tmp_path)
    from agri_backend.missions import LocalFrame
    from pyproj.enums import TransformDirection
    frame = LocalFrame(origin['latitude'], origin['longitude'])
    def preview(polygon, distance):
        rings = []
        for ring in [polygon.exterior, *polygon.interiors]:
            rings.append([list(frame.transformer.transform(frame.e0+x, frame.n0+y,
                          direction=TransformDirection.INVERSE)) for x, y in ring.coords])
        mission = Mission(name='inset', mode='coverage', boundary={'type':'Polygon','coordinates':rings},
                          parameters={'headland_width':distance})
        return store.preview(mission, origin)
    field = Polygon([(0,0),(10,0),(10,10),(0,10)], holes=[[(4,4),(6,4),(6,6),(4,6)]])
    result = preview(field, 1)
    inset = result['inset_polygons'][0]
    polygon = Polygon([(p['x'],p['y']) for p in inset[0]],
                      [[(p['x'],p['y']) for p in ring] for ring in inset[1:]])
    assert polygon.area == pytest.approx(field.buffer(-1).area, abs=1e-6)
    assert len(inset) == 2
    narrow = Polygon([(0,0),(4,0),(4,1.8),(6,1.8),(6,0),(10,0),(10,4),(6,4),(6,2.2),(4,2.2),(4,4),(0,4)])
    assert len(preview(narrow, .5)['inset_polygons']) == 2
    assert preview(box(0,0,1,1), 2)['inset_polygons'] == []


def test_actual_starts_empty_and_tails_complete_rows_only(tmp_path):
    _, run, directory, origin, points = fixture_run(tmp_path)
    state = MapState(directory)
    initial = state.update()
    assert initial['meta']['sources']['actual']['status'] == 'NOT_STARTED'
    assert state.sweep.counts.max() == 0 and state.planned.max() == 1
    # Approach/alignment records must not colour a single map cell.
    (directory/'tracks').mkdir(exist_ok=True)
    (directory/'tracks/approach_fake.csv').write_text('stamp,x,y\n100,0,1\n100.1,.1,1\n')
    assert state.update()['meta']['sources']['actual']['samples'] == 0
    run.update(stage='execution', state='RUNNING'); atomic_json(directory/'run.json', run)
    write_track(directory, origin, points, [[100,0,1,0,0,0,0],[100.1,.1,1,0,0,0,.1]])
    first = state.update()
    before = state.sweep.counts.copy()
    assert first['meta']['sources']['actual']['status'] == 'OBSERVED'
    assert 0 < state.grid.metrics(before)['field']['covered_m2'] < 2
    assert state.update()['meta']['sources']['actual']['samples'] == 2
    assert np.array_equal(before, state.sweep.counts)
    path = directory/'tracks/track_test.csv'
    with path.open('a') as stream: stream.write('100.2,.2,1,0,0,0,.2')
    assert state.update()['meta']['sources']['actual']['samples'] == 2
    with path.open('a') as stream: stream.write('\n')
    assert state.update()['meta']['sources']['actual']['samples'] == 3
    assert state.sweep.arc == pytest.approx(.2)


def test_live_gaps_pause_boundaries_and_final_report_agree(tmp_path):
    _, run, directory, origin, points = fixture_run(tmp_path)
    run.update(stage='execution',state='RUNNING'); atomic_json(directory/'run.json', run)
    rows = [[100,0,1,0,0,0,0],[100.1,.1,1,0,0,0,.1],
            [100.2,.2,1,0,0,0,.2],[100.3,.3,1,0,0,0,.3],
            [105,4,1,0,0,0,4],[105.1,4.1,1,0,0,0,4.1]]
    write_track(directory, origin, points, rows)
    status = {'topic':'/tracking/status','received_ros_ns':100150000000,
              'data':{'run_directory':str(directory/'tracks'),'state':'PAUSED','phase':'TRACKING'}}
    (directory/'telemetry').mkdir(exist_ok=True)
    (directory/'telemetry/status.jsonl').write_text(json.dumps(status)+'\n')
    state = MapState(directory)
    data = state.update()
    assert state.sweep.arc == pytest.approx(.3)
    assert data['meta']['sources']['actual']['splits'] == 2
    assert len(data['meta']['sources']['actual']['track_lines']) == 3
    final = completed_quality(directory, {'task_state':'CANCELED'})
    assert state.update()['meta']['sources']['actual']['status'] == 'FINAL'
    metric = state.grid.metrics(state.sweep.counts)['field']
    assert metric == final['actual']['metrics']['field']
    with (directory/'analysis/coverage_quality_actual_cells.csv').open() as stream:
        assert sum(float(r['field_weight_m2']) for r in csv.DictReader(stream)) == pytest.approx(24)


def test_invalid_frame_fingerprint_and_truncation_rejected(tmp_path):
    _, run, directory, origin, points = fixture_run(tmp_path)
    write_track(directory, {**origin,'epsg':32649}, points, [[100,0,1,0,0,0,0]])
    with pytest.raises(ValueError, match='坐标'): MapState(directory).update()
    run['reference_fingerprint'] = 'bad'; atomic_json(directory/'run.json',run)
    with pytest.raises(ValueError, match='指纹'): MapState(directory)
    tail = Tail(); path = directory/'partial.txt'; path.write_text('complete\nunfinished')
    assert tail.lines(path) == ['complete\n']
    path.write_text('x')
    with pytest.raises(ValueError, match='截断'): tail.lines(path)


def test_textures_are_north_up_hole_transparent_and_repeat_separate(tmp_path):
    _, _, directory, _, _ = fixture_run(tmp_path)
    state = MapState(directory)
    state.grid = AreaGrid(Polygon([(0,0),(3,0),(3,3),(0,3)], holes=[[(1,1),(2,1),(2,2),(1,2)]]),0,1)
    counts = np.zeros(9,dtype=np.uint32); counts[6]=1; counts[8]=2
    _, images = state.textures(counts)
    pixels = {name:np.asarray(Image.open(io.BytesIO(data))) for name,data in images.items()}
    assert pixels['completed'][0,0,3] > 0 and pixels['completed'][2,0,3] == 0
    assert pixels['overlap'][0,2,3] > 0 and pixels['overlap'][0,0,3] == 0
    assert all(p[1,1,3] == 0 for p in pixels.values())
    for i in range(4): state.textures(counts+i)
    assert len(state.images) == 2


def test_spawned_worker_and_http_layers_are_read_only(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    store, run, directory, _, _ = fixture_run(tmp_path)
    service = CoverageMaps()
    monkeypatch.setattr(main.app.state.workspace,'store',store)
    monkeypatch.setattr(main.app.state.workspace,'coverage_maps',service)
    client = TestClient(main.app)  # No lifespan / ROS.
    before = sorted(p.relative_to(directory) for p in directory.rglob('*'))
    try:
        result = None
        deadline = time.monotonic()+20
        while time.monotonic()<deadline:
            response = client.get(f'/api/runs/{run["id"]}/coverage-map')
            assert response.status_code == 200
            result = response.json()
            if result['state'] != 'PENDING': break
            time.sleep(.05)
        assert result['state'] == 'READY',result
        for source in ('planned','actual'):
            version = result['sources'][source]['version']
            for layer in ('completed','remaining','overlap'):
                image = client.get(f'/api/runs/{run["id"]}/coverage-map/{source}/{layer}.png?v={version}')
                assert image.status_code == 200 and image.headers['content-type'] == 'image/png'
                assert image.content.startswith(b'\x89PNG')
        assert client.get(f'/api/runs/{run["id"]}/coverage-map/actual/other.png?v=x').status_code == 404
        assert client.get(f'/api/runs/{run["id"]}/coverage-map/actual/completed.png?v=old').status_code == 404
        assert client.get('/api/runs/not-a-run/coverage-map').status_code == 404
        assert before == sorted(p.relative_to(directory) for p in directory.rglob('*'))
    finally:
        service.close(wait=True)
