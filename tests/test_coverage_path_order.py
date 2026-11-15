"""Real row visits and exact retracing are different from turns/footprint overlap."""
import copy
import json

import pytest
from shapely.geometry import LineString

from agri_backend.coverage_path_order import path_display, same_traversal, work_row_order


def saved(points):
    return {'points': [{'x': x, 'y': y} for x, y in points]}


def native_saved():
    value = saved([(0,0),(5,0),(10,0),(10,0),(10,2),(0,2),(0,4),(10,4)])
    value['planning'] = {'path_details': {
        'swath_visits': [{'swath_id': i} for i in (8, 2, 6)],
        'native_route_connections': [[], [(10,0),(10,2)], [(0,2),(0,4)]]}}
    return value


def length(value):
    return sum(LineString([(p['x'],p['y']) for p in line]).length for line in value['retraced_lines'])


def test_actual_visit_order_not_swath_ids_or_coordinate_sort():
    value = native_saved()
    before = copy.deepcopy(value)
    result = work_row_order(value)
    assert result['state'] == 'READY'
    assert [r['visit'] for r in result['rows']] == [1,2,3]
    assert result['rows'][1]['start'] == {'x':10.,'y':2.}
    assert result['rows'][1]['end'] == {'x':0.,'y':2.}
    assert value == before


def test_reversed_execution_is_numbered_again_from_new_start():
    value = native_saved()
    value['points'].reverse()
    value['planning']['path_details'] = {
        'swath_visits': [{'swath_id': i} for i in (6,2,8)],
        'native_route_connections': [[], [(0,4),(0,2)], [(10,2),(10,0)]]}
    result = work_row_order(value)
    assert result['state'] == 'READY'
    assert result['rows'][0] == {'visit':1,'start':{'x':10.,'y':4.},'end':{'x':0.,'y':4.}}


def test_polyline_validation_tolerates_sampling_noise_but_keeps_reversal_order():
    assert same_traversal([(0,0),(10,0)],[(0,0),(3,1e-8),(10,0)])
    assert not same_traversal([(0,0),(10,0)],[(0,0),(10,0),(0,0),(10,0)])
    assert not same_traversal([(0,0),(10,0)],[(0,0),(3,.001),(10,0)])


@pytest.mark.parametrize('change', ['geometry','duplicate','count','no_boundary','structure'])
def test_inconsistent_or_ambiguous_metadata_never_fabricates_work_rows(change):
    value = native_saved()
    d = value['planning']['path_details']
    if change == 'geometry': d['native_route_connections'][1][1] = (9,2)
    elif change == 'duplicate': d['swath_visits'][1]['swath_id'] = 8
    elif change == 'count': d['swath_visits'].pop()
    elif change == 'no_boundary': d['native_route_connections'][1] = []
    else: value['planning']['path_details'] = 'invalid'
    result = work_row_order(value)
    assert result['state'] == 'UNAVAILABLE' and result['rows'] == []


def test_only_partial_repeated_centerline_is_red_not_whole_visited_row():
    result = path_display(saved([(0,0),(10,0),(10,3),(3,3),(3,0),(7,0)]))
    assert result['retracing_state'] == 'READY' and length(result) == pytest.approx(4)
    assert result['state'] == 'UNAVAILABLE'  # Still no guessed row labels.
    assert all(abs(p['y'])<1e-8 and 3-1e-8<=p['x']<=7+1e-8 for line in result['retraced_lines'] for p in line)


@pytest.mark.parametrize('points', [
    [(0,0),(5,5),(0,5),(5,0)],  # Proper crossing is a point, not a repeated line.
    [(0,0),(10,0),(10,.001),(0,.001)],  # Nearby parallel rows/footprints are not centerline retracing.
    [(0,0),(0,0),(10,0)],  # Stationary turn duplicates.
])
def test_crossings_nearby_rows_and_stationary_poses_do_not_turn_red(points):
    result = path_display(saved(points))
    assert result['retracing_state'] == 'READY' and result['retraced_lines'] == []


def test_reverse_and_three_traversals_highlight_unique_repeated_geometry():
    result = path_display(saved([(0,0),(10,0),(4,0)]))
    assert length(result) == pytest.approx(6)
    assert length(path_display(saved([(0,0),(10,0),(0,0),(10,0)]))) == pytest.approx(10)


def test_map_and_saved_preview_deliver_the_same_verified_labels_without_rewriting_files(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.coverage_map import MapState
    from test_coverage_map import fixture_run
    store,run,directory,origin,points = fixture_run(tmp_path)
    path = directory/'paths/coverage_test.json'
    doc = json.loads(path.read_text())
    doc['planning'] = {'path_details':{'swath_visits':[{'swath_id':5}], 'native_route_connections':[[]]}}
    path.write_text(json.dumps(doc))
    before = path.read_bytes()
    expected = path_display(doc)
    assert expected['rows'][0]['visit'] == 1
    assert MapState(directory).base['path_order'] == expected
    mission = json.loads((directory/'config/mission.json').read_text())
    monkeypatch.setattr(main.app.state.workspace,'store',store)
    monkeypatch.setattr(main.app.state.workspace,'origin',lambda:(origin,'configured_yaml'))
    monkeypatch.setattr('agri_backend.workspace.read_plan',lambda *a,**k:(run,doc,[(p['x'],p['y'],0) for p in points],run['reference_fingerprint']))
    response = TestClient(main.app).get(f'/api/missions/{mission["id"]}/plans/{run["id"]}/path')
    assert response.status_code == 200,response.text
    assert response.json()['path_order'] == expected
    assert path.read_bytes() == before
