"""Coordinate imports preserve order/origin and never acquire control."""
import hashlib
import json
import math

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from pyproj.enums import TransformDirection

from agri_base.coordinates import LocalFrame
from agri_base.path_geometry import directed_poses, point_turn_indices
from agri_backend.devices import device_cache
from agri_backend.path_import import import_path, parse_coordinates, MAX_BYTES
from agri_backend.taught_paths import TaughtPathStore

ORIGIN = {'epsg':32650,'latitude':36.6468527633281,'longitude':117.021433066391}


def geo(x, y):
    frame = LocalFrame(ORIGIN['latitude'], ORIGIN['longitude'])
    lon, lat = frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
    return {'longitude':lon,'latitude':lat}


def content(kind='csv'):
    vertices = [(0,0), (1,0), (2,.3), (3,1), (4,2)]
    points = [geo(*p) for p in vertices]
    if kind == 'csv':
        return 'longitude,latitude\n'+'\n'.join(f'{p["longitude"]},{p["latitude"]}' for p in points)
    if kind == 'geojson':
        return json.dumps({'type':'Feature','properties':{},'geometry':{'type':'LineString',
            'coordinates':[[p['longitude'],p['latitude']] for p in points]}})
    if kind == 'map':
        return json.dumps({'frame_id':'map','projection':ORIGIN,'points':[dict(x=x,y=y) for x,y in vertices]})
    return json.dumps({'points':points})


@pytest.mark.parametrize('kind', ['csv','geojson','json','map'])
def test_import_formats_use_fixed_origin_preserve_every_point_and_archive_source(tmp_path, kind):
    store = TaughtPathStore(tmp_path)
    text = content(kind)
    ext = 'json' if kind == 'map' else kind
    value = import_path(store,'导入道路',ORIGIN,'../road.'+ext,text,{})
    poses = [(p['x'],p['y'],p['yaw']) for p in value['points']]
    assert value['projection'] == ORIGIN and value['frame_id'] == 'map'
    assert value['state'] == 'READY' and value['source_kind'] == 'import'
    assert poses[0][:2] == pytest.approx((0,0),abs=1e-6)
    assert poses[-1][:2] == pytest.approx((4,2),abs=1e-6)
    original,_,_,_=parse_coordinates('road.'+ext,text,ORIGIN)
    assert value['points'] == original
    assert value['statistics']['length_m'] == pytest.approx(1+math.hypot(1,.3)+math.hypot(1,.7)+math.sqrt(2),abs=1e-6)
    assert len(point_turn_indices(poses)) == 0
    assert value['statistics']['raw_samples'] == value['statistics']['path_points'] == 5
    assert value['processing'] == {'mode':'original_points','allow_reverse':False}
    assert all(d == 1 for d in directed_poses(poses)[1])
    assert all('stamp' not in p for p in value['points'])
    assert 'stamp' not in value['units']
    provenance = value['import_source']
    assert provenance['timing_available'] is False and provenance['filename'] == 'road.'+ext
    directory = store.directory(value['id'])
    assert (directory/provenance['archive']).read_text() == text
    assert provenance['sha256'] == hashlib.sha256(text.encode()).hexdigest()
    assert not (directory/'raw.csv').exists()
    assert len(store.list()) == 1


def test_reverse_requires_supplied_body_heading_and_explicit_permission(tmp_path):
    data = {'frame_id':'map','projection':ORIGIN,'points':[
        {'x':x,'y':0,'yaw':0} for x in (0,1,2,1,0)]}
    store = TaughtPathStore(tmp_path)
    with pytest.raises(ValueError,match='倒车'):
        import_path(store,'倒车',ORIGIN,'reverse.json',json.dumps(data),{})
    assert not store.list()
    value = import_path(store,'倒车',ORIGIN,'reverse.json',json.dumps(data),{'allow_reverse':True})
    assert value['import_source']['yaw_supplied']
    assert -1 in directed_poses([(p['x'],p['y'],p['yaw']) for p in value['points']])[1]


@pytest.mark.parametrize('mutation', ['missing_origin','wrong_origin','nonfinite','missing','mixed_yaw','degree_yaw','lateral','far','short','too_many','bool'])
def test_invalid_import_does_not_create_a_library_record(tmp_path, mutation):
    data = json.loads(content('map'))
    if mutation == 'missing_origin':data.pop('projection')
    if mutation == 'wrong_origin':data['projection'] = {**ORIGIN,'latitude':36.5}
    if mutation == 'nonfinite':data['points'][1]['x'] = math.nan
    if mutation == 'bool':data['points'][1]['x'] = True
    if mutation == 'missing':data['points'][1].pop('x')
    if mutation in ('mixed_yaw','degree_yaw','lateral'):
        for p in data['points']:p['yaw'] = 0
        if mutation == 'mixed_yaw':data['points'][0].pop('yaw')
        if mutation == 'degree_yaw':data['points'][0]['yaw'] = 90
        if mutation == 'lateral':
            for p in data['points']:p['yaw'] = math.pi/2
    if mutation == 'far':data['points'][0]['x'] = 6000
    if mutation == 'short':data['points'] = data['points'][:2]
    if mutation == 'too_many':data['points'] *= 4001
    store = TaughtPathStore(tmp_path)
    with pytest.raises(ValueError):import_path(store,'错误输入',ORIGIN,'bad.json',json.dumps(data),{})
    assert not store.list()


@pytest.mark.parametrize('filename,text', [
    ('a.geojson',json.dumps({'type':'Polygon','coordinates':[]})),
    ('a.geojson',json.dumps({'type':'FeatureCollection','features':[]})),
    ('a.csv','latitude\n36.6\n36.7\n36.8'),
    ('a.csv','longitude,latitude\n117,36,3\n117,36\n117,36'),
    ('a.json','{"points":'),
    ('a.xlsx','not a spreadsheet importer'),
    ('a.csv','x'*(MAX_BYTES+1)),
])
def test_wrong_formats_and_file_limit_are_explicit(filename,text):
    with pytest.raises(ValueError):parse_coordinates(filename,text,ORIGIN)


def test_long_reference_never_expands_to_resampled_points(tmp_path):
    data = {'frame_id':'map','projection':ORIGIN,'points':[{'x':0,'y':0},{'x':2000,'y':0},{'x':4000,'y':0}]}
    value = import_path(TaughtPathStore(tmp_path),'长路径',ORIGIN,'long.json',json.dumps(data),{'spacing':.03})
    assert len(value['points']) == 3
    assert value['statistics']['length_m'] == 4000


def test_dense_irregular_points_and_supplied_yaw_survive_legacy_processing_options(tmp_path):
    points = [{'x':x,'y':.01*math.sin(x),'yaw':yaw} for x,yaw in zip(
        (0., .001, .017, .2, .9), (.05, .06, .07, .08, .09))]
    data = {'frame_id':'map','projection':ORIGIN,'points':points}
    value = import_path(TaughtPathStore(tmp_path),'原始数据',ORIGIN,'raw.json',json.dumps(data),
                        {'spacing':1.,'smoothing':.1,'min_spacing':.1})
    assert value['points'] == points
    assert value['statistics']['retained_samples'] == len(points)
    assert value['statistics']['rejected']['stationary'] == 0
    assert value['import_source']['yaw_supplied']


def test_explicit_turn_and_duplicate_points_are_preserved_without_inventing_poses(tmp_path):
    points=[{'x':x,'y':y,'yaw':yaw} for x,y,yaw in
            [(0,0,0),(0,0,0),(1,0,0),(1,0,math.pi/2),(1,1,math.pi/2)]]
    data={'frame_id':'map','projection':ORIGIN,'points':points}
    value=import_path(TaughtPathStore(tmp_path),'停车转向',ORIGIN,'turn.json',json.dumps(data),{})
    assert value['points']==points
    assert value['statistics']['point_turn_count']==1


def test_missing_turn_marker_is_rejected_instead_of_rewriting_sharp_corner(tmp_path):
    data={'frame_id':'map','projection':ORIGIN,'points':[
        {'x':0,'y':0},{'x':1,'y':0},{'x':1,'y':1}]}
    store=TaughtPathStore(tmp_path)
    with pytest.raises(ValueError,match='急角缺少原地转向标记'):
        import_path(store,'急角',ORIGIN,'turn.json',json.dumps(data),{})
    assert not store.list()


@pytest.fixture
def api(tmp_path,monkeypatch):
    import agri_backend.following as following
    from agri_backend.workspace import Workspace
    from agri_backend.missions import MissionStore
    manager = following.FollowingManager(tmp_path)
    monkeypatch.setattr(following,'following_manager',manager)
    monkeypatch.setattr(device_cache,'values',{})
    calls = []
    def bridge():
        calls.append('bridge')
        raise AssertionError('Import must not call ROS')
    app = FastAPI()
    app.state.workspace = Workspace(bridge,lambda:{},MissionStore(tmp_path))
    monkeypatch.setattr(app.state.workspace.store,'configured_origin',lambda:ORIGIN)
    following.install_routes(app,bridge)
    yield TestClient(app),manager,calls
    manager.close()


def test_offline_api_import_only_adds_library_item_and_does_not_load_or_move(api):
    client, manager, calls = api
    manager.loaded = {'id':'previous'}
    manager.reference = [{'x':-1,'y':0,'yaw':0}]
    response = client.post('/api/following/import',json={'name':'新路径','filename':'road.csv','content':content()})
    assert response.status_code == 200,response.text
    value = response.json()
    assert value['state'] == 'READY' and value['source_kind'] == 'import'
    assert manager.loaded == {'id':'previous'}
    assert manager.reference == [{'x':-1,'y':0,'yaw':0}]
    assert not calls
    assert client.get('/api/following/paths/'+value['id']).status_code == 200
    assert client.post('/api/following/start',json={}).status_code == 409
    assert not calls


@pytest.mark.parametrize('state', ['RECORDING','PROCESSING','RUNNING','PAUSED','ACQUIRING'])
def test_import_is_excluded_from_current_recording_or_following(api,state):
    client,manager,calls = api
    if state in ('RECORDING','PROCESSING'):manager.state = state
    else:device_cache.update('following',{'state':state})
    response = client.post('/api/following/import',json={'name':'新路径','filename':'road.csv','content':content()})
    assert response.status_code == 409 and not calls and not manager.store.list()
