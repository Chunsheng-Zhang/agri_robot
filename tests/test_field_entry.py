"""Manual entry selection, native ordering and separate safe approach execution."""
import copy
import json
import math
from pathlib import Path

import pytest
from shapely.geometry import Polygon, Point, LineString
from pyproj.enums import TransformDirection

from agri_base.coordinates import LocalFrame
from agri_backend.missions import Mission, MissionStore, CoverageParameters
from agri_path_planning.entry import EntryConnector, parse_entry, validate_entry_metadata
from agri_path_planning.entry_planner import manual_entry_path, contour_entry_path
from agri_path_planning.optimizer import optimize

ROOT=Path(__file__).resolve().parents[1]


def parameters(**changes):
    return {**CoverageParameters(operation_width=2.,headland_width=2.,min_turning_radius=.5,
        sampling_distance=.15,path_continuity='DUBINS',quality_resolution=.2).model_dump(),
        'linear_curv_change':2.,'planning_timeout':120.,**changes}


def entry(x,y):
    return {'latitude':36.,'longitude':117.,'source':'map','x':float(x),'y':float(y)}


@pytest.mark.parametrize('mode',['BOUSTROPHEDON','SNAKE','SPIRAL'])
def test_native_manual_planner_changes_start_without_dropping_any_swath(mode):
    field=Polygon([(0,0),(20,0),(20,12),(0,12)])
    rows=[((2.,float(y)),(18.,float(y))) for y in (2,4,6,8,10)]
    a=manual_entry_path(field.wkb,rows,parameters(route_mode=mode,entry=entry(1,1)))
    b=manual_entry_path(field.wkb,rows,parameters(route_mode=mode,entry=entry(19,11)))
    assert math.dist(a['points'][0][:2],(1,1))<3
    assert math.dist(b['points'][0][:2],(19,11))<3
    assert a['points'][0][:2]!=b['points'][0][:2]
    assert len(a['metadata']['entry_candidates'])==4
    path=LineString([v[:2] for v in a['points']])
    assert field.buffer(-.099).covers(path)
    for row in rows:
        assert path.distance(Point(row[0]))<1e-5 and path.distance(Point(row[1]))<1e-5
    assert a['metadata']['swath_source']=='OpenNav Coverage'


@pytest.mark.parametrize('curve',['DUBINS_CC','REEDS_SHEPP','REEDS_SHEPP_HC'])
def test_entry_native_curve_selection_is_preserved(curve):
    field=Polygon([(0,0),(20,0),(20,12),(0,12)])
    rows=[((3.,float(y)),(17.,float(y))) for y in (3,5,7,9)]
    value=manual_entry_path(field.wkb,rows,parameters(path_continuity=curve,entry=entry(1,1),allow_reverse=True))
    assert value['metadata']['curve_type']==curve
    assert value['metadata']['allow_reverse'] is True
    assert value['metadata']['entry']['coverage_start']['x']==value['points'][0][0]


def test_entry_connection_detours_hole_and_outside_access_is_not_claimed_safe():
    field=Polygon([(0,0),(20,0),(20,12),(0,12)],holes=[[(8,4),(12,4),(12,8),(8,8)]])
    connector=EntryConnector(field,parameters(),entry(2,6))
    plan=connector.assess((18,6,0))
    assert plan['connection_length_m']>16 and len(plan['connection'])>2
    assert plan['access_verified']
    assert connector.safe.buffer(1e-5).covers(LineString([(p['x'],p['y']) for p in plan['connection']]))
    outside=EntryConnector(field,parameters(),entry(-2,6)).assess((18,6,0))
    assert outside['access_verified'] is False and outside['reference_to_anchor_m']>2
    assert outside['distance_m']>outside['connection_length_m']
    with pytest.raises(ValueError,match='禁入'):EntryConnector(field,parameters(),entry(10,6))


def test_saved_entry_metadata_cannot_be_retargeted_or_connect_through_a_hole():
    field=Polygon([(0,0),(20,0),(20,12),(0,12)],holes=[[(8,4),(12,4),(12,8),(8,8)]])
    expected=entry(2,6);start=(18,6,0)
    plan=EntryConnector(field,parameters(),expected).assess(start)
    validate_entry_metadata(plan,field,parameters(),expected,start)
    altered=copy.deepcopy(plan);altered['reference']['x']=3
    with pytest.raises(ValueError):validate_entry_metadata(altered,field,parameters(),expected,start)
    altered=copy.deepcopy(plan);altered['connection']=[{'x':2,'y':6},{'x':18,'y':6}]
    with pytest.raises(ValueError,match='穿越'):validate_entry_metadata(altered,field,parameters(),expected,start)


def test_smart_search_compares_native_open_route_ends_and_counts_connection_only_in_length():
    field=Polygon([(0,0),(14,0),(14,10),(0,10)])
    p=parameters(entry=entry(1,1),operation_width=2.5)
    result=optimize(field.wkb,p)
    report=result['optimization']
    assert report['state']=='SUCCEEDED' and report['search_complete']
    assert {r['variant'] for r in report['candidates']}=={0,1,2,3}
    assert report['weights']=={'coverage':.4,'overlap':.25,'length':.2,'turns':.15}
    selected=report['selected'];connection=result['metadata']['entry']
    assert selected['entry_length_m']==pytest.approx(connection['distance_m'])
    assert selected['length_m']==pytest.approx(selected['entry_length_m']+selected['working_length_m'])
    from agri_path_planning.coverage_geometry import AreaGrid
    grid=AreaGrid(field,result['metadata']['headland_search']['headland_width_m'],p['quality_resolution'])
    counts,_=grid.visits([[point[:2] for point in result['points']]],p['operation_width'])
    assert grid.metrics(counts)['field']['coverage_percent']==pytest.approx(selected['coverage_percent'])


def test_contour_entry_preserves_inside_out_open_layers():
    field=Polygon([(0,0),(18,0),(18,14),(0,14)])
    result=contour_entry_path(field.wkb,parameters(entry=entry(1,1),headland_width=0.,operation_width=1.5))
    meta=result['metadata']
    assert meta['direction']=='INSIDE_OUT' and meta['closed_laps'] is False
    assert meta['ring_count']>1 and meta['entry_seams_tested']==4
    assert meta['layers'][0]['level']>=meta['layers'][-1]['level']
    assert meta['entry']['coverage_start']['x']==result['points'][0][0]


def test_entry_task_and_map_conversion_round_trip_keep_fixed_origin_and_revision(tmp_path,monkeypatch):
    from fastapi.testclient import TestClient
    from agri_backend import main
    from agri_backend.devices import device_cache
    store=MissionStore(tmp_path);store.config_path=ROOT/'config/jetson_system.yaml'
    origin=store.configured_origin();frame=LocalFrame(origin['latitude'],origin['longitude'])
    def geo(x,y):
        lon,lat=frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
        return {'latitude':lat,'longitude':lon}
    payload={'name':'入口任务','mode':'coverage','boundary':{'points':[geo(0,0),geo(20,0),geo(20,12),geo(0,12)]},
             'entry':{**geo(1,1),'source':'coordinates'}}
    saved=store.save(Mission(**payload),origin)
    assert saved['preview']['entry']['x']==pytest.approx(1,abs=1e-7)
    assert saved['projection']==origin and saved['entry']==payload['entry']
    second=store.save(Mission(**{**payload,'entry':{**geo(19,11),'source':'map'},'revision':1}),origin,saved['id'])
    assert second['revision']==2 and second['preview']['entry']['x']>18
    monkeypatch.setattr(device_cache,'values',{})
    monkeypatch.setattr(main.app.state.workspace,'store',store)
    client=TestClient(main.app)
    response=client.post('/api/coordinates/map-to-wgs84',json={'x':1.,'y':1.,'projection':origin})
    assert response.status_code==200,response.text
    local=parse_entry({k:response.json()[k] for k in ('latitude','longitude')},frame)
    assert local['x']==pytest.approx(1,abs=1e-7) and local['y']==pytest.approx(1,abs=1e-7)
    assert client.post('/api/coordinates/map-to-wgs84',json={'x':1.,'y':1.,'projection':{**origin,'latitude':0}}).status_code==409
    assert client.post('/api/coordinates/map-to-wgs84',json={'x':6000.,'y':1.,'projection':origin}).status_code==422


def test_original_single_waypoint_approach_starts_far_away_then_aligns_and_tracks():
    from agri_path_tracking.controller import Polyline
    from agri_path_tracking.coverage import CoverageController
    from agri_waypoint.controller import WaypointController
    controller=CoverageController(Polyline([(3,3),(5,3)]),(-8,-6),
        lookahead=.2,speed=.4,max_angular=1.,completion_tolerance=.05)
    assert isinstance(controller.approach,WaypointController)
    assert controller.phase=='APPROACHING' and controller.target==(3,3)
    x,y,yaw=-8.,-6.,math.atan2(9,11);seen=set();paused=False;done=False
    for i in range(4000):
        seen.add(controller.phase)
        if controller.phase=='APPROACHING':
            assert controller.progress==0
            if not paused and x>-7:
                controller.align_for_resume();paused=True
        v,w,*_,done=controller.command(x,y,yaw,now=i*.05,sample_stamp=100+i*.05)
        x+=v*math.cos(yaw)*.05;y+=v*math.sin(yaw)*.05;yaw+=w*.05
        if done:break
    assert done and paused and {'APPROACHING','ALIGNING','TRACKING'}<=seen
    assert math.dist((x,y),(5,3))<.06
