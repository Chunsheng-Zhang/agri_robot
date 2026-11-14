"""Opt-in terrain matrix through real Web/ROS planners; never start a task.

Planning infeasibility is recorded separately from unsafe published geometry.
Set AGRI_TERRAIN_REPORT to retain the matrix and complete path evidence.
"""
import json
import math
import os
import shutil
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(os.getenv('AGRI_TERRAIN_TESTS') != '1', reason='Opt-in isolated terrain matrix')


def test_terrain_matrix_never_publishes_outside_or_hole_crossing_paths():
    from fastapi.testclient import TestClient
    from pyproj.enums import TransformDirection
    from shapely.geometry import Polygon, LineString, box, mapping
    from shapely.ops import transform
    from agri_base.path_geometry import directed_poses, validate_point_turn_path
    from agri_path_planning.field import load_field
    from web_test_runtime import SyntheticSystem

    report = Path(os.environ['AGRI_TERRAIN_REPORT'])
    report.parent.mkdir(parents=True, exist_ok=True)
    system = SyntheticSystem()
    client = TestClient(system.app)
    frame = system.localization.frame
    fields = {
        'convex_rectangle': box(0,0,24,18),
        'convex_pentagon': Polygon([(0,0),(22,0),(26,12),(14,22),(0,16)]),
        'concave_L': Polygon([(0,0),(24,0),(24,10),(12,10),(12,22),(0,22)]),
        'concave_U': Polygon([(0,0),(24,0),(24,22),(16,22),(16,8),(8,8),(8,22),(0,22)]),
        'one_hole': Polygon([(0,0),(28,0),(28,22),(0,22)],holes=[[(11,8),(17,8),(17,14),(11,14)]]),
        'two_holes': Polygon([(0,0),(32,0),(32,24),(0,24)],holes=[[(7,8),(11,8),(11,12),(7,12)],[(21,12),(25,12),(25,16),(21,16)]]),
        'concave_with_hole': Polygon([(0,0),(28,0),(28,12),(18,12),(18,24),(0,24)],holes=[[(6,8),(10,8),(10,12),(6,12)]]),
        'narrow_neck': Polygon([(0,0),(8,0),(8,5),(14,5),(14,0),(22,0),(22,12),(14,12),(14,5.2),(8,5.2),(8,12),(0,12)]),
        'drivable_neck': Polygon([(0,0),(8,0),(8,5),(14,5),(14,0),(22,0),(22,12),(14,12),(14,6.2),(8,6.2),(8,12),(0,12)]),
    }
    root = Path(__file__).resolve().parents[1]
    for key in ('a','b'):
        file = root/f'data/fields/import_examples/concave_field_{key}.geojson'
        if file.exists():
            polygon = load_field(file,frame)
            # Preserve the shared map coordinates, including their offsets.
            fields['generated_concave_'+key] = polygon
    if os.getenv('AGRI_TERRAIN_FILTER'):
        selected=set(os.environ['AGRI_TERRAIN_FILTER'].split(','))
        fields={name:field for name,field in fields.items() if name in selected}
        assert fields and len(fields)==len(selected), 'Unknown terrain filter'
    results = []
    settings = {'operation_width':.8,'robot_width':.2,'robot_length':.45,
                'rotation_safety_margin':.05,'headland_width':3.,'min_turning_radius':1.,
                'sampling_distance':.1,'quality_resolution':.15,
                'allow_reverse':os.getenv('AGRI_TERRAIN_REVERSE')=='1'}
    def get(url):
        response = client.get(url)
        assert response.status_code==200, response.text
        return response.json()
    def post(url,body):
        response = client.post(url,json=body)
        assert response.status_code==200, response.text
        return response.json()
    def wait(predicate, timeout=130):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            value=predicate()
            if value:return value
            time.sleep(.04)
        raise AssertionError(f'Terrain timeout: {system.planner.state}: {system.planner.reason}')
    def geo(x,y,z=None):
        return frame.transformer.transform(frame.e0+x,frame.n0+y,direction=TransformDirection.INVERSE)
    def persist():
        report.write_text(json.dumps({'schema_version':1,'projection':frame.identity,'parameters':settings,
            'ros_domain':int(os.environ['ROS_DOMAIN_ID']),'hardware':False,'cases':results},ensure_ascii=False,indent=2)+'\n')
    try:
        wait(lambda:get('/api/devices')['safety']['fresh'])
        post('/api/system/mode',{'mode':'coverage'})
        wait(system.planner.mode_ready)
        wait(lambda:system.planner.client.server_is_ready())
        for field_name, field in fields.items():
            assert field.is_valid
            field_dir=report.parent/'fields';field_dir.mkdir(exist_ok=True)
            (field_dir/f'{field_name}.geojson').write_text(json.dumps(mapping(transform(geo,field))))
            variants=[(name,'DUBINS_CC',False) for name in ('BOUSTROPHEDON','SNAKE','SPIRAL','CONTOUR','SMART')]
            if field_name in ('convex_rectangle','one_hole'):
                variants += [('BOUSTROPHEDON',curve,False) for curve in ('DUBINS','REEDS_SHEPP','REEDS_SHEPP_HC')]
            if field_name in ('convex_rectangle','concave_L','one_hole','narrow_neck'):
                variants += [('BOUSTROPHEDON','DUBINS_CC',True),('SMART','DUBINS_CC',True)]
            if os.getenv('AGRI_TERRAIN_CURVES'):
                curves=set(os.environ['AGRI_TERRAIN_CURVES'].split(','))
                variants=[v for v in variants if v[0]=='BOUSTROPHEDON' and v[1] in curves and not v[2]]
            for mode,curve,entry in variants:
                label=f'{field_name}__{mode}__{curve}'+('__entry' if entry else '')
                started=time.monotonic()
                row={'case':label,'terrain':field_name,'mode':mode,'curve':curve,'entry':entry,
                     'field_area_m2':field.area,'holes':len(field.interiors),'state':'PENDING'}
                results.append(row);persist()
                mission_body={'name':label[:80],'mode':'coverage','boundary':mapping(transform(geo,field)),
                    'parameters':{**settings,'route_mode':'BOUSTROPHEDON' if mode=='SMART' else mode,'path_continuity':curve}}
                if entry:
                    lon,lat=geo(field.bounds[0]-2,field.bounds[1]-2)
                    mission_body['entry']={'longitude':lon,'latitude':lat,'source':'coordinates'}
                mission=post('/api/missions',mission_body)
                row['mission_id']=mission['id']
                endpoint='optimize' if mode=='SMART' else 'plan'
                response=client.post(f'/api/missions/{mission["id"]}/{endpoint}')
                if response.status_code!=200:
                    row.update(state='REQUEST_REJECTED',reason=response.text)
                    # Immediate interface validation fails the planner service;
                    # the backend records the failed run and clears reference.
                    if mode in ('BOUSTROPHEDON','SNAKE','SPIRAL') and field.buffer(-settings['headland_width']).geom_type=='MultiPolygon':
                        assert '分裂' in response.text and '拒绝' in response.text
                        wait(lambda:get('/api/robot/path/reference')['count']==0)
                        row['failure_reference_cleared']=True
                else:
                    run_id=response.json()['run']['id']
                    row['run_id']=run_id
                    def finished():
                        active=get('/api/workspace')['active_run']
                        return active if active and active['id']==run_id and active['state'] in ('READY','FAILED','FAULT','CANCELED') else None
                    run=wait(finished)
                    row.update(state=run['state'],reason=run.get('reason',''))
                    directory=system.workspace.store.run_path(run_id)
                    evidence=report.parent/'runs'/run_id;evidence.mkdir(parents=True,exist_ok=True)
                    row['evidence_dir']=str(evidence)
                    for file in [directory/'run.json',directory/'config/mission.json',*list((directory/'paths').glob('*.json'))]:
                        if file.exists():shutil.copy2(file,evidence/file.name)
                    if run['state']=='READY':
                        saved=json.loads(next((directory/'paths').glob('coverage_*.json')).read_text())
                        poses=[(p['x'],p['y'],p['yaw']) for p in saved['points']]
                        path=LineString([p[:2] for p in poses])
                        margin=settings['robot_width']/2
                        if mode=='SMART':
                            envelope=saved['planning']['rotation_envelope']
                            validate_point_turn_path(poses,field,envelope)
                            margin=envelope['rotation_radius_m']
                        directed_poses(poses)
                        outside=path.difference(field.buffer(-margin).buffer(1e-5)).length
                        hole_crossing=sum(path.intersection(Polygon(h)).length for h in field.interiors)
                        row.update(points=len(poses),length_m=path.length,clearance_m=margin,
                            outside_length_m=outside,hole_crossing_length_m=hole_crossing,
                            safety_passed=outside<1e-5 and hole_crossing<1e-5)
                        if mode=='SMART':
                            optimization=json.loads((directory/'paths/optimization.json').read_text())
                            selected=optimization['selected']
                            row.update(coverage_percent=selected['coverage_percent'],overlap_percent=selected['overlap_percent'],
                                       angle_rad=selected['angle_rad'],candidates=optimization['evaluated_count'])
                        assert row['safety_passed'],row
                    else:
                        assert not list((directory/'paths').glob('coverage_*.json')),row
                        wait(lambda:get('/api/robot/path/reference')['count']==0)
                        row['failure_reference_cleared']=True
                row['elapsed_seconds']=time.monotonic()-started
                safety=get('/api/devices')['safety']
                assert safety['emergency_stop'] and not safety['owner']
                assert system.state['v']==system.state['w']==0
                row['motion_commands_zero']=True
                persist()
                print(json.dumps(row,ensure_ascii=False),flush=True)
        assert any(r['state']=='READY' for r in results)
    finally:
        persist()
        for log in system.logs:
            log.flush()
            shutil.copy2(log.name,report.parent/Path(log.name).name)
        system.close()
