#!/usr/bin/env python3
"""Offline 50-field paired benchmark. Never imports ROS or edits live settings.

Run after sourcing scripts/setup_environment.bash with the backend venv Python.
The native reference is a declared best-of-portfolio, not a global-best claim.
"""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_OUT=ROOT/'.diagnostics/fields50_20261114'


def atomic(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(value,ensure_ascii=False,indent=2));temp.replace(path)


def dump_gzip(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with gzip.open(path,'wt',encoding='utf-8') as stream:json.dump(value,stream,ensure_ascii=False)


def parameters(width):
    return dict(operation_width=width,robot_width=.2,robot_length=.45,rotation_safety_margin=.05,
                rotation_center_x=0.,rotation_center_y=0.,sampling_distance=.1,quality_resolution=.05)


def source_hashes():
    folder=ROOT/'agri_robot_ws/src/agri_path_planning/agri_path_planning'
    files=[folder/n for n in ('optimizer.py','edge_candidates.py','cell_routing.py','native_route.py',
        'point_turn_planner.py','planning_options.py','coverage_geometry.py','contour.py')]
    if (folder/'native_layouts.py').exists():files.append(folder/'native_layouts.py')
    files += [Path(__file__),Path(__file__).with_name('benchmark_coverage_metrics.py')]
    native=ROOT/'open_navigation_ws/src/Fields2Cover'
    files += [native/n for n in ('CMakeLists.txt','src/fields2cover/swath_generator/brute_force.cpp',
        'src/fields2cover/swath_generator/swath_generator_base.cpp','src/fields2cover/route_planning/route_planner_base.cpp')]
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def load_field(record):
    from shapely import wkt
    from shapely.ops import transform
    from shapely.affinity import translate
    from pyproj import Transformer
    path=ROOT/record['file'];assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256']
    raw=wkt.loads(path.read_text())
    if raw.geom_type!='Polygon' or not raw.is_valid:raise ValueError('INVALID_INPUT: original WKT is not a valid Polygon; no repair or replacement')
    projected=transform(Transformer.from_crs(4326,record['epsg'],always_xy=True).transform,raw)
    e,n=record['utm_anchor'];return translate(projected,xoff=-e,yoff=-n)


def identify_rows(cells,swaths,polygons,check):
    """Common ID/Route integrity adapter; no project edge layout or row repair."""
    import fields2cover as f
    from agri_path_planning.cell_routing import MAX_NATIVE_GRAPH_NODES
    result=f.SwathsByCells();expected={};ends=set()
    for cell_id in range(swaths.size()):
        rows=f.Swaths();part=swaths[cell_id]
        if not part.size():raise ValueError('Native swath generator returned an empty cell')
        for i in range(part.size()):
            check();row=part[i];row.setId(len(expected))
            a,b=row.startPoint(),row.endPoint();a,b=(a.getX(),a.getY()),(b.getX(),b.getY())
            if math.dist(a,b)<1e-5:raise ValueError('Native swath too short for shared executor')
            keys=[tuple(round(x,7) for x in p) for p in (a,b)]
            if any(k in ends for k in keys):raise ValueError('Native swaths share an endpoint; common Route adapter rejects ambiguity')
            ends.update(keys);expected[row.getId()]={'cell_id':cell_id,'start':a,'end':b};rows.push_back(row)
        result.emplace_back(rows)
    if not expected:raise ValueError('No native swaths')
    if len(expected)>1500:raise ValueError('作业行数量超过1500条几何预算')
    junctions={tuple(p[:2]) for poly in polygons for p in poly.exterior.coords}
    if len(ends)+len(junctions)>MAX_NATIVE_GRAPH_NODES:raise ValueError('原生TSP端点/通道图超过内存保护规模')
    return {'cells':cells,'swaths':result,'expected':expected,'angle':0.,'polygons':polygons,
        'decomposition':'NATIVE_BENCHMARK','precision_m':1e-6,'fallback_reasons':[],'row_layout':None}


def native_portfolio(field,p,progress):
    import fields2cover as f
    from shapely import wkt
    from shapely.ops import unary_union
    from agri_base.path_geometry import rotation_envelope,validate_point_turn_path
    from agri_path_planning.cell_routing import snapped
    from agri_path_planning.point_turn_planner import field_cell,sample_path
    from agri_path_planning.native_route import route_vertices
    from agri_path_planning.coverage_geometry import AreaGrid
    from agri_path_planning.optimizer import score,objective_terms
    env=rotation_envelope(p['robot_length'],p['robot_width'],p['rotation_safety_margin'])
    safe=field.buffer(-env['headland_width_m'],join_style=2)
    if safe.is_empty or safe.geom_type!='Polygon':return None,[],0
    stable=snapped(safe);input_cells=f.Cells(field_cell(stable,f));grid=AreaGrid(field,env['headland_width_m'],p['quality_resolution'])
    options=[('NONE',None,None)]+[(kind,cls,angle) for kind,cls in [('BCD',f.DECOMP_Boustrophedon),('TRAPEZOIDAL',f.DECOMP_TrapezoidalDecomp)] for angle in (0.,math.pi/2)]
    candidates=[];best=None;calls=0
    for kind,cls,angle in options:
        try:
            progress({'stage':'decomposition','kind':kind,'split_angle':angle,'candidates':len(candidates)})
            if cls is None:cells=input_cells
            else:
                decomp=cls();decomp.setSplitAngle(angle);cells=decomp.decompose(input_cells)
            polygons=[wkt.loads(cells.getGeometry(i).exportToWkt()) for i in range(cells.size())]
            union=unary_union(polygons)
            if (not polygons or any(not g.is_valid or g.is_empty for g in polygons)
                    or not union.buffer(1e-5).covers(safe) or not safe.buffer(1e-5).covers(union)):
                raise ValueError('Native decomposition did not preserve the full safe field')
        except Exception as exc:
            candidates.append({'decomposition':kind,'split_angle':angle,'state':'REJECTED','reason':str(exc)});continue
        for objective in ('NSwath','NSwathModified','SwathLength'):
            label={'decomposition':kind,'split_angle':angle,'swath_objective':objective}
            try:
                progress({'stage':'native_swath_search',**label,'candidates':len(candidates)})
                generator=f.SG_BruteForce();generator.setStepAngle(math.pi/180)
                generated=generator.generateBestSwaths(getattr(f,'OBJ_'+objective)(),p['operation_width'],cells)
                prepared=identify_rows(cells,generated,polygons,lambda:None)
            except Exception as exc:
                candidates.append({**label,'state':'REJECTED','reason':str(exc)});continue
            for guided in (False,True):
                row={**label,'guided_local_search':guided,'state':'REJECTED'}
                try:
                    progress({'stage':'native_tsp',**label,'guided':guided,'candidates':len(candidates)})
                    calls+=1
                    route=f.RP_RoutePlannerBase().genRoute(cells,prepared['swaths'],False,1e-4,True,1,guided)
                    vertices,details=route_vertices(route,prepared,lambda:None)
                    poses,vertices,turns=sample_path(vertices,p['sampling_distance'],lambda:None)
                    validate_point_turn_path(poses,field,env)
                    counts,_=grid.visits([vertices],p['operation_width']);metrics=grid.metrics(counts)['field']
                    length=sum(math.dist(a,b) for a,b in zip(vertices,vertices[1:]));n0=max(1.,2*math.sqrt(field.area)/p['operation_width'])
                    cost=score(metrics['coverage_percent'],metrics['overlap_percent'],length,field.area/p['operation_width'],turns['point_turn_count'],n0)
                    row.update(state='EVALUATED',score=cost,coverage_percent=metrics['coverage_percent'],overlap_percent=metrics['overlap_percent'],
                        length_m=length,point_turn_count=turns['point_turn_count'],swath_count=details['swath_count'],
                        objective_terms=objective_terms(metrics['coverage_percent'],metrics['overlap_percent'],length,field.area/p['operation_width'],turns['point_turn_count'],n0),
                        cell_count=len(polygons),actual_grid_m=grid.resolution)
                    if best is None or (cost,-metrics['coverage_percent'],length)<(best['selected']['score'],-best['selected']['coverage_percent'],best['selected']['length_m']):
                        best={'points':poses,'selected':row,'rotation_envelope':env,'details':details}
                except Exception as exc:row['reason']=str(exc)
                candidates.append(row)
    return best,candidates,calls


def worker(args):
    # Process-local CPU affinity bounds TBB's native angle parallelism as well.
    cpus=sorted(os.sched_getaffinity(0));start=(args.cpu_slot*2)%max(2,len(cpus));os.sched_setaffinity(0,cpus[start:start+2])
    import resource
    from shapely.validation import explain_validity
    from agri_base.path_geometry import validate_point_turn_path,point_turn_indices,rotation_envelope
    from benchmark_coverage_metrics import swept_metrics,compact_path
    from agri_path_planning import optimizer
    from agri_path_planning.coverage_geometry import AreaGrid
    out=Path(args.out);manifest=json.loads((out/'manifest.json').read_text());record=manifest['fields'][args.index]
    key=f'{args.index:02d}_{Path(record["file"]).stem}_{args.width:g}_{args.worker}'
    target=out/'jobs'/f'{key}.json';p=parameters(args.width);started=time.monotonic();calls=0
    def progress(v):
        print(json.dumps({'seconds':round(time.monotonic()-started,2),**v},ensure_ascii=False),flush=True)
    result={'key':key,'index':args.index,'field':Path(record['file']).stem,'method':args.worker,'width_m':args.width,
        'source':record,'parameters':p,'hardware':False,'signature':args.signature}
    try:
        field=load_field(record);result['area_m2']=field.area
        if args.worker=='current':
            native=optimizer.plan_native_route
            def observed(*a,**kw):
                nonlocal calls
                calls+=1;return native(*a,**kw)
            optimizer.plan_native_route=observed
            last=[-1,'']
            def event(v):
                if v['evaluated_count']>=last[0]+10 or v['phase']!=last[1]:
                    progress(v);last[:]=[v['evaluated_count'],v['phase']]
            plan_started=time.monotonic()
            planned=optimizer.optimize(field.wkb,p,progress=event)
            result.update(planner_wall_seconds=time.monotonic()-plan_started,
                          reported_search_seconds=planned['optimization']['elapsed_seconds'],
                          planning_policy_version=planned['metadata'].get('policy_version'))
            dump_gzip(out/'jobs'/f'{key}_optimization.json.gz',planned['optimization'])
            candidates=planned['optimization']['candidates'];best=({'points':planned['points'],'selected':planned['optimization']['selected'],
                'rotation_envelope':planned['metadata']['rotation_envelope'],'details':planned['metadata']['path_details']} if planned['points'] else None)
        else:
            best,candidates,calls=native_portfolio(field,p,progress)
            dump_gzip(out/'jobs'/f'{key}_optimization.json.gz',{'candidates':candidates,'selected':best['selected'] if best else None})
        result.update(planning_seconds=time.monotonic()-started,native_tsp_calls=calls,
            evaluated_candidates=len(candidates),feasible_candidates=sum(c['state']=='EVALUATED' for c in candidates),
            rejection_counts=dict(Counter(c.get('reason','') for c in candidates if c['state']=='REJECTED')))
        if best is None:
            result.update(state='FAILED',reason='No complete feasible candidate; see rejection_counts')
        else:
            points=best['points'];validate_point_turn_path(points,field,best['rotation_envelope'])
            selected=dict(best['selected']);nturns=len(point_turn_indices(points));assert selected['point_turn_count']==nturns
            progress({'stage':'independent_vector_evaluation','points':len(points)})
            vector=swept_metrics(points,field,args.width)
            vector['point_turn_count']=nturns;vector['score']=optimizer.score(vector['coverage_percent'],vector['overlap_percent'],vector['length_m'],field.area/args.width,nturns,max(1.,2*math.sqrt(field.area)/args.width))
            grid=AreaGrid(field,best['rotation_envelope']['headland_width_m'],p['quality_resolution'])
            line=compact_path(points)
            outside=__import__('shapely.geometry',fromlist=['LineString']).LineString(line).difference(field.buffer(-best['rotation_envelope']['rotation_radius_m']).buffer(1e-5)).length
            assert outside<1e-5
            result.update(state='SUCCEEDED',selected=selected,independent=vector,actual_grid_m=grid.resolution,
                point_count=len(points),clearance_passed=True,outside_length_m=outside,
                grid_coarser_than_half_width=grid.resolution>args.width/2,
                grid_vector_coverage_difference_pp=selected['coverage_percent']-vector['coverage_percent'],
                grid_vector_overlap_difference_pp=selected['overlap_percent']-vector['overlap_percent'])
            dump_gzip(out/'jobs'/f'{key}_path.json.gz',{'frame_id':'benchmark_local_utm','source_crs_assumption':'EPSG:4326 longitude/latitude',
                'epsg':record['epsg'],'utm_anchor':record['utm_anchor'],'points':points,'vertices':line,
                'field_wkt':field.wkt,'rotation_envelope':best['rotation_envelope'],'selected':selected,
                'path_details':best.get('details')})
    except Exception as exc:
        result.update(state='INVALID_INPUT' if str(exc).startswith('INVALID_INPUT:') else 'ERROR',reason=str(exc),traceback=traceback.format_exc())
    result['total_seconds']=time.monotonic()-started;result['peak_rss_mb']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    atomic(target,result);print(json.dumps({'state':result['state'],'key':key,'seconds':result['total_seconds']},ensure_ascii=False),flush=True)


def run(args):
    out=Path(args.out);manifest=json.loads((out/'manifest.json').read_text())
    indices=args.indices if args.indices is not None else list(range(min(args.limit,len(manifest['fields']))))
    if len(set(indices))!=len(indices) or any(i<0 or i>=len(manifest['fields']) for i in indices):
        raise ValueError('Invalid or duplicate benchmark input indices')
    config={'schema_version':1,'sampling_seed':manifest['seed'],'profiles':args.widths,
        'input_indices':indices,'methods':args.methods,
        'parameters':parameters(args.widths[0]),'source_hashes':source_hashes(),
        'native_portfolio':{'partitions':['NONE','BCD_0','BCD_90','TRAPEZOIDAL_0','TRAPEZOIDAL_90'],
            'swath_objectives':['NSwath','NSwathModified','SwathLength'],'angle_step_deg':1,
            'route_search':['AUTOMATIC_1s','GUIDED_LOCAL_SEARCH_1s'],'max_complete_configs':30,
            'motion_model':'shared stopped-turn executor; not native Ackermann curves'},
        'guards':{'max_job_seconds':1200,'max_rss_mb':1400,'worker_count':args.workers,'worker_cpus':2,
            'minimum_start_available_mb':args.min_start_memory_mb,'global_available_memory_floor_mb':350,
            'production_geometry_and_sampling_limits_unchanged':True},
        'source_crs_assumption':'EPSG:4326 lon/lat; UTM per field from centroid; same transform for both methods; no scaling',
        'no_live_ros':True}
    from concurrent.futures import ThreadPoolExecutor,as_completed
    from queue import Queue
    from threading import Lock
    signature=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest();config['signature']=signature
    atomic(out/'configuration.json',config);results=[]
    slots=Queue()
    for i in range(args.workers):slots.put(i)
    active={};lock=Lock()
    def one(job):
        index,record,width,method=job;key=f'{index:02d}_{Path(record["file"]).stem}_{width:g}_{method}'
        target=out/'jobs'/f'{key}.json'
        if target.exists() and json.loads(target.read_text()).get('signature')==signature:return json.loads(target.read_text())
        target.parent.mkdir(exist_ok=True);slot=slots.get();process=None
        try:
            cmd=[sys.executable,'-B',str(Path(__file__).resolve()),'--worker',method,'--index',str(index),
                '--width',str(width),'--out',str(out),'--signature',signature,'--cpu-slot',str(slot)]
            print(f'START {key} slot={slot}',flush=True)
            last_notice=0.
            while args.min_start_memory_mb:
                available=next(float(v.split()[1])/1024 for v in Path('/proc/meminfo').read_text().splitlines() if v.startswith('MemAvailable:'))
                if available>=args.min_start_memory_mb:break
                if time.monotonic()-last_notice>30:
                    print(f'WAIT_MEMORY {key}: {available:.0f}MiB available; start requires {args.min_start_memory_mb}MiB',flush=True)
                    last_notice=time.monotonic()
                time.sleep(1)
            with (out/'jobs'/f'{key}.log').open('w') as stream:
                process=subprocess.Popen(cmd,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                start=time.monotonic();peak=0.;terminated=None
                with lock:active[process.pid]=0.
                while process.poll() is None:
                    try:
                        status=Path(f'/proc/{process.pid}/status').read_text()
                        rss=next(float(v.split()[1])/1024 for v in status.splitlines() if v.startswith('VmRSS:'));peak=max(peak,rss)
                        with lock:active[process.pid]=rss
                    except (OSError,StopIteration):pass
                    available=next(float(v.split()[1])/1024 for v in Path('/proc/meminfo').read_text().splitlines() if v.startswith('MemAvailable:'))
                    with lock:largest=max(active,key=active.get) if active else None
                    if peak>1400:terminated='BENCHMARK_MEMORY_GUARD'
                    elif available<350 and largest==process.pid:terminated='BENCHMARK_GLOBAL_MEMORY_GUARD'
                    elif time.monotonic()-start>1200:terminated='BENCHMARK_TIMEOUT'
                    if terminated:
                        os.killpg(process.pid,signal.SIGTERM)
                        try:process.wait(timeout=5)
                        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
                        break
                    time.sleep(.25)
            if terminated or process.returncode!=0 or not target.exists():
                row={'key':key,'index':index,'field':Path(record['file']).stem,'method':method,'width_m':width,
                    'state':terminated or 'WORKER_CRASH','reason':f'exit={process.returncode}; inspect job log',
                    'planning_seconds':time.monotonic()-start,'peak_rss_mb':peak,'source':record,'signature':signature}
                atomic(target,row)
            return json.loads(target.read_text())
        finally:
            if process is not None:
                with lock:active.pop(process.pid,None)
            slots.put(slot)
    jobs=[(index,manifest['fields'][index],width,method) for index in indices
          for width in args.widths for method in args.methods]
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures=[executor.submit(one,job) for job in jobs]
        for future in as_completed(futures):
            row=future.result();results.append(row);atomic(out/'results.json',sorted(results,key=lambda r:r['key']))
            print(json.dumps({'completed':len(results),'total':len(jobs),'key':row['key'],'state':row['state'],
                'planning_seconds':row.get('planning_seconds'),'coverage':row.get('independent',{}).get('coverage_percent')},ensure_ascii=False),flush=True)
    atomic(out/'results.json',sorted(results,key=lambda r:r['key']))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',default=str(DEFAULT_OUT));parser.add_argument('--run',action='store_true')
    parser.add_argument('--limit',type=int,default=50);parser.add_argument('--worker',choices=['current','native'])
    parser.add_argument('--indices',type=int,nargs='+');parser.add_argument('--widths',type=float,nargs='+',default=[.8,6.])
    parser.add_argument('--methods',nargs='+',choices=['current','native'],default=['current','native'])
    parser.add_argument('--workers',type=int,choices=range(1,5),default=4)
    parser.add_argument('--min-start-memory-mb',type=int,default=0)
    parser.add_argument('--cpu-slot',type=int,default=0);parser.add_argument('--index',type=int);parser.add_argument('--width',type=float);parser.add_argument('--signature',default='pilot')
    args=parser.parse_args()
    if args.worker:worker(args)
    elif args.run:run(args)
    else:parser.error('Specify --run or --worker')

if __name__=='__main__':main()
