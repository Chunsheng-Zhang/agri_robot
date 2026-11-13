"""Complete finite angle search with stopped turns and fixed C/O/L objectives."""
import hashlib
import math
from pathlib import Path
from time import monotonic
from shapely import wkb
from . import contour
from .coverage_geometry import AreaGrid
from .planning_options import WEIGHTS
from .point_turn_planner import StraightRouter, working_rows, improve_rows, compose, field_cell
from agri_base.path_geometry import rotation_envelope, validate_point_turn_path


def score(coverage_percent, overlap_percent, length, reference_length):
    # Dimensionless bounded terms, with a fixed field-based length reference.
    return (.5*(1-coverage_percent/100)+.3*overlap_percent/100+
            .2*length/(length+reference_length))


def pca_angle(field):
    points=[];step=max(.1,field.length/256)
    ring=field.exterior
    for i in range(max(3,min(1024,math.ceil(ring.length/step)))):
        p=ring.interpolate(i*step);points.append((p.x,p.y))
    x=sum(p[0] for p in points)/len(points);y=sum(p[1] for p in points)/len(points)
    xx=sum((a-x)**2 for a,b in points);yy=sum((b-y)**2 for a,b in points)
    xy=sum((a-x)*(b-y) for a,b in points)
    return (.5*math.atan2(2*xy,xx-yy))%math.pi


def optimize(field_wkb, parameters, progress=None):
    field=wkb.loads(field_wkb);p=parameters
    if field.geom_type!='Polygon' or not field.is_valid or field.area<.1:
        raise ValueError('智能规划要求有效地块多边形')
    width=p['operation_width'];spacing=p['sampling_distance'];resolution=p['quality_resolution']
    for name,v,lo,hi in [('operation_width',width,.1,20),('sampling_distance',spacing,.02,1),
                         ('quality_resolution',resolution,.01,.5)]:
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError(f'{name}超出范围')
    envelope=rotation_envelope(p.get('robot_length',.45),p['robot_width'],p.get('rotation_safety_margin',.05),
        p.get('rotation_center_x',0.),p.get('rotation_center_y',0.))
    headland=envelope['headland_width_m'];safe=field.buffer(-headland)
    start=monotonic();rows=[];chosen=None;feasible_count=0;seen=set()
    def check():
        if contour._CANCEL is not None and contour._CANCEL.is_set():raise ValueError('智能规划已取消')
    def emit(phase,completed=0,total=0,**details):
        check()
        if progress is not None:progress({'phase':phase,'stage_completed':completed,'stage_total':total,
            'evaluated_count':len(rows),'feasible_count':feasible_count,
            'elapsed_seconds':monotonic()-start,**details})
    def rank(value):
        return value['score'],-value['coverage_percent'],value['length_m']
    emit('INITIALIZING')
    grid=AreaGrid(field,headland,resolution) if not safe.is_empty else None
    if grid is not None:
        router=StraightRouter(safe,check);reference_length=field.area/width
        def evaluate(angle,phase,completed,total):
            nonlocal chosen,feasible_count
            angle=angle%math.pi;key=round(angle,8)
            if key in seen:return
            seen.add(key);working=None
            for order in ['BOUSTROPHEDON','LOCAL_2OPT']:
                emit(phase,completed,total,angle_rad=angle,order=order)
                row={'id':len(rows)+1,'mode':'BOUSTROPHEDON','headland_width_m':headland,
                     'angle_rad':angle,'variant':0,'order':order,'state':'REJECTED',
                     'headland_search_phase':phase,'executable':False}
                try:
                    check()
                    if working is None:working=working_rows(safe,width,angle)
                    if order=='LOCAL_2OPT':
                        working,changes=improve_rows(working,router,check)
                        if not changes:return
                    points,vertices,details=compose(working,router,spacing,check)
                    validate_point_turn_path(points,field,envelope)
                    counts,_=grid.visits([vertices],width,check=check)
                    metrics=grid.metrics(counts)['field']
                    length=sum(math.dist(a,b) for a,b in zip(vertices,vertices[1:]))
                    value=score(metrics['coverage_percent'],metrics['overlap_percent'],length,reference_length)
                    row.update(state='EVALUATED',executable=True,score=value,length_m=length,
                        coverage_percent=metrics['coverage_percent'],overlap_percent=metrics['overlap_percent'],
                        fine_coverage_percent=metrics['coverage_percent'],fine_overlap_percent=metrics['overlap_percent'],
                        **details)
                    feasible_count+=1
                    # Keep metrics for every candidate, but only the best geometry.
                    # Memory usage must not truncate the remaining angle search.
                    if chosen is None or rank(row)<rank(chosen[0]):chosen=(row,points,details)
                except ValueError as exc:
                    check();row['reason']=str(exc)
                rows.append(row)
                if working is None:return
        axis=pca_angle(field)
        initial=[];initial_seen=set()
        for angle in [axis,(axis+math.pi/2)%math.pi]+[math.radians(i) for i in range(0,180,15)]:
            key=round(angle%math.pi,8)
            if key not in initial_seen:initial.append(angle);initial_seen.add(key)
        for i,angle in enumerate(initial):
            evaluate(angle,'COARSE',i,len(initial))
            emit('COARSE',i+1,len(initial),angle_rad=angle)
        if chosen is not None:
            for i,delta in enumerate((5.,2.,.5)):
                angle=chosen[0]['angle_rad']
                for j,sign in enumerate((-1,1)):
                    target=angle+sign*math.radians(delta)
                    evaluate(target,'REFINE',i*2+j,6)
                    emit('REFINE',i*2+j+1,6,angle_rad=target%math.pi)
    emit('FINALIZING')
    search={'schema_version':2,**envelope,'selected_width_m':headland,
        'range_min_m':headland,'range_max_m':headland,'initial_widths_m':[headland],'refinement_widths_m':[],
        'coarse_evaluated_count':sum(r['headland_search_phase']=='COARSE' for r in rows),
        'refinement_evaluated_count':sum(r['headland_search_phase']=='REFINE' for r in rows)}
    reason=('地块无法容纳车辆原地转向安全包络' if safe.is_empty else
            '没有满足原地转向扫掠净空的完整往复式路径')
    report={'schema_version':2,'strategy':'SMART','policy':'POINT_TURN_ANGLE',
        'state':'SUCCEEDED' if chosen else 'FAILED','reason':'' if chosen else reason,
        'candidates':rows,'evaluated_count':len(rows),'possible_count':len(rows),
        'search_complete':True,'candidate_limit_enabled':False,'search_time_limit_enabled':False,
        'weights':WEIGHTS,'headland_search':search,
        'normalization':{'coverage':'1-C/100','overlap':'O/100','length':'L/(L+L0)',
            'length_reference_m':field.area/width},'coverage_constraint_enabled':False,'coverage_objective_enabled':True,
        'curve_type':'POINT_TURN','turn_mode':'POINT_TURN','allow_reverse':False,
        'reverse_execution_supported':True,'search_resolution_m':grid.resolution if grid is not None else resolution,
        'final_resolution_m':grid.resolution if grid is not None else resolution,
        'selection_scope':'固定机具幅宽；完成预定角度及局部行顺序比较后，按覆盖率0.5、重复覆盖0.3、总长度0.2择优，不保证全局最优。',
        'configuration':{},'elapsed_seconds':monotonic()-start,
        'limitations':['车体按长宽及旋转中心的完整圆包络检查；尺寸须实测，默认值未实车标定。',
            '所有行驶段均按机具开启估计；尚无实际机具开关数据。',
            '接近起点不在总路径长度和覆盖评分中，仍须独立确认可通行。',
            '总长度包含作业行及全部换行连接；原地旋转平移长度为零，不评价旋转时间或能源。'],
        'implementation_sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in ('optimizer.py','point_turn_planner.py','planning_options.py','coverage_geometry.py')}}
    emit('COMPLETE' if chosen else 'FAILED')
    if not chosen:return {'points':[],'metadata':{'strategy':'SMART'},'optimization':report}
    row,points,details=chosen;report['selected']=row
    metadata={'mode':'BOUSTROPHEDON','strategy':'SMART','turn_mode':'POINT_TURN','policy_version':2,
        'rotation_envelope':envelope,'headland_search':search,'path_details':details,'selected_candidate_id':row['id'],
        'effective_parameters':{'headland_width':headland,'swath_angle':row['angle_rad'],
            'operation_width':width,'evaluation_width':width,'robot_width':p['robot_width'],
            'robot_length':envelope['robot_length_m'],'rotation_safety_margin':envelope['safety_margin_m'],
            'path_continuity':'POINT_TURN','allow_reverse':False}}
    return {'points':points,'metadata':metadata,'optimization':report}
