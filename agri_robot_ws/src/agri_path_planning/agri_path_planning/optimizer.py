"""Complete layout portfolio with stopped turns and four fixed objectives."""
import hashlib
import math
from pathlib import Path
from time import monotonic
from shapely import wkb
from shapely.affinity import rotate
from . import contour
from .coverage_geometry import AreaGrid
from .planning_options import WEIGHTS
from .entry import EntryConnector
from .point_turn_planner import sample_path
from .cell_routing import decompose_cells, rows_from_cells
from .edge_candidates import boundary_bases, layout_choices
from .native_layouts import layouts as native_layouts
from .native_route import plan_native_route, reversed_route
from agri_base.path_geometry import rotation_envelope, validate_point_turn_path


def objective_terms(coverage_percent, overlap_percent, length, reference_length,
                    turn_count=0, turn_reference=1.):
    return {'coverage':1-coverage_percent/100,'overlap':overlap_percent/100,
            'length':length/(length+reference_length),'turns':turn_count/(turn_count+turn_reference)}


def score(coverage_percent, overlap_percent, length, reference_length, turn_count=0, turn_reference=1.):
    terms=objective_terms(coverage_percent,overlap_percent,length,reference_length,turn_count,turn_reference)
    return sum(WEIGHTS[k]*v for k,v in terms.items())


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
    # A conservative mitre offset encloses rounded obstacle-corner offsets.
    # It gives a point-turn chassis one corner to align at, rather than many
    # tiny stop/turns along a tessellated arc. Disk clearance is checked again.
    headland=envelope['headland_width_m'];safe=field.buffer(-headland,join_style=2)
    start=monotonic();rows=[];chosen=None;feasible_count=0;seen=set();deduplicated=0
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
    turn_reference=max(1.,2*math.sqrt(field.area)/width)
    edge_bases=boundary_bases(field,width,headland,check) if grid is not None else []
    if grid is not None:
        reference_length=field.area/width
        entry=EntryConnector(field,p,p['entry'],check) if p.get('entry') else None
        def consider(prepared, angle, phase, completed, total, basis, layout_id, guided=False):
            nonlocal chosen, feasible_count
            attributes={k: prepared[k] for k in ('layout_family','direction_mode','split_angle_rad',
                'swath_objective','cell_directions','dominant_angle_rad','angle_search_step_deg') if k in prepared}
            attributes.setdefault('layout_family','EDGE_GLOBAL')
            attributes.setdefault('direction_mode','GLOBAL')
            attributes['guided_local_search']=guided
            def rejected(exc, basis, layout_id=0, variant=0):
                check()
                rows.append({'id':len(rows)+1,'mode':'TSP','headland_width_m':headland,
                    'angle_rad':angle,'variant':variant,'order':'F2C_TSP','state':'REJECTED',
                    'headland_search_phase':phase,'executable':False,'reason':str(exc),
                    'basis':basis,'layout_id':layout_id,**attributes})
            for seed in ((0,1) if entry else (0,)):
                try:
                    emit(phase,completed,total,angle_rad=angle,order='F2C_TSP',
                         variant=seed*2,layout_id=layout_id,basis=basis, layout_family=attributes['layout_family'])
                    native_vertices,native_details=plan_native_route(prepared,check,entry.anchor if seed else None, guided_local_search=guided)
                except (ValueError, RuntimeError) as exc:
                    rejected(exc,basis,layout_id,seed*2);continue
                for reverse in ((0,1) if entry else (0,)):
                    variant=seed*2+reverse
                    emit(phase,completed,total,angle_rad=angle,order='F2C_TSP',variant=variant,layout_id=layout_id)
                    row={'id':len(rows)+1,'mode':'TSP','headland_width_m':headland,
                        'angle_rad':angle,'variant':variant,'order':'F2C_TSP','state':'REJECTED',
                        'headland_search_phase':phase,'executable':False,'layout_id':layout_id,
                        'basis':basis, **attributes}
                    try:
                        check()
                        vertices,partition=(reversed_route(native_vertices,native_details) if reverse else
                                            (native_vertices,native_details))
                        points,vertices,details=sample_path(vertices,spacing,check)
                        details.update(partition)
                        details.update(attributes)
                        validate_point_turn_path(points,field,envelope)
                        connection=entry.assess(points[0]) if entry else None
                        counts,_=grid.visits([vertices],width,check=check)
                        metrics=grid.metrics(counts)['field']
                        working_length=sum(math.dist(a,b) for a,b in zip(vertices,vertices[1:]))
                        entry_length=connection['distance_m'] if connection else 0.
                        length=working_length+entry_length
                        terms=objective_terms(metrics['coverage_percent'],metrics['overlap_percent'],
                                              length,reference_length,details['point_turn_count'],turn_reference)
                        value=sum(WEIGHTS[k]*v for k,v in terms.items())
                        row.update(state='EVALUATED',executable=True,score=value,length_m=length,
                            working_length_m=working_length,entry_length_m=entry_length,
                            coverage_percent=metrics['coverage_percent'],overlap_percent=metrics['overlap_percent'],
                            fine_coverage_percent=metrics['coverage_percent'],fine_overlap_percent=metrics['overlap_percent'],
                            objective_terms=terms,weighted_terms={k:WEIGHTS[k]*v for k,v in terms.items()},
                            start={'x':points[0][0],'y':points[0][1]},end={'x':points[-1][0],'y':points[-1][1]},
                            **{k:v for k,v in details.items() if k not in
                               ('cell_polygons','native_route_connections','swath_visits','omitted_depot_connections')})
                        feasible_count+=1
                        if chosen is None or rank(row)<rank(chosen[0]):chosen=(row,points,{**details,'entry':connection})
                    except (ValueError, RuntimeError) as exc:
                        check();row['reason']=str(exc)
                    rows.append(row)
        def evaluate(angle,phase,completed,total,bases=None):
            nonlocal chosen,feasible_count,deduplicated
            angle=angle%math.pi;key=round(angle,8)
            if key in seen:return
            seen.add(key)
            def rejected(exc,basis,layout_id=0,variant=0):
                check()
                rows.append({'id':len(rows)+1,'mode':'TSP','headland_width_m':headland,
                    'angle_rad':angle,'variant':variant,'order':'F2C_TSP','state':'REJECTED',
                    'headland_search_phase':phase,'executable':False,'reason':str(exc),
                    'basis':basis,'layout_id':layout_id})
            try:
                emit(phase,completed,total,angle_rad=angle,order='F2C_TSP',variant=0)
                cells=decompose_cells(safe,angle,check)
                original=rotate(field,-angle,origin=(0,0),use_radians=True)
            except ValueError as exc:
                rejected(exc,(bases or [{'kind':phase}])[0]);return
            geometries=set();layout_id=0
            for basis in bases or [{'kind':phase,'angle_rad':angle}]:
                try:layouts=layout_choices(original,cells['safe'],cells['polygons'],width,basis,check)
                except ValueError as exc:
                    rejected(exc,basis);continue
                for layout in layouts:
                    layout_id+=1
                    try:
                        prepared=rows_from_cells(cells,width,check,coverage_field=field,row_layout=layout)
                        geometry=tuple(sorted((r['cell_id'],tuple(round(x,6) for x in r['start']),
                                                tuple(round(x,6) for x in r['end']))
                                               for r in prepared['expected'].values()))
                        if geometry in geometries:
                            deduplicated+=1;continue
                        geometries.add(geometry)
                    except ValueError as exc:
                        rejected(exc,basis,layout_id);continue
                    consider(prepared,angle,phase,completed,total,layout['basis'],layout_id)
        axis=pca_angle(field)
        initial={}
        for basis in edge_bases:
            key=round(basis['angle_rad'],8)
            initial.setdefault(key,{'angle':basis['angle_rad'],'bases':[]})['bases'].append(basis)
        for angle in [axis,(axis+math.pi/2)%math.pi]+[math.radians(i) for i in range(0,180,15)]:
            key=round(angle%math.pi,8)
            initial.setdefault(key,{'angle':angle,'bases':[]})['bases'].append({'kind':'PCA' if angle in (axis,(axis+math.pi/2)%math.pi) else 'COARSE','angle_rad':angle})
        for i,item in enumerate(initial.values()):
            evaluate(item['angle'],'COARSE',i,len(initial),item['bases'])
            emit('COARSE',i+1,len(initial),angle_rad=item['angle'])
        if chosen is not None:
            for i,delta in enumerate((5.,2.,.5)):
                angle=chosen[0]['angle_rad']
                for j,sign in enumerate((-1,1)):
                    target=angle+sign*math.radians(delta)
                    evaluate(target,'REFINE',i*2+j,6)
                    emit('REFINE',i*2+j+1,6,angle_rad=target%math.pi)
        def native_progress(completed,total,**details):
            emit('NATIVE_LAYOUTS',completed,total,**details)
        native_seen={}
        for layout_id,(label,prepared,error) in enumerate(native_layouts(safe,width,check,native_progress),1):
            basis={'kind':label['layout_family'],'angle_rad':prepared['dominant_angle_rad'] if prepared else 0.}
            if error:
                rows.append({'id':len(rows)+1,'mode':'TSP','headland_width_m':headland,
                    'angle_rad':0.,'variant':0,'order':'F2C_TSP','state':'REJECTED',
                    'headland_search_phase':'NATIVE_LAYOUTS','executable':False,'reason':error,
                    'basis':basis,'layout_id':layout_id,**label})
                continue
            signature=(label['layout_family'],tuple(p.wkb for p in prepared['polygons']),
                       tuple((r['cell_id'],r['start'],r['end']) for r in prepared['expected'].values()))
            if signature in native_seen:
                deduplicated+=1
                rows.append({'id':len(rows)+1,'mode':'TSP','headland_width_m':headland,
                    'angle_rad':prepared['dominant_angle_rad'],'variant':0,'order':'F2C_TSP',
                    'state':'DEDUPLICATED','headland_search_phase':'NATIVE_LAYOUTS',
                    'executable':False,'reason':'与已评价的同类原生作业图完全相同，复用候选集合',
                    'equivalent_layout_id':native_seen[signature],'basis':basis,'layout_id':layout_id,**label})
                continue
            native_seen[signature]=layout_id
            for guided in (False,True):
                consider(prepared,prepared['dominant_angle_rad'],'NATIVE_LAYOUTS',layout_id,15,
                         basis,layout_id,guided=guided)
    emit('FINALIZING')
    search={'schema_version':2,**envelope,'selected_width_m':headland,
        'range_min_m':headland,'range_max_m':headland,'initial_widths_m':[headland],'refinement_widths_m':[],
        'coarse_evaluated_count':sum(r['headland_search_phase']=='COARSE' for r in rows),
        'refinement_evaluated_count':sum(r['headland_search_phase']=='REFINE' for r in rows)}
    reason=('地块无法容纳车辆原地转向安全包络' if safe.is_empty else
            '没有满足原地转向扫掠净空的完整原生TSP路径')
    report={'schema_version':3,'strategy':'SMART','policy':'POINT_TURN_NATIVE_TSP',
        'state':'SUCCEEDED' if chosen else 'FAILED','reason':'' if chosen else reason,
        'candidates':rows,'evaluated_count':len(rows),'possible_count':len(rows),
        'search_complete':True,'candidate_limit_enabled':False,'search_time_limit_enabled':False,
        'weights':dict(WEIGHTS),'headland_search':search,
        'angle_count':len(seen),'boundary_basis_count':len(edge_bases),'deduplicated_layout_count':deduplicated,
        'layout_search':{'families':['EDGE_GLOBAL','NATIVE_WHOLE_FIELD','NATIVE_PER_CELL'],
            'native_angle_step_deg':1.,'partition_angles_rad':[0.,math.pi/2],
            'native_objectives':['NSwath','NSwathModified','SwathLength'],
            'native_route_search':['AUTOMATIC','GUIDED_LOCAL_SEARCH'],
            'family_candidate_counts':{family:sum(r.get('layout_family','EDGE_GLOBAL')==family for r in rows)
                for family in ('EDGE_GLOBAL','NATIVE_WHOLE_FIELD','NATIVE_PER_CELL')}},
        'weight_policy':'COVERAGE_PRIORITY_FOUR_OBJECTIVES_V1',
        'normalization':{'coverage':'1-C/100','overlap':'O/100','length':'L/(L+L0)',
            'length_reference_m':field.area/width,'turns':'N/(N+N0)',
            'turn_reference_count':turn_reference,'turn_reference_definition':'max(1,2*sqrt(A)/W)',
            'turn_count_definition':'Final executable stationary-turn markers; excludes entry approach and initial alignment.'},'coverage_constraint_enabled':False,'coverage_objective_enabled':True,
        'curve_type':'POINT_TURN','turn_mode':'POINT_TURN','allow_reverse':False,
        'reverse_execution_supported':True,'search_resolution_m':grid.resolution if grid is not None else resolution,
        'final_resolution_m':grid.resolution if grid is not None else resolution,
        'selection_scope':'完整比较沿边统一方向、原生整田不分解、BCD/梯形分区独立方向布局；完整保留原生TSP连接和人工入口选择，按覆盖0.40、重复0.25、长度0.20、转弯0.15择优；不保证全局最优。',
        'configuration':{},'elapsed_seconds':monotonic()-start,
        'limitations':['车体按长宽及旋转中心的完整圆包络检查；尺寸须实测，默认值未实车标定。',
            '每次原生TSP调用采用库默认1秒启发式求解；完整执行预定角度不等于穷举全部访问顺序或证明全局最优。',
            '人工入口比较自由路线及原生入口引导路线的两个起端；入口引导仅保留作业行间连接，不执行原生闭环的入口接近或返程，不代表固定入口开放路线全局最优。',
            '每条有效边绑定平行方向及半幅宽起排位置，排线向两侧铺满；比较原布局与正增益补边集合，不穷举补边子集，不保证全部边角覆盖。',
            '所有行驶段均按机具开启估计；尚无实际机具开关数据。',
            ('入口连接计入总长度，覆盖率不计接近段；田外入口到安全区域仅计距离示意，不自动执行。' if p.get('entry') else
             '未设置入口，接近起点不在总路径长度和覆盖评分中，仍须独立确认可通行。'),
            '分区独立方向的angle_rad仅为最长作业分区的代表方向，各区方向见cell_directions；分区切线不额外内缩，不穷举分区角度与各区角度的所有组合。',
            '总长度包含作业行及全部换行连接；原地旋转平移长度为零，转弯目标按执行停车转向标记计数；不评价旋转角度、时间或能源。'],
        'implementation_sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in ('optimizer.py','point_turn_planner.py','cell_routing.py','edge_candidates.py','native_route.py','native_layouts.py','planning_options.py','coverage_geometry.py')}}
    emit('COMPLETE' if chosen else 'FAILED')
    if not chosen:return {'points':[],'metadata':{'strategy':'SMART'},'optimization':report}
    row,points,details=chosen;report['selected']=row
    metadata={'entry':details.get('entry'),'mode':'TSP','strategy':'SMART','turn_mode':'POINT_TURN','policy_version':7,
        'rotation_envelope':envelope,'headland_search':search,'path_details':details,'selected_candidate_id':row['id'],
        'effective_parameters':{'headland_width':headland,'swath_angle':row['angle_rad'],
            'layout_family':row.get('layout_family'),'direction_mode':row.get('direction_mode'),
            'cell_directions':row.get('cell_directions',[]),
            'operation_width':width,'evaluation_width':width,'robot_width':p['robot_width'],
            'robot_length':envelope['robot_length_m'],'rotation_safety_margin':envelope['safety_margin_m'],
            'path_continuity':'POINT_TURN','allow_reverse':False}}
    return {'points':points,'metadata':metadata,'optimization':report}
