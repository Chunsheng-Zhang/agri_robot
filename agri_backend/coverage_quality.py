"""Reproducible coverage-quality reports from immutable per-run geometry.

The journal references motivate the evaluation dimensions; the disk/visit model
and overlap denominator below are explicit project conventions, not an asserted
universal definition copied from those papers. No ROS imports or control calls.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
import os
import platform
import uuid
from pathlib import Path

import numpy as np
from shapely.geometry import LineString
from shapely.ops import unary_union

from .coverage_geometry import AreaGrid, MAX_CELLS, polygons
from . import coverage_geometry as geometry_module
from .missions import LocalFrame, field_from_data, atomic_json, now

METHOD = {
    'version': 1, 'coordinate_unit': 'm', 'area_unit': 'm2',
    'footprint': '二维圆盘足迹，半径=评价幅宽/2，中心沿 base_link 折线移动；不含未标定的机具杆臂或作业深度。',
    'pass_definition': '网格代表点被足迹包含的沿路径弧长连通区间算一次经过；相邻线段、密集采样和静止重复点不重复计数。离开后再次进入才增加次数。',
    'coverage_formula': 'C = 100 * area(N >= 1) / A',
    'miss_formula': 'M = 100 * area(N == 0) / A',
    'overlap_formula': 'O = 100 * area(N >= 2) / A',
    'extra_pass_formula': 'R = 100 * integral(max(N-1,0)) / A; may exceed 100%',
    'denominator': 'A 为所选评价区域面积；默认整块田减去禁入区域，保留地头，不以已覆盖面积作主分母。',
    'integration': '规则网格代表点计次，边界格权重为与 ROI 的精确相交面积；边界代表点必要时移至相交区域内。',
    'headland_definition': '内部区=田块向内 buffer(-headland_width)，地头=田块减内部区；基于配置的几何分区，不是规划器原生作业标签。',
    'gap_policy': '正式跟踪 CSV 分段；非递增时间、>0.5s间隔、疑似跳变或已记录暂停/对齐边界不连线。空档中的实际运动未知。',
}


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def inputs(directory):
    run = read_json(directory/'run.json')
    if run.get('coverage_quality_version') != 1 or run.get('mode') != 'coverage':
        raise ValueError('本轮未启用覆盖质量评价；历史任务不自动回填')
    mission = read_json(directory/'config/mission.json')
    projection = run['projection']
    if mission['projection'] != projection:
        raise ValueError('任务与执行的坐标原点不一致')
    frame = LocalFrame(projection['latitude'], projection['longitude'])
    if frame.identity != projection:
        raise ValueError('投影定义不一致')
    field = field_from_data(mission['boundary'], frame)
    paths = sorted((directory/'paths').glob('coverage_*.json'))
    if len(paths) != 1:
        raise ValueError('质量评价要求恰好一个本轮参考路径文件')
    saved = read_json(paths[0])
    if saved.get('projection') != projection or saved.get('frame_id') != 'map':
        raise ValueError('参考路径坐标原点或 frame 不一致')
    points = [(float(p['x']), float(p['y'])) for p in saved['points']]
    if len(points)<2 or not all(math.isfinite(x+y) for x,y in points):
        raise ValueError('无有效规划路径')
    p = mission['parameters']
    width = float(p.get('evaluation_width', 0) or p['operation_width'])
    resolution = float(p.get('quality_resolution', .05))
    headland = float(p['headland_width'])
    planning=saved.get('planning') or {}
    point_turn=planning.get('turn_mode')=='POINT_TURN'
    if point_turn:width=float(p['operation_width'])
    if planning.get('strategy')=='SMART':
        headland=float(planning['effective_parameters']['headland_width'])
        if point_turn:
            from agri_base.path_geometry import rotation_envelope, validate_point_turn_path
            expected=rotation_envelope(p.get('robot_length',.45),p.get('robot_width',.2),
                p.get('rotation_safety_margin',.05),p.get('rotation_center_x',0.),p.get('rotation_center_y',0.))
            if planning.get('rotation_envelope')!=expected or abs(headland-expected['headland_width_m'])>1e-6:
                raise ValueError('作业评价的原地转向包络与任务不一致')
            validate_point_turn_path([(v['x'],v['y'],v['yaw']) for v in saved['points']],field,expected)
        elif p.get('optimization_headland_mode','AUTO')=='AUTO':
            adaptive=planning.get('headland_search') or {}
            if adaptive.get('schema_version')==1:
                lo,hi=float(adaptive['range_min_m']),float(adaptive['range_max_m'])
                if adaptive.get('method')!='GEOMETRY_NATIVE_ADAPTIVE' or not all(math.isfinite(v) for v in (lo,hi)) or not 0<=lo<=hi<=50:
                    raise ValueError('自动地头搜索范围元数据无效')
            else:
                # Read old immutable reports under the convention they used.
                lo,hi=float(p.get('optimization_headland_min',.5)),float(p.get('optimization_headland_max',4.))
            if not lo-1e-6<=headland<=hi+1e-6:
                raise ValueError('选中的自动地头宽度超出本轮配置范围')
        elif not math.isclose(headland,float(p['headland_width']),abs_tol=1e-6):
            raise ValueError('选中宽度与人工地头配置不一致')
    if not .1 <= width <= 20 or not .01 <= resolution <= .5 or not 0 <= headland <= 50:
        raise ValueError('评价参数越界')
    return run, mission, field, points, paths[0], width, resolution, headland


def tracking_break(previous, current, barriers):
    """One continuity policy shared by live map and final task report."""
    t,x,y = current
    dt = t-previous[0]
    boundary = bisect.bisect_right(barriers,previous[0]) < bisect.bisect_right(barriers,t)
    return dt<=0 or dt>.5 or math.dist((x,y),previous[1:]) > max(0,dt)+.3 or boundary


def tracking_chunks(directory, reference):
    files = sorted((directory/'tracks').glob('track_*.csv'))
    diagnostics = {'gap_limit_s': .5, 'invalid_rows': 0, 'split_segments': 0,
                   'jump_rule': '相邻位移 > 1m/s * dt + 0.3m 切断，不把跳点连成作业带'}
    if not files:
        return [], diagnostics, []
    if len(files) != 1:
        raise ValueError('多个正式跟踪文件，不能混合尝试')
    sidecar = read_json(files[0].with_suffix('.json'))
    run = read_json(directory/'run.json')
    if sidecar.get('projection') != run['projection'] or sidecar.get('frame_id') != 'map':
        raise ValueError('实际轨迹坐标基准不一致')
    if [(p['x'],p['y']) for p in sidecar['reference_path']['points']] != reference:
        raise ValueError('实际跟踪参考与规划路径不一致')
    barriers = []
    status_path = directory/'telemetry/status.jsonl'
    if status_path.exists():
        for line in status_path.read_text().splitlines():
            try:
                item = json.loads(line)
                data = item['data']
                if (item['topic']=='/tracking/status' and data.get('run_directory')==str(directory/'tracks')
                        and (data.get('state')!='RUNNING' or data.get('phase') not in (None,'TRACKING'))):
                    barriers.append(float(item['received_ros_ns'])/1e9)
            except (ValueError, KeyError, TypeError):
                diagnostics['invalid_rows'] += 1
    barriers.sort()
    chunks, current, previous = [], [], None
    with files[0].open(newline='',encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            try:
                t,x,y = (float(row[k]) for k in ('stamp','x','y'))
                if not all(math.isfinite(v) for v in (t,x,y)):
                    raise ValueError()
            except (KeyError,ValueError,TypeError):
                diagnostics['invalid_rows'] += 1
                if len(current)>1: chunks.append(current)
                current, previous = [], None
                continue
            if previous:
                if tracking_break(previous,(t,x,y),barriers):
                    diagnostics['split_segments'] += 1
                    if len(current)>1: chunks.append(current)
                    current = []
            current.append((x,y))
            previous = (t,x,y)
    if len(current)>1: chunks.append(current)
    return chunks, diagnostics, [files[0],files[0].with_suffix('.json')]+([status_path] if status_path.exists() else [])


def evaluate(field, chunks, width, resolution, headland, grid=None):
    grid = grid or AreaGrid(field,headland,resolution)
    counts, path = grid.visits(chunks,width)
    if not path['segments']:
        return {'available': False, 'reason': '没有可连接的有效正式跟踪线段；覆盖和重叠未知'}, grid, counts
    coarse = AreaGrid(field,headland,grid.resolution*2)
    coarse_counts, _ = coarse.visits(chunks,width)
    metrics, coarse_metrics = grid.metrics(counts), coarse.metrics(coarse_counts)
    # A second geometry implementation checks coverage integration, without
    # pretending that it measures implement action or overlap multiplicity.
    swept = unary_union([LineString(points).buffer(width/2,resolution=16)
                         for points in chunks if len(points)>1])
    vector_covered = swept.intersection(field).area
    result = {'available': True, 'width_m': width, 'requested_resolution_m': resolution,
              'resolution_m': grid.resolution, 'grid_cells': grid.nx*grid.ny,
              'grid_cell_budget': MAX_CELLS, 'metrics': metrics, 'path': path,
              'sensitivity': {'coarse_resolution_m': coarse.resolution,
                'coverage_difference_pp': abs(metrics['field']['coverage_percent']-coarse_metrics['field']['coverage_percent']),
                'overlap_difference_pp': abs(metrics['field']['overlap_percent']-coarse_metrics['field']['overlap_percent']),
                'meaning': '两级网格的结果差（百分点），是数值敏感性检查，不是置信区间或误差上界。'},
              'vector_check': {'covered_m2':vector_covered,'coverage_percent':100*vector_covered/field.area,
                  'outside_field_m2':swept.difference(field).area,
                  'grid_difference_pp':abs(metrics['field']['coverage_percent']-100*vector_covered/field.area),
                  'circle_quadrant_segments':16,
                  'meaning':'同一圆盘足迹的矢量缓冲并集校核；圆弧离散，每象限16段，不是独立真值。'}}
    return result, grid, counts


def export_grid(directory, name, grid, counts):
    path = directory/'analysis'/f'coverage_quality_{name}_cells.csv'
    temporary = path.with_name('.'+uuid.uuid4().hex+'.csv')
    try:
        with temporary.open('w',newline='',encoding='utf-8') as stream:
            writer=csv.writer(stream)
            writer.writerow(['row','column','sample_x_m','sample_y_m','field_weight_m2','interior_weight_m2','observed_passes'])
            for i in np.flatnonzero(grid.weights>0):
                row,col=divmod(int(i),grid.nx)
                writer.writerow([row,col,float(grid.x[i]),float(grid.y[i]),float(grid.weights[i]),float(grid.interior_weights[i]),int(counts[i])])
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def figure(directory, name, grid, counts, chunks):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.path import Path as PlotPath
    from matplotlib.patches import Patch, PathPatch
    from shapely.geometry.polygon import orient
    values = np.minimum(counts,2).astype(float)
    values[grid.weights<=0]=np.nan
    fig,ax=plt.subplots(figsize=(8,6))
    cmap=ListedColormap(['#f6c8c5','#77ba96','#efb65b'])
    image=ax.imshow(values.reshape(grid.ny,grid.nx),origin='lower',interpolation='nearest',
        extent=(grid.x0,grid.x0+grid.nx*grid.resolution,grid.y0,grid.y0+grid.ny*grid.resolution),
        cmap=cmap,norm=BoundaryNorm([-.5,.5,1.5,2.5],3),rasterized=True)
    # A tiny clipped boundary cell must not paint its whole square outside the
    # ROI. Preserve holes in the figure as well as in the numeric denominator.
    vertices,codes=[],[]
    for polygon in polygons(grid.field):
        polygon=orient(polygon,sign=1.0)
        for ring in [polygon.exterior,*polygon.interiors]:
            coordinates=list(ring.coords)
            vertices.extend(coordinates)
            codes.extend([PlotPath.MOVETO]+[PlotPath.LINETO]*(len(coordinates)-2)+[PlotPath.CLOSEPOLY])
    image.set_clip_path(PathPatch(PlotPath(vertices,codes),transform=ax.transData))
    for polygon in polygons(grid.field):
        for ring in [polygon.exterior,*polygon.interiors]:
            x,y=ring.xy; ax.plot(x,y,color='#243c31',linewidth=1)
    for polygon in polygons(grid.interior):
        x,y=polygon.exterior.xy; ax.plot(x,y,'--',color='#56627c',linewidth=.8)
    # Each observed chunk is independent; never draw across missing intervals.
    for points in chunks:
        ax.plot([p[0] for p in points],[p[1] for p in points],color='#336a9b',linewidth=.45,alpha=.7)
    ax.legend(handles=[Patch(color=color,label=label) for color,label in zip(cmap.colors,
        ('Uncovered','One observed visit','Two or more visits'))],loc='best',fontsize=8)
    ax.set(xlabel='Local map X (m)',ylabel='Local map Y (m)',
        title=('Planned geometric coverage' if name=='plan' else 'Observed tracking coverage estimate'))
    ax.set_aspect('equal');fig.tight_layout()
    for suffix in ('png','svg'):
        path=directory/'analysis'/f'coverage_quality_{name}.{suffix}'
        temporary=path.with_name('.'+uuid.uuid4().hex+'.'+suffix)
        try:
            fig.savefig(temporary,dpi=300)
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
    plt.close(fig)


def fingerprint(paths, directory):
    return {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths if path.exists()}


def planned_quality(directory):
    directory=Path(directory).resolve()
    (directory/'analysis').mkdir(exist_ok=True)
    run,mission,field,points,path,width,resolution,headland=inputs(directory)
    point_turn=(read_json(path).get('planning') or {}).get('turn_mode')=='POINT_TURN'
    result,grid,counts=evaluate(field,[points],width,resolution,headland)
    result.update(schema_version=1,run_id=run['id'],mission_revision=run['mission_revision'],
                  generated_at=now(),projection=run['projection'],method=METHOD,
                  width_source=('operation_width_effective' if point_turn else
                      'evaluation_width' if mission['parameters'].get('evaluation_width') else 'operation_width_nominal'),
                  sources_sha256=fingerprint([directory/'config/mission.json',path],directory))
    result['effective_headland_width_m']=headland
    export_grid(directory,'plan',grid,counts)
    figure(directory,'plan',grid,counts,[points])
    atomic_json(directory/'analysis/coverage_quality_plan.json',result)
    return result


def completed_quality(directory, metrics):
    directory=Path(directory).resolve()
    run,mission,field,points,path,width,resolution,headland=inputs(directory)
    plan=read_json(directory/'analysis/coverage_quality_plan.json') if (directory/'analysis/coverage_quality_plan.json').exists() else None
    current=fingerprint([directory/'config/mission.json',path],directory)
    if not plan or plan.get('sources_sha256')!=current:
        plan=planned_quality(directory)
    chunks,diagnostics,files=tracking_chunks(directory,points)
    actual,grid,counts=evaluate(field,chunks,width,resolution,headland)
    actual['recording']=diagnostics
    if actual['available']:
        export_grid(directory,'actual',grid,counts)
        figure(directory,'actual',grid,counts,chunks)
    import shapely
    result={'schema_version':1,'run_id':run['id'],'generated_at':now(),
            'task_state':metrics['task_state'],'projection':run['projection'],
            'planned':plan,'actual':actual,'method':METHOD,
            'tracking_summary':metrics.get('tracking_assessment',{}),
            'sources_sha256':fingerprint([directory/'config/mission.json',path,*files,
                                         directory/'telemetry/recording.json'],directory),
            'implementation_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                (Path(__file__),Path(geometry_module.__file__))},
            'environment':{'python':platform.python_version(),'numpy':np.__version__,'shapely':shapely.__version__},
            'limitations':[
                '规划是预测；实际是同一 MGX23 位姿的几何估计，无独立地面真值、作业开关或剂量反馈。',
                '评价幅宽未自动视为已标定；0 表示沿用名义作业幅宽。以 base_link 为中心，未补偿机具偏置。',
                '假设完整规划路径或正式跟踪期间机具一直开启，包括连接转弯；接近/对齐不算作业。',
                '圆盘等效足迹会包含端点圆帽，与真实横向机具扫掠形状不同；二维面积不包含坡面修正。',
                '缺失区间不连线；覆盖是已观测路段的估计，重叠次数可能漏计，不能据此声明实际全程无漏作。',
                '一次连续包含只算一次，未测持续停留的重复施药量；各论文重叠分母/计次可不同，跨论文比较前须统一口径。',
                '两级网格差是敏感性结果，不是统计置信区间；论文须做幅宽/外参标定及多次重复实车实验。',
                '控制器完成与整田作业完成不同；失败、取消或故障结果只代表本轮已有轨迹。']}
    if run.get('planning_strategy') in ('SMART','REUSED'):
        from .optimization_results import summary
        result['optimization']=summary(directory)
    write_report(directory,mission,result,metrics)
    atomic_json(directory/'analysis/coverage_quality.json',result)
    return result


def write_report(directory,mission,result,metrics):
    def f(value):return '未测' if value is None else f'{value:.4f}'
    lines=['# 全覆盖作业几何质量评价报告','',f'执行：`{result["run_id"]}`；结束状态：`{result["task_state"]}`；生成：{result["generated_at"]}。','',
           '本报告提供可复核的规划预测与实际轨迹估计，不能代替机具作业效果或独立真值验证。',
           '', '## 参数和坐标基准','',f'- 固定投影/原点：`{json.dumps(result["projection"],ensure_ascii=False)}`；全部路径保持同一 map 米制坐标。',
           f'- 评价幅宽：{f(result["planned"]["width_m"])} m；来源：`{result["planned"]["width_source"]}`。',
           f'- 规划作业幅宽：{mission["parameters"]["operation_width"]} m；任务人工地头宽度：{mission["parameters"]["headland_width"]} m；实际选定宽度：{result["planned"].get("effective_headland_width_m",mission["parameters"]["headland_width"])} m。','',
           '## 面积指标','',
           '| 数据 | 区域 | 区域 m² | 覆盖 m² | 漏覆盖 m² | 重叠 m² | 覆盖率 % | 漏覆盖率 % | 重叠率 % | 累计重复率 % |',
           '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    with (directory/'analysis/coverage_quality.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['source','region','metric','value','unit'])
        for name,label in (('planned','规划预测'),('actual','实际轨迹估计')):
            item=result[name]
            if not item['available']:
                lines+=['',label+'：'+item['reason']+'。'];continue
            for key,region_name in (('field','整块田（扣除孔洞）'),('interior','内部作业区'),('headland','地头区')):
                region=item['metrics'][key]
                if not region['available']:continue
                keys=('area_m2','covered_m2','missed_m2','overlap_m2','coverage_percent','miss_percent','overlap_percent','extra_pass_percent')
                lines.append('| '+' | '.join([label,region_name]+[f(region[k]) for k in keys])+' |')
                for metric,value in region.items():
                    if metric!='available':writer.writerow([name,key,metric,value,'m2' if metric.endswith('_m2') else '%'])
    lines+=['','重叠率的主分母是本行区域面积；累计重复率将第三次及以后每次额外经过都计入，可超过100%。另在 JSON 保存“重叠面积/已覆盖面积”供换算，不能混用。','',
            '## 数值分辨率与数据连续性','']
    for name,label in (('planned','规划'),('actual','实际')):
        item=result[name]
        if not item['available']:continue
        sensitivity=item['sensitivity']
        lines += [f'- {label}：请求网格 {f(item["requested_resolution_m"])} m，实际 {f(item["resolution_m"])} m，{item["grid_cells"]} 格；预算上限 {item["grid_cell_budget"]} 格。',
                  f'- {label}粗网格 {f(sensitivity["coarse_resolution_m"])} m；两级覆盖率差 {f(sensitivity["coverage_difference_pp"])} 个百分点，重叠率差 {f(sensitivity["overlap_difference_pp"])} 个百分点。',
                  f'- {label}已连接长度 {f(item["path"]["observed_length_m"])} m，连续片段 {item["path"]["chunks"]} 段。']
        check=item['vector_check']
        lines += [f'- {label}矢量缓冲并集覆盖率校核：{f(check["coverage_percent"])}%；与网格差 {f(check["grid_difference_pp"])} 个百分点；足迹越出田块及孔洞面积 {f(check["outside_field_m2"])} m²。圆弧按每象限16段近似。']
    lines+=['',f'实际轨迹切段/无效行：`{json.dumps(result["actual"].get("recording",{}),ensure_ascii=False)}`。',
            '网格估计不是精确连续几何积分；边界格面积权重精确，但格内覆盖状态取代表点。细窄区域和接近幅宽的间距应进一步减小网格做收敛验证。','',
            '## 跟踪精度与记录质量','',
            '[本轮完整跟踪、速度、状态与通信报告](report.md) 保留分区明细；以下汇总正式跟踪关键指标。',
            f'缺失话题：`{json.dumps(metrics.get("missing_topics",[]),ensure_ascii=False)}`；记录器丢弃：`{json.dumps(metrics.get("capture",{}).get("queue_drops_by_topic",{}),ensure_ascii=False)}`。','',
            '## 本项目公式与建模约定','']
    # Keep the per-run report numerical. Literature justification is maintained
    # once in docs rather than replicated in every experiment directory.
    assessment=metrics.get('tracking_assessment',{})
    subset=assessment.get('subsets',{}).get('all_tracking',{})
    summary=['## 本轮跟踪与运行数据','', '| 正式跟踪指标 | 本轮值 | 单位 |','| --- | ---: | --- |']
    for field,label,scale,unit in (('cross_track_m','横向误差',100,'cm'),('heading_error_deg','航向误差',1,'deg')):
        values=subset.get(field,{})
        for key,title in (('mae','MAE'),('rmse','RMSE'),('abs_p95','P95'),('abs_p99','P99'),('abs_max','最大绝对值')):
            value=values.get(key)
            summary.append(f'| {label} {title} | {f(value*scale if value is not None else None)} | {unit} |')
    speed=assessment.get('mgx23_horizontal_speed_m_s',{})
    for key,label in (('mean','MGX23 水平速度均值'),('abs_median','MGX23 水平速度中位数')):
        summary.append(f'| {label} | {f(speed.get(key))} | m/s |')
    states=assessment.get('task_status',{})
    if states.get('available'):
        durations=states.get('approximate_state_durations_s',{})
        summary += [f'| 状态观测运行时间（含接近/对齐） | {f(durations.get("RUNNING",0))} | s |',
                    f'| 状态观测暂停时间 | {f(durations.get("PAUSED",0))} | s |',
                    f'| 暂停次数 | {states.get("pause_count")} | 次 |']
    else:summary+=['| 状态运行/暂停时长与次数 | 未测 | — |']
    summary+=['','接近与对齐不混入正式跟踪误差；状态时长由接收时刻近似，不是独立作业计时。能耗、机具开关/剂量、独立真值下的实际农艺效果未测。','']
    # Insert ahead of the method section so all numerical results stay together.
    method_position=lines.index('## 本项目公式与建模约定')
    lines[method_position:method_position]=summary
    optimization=result.get('optimization') or {}
    if optimization:
        selected=optimization.get('selected') or {}
        optimization_lines=['','## 本轮一键优化选择','',
                f'- 状态：{optimization.get("state")}；候选比较 {optimization.get("evaluated_count")} / {optimization.get("possible_count")}。',
                f'- 形式：{selected.get("mode","未选定")}；地头：{selected.get("headland_width_m","未知")} m；路径：{selected.get("length_m","未知")} m。',
                f'- 相对能耗代理：{selected.get("energy_proxy","未知")}；未标定，不是 Wh 或实测能源。',
                f'- 归一化权重：`{json.dumps(optimization.get("weights",{}),ensure_ascii=False)}`。',
                '- 仅在本次有限完整方案中选择；连接采用指定的Fields2Cover原生曲线，倒车须由本轮任务明确允许，换向停车对齐。完整配置、评分、拒绝原因及代码指纹见 `../paths/optimization.json`。','']
        method_position=lines.index('## 本项目公式与建模约定')
        headland=optimization.get('headland_search') or {}
        if optimization.get('turn_mode')=='POINT_TURN':
            optimization_lines=['','## 本轮原地转向智能规划','',
                f'- 作业角度：{selected.get("angle_rad")} rad；行顺序：{selected.get("order")}。',
                f'- 最小安全地头：{selected.get("headland_width_m")} m；原地转向点：{selected.get("point_turn_count")}。',
                f'- 车辆包络：长{headland.get("robot_length_m")} m，宽{headland.get("robot_width_m")} m，安全余量{headland.get("safety_margin_m")} m。',
                '- 评分：0.5×(1−覆盖率/100)+0.3×重复覆盖率/100+0.2×L/(L+A/W)，越小越好；总长度包含全部换行连接。',
                '- 机具有效幅宽固定，整田分母包含地头并扣除禁入区；每个几何拐角停车后原地对齐。',
                '- 未评价能耗或作业时间；在有限候选和预算内择优，不保证全局最优。',
                '- 完整参数、候选、拒绝原因和代码指纹见 `../paths/optimization.json`。','']
        elif headland.get('method')=='MANUAL':
            optimization_lines+=['- 地头选择：严格采用任务人工地头宽度，未搜索其他宽度。','']
        elif headland:
            optimization_lines+=[f'- 自动地头估计范围：{headland.get("range_min_m")}～{headland.get("range_max_m")} m；粗搜索 {headland.get("coarse_evaluated_count",0)} 套、细化 {headland.get("refinement_evaluated_count",0)} 套。',
                '- 搜索范围由地块及原生转弯包络估计；有限预算及当前评分下择优，不保证全局最优。','']
        if optimization.get('coverage_constraint_enabled') is False and optimization.get('turn_mode')!='POINT_TURN':
            optimization_lines+=['- 本轮不设最低覆盖率门槛；覆盖率仅作评价统计，按重复覆盖、路径长度与能源代理评分选择方案。','']
        lines[method_position:method_position]=optimization_lines
    lines += ['- '+value for key,value in METHOD.items() if isinstance(value,str)]
    lines += ['','## 限制与论文使用要求','']+['- '+value for value in result['limitations']]
    lines += ['','## 图表与复现','',
              '![规划覆盖、漏覆盖与重复覆盖分布](coverage_quality_plan.png)','']
    if result['actual']['available']:lines+=['![实际轨迹几何作业质量估计](coverage_quality_actual.png)','']
    lines+=['原始数值：`coverage_quality.json` / `coverage_quality.csv`；每格坐标、面积权重和经过次数：`coverage_quality_plan_cells.csv` / `coverage_quality_actual_cells.csv`（实际有数据时）。',
            '图像同时提供300 dpi PNG与SVG。输入和评价代码SHA256、Python/NumPy/Shapely版本写入JSON；原始路径/轨迹及配置随本轮归档保存。','',
            '指标选取依据统一保存在项目说明 `docs/全覆盖作业质量评价.md`；每轮仅保存本轮数据及方法版本。','']
    (directory/'analysis/coverage_quality_report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=['plan'])
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    if hasattr(os,'nice'):os.nice(10)
    planned_quality(args.directory)


if __name__=='__main__':
    main()
