"""Post-task GB/T 46270--2025 formula (1)/(2) engineering statistics.

Exactly 50 spatially equidistant samples per observed continuous section.
This does not certify the standard's speed, site, reference instrument or trial
matrix. No ROS imports, control calls, filtering or trajectory registration.
"""
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point, Polygon

from .coverage_path_order import work_row_order
from .coverage_quality import tracking_break
from .missions import LocalFrame, atomic_json, field_from_data

N = 50
EPS = 1e-6
MAX_SECTIONS = 6000
TITLES = {'straight': '直线作业精度', 'handover': '交接行精度', 'curve': '曲线作业精度'}
FORMULAS = {
    'straight': 'P1=sqrt(sum((x_i-mean(x))^2)/(N-1)); x_i为有符号横向垂距，cm',
    'handover': 'P2=sqrt(sum((abs(h_i-H)-mean(abs(h-H)))^2)/(N-1)); h_i、H均为cm',
    'curve': 'P1=sqrt(sum((x_i-mean(x))^2)/(N-1)); x_i为到对应参考曲线的最小距离，cm',
}


def formula_1_cm(errors_m):
    if len(errors_m) != N or not all(math.isfinite(x) for x in errors_m):
        raise ValueError('公式(1)要求50个有效等距样本')
    mean = math.fsum(errors_m)/N
    return 100*math.sqrt(math.fsum((x-mean)**2 for x in errors_m)/(N-1))


def formula_2_cm(spacings_m, preset_m):
    if not math.isfinite(preset_m) or preset_m <= 0:
        raise ValueError('预设交接行间距无效')
    return formula_1_cm([abs(h-preset_m) for h in spacings_m])


def empty_result(reason):
    return {'schema_version': 1, 'standard': 'GB/T 46270—2025', 'samples_per_section': N,
            'formulas': FORMULAS, 'standard_test_conformance': 'NOT_VERIFIED',
            'measurement': '同一MGX23定位的工程统计，无独立真值；不自动判定国标合格',
            'aggregation': '每个连续观测段单独计算，汇总取有效段最大值；不替代标准完整试验组合',
            **{key: {'available': False, 'state': 'UNAVAILABLE', 'value_cm': None,
                     'reason': reason, 'sections': []} for key in TITLES}}


def barriers_from_status(directory, run):
    path = directory/'telemetry/status.jsonl'
    values = []
    if not path.exists():
        return values
    topic = '/following/status' if run['mode'] == 'following' else '/tracking/status'
    for line in path.read_text().splitlines():
        try:
            item = json.loads(line); data = item['data']
            stamp = float(item['received_ros_ns'])/1e9
            if (item['topic'] == topic and data.get('run_directory') == str(directory/'tracks')
                    and (data.get('state') != 'RUNNING' or data.get('phase') not in (None, 'TRACKING'))
                    and math.isfinite(stamp)):
                values.append(stamp)
        except (ValueError, TypeError, KeyError):
            continue
    return sorted(values)


def reference_units(document, mode):
    """Prefer verified native work rows; never label point-turn connectors curves."""
    points = [(p['x'], p['y']) for p in document['points']]
    planning = document.get('planning') or {}
    native = work_row_order(document)
    if native['state'] == 'READY':
        units, arc = [], 0.
        connections = planning['path_details']['native_route_connections']
        for row, connection in zip(native['rows'], connections):
            arc += sum(math.dist(a, b) for a, b in zip(connection, connection[1:]))
            a, b = row['start'], row['end']
            a, b = (a['x'], a['y']), (b['x'], b['y'])
            length = math.dist(a, b)
            units.append({'id': f'row_{row["visit"]}', 'visit': row['visit'], 'kind': 'straight',
                          'start': a, 'end': b, 's0': arc, 's1': arc+length, 'length_m': length,
                          'source': 'VERIFIED_NATIVE_WORK_ROW'})
            arc += length
        return units, native, '原生Route中核对过的作业行；排除换行与绕障连接'

    # A single straight reference is unambiguous even without row metadata.
    distinct = [points[0]] if points else []
    for p in points[1:]:
        if math.dist(distinct[-1], p) > 1e-9:
            distinct.append(p)
    if len(distinct) < 2:
        return [], native, '参考路径没有有效长度'
    a, b = distinct[0], distinct[-1]; chord = math.dist(a, b)
    arc = sum(math.dist(p, q) for p, q in zip(distinct, distinct[1:]))
    if chord > EPS and abs(arc-chord) <= 1e-5 and all(
            abs((p[0]-a[0])*(b[1]-a[1])-(p[1]-a[1])*(b[0]-a[0]))/chord <= EPS for p in distinct):
        return [{'id':'reference_line', 'visit':None, 'kind':'straight', 'start':a, 'end':b,
                 's0':0., 's1':arc, 'length_m':chord, 'source':'SINGLE_REFERENCE_LINE'}], native, '单一直线参考'

    # No work-label inference for ordinary coverage: headland curves are not
    # evidence of a dedicated curved work pass. Following paths and continuous
    # contour passes are explicitly reported as engineering curve references.
    if planning.get('turn_mode') != 'POINT_TURN' and (mode == 'following' or planning.get('mode') == 'CONTOUR'):
        headings = [math.atan2(q[1]-p[1], q[0]-p[0]) for p, q in zip(distinct, distinct[1:])]
        bends = [abs((b-a+math.pi)%(2*math.pi)-math.pi) for a, b in zip(headings, headings[1:])]
        if sum(1e-6 < angle < math.pi/4 for angle in bends) >= 2:
            return [{'id':'reference_curve', 'visit':None, 'kind':'curve', 'points':distinct,
                     's0':0., 's1':arc, 'length_m':arc, 'source':'CONTINUOUS_REFERENCE_CURVE'}], native, '连续曲线参考的已观测段，未核验标准专用曲线工况'
    return [], native, '缺少可核对的作业行/曲线作业段元数据，未将连接或折线拐角当作作业曲线'


def observed_chunks(units, rows, barriers):
    starts = [u['s0'] for u in units]
    chunks = {u['id']: [] for u in units}
    previous = None; current = []; current_id = None
    invalid = ignored = 0
    for row in rows:
        try:
            stamp, x, y, progress = (float(row[k]) for k in ('stamp','x','y','progress'))
            if not all(math.isfinite(v) for v in (stamp,x,y,progress)):
                raise ValueError()
        except (KeyError, TypeError, ValueError):
            if current: chunks[current_id].append(current)
            current = []; current_id = None; previous = None; invalid += 1
            continue
        index = bisect.bisect_right(starts, progress+1e-8)-1
        unit = units[index] if index >= 0 and progress <= units[index]['s1']+EPS else None
        sample = {'stamp':stamp,'x':x,'y':y,'progress':progress}
        cut = (previous is None or tracking_break((previous['stamp'],previous['x'],previous['y']), (stamp,x,y), barriers)
               or progress < previous['progress']-EPS)
        identity = unit['id'] if unit else None
        if cut or identity != current_id:
            if current: chunks[current_id].append(current)
            current = []
        current_id = identity
        if unit:
            current.append(sample)
        else:
            ignored += 1
        previous = sample
    if current: chunks[current_id].append(current)
    return chunks, {'invalid_samples':invalid,'non_work_or_unmatched_samples':ignored,
                    'gap_policy':'不跨暂停/对齐、非递增时间、>0.5s空档、定位跳变或进度回退插值'}


def moving_points(chunk):
    points = []
    for sample in chunk:
        if not points or math.hypot(sample['x']-points[-1]['x'], sample['y']-points[-1]['y']) > 1e-9:
            points.append(sample)
    return points


def crossing_samples(chunk, axis, stations):
    """First chronological forward crossing; never choose the smallest error."""
    points = moving_points(chunk)
    if len(points) < N:
        raise ValueError('连续移动原始样本不足50个，不将少量点上采样冒充50次观测')
    direction = 1 if stations[-1] > stations[0] else -1
    targets = [direction*s for s in stations]
    coordinates = [direction*(p['x']*axis[0]+p['y']*axis[1]) for p in points]
    participating = set()
    for i, (a,b) in enumerate(zip(coordinates,coordinates[1:])):
        if b>a+1e-10 and b>=targets[0]-EPS and a<=targets[-1]+EPS:
            participating.update((i,i+1))
    if len(participating)<N:
        raise ValueError('本次等距评价跨度内的移动原始样本不足50个')
    result = []
    for a, b in zip(points, points[1:]):
        sa = direction*(a['x']*axis[0]+a['y']*axis[1])
        sb = direction*(b['x']*axis[0]+b['y']*axis[1])
        if sb <= sa+1e-10:
            continue
        while len(result) < N and sa-EPS <= targets[len(result)] <= sb+EPS:
            target = targets[len(result)]; t = max(0., min(1., (target-sa)/(sb-sa)))
            sample = {key:a[key]+t*(b[key]-a[key]) for key in ('stamp','x','y')}
            sample['station_m'] = stations[len(result)]
            result.append(sample)
        if len(result) == N:
            return result,len(participating)
    raise ValueError('等距站位缺少连续轨迹交点，未跨空档或外推补点')


def axis_for(unit):
    a, b = unit['start'], unit['end']; length = math.dist(a,b)
    return ((b[0]-a[0])/length, (b[1]-a[1])/length)


def bounds_on_axis(chunk, axis):
    values = [p['x']*axis[0]+p['y']*axis[1] for p in chunk]
    return min(values), max(values)


def summarize(sections, reason):
    valid = [s for s in sections if s['state'] == 'AVAILABLE']
    return {'available':bool(valid), 'state':'AVAILABLE' if valid else 'UNAVAILABLE',
            'value_cm':max((s['precision_cm'] for s in valid), default=None),
            'reason':'' if valid else reason, 'valid_sections':len(valid), 'sections':sections}


def section_precision(unit, chunk, number, samples):
    record = {'id':f'{unit["id"]}_part_{number}', 'reference_id':unit['id'], 'kind':unit['kind'],
              'source':unit['source'], 'raw_samples':len(moving_points(chunk)), 'samples':0,
              'reference_length_m':unit['length_m'], 'state':'UNAVAILABLE','precision_cm':None}
    try:
        if record['raw_samples'] < N:
            raise ValueError('连续移动原始样本不足50个')
        if unit['kind'] == 'straight':
            axis = axis_for(unit); a = unit['start']; origin = a[0]*axis[0]+a[1]*axis[1]
            lo, hi = bounds_on_axis(chunk, axis)
            lo, hi = max(lo,origin), min(hi,origin+unit['length_m'])
            if hi-lo <= EPS:
                raise ValueError('没有有效直线观测跨度')
            measured, raw_samples = crossing_samples(chunk, axis, np.linspace(lo,hi,N))
            errors = [(p['y']-a[1])*axis[0]-(p['x']-a[0])*axis[1] for p in measured]
            for p, error in zip(measured, errors):
                p['station_m'] -= origin
                p['error_cm'] = 100*error
            record.update(raw_samples=raw_samples, evaluated_length_m=hi-lo, start_station_m=lo-origin, end_station_m=hi-origin)
        else:
            points = moving_points(chunk)
            line = LineString([(p['x'],p['y']) for p in points])
            if line.length <= EPS:
                raise ValueError('没有有效曲线观测跨度')
            cumulative = [0.]
            for a,b in zip(points,points[1:]):cumulative.append(cumulative[-1]+math.hypot(b['x']-a['x'],b['y']-a['y']))
            # Corresponding reference is this curve only, not another work row.
            reference = LineString(unit['points'])
            measured, errors = [], []
            for station in np.linspace(0.,line.length,N):
                i = min(len(points)-2,max(0,bisect.bisect_right(cumulative,station)-1))
                t = (station-cumulative[i])/(cumulative[i+1]-cumulative[i])
                a,b=points[i:i+2]
                p={key:a[key]+t*(b[key]-a[key]) for key in ('stamp','x','y')}
                error=Point(p['x'],p['y']).distance(reference)
                p.update(station_m=float(station),error_cm=100*error);measured.append(p);errors.append(error)
            record.update(evaluated_length_m=line.length,start_station_m=0.,end_station_m=line.length)
        record.update(state='AVAILABLE', samples=N, precision_cm=formula_1_cm(errors),
                      mean_input_cm=100*math.fsum(errors)/N)
        samples.extend({'indicator':unit['kind'],'section_id':record['id'],'sample_index':i+1,**p} for i,p in enumerate(measured))
    except ValueError as exc:
        record['reason'] = str(exc)
    return record


def adjacent_pairs(units, field):
    """Spatial neighbours, even when TSP visits different rows consecutively."""
    groups = {}
    for unit in units:
        if unit['kind'] != 'straight' or unit['source'] != 'VERIFIED_NATIVE_WORK_ROW':continue
        u = axis_for(unit); angle = math.atan2(u[1],u[0])%math.pi
        if angle < 1e-8 or math.pi-angle < 1e-8:angle=0.
        groups.setdefault(round(angle,8),[]).append(unit)
    intervals = []
    cover = field.buffer(EPS) if field is not None else None
    for angle, group in groups.items():
        axis = (math.cos(angle),math.sin(angle)); normal=(-axis[1],axis[0]); projected=[]
        for unit in group:
            lo,hi=sorted(p[0]*axis[0]+p[1]*axis[1] for p in (unit['start'],unit['end']))
            offset=sum(p[0]*normal[0]+p[1]*normal[1] for p in (unit['start'],unit['end']))/2
            projected.append((lo,hi,offset,unit))
        edges=sorted({v for lo,hi,_,_ in projected for v in (lo,hi)})
        if field is not None:
            rings=[field.exterior,*field.interiors]
            edges=sorted({*edges,*(p[0]*axis[0]+p[1]*axis[1] for ring in rings for p in ring.coords)})
        merged={}
        for lo,hi in zip(edges,edges[1:]):
            if hi-lo<=EPS:continue
            mid=(lo+hi)/2
            active=sorted((p for p in projected if p[0]<mid<p[1]),key=lambda p:p[2])
            for a,b in zip(active,active[1:]):
                if b[2]-a[2]<=EPS:continue
                def point(s,t):return (axis[0]*s+normal[0]*t,axis[1]*s+normal[1]*t)
                if cover is not None and not cover.covers(Polygon([
                        point(lo,a[2]),point(hi,a[2]),point(hi,b[2]),point(lo,b[2])])):continue
                key=(a[3]['id'],b[3]['id']);windows=merged.setdefault(key,[])
                if windows and abs(windows[-1][1]-lo)<=EPS:windows[-1]=(windows[-1][0],hi,a,b)
                else:windows.append((lo,hi,a,b))
        for windows in merged.values():
            for lo,hi,a,b in windows:
                intervals.append((a[3],b[3],axis,lo,hi,b[2]-a[2]))
                if len(intervals)>MAX_SECTIONS:raise ValueError('交接行评价分段超出分析规模')
    return intervals


def handover_sections(units, chunks, field, samples):
    sections=[]
    for window,(ua,ub,axis,lower,upper,preset) in enumerate(adjacent_pairs(units,field),1):
        for ia,ca in enumerate(chunks[ua['id']]):
            for ib,cb in enumerate(chunks[ub['id']]):
                if len(sections)>=MAX_SECTIONS:raise ValueError('交接行评价分段超出分析规模')
                lo=max(lower,bounds_on_axis(ca,axis)[0],bounds_on_axis(cb,axis)[0])
                hi=min(upper,bounds_on_axis(ca,axis)[1],bounds_on_axis(cb,axis)[1])
                if hi-lo<=EPS:continue
                record={'id':f'{ua["id"]}__{ub["id"]}_window_{window}_{ia+1}_{ib+1}', 'kind':'handover',
                        'first_reference_id':ua['id'],'second_reference_id':ub['id'],
                        'state':'UNAVAILABLE','precision_cm':None,'samples':0,
                        'raw_samples':min(len(moving_points(ca)),len(moving_points(cb))),
                        'preset_spacing_cm':100*preset,'evaluated_length_m':hi-lo}
                try:
                    stations=np.linspace(lo,hi,N);paired=[];raw_counts=[]
                    for unit,chunk in ((ua,ca),(ub,cb)):
                        forward=sum(a*b for a,b in zip(axis,axis_for(unit)))>0
                        values,raw_count=crossing_samples(chunk,axis,stations if forward else list(reversed(stations)))
                        paired.append(values if forward else list(reversed(values)))
                        raw_counts.append(raw_count)
                    spacings=[]
                    for i,(a,b) in enumerate(zip(*paired)):
                        h=abs(-(b['x']-a['x'])*axis[1]+(b['y']-a['y'])*axis[0]);spacings.append(h)
                    record.update(state='AVAILABLE',samples=N,raw_samples=min(raw_counts),first_raw_samples=raw_counts[0],second_raw_samples=raw_counts[1],precision_cm=formula_2_cm(spacings,preset),
                                  mean_input_cm=100*sum(abs(h-preset) for h in spacings)/N)
                    samples.extend({'indicator':'handover','section_id':record['id'],'sample_index':i+1,
                        **a,'paired_stamp':b['stamp'],'paired_x':b['x'],'paired_y':b['y'],
                        'spacing_cm':100*h,'preset_spacing_cm':100*preset,'error_cm':100*abs(h-preset)}
                        for i,(a,b,h) in enumerate(zip(paired[0],paired[1],spacings)))
                except ValueError as exc:record['reason']=str(exc)
                sections.append(record)
    return sections


def assess(directory, mission, run, document, rows):
    result=empty_result('没有适用的正式跟踪数据');samples=[]
    if run['mode'] not in ('coverage','following') or not document:
        return result,samples
    if not rows:return result,samples
    try:
        if document.get('frame_id')!='map' or document.get('projection')!=run.get('projection'):
            raise ValueError('参考路径固定原点或坐标系不一致')
        if mission.get('projection')!=run.get('projection'):
            raise ValueError('任务与本轮固定原点不一致')
        units,native,note=reference_units(document,run['mode']);result['reference_source']=note
        if not units:
            return empty_result(note),samples
        chunks,recording=observed_chunks(units,rows,barriers_from_status(directory,run));result['recording']=recording
        sections={'straight':[],'curve':[]}
        for unit in units:
            parts=chunks[unit['id']]
            if not parts:sections[unit['kind']].append({'id':unit['id'],'state':'UNAVAILABLE','precision_cm':None,'samples':0,'reason':'本轮没有该参考作业段的有效记录'})
            for i,chunk in enumerate(parts):
                sections[unit['kind']].append(section_precision(unit,chunk,i+1,samples))
                if sum(len(v) for v in sections.values())>MAX_SECTIONS:raise ValueError('作业精度评价分段超出分析规模')
        result['straight']=summarize(sections['straight'],'没有满足50点连续观测要求的直线作业段')
        result['curve']=summarize(sections['curve'],'无适用曲线作业样本；原地转向/换行连接不冒充曲线作业')
        for kind in ('straight','curve'):
            if not any(unit['kind']==kind for unit in units):result[kind]['state']='NOT_APPLICABLE'
        field=None
        if run['mode']=='coverage':
            frame=LocalFrame(run['projection']['latitude'],run['projection']['longitude'])
            field=field_from_data(mission['boundary'],frame)
        pairs=handover_sections(units,chunks,field,samples)
        result['handover']=summarize(pairs,'没有可配对且两侧各有50个连续移动样本的几何相邻平行作业行')
        if native['state']!='READY' or len(units)<2:result['handover']['state']='NOT_APPLICABLE'
        result['nominal_operation_width_m']=mission.get('parameters',{}).get('operation_width')
        result['notes']=[
            '50点在已观测连续跨度内等距插值；每侧至少50个移动原始样本，不外推、不跨空档、不按误差大小挑点。',
            '直线/交接行按参考线纵向站位等距；曲线按实际轨迹弧长等距，再求到对应参考曲线的最小距离。',
            '各段观测跨度单独披露，可能短于整条参考行；未知标准速度/150m直线/专用曲线/独立测量及重复工况不标合格。',
            '交接行H取两条规划平行作业线的垂距，另存机具幅宽；按几何邻接，不按TSP访问先后配对。',
            'P1/P2是样本标准差：恒定偏移可得到0，须同时看输入均值、原有MAE/RMSE及逐点数据。']
    except (ValueError,KeyError,TypeError,IndexError) as exc:
        return empty_result(f'作业精度统计不可用：{exc}'),[]
    return result,samples


def write_artifacts(directory, result, samples):
    output=directory/'analysis'
    result['implementation_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    atomic_json(output/'standard_accuracy.json',result)
    columns=['indicator','section_id','state','precision_cm','samples','raw_samples','mean_input_cm',
             'reference_length_m','evaluated_length_m','preset_spacing_cm','reason']
    with (output/'standard_accuracy.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader()
        for key in TITLES:
            item=result[key]
            writer.writerow({'indicator':key,'section_id':'SUMMARY_MAX','state':item['state'],
                             'precision_cm':item['value_cm'],'reason':item['reason']})
            for section in item['sections']:
                writer.writerow({**{k:section.get(k) for k in columns},'indicator':key,'section_id':section['id']})
    columns=['indicator','section_id','sample_index','station_m','stamp','x','y','paired_stamp','paired_x','paired_y',
             'error_cm','spacing_cm','preset_spacing_cm']
    with (output/'standard_accuracy_samples.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(samples)
    (output/'standard_accuracy_report.md').write_text(report_section(result),encoding='utf-8')


def report_section(result):
    lines=['## 按GB/T 46270—2025公式计算的作业精度','',
           '按文件公式(1)/(2)，每组50个空间等距样本，分母N−1。以下为已观测路段的工程统计；未核验完整标准试验工况，不自动判定国标合格。','',
           '| 指标 | 有效观测段数 | 最大段指标 cm | 状态/原因 |','| --- | ---: | ---: | --- |']
    for key,title in TITLES.items():
        item=result[key];value='未测' if item['value_cm'] is None else f'{item["value_cm"]:.6f}'
        lines.append(f'| {title} | {item.get("valid_sections",0)} | {value} | {item["reason"] or "已计算"} |')
    lines+=['','公式：','']+[f'- {title}：`{FORMULAS[key]}`。' for key,title in TITLES.items()]
    lines+=['','采样与解释：','']+['- '+note for note in result.get('notes',[])]
    lines+=['','恒定偏移的标准差可能为0，不能据此认定没有偏差。现有MAE、RMSE、P95、航向和速度指标继续保留。',
            '分段数值与输入均值：`standard_accuracy.csv`；50点及配对坐标：`standard_accuracy_samples.csv`；完整方法：`standard_accuracy.json`。','']
    return '\n'.join(lines)
