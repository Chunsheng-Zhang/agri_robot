"""Per-run tracking assessment and a human-readable Chinese report, offline only."""
import bisect
import csv
import json
import math

from .experiment_communication import numeric


def error_indicators(rows, statistics):
    lateral=statistics([r.get('cross_track') for r in rows])
    heading=statistics([math.degrees(r['heading_error']) if r.get('heading_error') is not None else None for r in rows])
    values=[abs(r['cross_track']) for r in rows if r.get('cross_track') is not None]
    return {'samples':len(rows),'cross_track_m':lateral,'heading_error_deg':heading,
            'within_cross_track_percent':{str(limit):100*sum(v<=limit for v in values)/len(values) if values else None
                                          for limit in (.05,.1,.2)}}


def geometry(reference):
    points=[]
    for point in reference:
        if not points or math.dist(points[-1],point)>1e-6:points.append(point)
    cumulative=[0.0]
    headings=[]
    for a,b in zip(points,points[1:]):
        cumulative.append(cumulative[-1]+math.dist(a,b))
        headings.append(math.atan2(b[1]-a[1],b[0]-a[0]))
    turns=[]
    for i in range(1,len(points)-1):
        angle=(headings[i]-headings[i-1]+math.pi)%(2*math.pi)-math.pi
        turns.append((cumulative[i],angle/((cumulative[i+1]-cumulative[i-1])/2)))
    return cumulative,turns


def task_statuses(directory, mode):
    path=directory/'telemetry/status.jsonl'
    if not path.exists():return {'available':False,'reason':'无任务状态记录'}
    topic='/tracking/status' if mode=='coverage' else '/waypoint/status'
    rows=[];malformed=0
    for line in path.read_text().splitlines():
        try:
            item=json.loads(line);data=item['data'];stamp=float(item['received_ros_ns'])/1e9
            if item['topic']==topic and data.get('run_directory')==str(directory/'tracks') and math.isfinite(stamp):
                rows.append((stamp,data))
        except (ValueError,TypeError,KeyError):malformed+=1
    if not rows:return {'available':False,'reason':'无匹配本轮目录的任务状态','malformed_rows':malformed}
    durations={};pauses=0;last_state=None;max_gap=0;backwards=0
    for i,(stamp,data) in enumerate(rows):
        state=data.get('state','UNKNOWN')
        if state=='PAUSED' and last_state!='PAUSED':pauses+=1
        last_state=state
        if i+1<len(rows):
            dt=rows[i+1][0]-stamp
            if dt<0:backwards+=1;continue
            max_gap=max(max_gap,dt)
            durations[state]=durations.get(state,0)+dt
    return {'available':True,'samples':len(rows),'pause_count':pauses,'approximate_state_durations_s':durations,
            'max_status_interval_s':max_gap,'backward_timestamps':backwards,'malformed_rows':malformed,
            'last_state':rows[-1][1].get('state'),'last_reason':rows[-1][1].get('reason',''),
            'note':'按相邻状态接收时刻估算；状态中断时不能视为精确运行时长'}


def tracking_assessment(directory, mission, run, reference, phases, motion, mgx, statistics):
    rows=phases.get('tracking',[])
    result={'available':bool(rows),'method':{
        'weighting':'按记录样本统计，不是按时间/距离加权；同一 MGX23 定位用于控制和评价，无独立真值',
        'startup_excluded_s':5.0,'straight_window_m':1.0,'straight_curvature_limit_rad_per_m':.15,
        'straight_definition':'参考路径进度前后1m内全部顶点曲率绝对值<0.15 rad/m，且离路径两端>1m；其余为转弯/衔接',
        'curvature_definition':'相邻线段夹角（wrap到±pi）除以两段平均长度；依赖参考路径采样分辨率',
        'descriptive_error_bands_m':[.05,.1,.2], 'bands_are_acceptance_requirements':False},
        'task_status':task_statuses(directory,run['mode'])}
    if not rows:
        result['reason']='没有正式覆盖跟踪样本；多点任务的目标点距离不能作为横向跟踪误差'
        return result
    cumulative,turns=geometry(reference)
    total=cumulative[-1] if cumulative else 0
    turn_positions=[p for p,_ in turns]
    startup=rows[0]['stamp']+5
    groups={'all_tracking':rows,'after_startup':[],'straight_core_after_startup':[],'turn_transition_after_startup':[]}
    labels=[]
    for row in rows:
        progress=row.get('progress')
        straight=False
        if progress is not None and 1<progress<total-1:
            lo,hi=bisect.bisect_left(turn_positions,progress-1),bisect.bisect_right(turn_positions,progress+1)
            straight=all(abs(k)<.15 for _,k in turns[lo:hi])
        label='startup' if row['stamp']<startup else 'straight_core' if straight else 'turn_transition'
        labels.append(label)
        if label!='startup':
            groups['after_startup'].append(row)
            groups['straight_core_after_startup' if straight else 'turn_transition_after_startup'].append(row)
    result['subsets']={name:error_indicators(values,statistics) for name,values in groups.items()}
    result['final_progress_percent']=100*max(0,min(total,rows[-1]['progress']))/total if total and rows[-1].get('progress') is not None else None
    result['final_endpoint_distance_m']=math.dist((rows[-1]['x'],rows[-1]['y']),reference[-1]) if reference else None
    with (directory/'analysis/tracking_samples.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['stamp','progress_m','cross_track_m','heading_error_deg','subset'])
        for row,label in zip(rows,labels):writer.writerow([row['stamp'],row.get('progress'),row.get('cross_track'),
            math.degrees(row['heading_error']) if row.get('heading_error') is not None else None,label])
    # Only compare samples within 0.25s of an actual formal-tracking observation.
    track_stamps=sorted(r['stamp'] for r in rows)
    def in_tracking(stamp):
        if stamp is None or not track_stamps[0]<=stamp<=track_stamps[-1]:return False
        index=bisect.bisect_left(track_stamps,stamp)
        return any(0<=j<len(track_stamps) and abs(stamp-track_stamps[j])<=.25 for j in (index-1,index))
    commands=[]
    for row in motion:
        if row['topic']!='/control/tracking/cmd_vel':continue
        stamp=numeric(row.get('source_stamp_ns'))
        stamp=stamp/1e9 if stamp is not None else None
        v,w,lateral=numeric(row.get('linear_x')),numeric(row.get('angular_z')),numeric(row.get('linear_y'))
        if in_tracking(stamp) and None not in (v,w,lateral):commands.append((stamp,v,w,lateral))
    commands.sort();command_times=[r[0] for r in commands]
    max_angular=numeric(mission.get('parameters',{}).get('max_angular'))
    result['angular_command']={'samples':len(commands),'configured_max_rad_s':max_angular,
        'near_limit_percent':100*sum(abs(r[2])>=.99*max_angular for r in commands)/len(commands) if commands and max_angular else None,
        'meaning':'正式跟踪附近控制器指令样本中，|角速度|≥任务上限99%的比例；按样本统计，非执行器饱和实测'}
    speeds,errors=[],[]
    for row in mgx:
        if row['topic']!='/mgx23/velocity_stamped':continue
        stamp,n,e=numeric(row.get('source_stamp_ns')),numeric(row.get('v1')),numeric(row.get('v2'))
        stamp=stamp/1e9 if stamp is not None else None
        if not in_tracking(stamp) or None in (n,e):continue
        speed=math.hypot(n,e);speeds.append(speed)
        index=bisect.bisect_right(command_times,stamp)-1
        if index>=0 and 0<=stamp-commands[index][0]<=.25:
            command=commands[index]
            errors.append(speed-math.hypot(command[1],command[3]))
    result['mgx23_horizontal_speed_m_s']=statistics(speeds)
    result['mgx23_minus_command_speed_m_s']=statistics(errors)
    result['speed_comparison_note']='源时间对齐最近一条不超过0.25s的控制器指令；不补偿惯性/滤波滞后，不能单独推断限速或纯通信延迟'
    result['task_configured_speed_m_s']=numeric(mission.get('parameters',{}).get('speed'))
    return result


def fmt(value, scale=1, digits=3):
    return f'{value*scale:.{digits}f}' if value is not None else '未测/无样本'


def write_summary(directory, mission, metrics):
    assessment=metrics['tracking_assessment'];communication=metrics['communication']
    lines=['# 本轮实验结果汇总','',f'- 运行编号：`{metrics["run_id"]}`',
           f'- 任务：{str(mission.get("name", "未命名")).replace(chr(10), " ")}；模式：`{metrics["mode"]}`',
           f'- 最终记录状态：`{metrics["task_state"]}`；该状态不等于整块地作业覆盖率或独立精度验证。',
           '- 结束原因：'+str((metrics['capture'].get('terminal') or {}).get('reason') or '未记录额外原因').replace(chr(10),' '),
           f'- 生成时间：{metrics["generated_at"]}',
           f'- 采集开始：{metrics["capture"].get("started_at", "未测")}；记录结束观察时间：{(metrics["capture"].get("terminal") or {}).get("observed_at", "未测")}',
           f'- 固定原点：`{json.dumps(metrics["projection"],ensure_ascii=False)}`',
           '- 坐标：固定 UTM 原点下的 map/base_link；未平移、旋转或逐条轨迹归零。','',
           '## 任务参数','', '| 参数 | 本轮保存值 |','| --- | --- |']
    for name,value in mission.get('parameters',{}).items():
        lines.append(f'| `{name}` | {str(value).replace("|", "/").replace(chr(10), " ")} |')
    lines+=['','## 跟踪精度','',
            '横向误差使用控制器相对当前参考路径段的带符号距离。MAE 是平均绝对误差，RMSE 更突出大误差，P95 表示95%的样本绝对误差不超过该值。带符号均值可能正负抵消，不能单独评价精度。','',
            '| 统计范围 | 样本数 | 横向均值 cm | MAE cm | RMSE cm | P95 cm | P99 cm | 最大 cm | 航向 MAE ° | 航向 RMSE ° | 航向 P95 ° | 航向最大 ° |',
            '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    names={'all_tracking':'全部正式跟踪','after_startup':'去掉起始5秒','straight_core_after_startup':'直线核心区（去起始5秒）','turn_transition_after_startup':'转弯/衔接（去起始5秒）'}
    for name,item in assessment.get('subsets',{}).items():
        e,h=item['cross_track_m'],item['heading_error_deg']
        lines.append('| '+ ' | '.join([names[name],str(item['samples'])]+[fmt(e[k],100) for k in ('mean','mae','rmse','abs_p95','abs_p99','abs_max')]+[fmt(h[k]) for k in ('mae','rmse','abs_p95','abs_max')])+' |')
    if not assessment['available']:lines+=['',assessment['reason']]
    else:
        bands=assessment['subsets']['all_tracking']['within_cross_track_percent']
        lines+=['',f'正式跟踪横向误差在 5 / 10 / 20 cm 内的样本比例：{fmt(bands["0.05"])}% / {fmt(bands["0.1"])}% / {fmt(bands["0.2"])}%。这些是描述性分档，不是已确认的验收标准。',
                f'正式跟踪横向误差总体标准差：{fmt(assessment["subsets"]["all_tracking"]["cross_track_m"]["std"],100)} cm；绝对误差中位数：{fmt(assessment["subsets"]["all_tracking"]["cross_track_m"]["abs_median"],100)} cm；航向绝对误差 P99：{fmt(assessment["subsets"]["all_tracking"]["heading_error_deg"]["abs_p99"])}°。',
                f'最后记录位置距终点：{fmt(assessment["final_endpoint_distance_m"],100)} cm；记录进度：{fmt(assessment["final_progress_percent"])}%。未完成任务的误差只代表已记录路段。',
                '',assessment['method']['straight_definition']+'；'+assessment['method']['curvature_definition']+'。']
    lines+=['','## 阶段、暂停与速度','', '| 阶段 | 样本数 | 记录跨度 s（含空档） | 航向 MAE ° | 航向 RMSE ° |','| --- | ---: | ---: | ---: | ---: |']
    for name,phase in metrics['phases'].items():
        h=phase['heading_error_deg']
        lines.append(f'| {name} | {phase["samples"]} | {fmt(phase["recorded_span_s"])} | {fmt(h["mae"])} | {fmt(h["rmse"])} |')
    status=assessment['task_status']
    lines+=['','接近、原地对齐与正式跟踪分别统计；对齐文件跨多次恢复时，记录跨度和位置折线长度不能当作原地转向持续时间/行驶距离。']
    if status['available']:
        lines+=[f'暂停次数：{status["pause_count"]}；状态推算暂停时长：{fmt(status["approximate_state_durations_s"].get("PAUSED",0))} s；最大状态采样间隔：{fmt(status["max_status_interval_s"])} s。{status["note"]}。']
    else:lines+=['暂停统计：未测，'+status['reason']+'。']
    if assessment['available']:
        speed=assessment['mgx23_horizontal_speed_m_s'];error=assessment['mgx23_minus_command_speed_m_s']
        lines+=[f'正式跟踪配置速度：{fmt(assessment["task_configured_speed_m_s"])} m/s；MGX23 实测水平速度均值/中位数：{fmt(speed["mean"])} / {fmt(speed["abs_median"])} m/s。',
                f'MGX23−匹配控制指令速度：{error["samples"]} 对样本，平均差 {fmt(error["mean"])} m/s，RMSE {fmt(error["rmse"])} m/s。',
                assessment['speed_comparison_note']+'。',
                f'接近角速度指令上限的样本比例：{fmt(assessment["angular_command"]["near_limit_percent"])}%；'+assessment['angular_command']['meaning']+'。']
    lines+=['','## 通信与记录完整性','']
    if communication['available']:
        lines+=['| 数据流 | 接收数 | 实收 Hz | 延迟均值 ms | P95 ms | P99 ms | 最大 ms | 超过100ms比例 % | 最长接收间隔 ms |',
                '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
        for topic,value in communication['topics'].items():
            age=value['source_to_recorder_ms']
            lines.append('| '+' | '.join([f'`{topic}`',str(value['received']),fmt(value['observed_hz'])]+[fmt(age[k]) for k in ('mean','p95','p99','max','over_threshold_percent')]+[fmt(value['interarrival_ms']['max'])])+' |')
        lines+=['','100 ms 是统一诊断参考线，不是停车阈值或统一合格线。没有源时间戳的消息显示未测；雷达 Header 通常是首束采集时刻，其年龄包含扫描成帧时间。']
        lines+=['']
        for topic,value in communication['topics'].items():
            if value['negative_age_samples']:
                lines.append(f'- `{topic}` 有 {value["negative_age_samples"]} 个负消息年龄，已从延迟分布排除，需检查时钟。')
        for session,item in communication['raw_sequence_by_session'].items():
            lines.append(f'- 原始块会话 `{session}`：首末序号范围应有 {item["expected_between_first_last"]} 块，记录唯一块 {item["unique"]}，缺口 {item["missing"]}，缺口率 {fmt(item["missing_percent"])}%。该比例不是 IP 丢包率。')
        for topic,item in communication['matched_source_stamps'].items():
            lines.append(f'- 同源时间戳匹配 `{topic}`：位置参考 {item["position_reference_samples"]} 条，未匹配 {item["unmatched"]} 条，比例 {fmt(item["unmatched_percent"])}%。去掉位置观察窗口首尾1秒；四路共同缺失无法由此识别。')
        navi=communication['navi']
        if navi['available']:
            lines.append(f'- NAVI：{navi["frames"]} 帧，校验失败 {navi["checksum_failures"]} 帧，重复设备时刻 {navi["repeated_device_times"]} 次，按10Hz估算缺少 {fmt(navi["estimated_missing_at_10hz"],digits=0)} 个时间槽；不判定缺口原因，不作为网络丢包证明。')
            if navi['unclassified_long_intervals'] or navi['invalid_or_backward_device_times']:
                lines.append('- NAVI 设备时间存在长间断或无效/后退变化，缺失估算保持未知，需单独检查设备/时钟/记录。')
        else:lines.append('- NAVI：未测；'+navi['reason']+'。')
        lines+=['','| 后端查询接口 | 请求数 | 失败/未完成数 | 处理均值 ms | P95 ms | P99 ms | 最大 ms | 超过2秒比例 % |',
                '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
        for path,value in communication['http_server'].items():
            age=value['server_duration_ms']
            lines.append('| '+' | '.join([f'`{path}`',str(value['requests']),str(value['failed_or_incomplete'])]+[fmt(age[k]) for k in ('mean','p95','p99','max','over_threshold_percent')])+' |')
        if not communication['http_server']:lines+=['','本轮没有观察到受监测的后端查询请求，不能视为零延迟或零超时。']
        lines+=['','以下指标未测：','']+['- '+value for value in communication['unmeasured'].values()]
    else:lines+=[communication['reason']]
    lines+=['',f'- 记录器队列丢弃：`{json.dumps(metrics["capture"].get("queue_drops_by_topic",{}),ensure_ascii=False)}`',
            f'- 记录器错误：`{json.dumps(metrics["capture"].get("errors",[]),ensure_ascii=False)}`',
            '- 缺失必要话题：'+(', '.join(f'`{topic}`' for topic in metrics['missing_topics']) or '未发现（仅指记录器观测）'),'','## 评价边界与文件','',
            '- 本报告使用同一套 MGX23 定位评价跟踪，不代表独立真值下的绝对定位精度。',
            '- 各项按记录样本统计，未按距离或时间重新加权；缺失和暂停可能改变样本权重。',
            '- 未设定自动“合格/不合格”总评分；故障、取消、未开始等状态如实保留。',
            '- `metrics.json` / `metrics.csv`：详细指标；`tracking_samples.csv`：正式跟踪分区明细（有样本时）。',
            '- `communication.json` 和 `../communication/`：通信汇总与逐条时序，`../telemetry/recording.json`：采集清单。',
            '- 原始文件分别在 `../raw/`、`../telemetry/`、`../tracks/`，配置和参考路径在 `../config/`、`../paths/`。','',
            '![轨迹、速度、横向误差和航向误差](trajectory_and_speed.png)','']
    lines+=['## 通信诊断图','',
            '两图沿用跟踪图时间零点，底色标识 Approach（接近）、Align（对齐）、Track（跟踪）、Pause（暂停）；缺少匹配状态时不猜测阶段。',
            '缺失率按10秒窗口统计，原始块序号缺口和对应分母统一归入发现缺口的后一个片段窗口，不代表缺失数据确切发送时刻；空窗口不补零。记录器丢弃可能与序号缺口重叠，不能相加。',
            '延迟曲线为1秒窗口P95（实线）与最大值（点线），表格为整轮采集窗口的均值、P95/P99、最大值和超参考线比例；参考线不改变安全策略。',
            '![消息缺失率与记录器丢弃](communication_loss.png)','',
            '![消息延迟与后端处理耗时](communication_latency.png)','',
            '图表数据保存在 `communication_plot_data.json`。底层IP丢包、浏览器完整RTT和车体执行延迟仍未测，不能由这两张图直接推断。','']
    lines+=['## 本轮分析备注','']+['- '+note for note in metrics['notes']]+['']
    (directory/'analysis/report.md').write_text('\n'.join(lines),encoding='utf-8')
