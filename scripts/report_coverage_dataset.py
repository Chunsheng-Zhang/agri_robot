#!/usr/bin/env python3
"""Single-strategy coverage/time/topology report over the frozen WKT sample."""
import argparse
from collections import Counter
import csv
from datetime import datetime
from zoneinfo import ZoneInfo
import gc
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import numpy as np
from shapely import wkt
from shapely.geometry import shape
from shapely.affinity import translate
from shapely.ops import transform
from pyproj import Transformer
from benchmark_path_topology import topology_metrics,route_segments,line_parts

ROOT=Path(__file__).resolve().parents[1]


def stats(values):
    values=[float(v) for v in values if v is not None]
    if not values:return {'n':0,'mean':None,'median':None,'p95':None,'min':None,'max':None}
    return {'n':len(values),'mean':statistics.mean(values),'median':statistics.median(values),
            'p95':float(np.quantile(values,.95)),'min':min(values),'max':max(values)}


def projected(record):
    raw=wkt.loads((ROOT/record['file']).read_text())
    p=transform(Transformer.from_crs(4326,record['epsg'],always_xy=True).transform,raw)
    return translate(p,xoff=-record['utm_anchor'][0],yoff=-record['utm_anchor'][1])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=ROOT/'.diagnostics/coverage50_10m_20261115')
    parser.add_argument('--plots',action='store_true');parser.add_argument('--partial',action='store_true')
    args=parser.parse_args();out=args.out
    manifest=json.loads((out/'manifest.json').read_text());rows=json.loads((out/'results.json').read_text())
    config=json.loads((out/'configuration.json').read_text());size=len(manifest['fields'])
    if not args.partial:assert len(rows)==size and {r['index'] for r in rows}==set(range(size))
    assert len({r['index'] for r in rows})==len(rows)
    assert all(r['method']=='current' and r['width_m']==10 for r in rows)
    assert all(r['signature']==config['signature'] for r in rows)
    for name,h in {**config['source_hashes'],**json.loads((out/'extra_source_hashes.json').read_text())}.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h,name
    for source in manifest['fields']:
        assert hashlib.sha256((ROOT/source['file']).read_bytes()).hexdigest()==source['sha256']
    topology_dir=out/'topology';topology_dir.mkdir(exist_ok=True);augmented=[]
    for row in sorted(rows,key=lambda r:r['index']):
        value=dict(row)
        if row['state']=='SUCCEEDED':
            assert row['planning_policy_version']==7 and row.get('clearance_passed') is True
            with gzip.open(out/'jobs'/f"{row['key']}_optimization.json.gz",'rt') as stream:optimization=json.load(stream)
            assert optimization['search_complete'] and not optimization['candidate_limit_enabled'] and not optimization['search_time_limit_enabled']
            with gzip.open(out/'jobs'/f"{row['key']}_path.json.gz",'rt') as stream:data=json.load(stream)
            assert data['frame_id']=='benchmark_local_utm' and data['epsg']==row['source']['epsg']
            assert data['utm_anchor']==row['source']['utm_anchor']
            assert wkt.loads(data['field_wkt']).equals_exact(projected(row['source']),1e-8)
            top=topology_metrics(data['vertices'],data.get('path_details'))
            rounding_bound=max(.02,top['segment_count']*2*np.sqrt(2)*top['tolerance_m'])
            top['length_rounding_error_m']=abs(top['centerline_length_m']-row['independent']['length_m'])
            assert top['length_rounding_error_m']<rounding_bound
            assert abs(sum(top['extra_retraced_by_current_role_m'].values())-top['extra_retraced_length_m'])<rounding_bound
            (topology_dir/f'{row["field"]}.json').write_text(json.dumps(top,ensure_ascii=False,indent=2))
            value['topology']={k:v for k,v in top.items() if k not in ('crossing_points','repeated_centerline_geometry','backtrack_points')}
            value['topology_file']=str((topology_dir/f'{row["field"]}.json').relative_to(out))
        augmented.append(value)
    good=[r for r in augmented if r['state']=='SUCCEEDED'];area=sum(r['independent']['field_area_m2'] for r in good)
    summary={'generated_at':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'width_m':10.,
        'sample_size':size,'completed':len(rows),'states':dict(Counter(r['state'] for r in rows)),
        'success_count':len(good),'success_rate_all_inputs':len(good)/size,
        'valid_input_count':sum(bool(r['valid']) for r in manifest['fields']),
        'success_rate_valid_inputs':len(good)/sum(bool(r['valid']) for r in manifest['fields']),
        'quality':{k:stats(r['independent'][k] for r in good) for k in ('coverage_percent','overlap_percent','length_m','point_turn_count','score')},
        'time':{k:stats(r.get(k) for r in rows)
                for k in ('planner_wall_seconds','reported_search_seconds','planning_seconds')},
        'success_planner_wall_seconds':stats(r.get('planner_wall_seconds') for r in good),
        'success_search_size':{k:stats(r.get(k) for r in good) for k in (
            'evaluated_candidates','feasible_candidates','native_tsp_calls','peak_rss_mb')},
        'selected_families':dict(Counter(r['selected']['layout_family'] for r in good)),
        'transition_fraction':stats(r['selected']['transition_length_m']/r['independent']['length_m'] for r in good),
        'area_weighted':{k:100*sum(r['independent'][part] for r in good)/area if area else None
                         for k,part in [('coverage_percent','covered_m2'),('overlap_percent','overlap_m2')]},
        'topology':{k:stats(r['topology'][k] for r in good) for k in (
            'proper_crossing_locations','proper_crossing_locations_per_km','extra_retraced_length_m',
            'extra_retraced_fraction','repeated_centerline_unique_m','immediate_collinear_backtracks')},
        'crossing_presence':{'none':sum(r['topology']['proper_crossing_locations']==0 for r in good),
                             'present':sum(r['topology']['proper_crossing_locations']>0 for r in good)},
        'crossing_role_field_counts':dict(Counter(k for r in good for k,v in r['topology']['crossing_pair_roles'].items() if v)),
        'role_reconstruction':dict(Counter(r['topology']['role_reconstruction'] for r in good)),
        'clearance_passed_count':sum(r.get('clearance_passed') is True for r in good),
        'grid_vector_difference_pp':{k:stats(r.get(k) for r in good) for k in (
            'grid_vector_coverage_difference_pp','grid_vector_overlap_difference_pp')},
        'rejection_reason_field_counts':dict(Counter(k for r in rows for k in r.get('rejection_counts',{}))),
        'all_source_hashes_verified':True,'hardware':False}
    summary['strata']={}
    for label,group in [('with_holes',[r for r in good if r['source']['holes']]),('without_holes',[r for r in good if not r['source']['holes']])]:
        inputs=[r for r in augmented if bool(r['source']['holes'])==(label=='with_holes')]
        summary['strata'][label]={'n':len(group),'input_count':len(inputs),'states':dict(Counter(r['state'] for r in inputs)),
                                 **{k:stats(r['independent'][k] for r in group) for k in ('coverage_percent','overlap_percent')},
                                 'planner_wall_seconds':stats(r.get('planner_wall_seconds') for r in group)}
    rankings={}
    for key,container,descending in [('overlap_percent','independent',True),('coverage_percent','independent',False),
            ('extra_retraced_fraction','topology',True),('proper_crossing_locations_per_km','topology',True)]:
        rankings[key]=[{'field':r['field'],'index':r['index']+1,'value':r[container][key]} for r in sorted(good,key=lambda r:r[container][key],reverse=descending)[:8]]
    summary['rankings']=rankings
    tradeoff_rows={r['index']:r for r in sorted(good,key=lambda r:r['independent']['coverage_percent'])[:3]}
    tradeoff_rows.update({r['index']:r for r in sorted(good,key=lambda r:r['independent']['overlap_percent'],reverse=True)[:3]})
    summary['candidate_tradeoffs']=[]
    candidate_columns=('coverage_percent','overlap_percent','length_m','point_turn_count','score','layout_family','angle_rad')
    for row in tradeoff_rows.values():
        with gzip.open(out/'jobs'/f"{row['key']}_optimization.json.gz",'rt') as stream:optimization=json.load(stream)
        evaluated=[c for c in optimization['candidates'] if c.get('state')=='EVALUATED']
        max_coverage=min(evaluated,key=lambda c:(-c['coverage_percent'],c['score']))
        summary['candidate_tradeoffs'].append({'field':row['field'],
            'selected':{k:optimization['selected'].get(k) for k in candidate_columns},
            'max_coverage_evaluated_candidate':{k:max_coverage.get(k) for k in candidate_columns},
            'metric_source':'candidate grid metrics, not independent vector metrics; max coverage is not the weighted objective winner'})
    attempts=[]
    for file in sorted((out/'attempts').glob('**/*.json')):
        record=json.loads(file.read_text())
        if str(record.get('state','')).startswith('BENCHMARK_') or record.get('state')=='WORKER_CRASH':
            log=file.with_suffix('.log')
            bootstrap=(record.get('state')=='WORKER_CRASH' and log.exists()
                       and "No module named 'agri_base'" in log.read_text(errors='replace'))
            attempts.append({'field':record.get('field'),'state':record['state'],
                             'classification':'OVERLAY_BOOTSTRAP_ERROR' if bootstrap else record['state'],
                             'archive':str(file.relative_to(out))})
    summary['prior_infrastructure_attempts']=attempts
    summary['prior_infrastructure_counts']=dict(Counter(r['classification'] for r in attempts))
    name='partial_' if args.partial else ''
    (out/f'{name}summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    (out/f'{name}results_with_topology.json').write_text(json.dumps(augmented,ensure_ascii=False,indent=2))
    write_csv(out/f'{name}per_field_results.csv',augmented)
    write_report(out/f'{name}report.md',summary,augmented)
    if args.plots:plot(out,manifest,augmented)
    print(json.dumps({'completed':len(rows),'states':summary['states'],'quality':summary['quality'],
                      'time':summary['success_planner_wall_seconds'],'topology':summary['topology']},ensure_ascii=False))


def write_csv(target,rows):
    names=['index','field','state','area_ha','holes','planner_wall_seconds','reported_search_seconds','planning_seconds',
           'evaluated_candidates','feasible_candidates','native_tsp_calls','peak_rss_mb',
           'coverage_percent','overlap_percent','length_m','point_turn_count','layout_family','cell_count',
           'swath_length_m','transition_length_m','transition_fraction',
           'proper_crossing_locations','proper_crossing_locations_per_km','extra_retraced_length_m','extra_retraced_fraction',
           'work_work_crossing_pairs','work_transition_crossing_pairs','transition_transition_crossing_pairs',
           'extra_retrace_work_m','extra_retrace_transition_m','role_reconstruction','clearance_passed','reason']
    with target.open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=names);writer.writeheader()
        for row in rows:
            value={k:row.get(k) for k in names};value.update(index=row['index']+1,area_ha=row['source']['area_m2']/1e4,holes=row['source']['holes'])
            value.update({k:row.get('independent',{}).get(k) for k in ('coverage_percent','overlap_percent','length_m','point_turn_count')})
            value.update({k:row.get('selected',{}).get(k) for k in ('layout_family','cell_count','swath_length_m','transition_length_m')})
            value['transition_fraction']=value['transition_length_m']/value['length_m'] if value.get('length_m') else None
            t=row.get('topology',{});value.update({k:t.get(k) for k in ('proper_crossing_locations','proper_crossing_locations_per_km','extra_retraced_length_m','extra_retraced_fraction','role_reconstruction')})
            for column,role in [('work_work_crossing_pairs','work/work'),('work_transition_crossing_pairs','transition/work'),('transition_transition_crossing_pairs','transition/transition')]:
                value[column]=t.get('crossing_pair_roles',{}).get(role,0) if t.get('role_reconstruction')=='reconstructed' else None
            for role in ('work','transition'):value[f'extra_retrace_{role}_m']=t.get('extra_retraced_by_current_role_m',{}).get(role,0.) if t.get('role_reconstruction')=='reconstructed' else None
            writer.writerow(value)


def write_report(target,s,rows):
    def val(v):return '未知' if v is None else f'{v:.4f}'
    lines=['# 当前一键优化规划：10m幅宽、固定50地块评估','',f'生成：{s["generated_at"]}。完成{s["completed"]}/{s["sample_size"]}；状态：`{json.dumps(s["states"],ensure_ascii=False)}`。',
        '沿用原随机种子20261114的50块，不按结果补抽；被测v7代码与输入指纹核验通过。只评估当前策略，没有重跑其他算法对照。',
        '以下质量只统计成功产出完整路径的样本；失败、无效与外部保护另列，不填零。','',
        '| 指标 | 样本数 | 均值 | 中位数 | P95 | 最小 | 最大 |','| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    measures=[('覆盖率 %',s['quality']['coverage_percent']),('重复覆盖率 %',s['quality']['overlap_percent']),
              ('成功路径优化调用用时 s',s['success_planner_wall_seconds']),
              ('所有完整结束优化调用用时 s，含完整失败',s['time']['planner_wall_seconds']),
              ('额外中心线重复行驶 m',s['topology']['extra_retraced_length_m']),
              ('额外重复行驶比例，0–1',s['topology']['extra_retraced_fraction']),('非相邻路段真交叉点',s['topology']['proper_crossing_locations'])]
    for label,d in measures:lines.append(f'| {label} | {d["n"]} | '+' | '.join(val(d[k]) for k in ('mean','median','p95','min','max'))+' |')
    lines += ['',f'面积加权：`{json.dumps(s["area_weighted"],ensure_ascii=False)}`；选定布局：`{json.dumps(s["selected_families"],ensure_ascii=False)}`。',
        f'无真交叉/有真交叉地块数：`{json.dumps(s["crossing_presence"],ensure_ascii=False)}`；各类交叉存在的地块数：`{json.dumps(s["crossing_role_field_counts"],ensure_ascii=False)}`。',
        f'作业/过渡角色重建：`{json.dumps(s["role_reconstruction"],ensure_ascii=False)}`。',
        '真交叉排除相邻转角、共享端点和T形接入；共享通道重走可能合理，交叉数不直接等同规划错误。中心线精确重走与10m足迹重复覆盖分开统计。',
        f'换行/换区连接长度占比（0–1）：`{json.dumps(s["transition_fraction"],ensure_ascii=False)}`。即使没有几何交叉，仍可能存在长连接或不连续访问，因此需要结合图中访问编号检查。',
        '图册左图按行驶顺序着色并标作业行访问编号，右图蓝色为作业行、橙色为连接、红色为重复中心线、紫叉为真交叉；绿点起点、红方块终点。','',
        '## 值得逐图检查的地块','']
    for key,ranking in s['rankings'].items():
        lines += [f'### {key}','', '| 地块 | 原样本编号 | 数值 |','| --- | ---: | ---: |']
        lines += [f'| {r["field"]} | {r["index"]} | {r["value"]:.5f} |' for r in ranking]
        lines.append('')
    lines += ['## 非成功记录','', '| 地块 | 状态 | 原因 |','| --- | --- | --- |']
    lines += [f'| {r["field"]} | {r["state"]} | {r.get("reason","")} {json.dumps(r.get("rejection_counts",{}),ensure_ascii=False)} |' for r in rows if r['state']!='SUCCEEDED']
    lines += ['','## 已评估候选的取舍','',
              '按低覆盖/高重复各取前三块去重。下表两方案均用优化器自身网格口径，不能与上方独立矢量评价混作精度差异。最高覆盖候选不一定是四目标代价最小的候选。','',
              '| 地块 | 当前 C/O %；长度m；转向；J | 候选中最高覆盖 C/O %；长度m；转向；J |',
              '| --- | --- | --- |']
    for item in s['candidate_tradeoffs']:
        def candidate(v):return f'{v["coverage_percent"]:.3f}/{v["overlap_percent"]:.3f}；{v["length_m"]:.1f}；{v["point_turn_count"]}；{v["score"]:.5f}'
        lines.append(f'| {item["field"]} | {candidate(item["selected"])} | {candidate(item["max_coverage_evaluated_candidate"])} |')
    lines += ['',f'历史基础设施中断统计：`{json.dumps(s["prior_infrastructure_counts"],ensure_ascii=False)}`。',
        '首次低内存保护和一次漏加载overlay的补测启动错误均保留在attempts中；启动错误没有进入优化算法。按原参数、相同源码补测后，最终结果替代中断项，不能把中断项当成算法完整失败或把补测等待计为规划用时。详见summary.json中的prior_infrastructure_attempts。',
        '单进程、2CPU；启动至少500MiB可用内存，全机350MiB/进程1400MiB/单项1200秒实验保护保留。等待和保护不属于优化算法主动截断；外部中断不当成算法完整失败。',
        '优化函数调用时间不含独立评价、归档和绘图；planning_seconds为旧兼容口径，另含输入准备与候选归档，不混用。仅单次主机运行，受系统负载和原生启发式影响，不是严格性能基准或Jetson实测工时。',
        '10m仅是圆盘等效幅宽，物理包络仍0.45×0.2m、余量0.05m。未建模真实10m机具外伸或机具开关，不代表实际农艺效果或真实机具旋转安全。',
        '输入采用逐地块UTM/共同质心平移，不改变机器人固定map原点；覆盖、重复分母为原地块扣孔洞、含地头。',
        '全部50块路径图和失败项保存在atlas_50_fields.pdf与figures；CSV、JSON、源码快照、候选/原始完整路径和首次中断均保留。','']
    target.write_text('\n'.join(lines))


def plot(out,manifest,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.collections import LineCollection
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name();plt.rcParams['axes.unicode_minus']=False
    lookup={r['index']:r for r in rows};images=out/'figures';images.mkdir(exist_ok=True)
    def boundary(ax,field):
        ax.fill(*field.exterior.xy,color='#edf3e8');ax.plot(*field.exterior.xy,c='#333',lw=1.)
        for ring in field.interiors:ax.fill(*ring.xy,color='#c8c8c8');ax.plot(*ring.xy,c='#333',lw=.8)
        ax.set_aspect('equal');ax.grid(alpha=.12)
    def route(ax,data,top,sequence=False,small=False):
        segments,_=route_segments(data['vertices'],data.get('path_details'))
        paths=[[a,b] for a,b,role in segments]
        if sequence:
            collection=LineCollection(paths,cmap='viridis',linewidths=.8);collection.set_array(np.linspace(0,1,len(paths)));ax.add_collection(collection)
            work=[(a,b) for a,b,role in segments if role=='work']
            step=max(1,int(np.ceil(len(work)/45)))
            for i,(a,b) in enumerate(work):
                if i%step==0:ax.text((a[0]+b[0])/2,(a[1]+b[1])/2,str(i+1),fontsize=5,color='#263b30')
        else:
            for role,color in [('work','#287c99'),('transition','#dc9231'),('unknown','#777777')]:
                paths=[[a,b] for a,b,r in segments if r==role]
                if paths:ax.add_collection(LineCollection(paths,colors=color,linewidths=.8))
            repeat=shape(top['repeated_centerline_geometry'])
            parts=line_parts(repeat)
            if parts:ax.add_collection(LineCollection([list(p.coords) for p in parts],colors='#cc3030',linewidths=1.7))
            if top['crossing_points']:
                x,y=zip(*top['crossing_points']);ax.scatter(x,y,c='#8b3db0',marker='x',s=12 if small else 22,zorder=5)
        a,b=data['vertices'][0],data['vertices'][-1]
        ax.scatter(*a,c='#198b3b',s=12 if small else 24,zorder=6);ax.scatter(*b,c='#d9443b',marker='s',s=12 if small else 24,zorder=6)
    with PdfPages(out/'atlas_50_fields.pdf') as pdf:
        for i,source in enumerate(manifest['fields']):
            row=lookup.get(i);field=projected(source);fig,axes=plt.subplots(1,2,figsize=(12,7),sharex=True,sharey=True)
            for ax in axes:boundary(ax,field);ax.set_xlabel('共同局部UTM X / m')
            axes[0].set_ylabel('Y / m')
            if row and row['state']=='SUCCEEDED':
                with gzip.open(out/'jobs'/f"{row['key']}_path.json.gz",'rt') as f:data=json.load(f)
                top=json.loads((out/row['topology_file']).read_text());m=row['independent']
                route(axes[0],data,top,True);route(axes[1],data,top)
                axes[0].set_title('路径按行驶顺序着色，数字为作业行访问编号');axes[1].set_title('蓝：作业 / 橙：连接 / 红：重走 / 紫叉：交叉')
                title=f'C {m["coverage_percent"]:.3f}% / O {m["overlap_percent"]:.3f}% / 规划 {row["planner_wall_seconds"]:.2f}s\n交叉 {top["proper_crossing_locations"]}处 / 额外重走 {top["extra_retraced_length_m"]:.1f}m ({100*top["extra_retraced_fraction"]:.2f}%) / {row["selected"]["layout_family"]}'
                del data,top
            else:
                title=(row or {}).get('state','MISSING')+' / '+str((row or {}).get('reason','未完成'))[:100]
                for ax in axes:ax.text(.5,.5,title,transform=ax.transAxes,ha='center',wrap=True,fontsize=9)
            fig.suptitle(f'{i+1}/50 · {Path(source["file"]).stem} · {source["area_m2"]/1e4:.3f}ha · 孔洞{source["holes"]}\n{title}\n10m等效幅宽，未建模真实机具外伸；离线预测；交叉/共享通道不直接判错',fontsize=10)
            fig.tight_layout(rect=(0,0,1,.85));pdf.savefig(fig,bbox_inches='tight',pad_inches=.15);fig.savefig(images/f'{i+1:02d}_{Path(source["file"]).stem}.png',dpi=160,bbox_inches='tight',pad_inches=.15);plt.close(fig);gc.collect()
    for start in (0,25):
        fig,axes=plt.subplots(5,5,figsize=(17,17))
        for i,ax in zip(range(start,start+25),axes.ravel()):
            source=manifest['fields'][i];row=lookup.get(i);boundary(ax,projected(source));ax.set_xticks([]);ax.set_yticks([])
            if row and row['state']=='SUCCEEDED':
                with gzip.open(out/'jobs'/f"{row['key']}_path.json.gz",'rt') as f:data=json.load(f)
                top=json.loads((out/row['topology_file']).read_text());route(ax,data,top,small=True)
                ax.set_title(f'{i+1}: {row["field"]}\nC{row["independent"]["coverage_percent"]:.2f} / O{row["independent"]["overlap_percent"]:.2f} / X{top["proper_crossing_locations"]}',fontsize=8)
                del data,top
            else:ax.set_title(f'{i+1}: {Path(source["file"]).stem}\n{(row or {}).get("state","MISSING")}',fontsize=8)
        fig.suptitle(f'10m幅宽全样本路径总览 {start+1}–{start+25}；蓝作业/橙连接/红重走/紫交叉',fontsize=14)
        fig.tight_layout(rect=(0,0,1,.97));fig.savefig(out/f'overview_{start+1:02d}_{start+25:02d}.png',dpi=180,bbox_inches='tight',pad_inches=.15);plt.close(fig);gc.collect()
    fig,axes=plt.subplots(2,2,figsize=(14,9),sharex=True)
    successful=[r for r in rows if r['state']=='SUCCEEDED']
    x=[r['index']+1 for r in successful]
    colors=['#cb7737' if r['source']['holes'] else '#277f9d' for r in successful]
    measurements=[('覆盖率 / %',[r['independent']['coverage_percent'] for r in successful]),
                  ('重复覆盖率 / %',[r['independent']['overlap_percent'] for r in successful]),
                  ('优化函数调用用时 / s',[r['planner_wall_seconds'] for r in successful]),
                  ('额外中心线重走 / m',[r['topology']['extra_retraced_length_m'] for r in successful])]
    for ax,(label,values) in zip(axes.ravel(),measurements):
        ax.bar(x,values,color=colors,width=.75)
        for row in rows:
            if row['state']!='SUCCEEDED':ax.axvspan(row['index']+.6,row['index']+1.4,color='#ccc',alpha=.7)
        ax.set_ylabel(label);ax.set_xlim(.2,50.8);ax.grid(axis='y',alpha=.2)
        ax.set_xticks(range(5,51,5))
    if successful:axes[0,0].set_ylim(max(0,min(measurements[0][1])-1),100.3)
    for ax in axes[1]:ax.set_xlabel('固定50块原样本编号；详见CSV')
    fig.suptitle('10m幅宽：覆盖、重复、用时和中心线重走\n蓝色无孔 / 橙色带孔；灰列为未产出完整路径，数值缺失不作零值',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.93));fig.savefig(out/'quality_and_runtime.png',dpi=180,bbox_inches='tight',pad_inches=.15);plt.close(fig);gc.collect()


if __name__=='__main__':main()
