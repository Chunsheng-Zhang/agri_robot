#!/usr/bin/env python3
"""Paired results, figures and a complete 50-field atlas for the offline study."""
import argparse
from collections import Counter
import csv
from datetime import datetime,timezone
import gzip
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
FIELDS=('coverage_percent','overlap_percent','length_m','point_turn_count','score')
LABELS={'coverage_percent':'覆盖率 %','overlap_percent':'重复覆盖率 %','length_m':'路径长度 m',
        'point_turn_count':'停车转向次数','score':'四目标代价 J','planning_seconds':'规划及归档耗时 s'}


def pairs(rows,width):
    lookup={(r['index'],r['method']):r for r in rows if r['width_m']==width}
    return [(lookup[i,'current'],lookup[i,'native']) for i in range(50)
            if (i,'current') in lookup and (i,'native') in lookup
            and lookup[i,'current']['state']==lookup[i,'native']['state']=='SUCCEEDED']


def paired_stats(values):
    values=np.asarray(values,dtype=float)
    if not len(values):return {'n':0,'mean':None,'median':None,'ci95_mean':None}
    ci=None
    if len(values)>1:
        rng=np.random.default_rng(20261114);boots=values[rng.integers(0,len(values),size=(5000,len(values)))].mean(axis=1)
        ci=[float(v) for v in np.quantile(boots,[.025,.975])]
    return {'n':len(values),'mean':float(values.mean()),'median':float(np.median(values)),
            'ci95_mean':ci,'negative':int((values<-1e-8).sum()),'positive':int((values>1e-8).sum()),
            'ties':int((np.abs(values)<=1e-8).sum())}


def summarize(rows,manifest=None):
    def native_label(row):
        selected=row.get('selected') or {}
        return (f'{selected.get("decomposition","UNKNOWN")}@{selected.get("split_angle")} / '
                f'{selected.get("swath_objective","UNKNOWN")} / '+('GUIDED' if selected.get('guided_local_search') else 'AUTO'))
    summary={'generated_at':datetime.now(timezone.utc).isoformat(),'jobs':len(rows),'profiles':{}}
    for width in (.8,6.):
        p=pairs(rows,width);item={'paired_successes':len(p),'status_counts':{},'metrics':{}}
        for method in ('current','native'):
            group=[r for r in rows if r['width_m']==width and r['method']==method]
            counts=Counter(r['state'] for r in group);counts['MISSING']=50-len(group)
            item['status_counts'][method]=dict(counts)
            item[method+'_reason_counts']=dict(Counter(reason for r in group for reason in r.get('rejection_counts',{})))
            item[method+'_failed_job_reason_counts']=dict(Counter(reason for r in group if r['state']=='FAILED' for reason in r.get('rejection_counts',{})))
        for metric in FIELDS+('planning_seconds',):
            left=[(a['planning_seconds'] if metric=='planning_seconds' else a['independent'][metric]) for a,b in p]
            right=[(b['planning_seconds'] if metric=='planning_seconds' else b['independent'][metric]) for a,b in p]
            d=paired_stats(np.asarray(left)-np.asarray(right))
            d.update(current_mean=float(np.mean(left)) if left else None,native_mean=float(np.mean(right)) if right else None)
            if metric in ('length_m','point_turn_count','score','planning_seconds'):
                d['relative_change_percent']=paired_stats([100*(a-b)/b for a,b in zip(left,right) if b>0])
            item['metrics'][metric]=d
        for metric in ('coverage_percent','overlap_percent'):
            areas=[a['independent']['field_area_m2'] for a,b in p]
            item[metric+'_area_weighted']={method:(float(np.average([pair[j]['independent'][metric] for pair in p],weights=areas)) if p else None)
                for j,method in enumerate(('current','native'))}
        dominance=Counter()
        tolerances={'coverage_percent':.01,'overlap_percent':.01,'length_m':.1,'point_turn_count':0.}
        for a,b in p:
            deltas={k:(a['independent'][k]-b['independent'][k])*(-1 if k=='coverage_percent' else 1) for k in tolerances}
            better=all(v<=tolerances[k] for k,v in deltas.items()) and any(v<-tolerances[k] for k,v in deltas.items())
            worse=all(v>=-tolerances[k] for k,v in deltas.items()) and any(v>tolerances[k] for k,v in deltas.items())
            equal=all(abs(v)<=tolerances[k] for k,v in deltas.items())
            dominance['current_dominates' if better else 'native_dominates' if worse else 'near_equal' if equal else 'tradeoff']+=1
        item['paired_pareto']={'counts':dict(dominance),'comparison_tolerances':tolerances,
            'scope':'two selected outputs, not a Pareto proof over all candidates'}
        item['grid_coarsening_risk_counts']={method:sum(bool(r.get('grid_coarser_than_half_width')) for r in rows if r['width_m']==width and r['method']==method and r['state']=='SUCCEEDED') for method in ('current','native')}
        item['grid_vector_differences']={method:{key:paired_stats([r[key] for r in rows if r['width_m']==width and r['method']==method and r['state']=='SUCCEEDED']) for key in ('grid_vector_coverage_difference_pp','grid_vector_overlap_difference_pp')} for method in ('current','native')}
        item['native_winning_combinations']=dict(Counter(
            native_label(r)
            for r in rows if r['width_m']==width and r['method']=='native' and r['state']=='SUCCEEDED'))
        summary['profiles'][str(width)]=item
    if manifest is not None:
        strata={
            'area_le_1ha':[i for i,r in enumerate(manifest['fields']) if r['area_m2']<=1e4],
            'area_1_to_5ha':[i for i,r in enumerate(manifest['fields']) if 1e4<r['area_m2']<=5e4],
            'area_gt_5ha':[i for i,r in enumerate(manifest['fields']) if r['area_m2']>5e4],
            'with_holes':[i for i,r in enumerate(manifest['fields']) if r['holes']>0],
            'without_holes':[i for i,r in enumerate(manifest['fields']) if not r['holes']],
        }
        summary['strata']={}
        for name,indices in strata.items():
            summary['strata'][name]={'sample_count':len(indices),'profiles':{}}
            for width in (.8,6.):
                group=[r for r in rows if r['index'] in indices and r['width_m']==width]
                common=pairs(group,width)
                summary['strata'][name]['profiles'][str(width)]={
                    'successes':{method:sum(r['state']=='SUCCEEDED' and r['method']==method for r in group) for method in ('current','native')},
                    'paired_successes':len(common),
                    'paired_score_delta':paired_stats([a['independent']['score']-b['independent']['score'] for a,b in common])}
    return summary


def write_csv(out,rows):
    with (out/'per_field_results.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        keys=['index','field','width_m','method','state','area_m2','planning_seconds','total_seconds','peak_rss_mb',
              'native_tsp_calls','evaluated_candidates','feasible_candidates','point_count','actual_grid_m',
              'grid_coarser_than_half_width','clearance_passed']
        keys += ['vector_'+k for k in FIELDS]+['grid_'+k for k in FIELDS]
        keys += ['native_decomposition','native_split_angle_rad','native_swath_objective','native_guided_local_search',
                 'current_basis_kind','current_angle_rad','current_fallback_cell_count','swath_count','cell_count','current_supplement','reason','rejection_counts']
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader()
        for r in rows:
            value={k:r.get(k) for k in keys};value.update({'vector_'+k:r.get('independent',{}).get(k) for k in FIELDS})
            value.update({'grid_'+k:r.get('selected',{}).get(k) for k in FIELDS})
            value['area_m2']=r.get('area_m2',r['source']['area_m2']);value['index']=r['index']+1
            selected=r.get('selected') or {};layout=selected.get('row_layout') or {}
            value.update(swath_count=selected.get('swath_count'),cell_count=selected.get('cell_count'))
            if r['method']=='native':
                value.update(native_decomposition=selected.get('decomposition'),native_split_angle_rad=selected.get('split_angle'),
                    native_swath_objective=selected.get('swath_objective'),native_guided_local_search=selected.get('guided_local_search'))
            else:
                value.update(current_basis_kind=(selected.get('basis') or {}).get('kind'),current_angle_rad=selected.get('angle_rad'),
                    current_fallback_cell_count=layout.get('fallback_cell_count'),current_supplement=layout.get('supplement'))
            value['rejection_counts']=json.dumps(r.get('rejection_counts',{}),ensure_ascii=False);writer.writerow(value)


def figures(out,rows,manifest,summary,atlas):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.backends.backend_pdf import PdfPages
    from shapely import wkt
    from shapely.ops import transform
    from shapely.affinity import translate
    from pyproj import Transformer
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(figsize=(9,5));colors={'SUCCEEDED':'#42886b','INVALID_INPUT':'#a9adb1','FAILED':'#c75b58','ERROR':'#b87145'}
    groups=[(.8,'current'),(.8,'native'),(6.,'current'),(6.,'native')];bottom=np.zeros(4)
    states=sorted(set(r['state'] for r in rows))
    for state in states:
        values=[sum(r['state']==state and (r['width_m'],r['method'])==g for r in rows) for g in groups]
        ax.bar(range(4),values,bottom=bottom,color=colors.get(state,'#c49842'),label=state);bottom+=values
    ax.set_xticks(range(4),['0.8m 一键优化','0.8m 原生强基线','6m 一键优化','6m 原生强基线']);ax.set_ylim(0,52)
    ax.set_ylabel('地块数（每组固定50）');ax.set_title('成功、不可行与无效输入分别统计');ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'success_rates.png',dpi=180);fig.savefig(out/'success_rates.svg');plt.close(fig)
    for width in (.8,6.):
        paired=pairs(rows,width);fig,axes=plt.subplots(2,3,figsize=(13,8));fig.subplots_adjust(top=.86,hspace=.35,wspace=.32)
        for ax,key in zip(axes.ravel(),FIELDS+('planning_seconds',)):
            if not paired:ax.text(.5,.5,'无双方成功配对',transform=ax.transAxes,ha='center');ax.set_title(LABELS[key]);continue
            x=np.array([b['planning_seconds'] if key=='planning_seconds' else b['independent'][key] for a,b in paired])
            y=np.array([a['planning_seconds'] if key=='planning_seconds' else a['independent'][key] for a,b in paired])
            ax.scatter(x,y,c='#287c99',s=20,alpha=.8);lo=min(x.min(),y.min());hi=max(x.max(),y.max());pad=max((hi-lo)*.04,1e-6)
            ax.plot([lo-pad,hi+pad],[lo-pad,hi+pad],'--',c='#999',lw=1)
            ax.set_xlabel('原生强基线');ax.set_ylabel('一键优化');ax.set_title(LABELS[key]);ax.grid(alpha=.15)
        fig.suptitle(f'{width:g} m幅宽 · 双方成功的{len(paired)}块地配对\n共用停车原地转向执行模型；覆盖/重复为独立矢量复核\n耗时含候选归档、受并发及单次采样影响',fontsize=12)
        fig.savefig(out/f'paired_{width:g}m.png',dpi=170);fig.savefig(out/f'paired_{width:g}m.svg');plt.close(fig)
    if not atlas:return
    lookup={(r['index'],r['width_m'],r['method']):r for r in rows};images=out/'figures';images.mkdir(exist_ok=True)
    with PdfPages(out/'atlas_50_fields.pdf') as pdf:
        for i,source in enumerate(manifest['fields']):
            raw=wkt.loads((ROOT/source['file']).read_text());projected=transform(Transformer.from_crs(4326,source['epsg'],always_xy=True).transform,raw)
            field=translate(projected,xoff=-source['utm_anchor'][0],yoff=-source['utm_anchor'][1])
            fig,axes=plt.subplots(2,2,figsize=(11,10),sharex=True,sharey=True);fig.subplots_adjust(top=.86,hspace=.34,wspace=.15)
            for ax,(width,method) in zip(axes.ravel(),groups):
                ax.fill(*field.exterior.xy,color='#eef4e8');ax.plot(*field.exterior.xy,c='#333',lw=.9)
                for hole in field.interiors:ax.fill(*hole.xy,color='white');ax.plot(*hole.xy,c='#9b3535',lw=1)
                row=lookup.get((i,width,method));title=f'{width:g}m · '+('一键优化' if method=='current' else 'Fields2Cover原生强基线')
                if row and row['state']=='SUCCEEDED':
                    with gzip.open(out/'jobs'/f'{row["key"]}_path.json.gz','rt') as stream:data=json.load(stream)
                    assert data['frame_id']=='benchmark_local_utm' and data['epsg']==source['epsg']
                    assert data['utm_anchor']==source['utm_anchor']
                    assert wkt.loads(data['field_wkt']).equals_exact(field,1e-8),row['key']
                    x,y=zip(*data['vertices']);ax.plot(x,y,c='#287c99',lw=.55)
                    ax.scatter(x[0],y[0],c='#198b3b',s=16,zorder=5);ax.scatter(x[-1],y[-1],c='#d9443b',s=16,zorder=5)
                    m=row['independent'];title+=f'\nC {m["coverage_percent"]:.3f}% · O {m["overlap_percent"]:.3f}%\nL {m["length_m"]:.1f}m · N {m["point_turn_count"]} · J {m["score"]:.5f}'
                else:
                    state=row['state'] if row else 'MISSING';reasons=(row or {}).get('rejection_counts',{})
                    reason=next(iter(reasons),(row or {}).get('reason','无结果'))
                    ax.text(.5,.45,state+'\n'+reason[:60],transform=ax.transAxes,ha='center',va='center',fontsize=8,wrap=True)
                ax.set_title(title,fontsize=10);ax.set_aspect('equal');ax.grid(alpha=.12);ax.set_xlabel('共同局部UTM X / m');ax.set_ylabel('Y / m')
            fig.suptitle(f'{i+1:02d}/50 · {Path(source["file"]).stem} · {source["area_m2"]/1e4:.3f} ha · 孔洞{source["holes"]}\n共用停车原地转向模型与投影；边界不缩放；绿起点/红终点；离线预测\n6m组仅为幅宽敏感性，车体包络未随幅宽放大',fontsize=11)
            fig.savefig(images/f'{i+1:02d}_{Path(source["file"]).stem}.png',dpi=130);pdf.savefig(fig);plt.close(fig)


def report(out,rows,manifest,summary,partial):
    def n(x,d=4):return '—' if x is None else f'{x:.{d}f}'
    lines=['# 50地块一键优化与Fields2Cover原生强基线对比','',
        '**阶段性结果，尚未完成全部实验。**' if partial else '**完整离线实验：固定50地块、两种幅宽、两种方法，共200项。**','',
        f'生成：{summary["generated_at"]}；随机种子20261114；本地总体350，均匀不放回抽样50，不按结果替换。',
        '主配置0.8m；6m为同一批地块的幅宽尺度对照。车长0.45m、车宽0.2m、余量0.05m、采样0.1m。',
        '50块含49个有效Polygon、1个原始无效Polygon、7个带孔地块；所有原始输入留在分母。','',
        '当前一键规划复用Fields2Cover；本实验比较项目的沿边/补边/统一方向/四目标策略，与原生分区逐角搜索/TSP组合，并不是两个互不相关的底层库。',
        '原生强基线最多比较5种分区×3种条带目标×2种TSP设置，按同一四目标选择集合内最佳可行结果；不是整个库的全局最好。两边共用停车原地转向执行层，不评价Dubins/Reeds–Shepp曲线优劣。','',
        '## 成功率与配对范围','',
        '| 幅宽 | 方法 | 成功/50 | 完整搜索无可行解 | 无效输入 | 错误/外部保护 | 尚未完成 |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for width in (.8,6.):
        item=summary['profiles'][str(width)]
        for method in ('current','native'):
            c=item['status_counts'][method];errors=sum(v for k,v in c.items() if k not in ('SUCCEEDED','FAILED','INVALID_INPUT','MISSING'))
            lines.append(f'| {width:g}m | '+('一键优化' if method=='current' else '原生强基线')+f' | {c.get("SUCCEEDED",0)} | {c.get("FAILED",0)} | {c.get("INVALID_INPUT",0)} | {errors} | {c.get("MISSING",0)} |')
    for width in (.8,6.):
        item=summary['profiles'][str(width)];lines += ['',f'## {width:g}m：双方成功的{item["paired_successes"]}块地配对','',
            '面积指标采用独立连续足迹复核。差值为“一键优化−原生强基线”：覆盖正值较好，其余负值较好。仅比较同一批双方成功地块。','',
            '| 指标 | 一键均值 | 原生均值 | 配对差均值 | 配对差中位数 | 差均值95% bootstrap区间 |',
            '| --- | ---: | ---: | ---: | ---: | --- |']
        for key,value in item['metrics'].items():
            ci=value['ci95_mean'];ci_text='—' if ci is None else '['+', '.join(n(x) for x in ci)+']'
            lines.append(f'| {LABELS[key]} | {n(value["current_mean"])} | {n(value["native_mean"])} | {n(value["mean"])} | {n(value["median"])} | {ci_text} |')
        cost=item['metrics']['score'];lines += ['',f'独立J：一键较低 {cost.get("negative",0)} 块，原生较低 {cost.get("positive",0)} 块，数值相同 {cost.get("ties",0)} 块（阈值1e−8）。',
            f'成功路径中，实际规划网格大于半幅宽的数量：`{json.dumps(item["grid_coarsening_risk_counts"],ensure_ascii=False)}`。',
            '面积加权覆盖/重复均值、相对长度/转弯/评分变化、网格与矢量差异见`summary.json`；不以不同成功子集的均值宣称优越。']
        dominance=item['paired_pareto']['counts']
        lines += ['',f'四项输出的近似Pareto比较：一键占优{dominance.get("current_dominates",0)}块，原生占优{dominance.get("native_dominates",0)}块，各有取舍{dominance.get("tradeoff",0)}块，近似相同{dominance.get("near_equal",0)}块。覆盖/重复容差0.01个百分点、长度0.1m、转向0次；仅比较两个选定输出，不证明候选全集的Pareto最优。']
        lines += ['','原生胜出组合分布（分区@分解角度rad / 条带目标 / TSP设置）：']
        lines += ['- '+name+f'：{count}块' for name,count in sorted(item['native_winning_combinations'].items(),key=lambda x:-x[1])] or ['- 无可行结果']
        for method in ('current','native'):
            lines += ['',('一键优化' if method=='current' else '原生强基线')+'的候选拒绝原因（含成功任务内部被淘汰的候选；作业数可重叠，不等于整项失败数）：']
            reasons=item[method+'_reason_counts']
            lines += ['- '+reason+f'：{count}' for reason,count in sorted(reasons.items(),key=lambda x:-x[1])[:10]] or ['- 无']
            lines += ['','仅完整规划失败作业中的原因标签（可重叠）：']
            reasons=item[method+'_failed_job_reason_counts']
            lines += ['- '+reason+f'：{count}' for reason,count in sorted(reasons.items(),key=lambda x:-x[1])[:10]] or ['- 无']
    lines += ['','## 采样表示容量与规模解释','',
        '当前完整参考路径最多150000点，采样间距0.1m，连续路径长度上限约14999.9m；停车标记和逐段向上取整还会消耗点数。',
        '半径W/2圆盘沿连续折线的足迹并集面积不超过 W×L+π(W/2)²。因此该点数预算下，0.8m幅宽最多表示约1.20ha的100%圆盘覆盖，6m最多约9.00ha；田外足迹、重复和换行会进一步降低有效容量。',
        '这是软件参考路径表示规模，不是车辆物理不能作业的证明，也不将100%变成新增覆盖率门槛。两种算法均未用此估计提前跳过搜索；实际失败仍以完整执行日志为准。逐地块理论上界见`sampling_capacity_bounds.json`。']
    case=out/'case_ee136_explanation.json'
    if case.exists():
        data=json.loads(case.read_text());a=data['current'];b=data['native'];near=data['nearby_current_candidates'][0]
        lines += ['','## 一个事后诊断反例：ee_field_136，6m','',
            f'一键选定约{math.degrees(a["selected"]["angle_rad"]):.3f}°、{a["selected"]["swath_count"]}行、{a["independent"]["point_turn_count"]}次停车转向；原生选定{b["selected"]["swath_count"]}行、{b["independent"]["point_turn_count"]}次转向，原生长直线主方向由路径反推约{data["native_dominant_long_segment_angle_inferred_deg"]:.1f}°。',
            f'当前搜索并未漏掉这个方向邻域：最近的约{math.degrees(near["angle_rad"]):.3f}°候选有{near.get("cell_count")}个分区，其中{(near.get("row_layout") or {}).get("fallback_cell_count")}个采用窄区保留行，转向{near.get("point_turn_count")}次。原生该例不分解、使用NSwath和自动TSP，未依赖guided local search。',
            '这指向分区和窄区必保留一行的策略差异，而不是简单的方向候选不足。尚未做逐因素消融，不能把全部差额归给单一因素；这是事后解释，不参与主统计或调参。完整证据见`case_ee136_explanation.json`，对应全量图册第9页。']
    if summary.get('strata'):
        lines += ['','## 面积与孔洞分层','',
            '| 分层 | 样本数 | 0.8m一键/原生成功 | 6m一键/原生成功 |',
            '| --- | ---: | ---: | ---: |']
        names={'area_le_1ha':'≤1ha','area_1_to_5ha':'1–5ha','area_gt_5ha':'>5ha','with_holes':'含孔洞','without_holes':'无孔洞'}
        for name,item in summary['strata'].items():
            a=item['profiles']['0.8']['successes'];b=item['profiles']['6.0']['successes']
            lines.append(f'| {names[name]} | {item["sample_count"]} | {a["current"]}/{a["native"]} | {b["current"]}/{b["native"]} |')
        lines += ['','面积层互斥；孔洞层为另一套分组，不将各行相加作为总样本数。分层配对J差见`summary.json`。']
    convergence=out/'convergence_checks.json'
    if convergence.exists():
        checks=[c for c in json.loads(convergence.read_text())['checks'] if c['state']=='SUCCEEDED']
        if checks:
            lines += ['','## 独立矢量数值检查','',
                f'固定样本索引0/10/20/30/40中，有结果的{len(checks)}条路径从每象限32段加密到64段，最大覆盖差{max(abs(c["coverage_difference_pp"]) for c in checks):.6f}个百分点，最大重复差{max(abs(c["overlap_difference_pp"]) for c in checks):.6f}个百分点；不根据结果重新选路径。明细见`convergence_checks.json`。']
    lines += ['','## 方法与解释限制','',
        '- 权重固定覆盖0.40、重复0.25、长度0.20、停车转向0.15；L0=A/W，N0=max(1,2√A/W)。规划J与独立复核J分别保存，不能混用排序口径。',
        '- 所有连接按机具开启的圆盘足迹估计，分母为整田扣孔洞、含地头；不是实测农艺质量、能耗、压实或真实跟踪精度。',
        '- 独立矢量评价采用每象限32段近似圆盘，重复按连续经过区间；不把同一连续转弯/相邻定位样本当重复经过。去除前进共线采样时保留折返，禁止普通RDP简化抹除往返。',
        '- WKT按EPSG:4326经纬度解释，逐地块UTM投影，两个方法共用同一平移基准，无几何缩放；不改变机器人实际map原点。',
        '- 6m组只改变抽象作业幅宽，车体包络保持0.45×0.2m；未建模真实6m刚性机具外伸，不能据此声称其实际地头转弯安全。',
        '- 原有1500行、768路由图节点估计、150000路径采样点等保护保留。原始无效、原资源限制、几何安全拒绝和额外实验进程保护分开记录。',
        '- 每项仅运行一次；4进程各2CPU并发，耗时包含候选归档，不包含矢量复核；受主机负载与原生1秒启发式求解波动影响，不作为严格串行耗时或确定性证明。',
        '- 配对bootstrap为探索性区间，固定5000次重采样、种子20261114；小样本区间尤其不代表普遍结论，未进行实车验证。','',
        '## 文件与复现','',
        '- `manifest.json`：50地块名单、源SHA256、有效性、投影/共同基准。',
        '- `configuration.json`：冻结参数、算法/基线源文件指纹、候选范围和资源保护。',
        '- `per_field_results.csv` / `results.json`：所有作业结果、失败原因、规划网格与独立指标；缺失项为空而非零。',
        '- `jobs/*_path.json.gz`：完整位姿、压缩后的保真折线、原始米制地块及安全包络。',
        '- `jobs/*_optimization.json.gz`：全部候选与拒绝原因；`jobs/*.log`：逐项进度。',
        '- `atlas_50_fields.pdf`及`figures/*.png`：全部50地块四面板对照，不仅展示好看的样例。','',
        '方案与出处：项目`docs/2026-11-14_50地块对比实验方案.md`；数据集[发布页](https://zenodo.org/records/14524735)，原生算法[官方条带教程](https://fields2cover.github.io/source/tutorials/swath_generator.html)。','',
        '![成功与失败分类](success_rates.png)','',
        '![0.8m配对](paired_0.8m.png)','',
        '![6m配对](paired_6m.png)','']
    (out/('partial_report.md' if partial else 'report.md')).write_text('\n'.join(lines),encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=ROOT/'.diagnostics/fields50_20261114')
    p.add_argument('--partial',action='store_true');p.add_argument('--atlas',action='store_true');args=p.parse_args();out=args.out
    rows=json.loads((out/'results.json').read_text());manifest=json.loads((out/'manifest.json').read_text())
    if not args.partial:assert len(rows)==200 and len({r['key'] for r in rows})==200,'Experiment incomplete'
    summary=summarize(rows,manifest);(out/('partial_summary.json' if args.partial else 'summary.json')).write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    write_csv(out,rows);figures(out,rows,manifest,summary,args.atlas);report(out,rows,manifest,summary,args.partial)
    print(json.dumps({'jobs':len(rows),'paired':{k:v['paired_successes'] for k,v in summary['profiles'].items()}},ensure_ascii=False))

if __name__=='__main__':main()
