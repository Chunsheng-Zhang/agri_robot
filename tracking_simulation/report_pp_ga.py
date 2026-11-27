#!/usr/bin/env python3
"""Summarize a frozen PP GA result and audit six straight-line P1 values."""
import argparse
import csv
from copy import deepcopy
import json
import hashlib
import math
from pathlib import Path

import numpy as np
from tune_pp import HERE, ROOT, BASELINE, evaluate, policy_check
from validate_pp_lookahead import load_case, HOLDOUT
from simulate import load_path
from agri_path_tracking.controller import Polyline


def straight_units(case):
    p=np.asarray(case['poses']);s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
    k=np.diff(np.unwrap(p[:,2]))/np.maximum(np.diff(s),1e-9)
    edges=np.diff(np.r_[False,abs(k)>.05,False].astype(int))
    arcs=[(a,b) for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)) if s[b]-s[a]>.5]
    result=[]
    for i,(a,b) in enumerate(zip([0]+[b for a,b in arcs],[a for a,b in arcs]+[len(p)-1])):
        origin=p[a,:2];axis=p[b,:2]-origin;length=np.linalg.norm(axis);axis=axis/length
        perpendicular=(p[a:b+1,1]-origin[1])*axis[0]-(p[a:b+1,0]-origin[0])*axis[1]
        if max(abs(perpendicular))>1e-6:raise ValueError('Straight geometry failed verification')
        result.append(dict(line=i+1,origin=origin,axis=axis,length=length,s0=s[a],s1=s[b]))
    return result


def first_crossings(u,e,t,stations):
    answer=[]
    for q in stations:
        indices=np.flatnonzero((u[:-1]<=q)&(u[1:]>=q)&(np.diff(u)>1e-8)&(np.diff(t)<.25))
        if not len(indices):return None
        i=indices[0];answer.append(e[i]+(q-u[i])/(u[i+1]-u[i])*(e[i+1]-e[i]))
    return np.asarray(answer)


def p1_audit(output,case):
    result=[]
    for name in ('v2_0.5_1','probe_0.5_1'):
        raw={}
        for variant in ('baseline','selected'):
            with (output/'examples'/name/variant/'tracking.csv').open(encoding='utf-8-sig') as f:
                raw[variant]=[r for r in csv.DictReader(f) if r['phase']=='TRACKING']
        for definition in ('full_straight','core'):
            for unit in straight_units(case):
                observations={}
                for variant,rows in raw.items():
                    data=np.array([[float(r[k]) for k in ('progress_m','x_true_m','y_true_m','time_s')] for r in rows])
                    keep=(data[:,0]>=unit['s0'])&(data[:,0]<=unit['s1'])
                    if definition=='core':
                        keep&=(data[:,3]>=data[0,3]+5)
                        if unit['line']>1:keep&=(data[:,0]>=unit['s0']+2.)
                        if unit['line']<6:keep&=(data[:,0]<=unit['s1']-1.2)
                        if unit['line']==6:keep&=(data[:,0]<=unit['s1']-1.)
                    data=data[keep];xy=data[:,1:3]-unit['origin']
                    u=xy@unit['axis'];e=(xy[:,1]*unit['axis'][0]-xy[:,0]*unit['axis'][1])*100
                    if len(data)<50:raise ValueError('Too few raw samples for 50-point P1')
                    observations[variant]=(u,e,data[:,3])
                lower=max(0.,*(min(d[0]) for d in observations.values()))
                upper=min(unit['length'],*(max(d[0]) for d in observations.values()))
                stations=np.linspace(lower+1e-7,upper-1e-7,50)
                for variant,(u,e,t) in observations.items():
                    samples=first_crossings(u,e,t,stations)
                    if samples is None:raise ValueError('No contiguous first crossing for P1 station')
                    result.append(dict(scenario=name,definition=definition,line=unit['line'],variant=variant,
                        span_m=upper-lower,p1_cm=float(np.std(samples,ddof=1)),mean_cm=float(np.mean(samples)),
                        rmse_cm=float(np.sqrt(np.mean(samples**2)))))
    (output/'straight_p1.json').write_text(json.dumps(result,indent=2))
    with (output/'straight_p1.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,list(result[0]));w.writeheader();w.writerows(result)
    return result


def extra_paths(output,case,parameters):
    """Postselection checks only; these never alter the chosen GA genes."""
    profile=json.loads((output/'models/v2.json').read_text())
    records=[]
    for radius in (1.,2.):
        points=[(float(x),0.,0.) for x in np.linspace(0.,10.,101)]
        points += [(10+radius*math.sin(a),radius*(1-math.cos(a)),a)
                   for a in np.linspace(0.,math.pi,101)[1:]]
        points += [(float(x),2*radius,math.pi) for x in np.linspace(10.,0.,101)[1:]]
        c=deepcopy(case);c['poses']=points;c['initial']=(0.,.1,math.radians(5.));c['progress']=0.
        c['arcs']=[(10.,10.+math.pi*radius)]
        for speed in (.3,.5):
            pair=dict(radius=radius,speed=speed)
            for variant,p in [('baseline',BASELINE),('selected',parameters)]:
                metrics,_=evaluate(c,profile,p,speed,seed=5)
                pair[variant]=metrics
            pair['rejected_by']=policy_check(pair['selected'],pair['baseline'])
            records.append(pair)
    (output/'additional_paths.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
    return records


def run(output):
    output=Path(output).resolve()
    report=json.loads((output/'result.json').read_text());selected=report['selection'];p=selected['parameters']
    if (output/'report.md').exists():raise ValueError('Refuse to overwrite report')
    case=load_case(HOLDOUT);p1=p1_audit(output,case);extra=extra_paths(output,case,p)
    lines=['# 当前 PP 的 L0/T/K 遗传算法离线搜索','',
        '仅离线修改三个前视系数，没有替换控制律、改任务速度或加载实车参数。以下“最佳”仅指本次范围、约束和冻结模型下的搜索结果，不是全局最优或实车最优。','',
        f"**复核结论：14组成对场景中{sum(r['passed'] for r in report['validation'])}组满足预设限制，仍有直线核心误差或指令波动退化；该候选不作为可直接替换实车的稳定最优参数。**",'',
        '## 搜索结果','',f"候选 L0={p['lookahead']:.4f} m，T={p['lookahead_gain']:.4f} s，K={p['curvature_gain']:.4f} m。",'',
        '```text',f"Ld = clip(({p['lookahead']:.4f} + {p['lookahead_gain']:.4f} × |v|) / (1 + {p['curvature_gain']:.4f} × |κ|), 0.65, 1.05)",'```','',
        '前视上下限0.65/1.05m、曲率窗口1m、虚拟轴距0.16m、后轴偏移0、转角上限35°均固定。任务速度0.30/0.50m/s、最大角速度1rad/s；没有新增速度规划、PID、滤波或前馈。', '',
        f"两个GA种子27/2811，每组12个体、7代，共评估{report['candidates']}组去重参数，执行{report['training_simulations']}次训练闭环（含额外基线核对）、{report['validation_simulations']}次冻结后成对验证及8次额外U型路径检查。",'',
        '实数编码，锦标赛选父代、混合交叉、高斯变异、2个精英及每代1个随机新个体；不复制AutoHarvester示例中的父代别名/旧适应度问题。两个种子并不保证全局收敛，三参数在窄速度范围和限幅下也可能相互抵消。','',
        '## 选择规则','',
        '只用09:01 PP的路径与第一帧位姿搜索；冻结v2及历史PP响应试验模型不重新拟合。训练场景为v2/0.50、PP试验模型/0.50、v2/0.30m/s。目标为三个场景等权的“候选/当前参数”等距进度横向RMSE比值均值。', '',
        '必须完成任务；各场景等距进度RMSE不超过基线1.05倍、P95/峰值不超过1.10倍；直线核心RMSE不超过max(基线×1.15,基线+0.15cm)；角速度指令总变差率不超过1.25倍；平均指令速度至少98%、用时最多105%。这些是预设工程筛选条件，不是国标或所有指标必须改善的承诺。', '',
        f"选中参数的训练目标比值为{selected['objective']:.6f}，相对下降{100*(1-selected['objective']):.2f}%。",'',
        '|训练场景|当前RMSE cm|候选RMSE cm|当前核心RMSE cm|候选核心RMSE cm|','|---|---:|---:|---:|---:|']
    for b,r in zip(report['training_baselines'],selected['results']):
        lines.append(f"|{r['scenario']}|{b['rmse_cm']:.3f}|{r['rmse_cm']:.3f}|{b['regions']['core']['rmse_cm']:.3f}|{r['regions']['core']['rmse_cm']:.3f}|")
    lines += ['', '## 冻结参数后的验证','',
        '09:25仅提供另一初始位姿，后续状态全部由模型计算，不回放实测轨迹。模型、速度、噪声、随机种子均成对一致。该实车路径此前已分析，不是未知地块或前瞻实车验证。','',
        '|场景|当前RMSE cm|候选RMSE cm|变化 %|筛选条件|','|---|---:|---:|---:|---|']
    for r in report['validation']:
        b,a=r['baseline'],r['selected']
        lines.append(f"|{r['scenario']}|{b['rmse_cm']:.3f}|{a['rmse_cm']:.3f}|{100*(a['rmse_cm']/b['rmse_cm']-1):+.2f}|{'满足' if r['passed'] else '超出：'+','.join(r['rejected_by'])}|")
    lines+=['','噪声压力为位置各轴0.03m、航向0.5°标准差；tau场景仅将角响应时间常数乘0.8/1.2，均非测量置信区间。ideal仅作对照，不参加搜索。','',
        '## 额外U型路径（选择冻结后，未参与调参）','',
        '|半径m|速度m/s|当前RMSE cm|候选RMSE cm|超出的筛选项|','|---:|---:|---:|---:|---|']
    for r in extra:
        lines.append(f"|{r['radius']}|{r['speed']}|{r['baseline']['rmse_cm']:.3f}|{r['selected']['rmse_cm']:.3f}|{','.join(r['rejected_by']) or '无'}|")
    lines+=['','U型路径：直线10m—半圆—直线10m；初始横向偏差0.10m、航向偏差5°；仅v2等效模型。不是新增实车试验。','',
        '## 直线P1复核','',
        '几何验证6条直线，每条在当前/候选共有的观测纵向跨度上取50个等距站位，首次向前穿越线性插值，有符号垂距样本标准差（分母49）。全直线保留起步及出弯调整；核心另去前5s、弯前1.2m、弯后2m及末端1m。不是完整国标试验，也不是只看P1就能忽略恒定偏移。','',
        '|模型|范围|当前最大P1 cm|候选最大P1 cm|','|---|---|---:|---:|']
    for name in ('v2_0.5_1','probe_0.5_1'):
        for definition in ('full_straight','core'):
            values=[max(r['p1_cm'] for r in p1 if r['scenario']==name and r['definition']==definition and r['variant']==v)
                    for v in ('baseline','selected')]
            lines.append(f'|{name}|{definition}|{values[0]:.3f}|{values[1]:.3f}|')
    lines+=['','## 局限与文件','',
        '- 同一路径的多次仿真不是多个独立实车证据。0.30m/s PP工况及额外U型均为模型推演。',
        '- 模型来自MGX观测的等效响应，未分离滑移、导航滤波、杆臂等；拟合滞后不是纯MCU通信延迟。',
        '- 搜索范围L0=0.45～1.20m、T=0～1s、K=0～1m；未证明范围外无更优解，也未做GA种群规模收敛研究。',
        '- 结果可能与理想模型的最优方向不同；必须如实查看所有复核场景，不能只选择改善的一组。',
        '- 参数已写入本目录candidate.yaml，项目默认YAML、生产PP和原始实验数据保持原样；没有启动ROS/硬件、运动、系统升级或提交Git。','',
        'protocol.json为运行前协议；search.jsonl/search_summary.csv为全部候选及拒绝原因；ga_history.json为逐代历史；selection.json为验证前冻结的选择；result.json为指标与输入指纹；models/为模型快照；examples/为逐周期CSV和图；straight_p1.csv为各条直线P1。','',
        '![遗传算法收敛](ga_convergence.png)','',
        '![冻结后RMSE](validation_rmse.png)','', '![横向误差与转向指令](validation_comparison.png)','',
        '## 复现','', '```bash', 'cd /home/jetson/agri_robot',
        'TMPDIR="$PWD/tracking_simulation/.cache/tmp" MPLCONFIGDIR="$PWD/tracking_simulation/.cache/matplotlib" OPENBLAS_NUM_THREADS=1 \\',
        '  agri_backend/.venv/bin/python -B tracking_simulation/tune_pp.py \\',
        '  --output tracking_simulation/results/pp_ga_L0_T_K_repeat',
        'TMPDIR="$PWD/tracking_simulation/.cache/tmp" MPLCONFIGDIR="$PWD/tracking_simulation/.cache/matplotlib" OPENBLAS_NUM_THREADS=1 \\',
        '  agri_backend/.venv/bin/python -B tracking_simulation/report_pp_ga.py \\',
        '  tracking_simulation/results/pp_ga_L0_T_K_repeat','```','',
        '仅运行候选参数：','', '```bash',f'bash tracking_simulation/run.bash --config {output.relative_to(ROOT)}/candidate.yaml --controller pp --no-animation','```','']
    (output/'report.md').write_text('\n'.join(lines))
    draw_ga(output,report)
    (output/'report_provenance.json').write_text(json.dumps(dict(
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        result_sha256=hashlib.sha256((output/'result.json').read_bytes()).hexdigest(),
        p1_definition='50 equally spaced common longitudinal stations, signed perpendicular error, sample std ddof=1',
        additional_simulations=8,selection_changed=False),indent=2))
    print(json.dumps(dict(parameters=p,validation_passed=sum(r['passed'] for r in report['validation']),
                         validation_total=len(report['validation']),extra=extra),ensure_ascii=False,indent=2))


def draw_ga(output,report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),constrained_layout=True)
    for item in report['ga_history']:
        axes[0].plot([r['generation'] for r in item['history']],
                     [r['best_cost'] for r in item['history']],marker='o',label='seed '+str(item['seed']))
    axes[0].set(xlabel='Generation',ylabel='Best feasible normalized objective',title='GA convergence (two seeds)')
    axes[0].axhline(1.,color='grey',ls='--',label='Current parameters');axes[0].legend();axes[0].grid(alpha=.25)
    data=[json.loads(line) for line in (output/'search.jsonl').read_text().splitlines()]
    good=[r for r in data if r['eligible']];bad=[r for r in data if not r['eligible']]
    axes[1].scatter([r['parameters']['lookahead'] for r in bad],[r['parameters']['lookahead_gain'] for r in bad],
                     color='lightgrey',marker='x',label='Rejected')
    points=axes[1].scatter([r['parameters']['lookahead'] for r in good],[r['parameters']['lookahead_gain'] for r in good],
                           c=[r['objective'] for r in good],cmap='viridis_r',label='Eligible')
    p=report['selection']['parameters']
    axes[1].scatter([p['lookahead']],[p['lookahead_gain']],marker='*',s=200,color='red',label='Selected')
    axes[1].set(xlabel='L0 (m)',ylabel='T (s)',title='Evaluated candidates (K also varies)')
    axes[1].legend();fig.colorbar(points,ax=axes[1],label='Normalized objective')
    fig.savefig(output/'ga_convergence.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output')
    run(parser.parse_args().output)
