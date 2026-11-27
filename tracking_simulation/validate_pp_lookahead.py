#!/usr/bin/env python3
"""Offline paired validation of an experimental PP speed/curvature lookahead.

No ROS, device, Web, production-controller or default-YAML changes. The default
K=0.25 hypothesis is declared before evaluation. A separate sensitivity choice
uses only the first PP case; the second case is used after that choice freezes.
"""
import argparse
from copy import deepcopy
import csv
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
from zoneinfo import ZoneInfo

HERE=Path(__file__).resolve().parent
for key,relative in (('TMPDIR','.cache/tmp'),('MPLCONFIGDIR','.cache/matplotlib')):
    os.environ.setdefault(key,str(HERE/relative));Path(os.environ[key]).mkdir(parents=True,exist_ok=True)

import numpy as np
from campaign_data import csv_rows,read_campaign_run,ROOT
from campaign_model import fit_moving_points,replay_response
from ground_model import load_profile
from pp_curvature_lookahead import CurvaturePursuit,LookaheadRule,SpeedPursuitBaseline
from simulate import SimulationConfig,load_path,simulate,write_outputs

TRAIN='20261127_090159_929630_web_05f93e4c'
HOLDOUT='20261127_092522_737619_web_45341384'
GRID=(0.,.1,.15,.25,.4,.6)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_case(name):
    directory=ROOT/'data/experiments'/name
    mission_file=directory/'config/mission.json';ctrl_file=directory/'config/tracking_controller.json'
    mission=json.loads(mission_file.read_text());ctrl=json.loads(ctrl_file.read_text())
    if ctrl['algorithm']!='pp':raise ValueError('仅接受冻结PP实验')
    path_file=next((directory/'paths').glob('coverage_*.json'))
    track_file=next((directory/'tracks').glob('track_*.csv'))
    poses,source=load_path(path_file);real=csv_rows(track_file)
    p=np.asarray(poses);stations=np.r_[0,np.cumsum(np.hypot(*np.diff(p[:,:2],axis=0).T))]
    curvature=np.diff(np.unwrap(p[:,2]))/np.maximum(np.diff(stations),1e-9)
    edges=np.diff(np.r_[False,abs(curvature)>.05,False].astype(int))
    arcs=[(float(stations[a]),float(stations[b])) for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)) if stations[b]-stations[a]>.5]
    return dict(name=name,directory=directory,mission=mission,controller=ctrl,poses=poses,source=source,real=real,arcs=arcs,
        initial=tuple(float(real[0][k]) for k in ('x','y','yaw')),progress=float(real[0]['progress']),
        source_sha256={str(f.relative_to(ROOT)):sha(f) for f in (mission_file,ctrl_file,path_file,track_file)})


def rms(a):return float(np.sqrt(np.mean(np.asarray(a)**2))) if len(a) else None


def score(rows,metrics,case):
    formal=[r for r in rows if r['phase']=='TRACKING']
    error=np.array([r['cross_track_true_m'] for r in formal]);s=np.array([r['progress_m'] for r in formal])
    times=np.array([r['time_s'] for r in formal]);w=np.array([r['w_command_rad_s'] for r in formal])
    phase=np.full(len(s),'core',dtype=object)
    for a,b in case['arcs']:
        phase[(s>=a-1.2)&(s<a)]='entry';phase[(s>=a)&(s<=b)]='arc';phase[(s>b)&(s<=b+2)]='exit'
    phase[s>metrics['reference_length_m']-1.]='endpoint'
    phase[times<times[0]+5]='startup'
    keep=np.r_[np.diff(s)>1e-8,True]
    uniform=np.arange(.5,metrics['reference_length_m']-.3,.1)
    sampled=np.interp(uniform,s[keep],error[keep])
    return dict(state=metrics['state'],rmse_cm=rms(error)*100,p95_cm=float(np.quantile(abs(error),.95))*100,
        max_cm=float(max(abs(error)))*100,heading_rmse_deg=metrics['heading_error_true_deg']['rmse'],
        after5_rmse_cm=rms(error[phase!='startup'])*100,common_progress_rmse_cm=rms(sampled)*100,
        regions={name:dict(n=int(sum(phase==name)),rmse_cm=None if rms(error[phase==name]) is None else rms(error[phase==name])*100)
            for name in ('core','entry','arc','exit','startup','endpoint')},
        angular_limit_fraction=float(np.mean(abs(w)>=case['mission']['parameters']['max_angular']-1e-6)),
        angular_command_total_variation_per_s=float(sum(abs(np.diff(w)))/max(times[-1]-times[0],1e-9)),
        command_v_mean_m_s=float(np.mean([r['v_command_m_s'] for r in formal])),
        duration_s=metrics['elapsed_simulation_s'],endpoint_distance_m=metrics['final_endpoint_distance_m'])


def evaluate(case,profile,variant,gain,seed,*,output=None,noise=False):
    params=case['mission']['parameters']
    cfg=SimulationConfig(**{k:params[k] for k in ('lookahead','speed','max_angular','completion_tolerance','search_window')},
        dt=.05,duration=260.,feedback_period=0. if profile is None else profile['feedback_period_s'],
        initial_lateral=0.,initial_heading_deg=0.,seed=seed,
        position_noise=.03 if noise else 0.,heading_noise_deg=.5 if noise else 0.)
    holder={}
    def factory(path,**settings):
        if variant=='original':tracker=SpeedPursuitBaseline(path,tuning=case['controller']['tuning'],**settings)
        else:tracker=CurvaturePursuit(path,rule=LookaheadRule(curvature_gain_m=gain),tuning=case['controller']['tuning'],**settings)
        holder['tracker']=tracker
        return tracker
    rows,metrics=simulate(case['poses'],cfg,initial_pose=case['initial'],initial_progress=case['progress'],
        controller_factory=factory,model_profile=profile)
    result=score(rows,metrics,case)
    tracker=holder['tracker']
    if hasattr(tracker,'lookahead_history'):
        h=np.asarray(tracker.lookahead_history)
        result['lookahead_m']={k:float(v) for k,v in zip(('min','median','max'),np.quantile(h[:,1],[0,.5,1]))}
    if output is not None:
        metrics.update(source=case['source'],experimental_lookahead=None if variant=='original' else asdict(tracker.rule))
        write_outputs(output,case['poses'],rows,metrics)
        if hasattr(tracker,'lookahead_history'):
            with (output/'lookahead.csv').open('w',newline='') as f:
                writer=csv.writer(f);writer.writerow(['progress_m','lookahead_m','reference_curvature_rad_m']);writer.writerows(tracker.lookahead_history)
    return result,rows


def run(output,seeds=(0,1,2)):
    output=Path(output).resolve()
    if not output.is_relative_to(HERE/'results') or output==HERE/'results':raise ValueError('结果只写入仿真results独立子目录')
    if output.exists() and any(output.iterdir()):raise ValueError('拒绝覆盖已有结果')
    output.mkdir(parents=True,exist_ok=True)
    v2,profile_source=load_profile(HERE/'profiles/x3_hard_ground_v2.json')
    train_data=read_campaign_run(ROOT/'data/experiments'/TRAIN)
    pp=deepcopy(v2)
    # Only two moving angular parameters are fitted to the first PP velocity
    # record. No lateral tracking-error objective, held-out data or formula is
    # involved. Other response/timing parameters remain frozen from v2.
    pp['moving_angular_points']=fit_moving_points([train_data],v2)
    pp['name']='PP velocity-response probe: v2 with angular gain/tau from first PP run only'
    pp['pp_angular_training_runs']=[TRAIN];pp['pp_angular_holdout_runs']=[HOLDOUT]
    (output/'pp_probe_profile.json').write_text(json.dumps(pp,ensure_ascii=False,indent=2))
    first=load_case(TRAIN)
    sensitivity=[]
    for gain in GRID:
        scores=[]
        for seed in (0,1):
            result,_=evaluate(first,pp,'formula',gain,seed)
            result.update(gain=gain,seed=seed);sensitivity.append(result);scores.append(result)
        print('sensitivity',gain,[(r['state'],round(r['rmse_cm'],3)) for r in scores],flush=True)
    eligible=[gain for gain in GRID if all(r['state']=='COMPLETED' for r in sensitivity if r['gain']==gain)]
    if not eligible:raise ValueError('灵敏度扫描没有完成的候选')
    selected=min(eligible,key=lambda gain:np.mean([r['common_progress_rmse_cm'] for r in sensitivity if r['gain']==gain]))
    selection=dict(declared_primary_gain=.25,selected_gain=selected,grid=GRID,
        selection_case=TRAIN,selection_seeds=[0,1],objective='minimum mean common-progress RMSE; completed only',
        note='Selection frozen before second case evaluation; both recorded paths were already inspected during hypothesis design, so not a prospectively unseen field trial')
    (output/'selection.json').write_text(json.dumps(selection,ensure_ascii=False,indent=2))
    # Held-out velocity observations are first loaded after gain/tau and the
    # sensitivity choice freeze. They never update either fitted parameter.
    second=load_case(HOLDOUT);holdout_data=read_campaign_run(ROOT/'data/experiments'/HOLDOUT)
    velocity=[]
    for data in (train_data,holdout_data):
        item=dict(run=data.name)
        for name,p in (('v2',v2),('pp_probe',pp)):
            prediction=replay_response(data,p)
            item[name]=dict(zip(('linear_rmse_m_s','angular_rmse_rad_s'),map(float,np.sqrt(np.mean((prediction[data.valid]-data.measured[data.valid])**2,axis=0)))))
        velocity.append(item)
    records=[];examples={}
    variants=[('original',0.),('bounded_zero',0.),('proposed',.25)]
    if selected not in (0.,.25):variants.append(('selected',selected))
    for case in (first,second):
        for model,p in (('ideal',None),('frozen_v2',v2),('pp_probe',pp)):
            for seed in seeds:
                for variant,gain in variants:
                    save=output/case['name']/model/variant if model=='pp_probe' and seed==seeds[0] and variant!='bounded_zero' else None
                    result,rows=evaluate(case,p,variant,gain,seed,output=save)
                    result.update(run=case['name'],model=model,variant=variant,gain=gain,seed=seed,noise=False)
                    records.append(result)
                    if seed==seeds[0]:examples[(case['name'],model,variant)]=rows
                    print('paired',case['name'][9:15],model,variant,seed,result['state'],round(result['rmse_cm'],3),flush=True)
        # Unfitted noise stress is separate from zero-noise primary results.
        for seed in seeds:
            for variant,gain in (('original',0.),('proposed',.25)):
                result,_=evaluate(case,pp,variant,gain,seed,noise=True)
                result.update(run=case['name'],model='pp_probe_noise_stress',variant=variant,gain=gain,seed=seed,noise=True)
                records.append(result)
    # Model uncertainty scenarios are exploratory, not fitted confidence bounds.
    for multiplier in (.8,1.2):
        p=deepcopy(pp)
        for point in p['moving_angular_points']:point['tau_s']*=multiplier
        for variant,gain in (('original',0.),('proposed',.25)):
            result,_=evaluate(second,p,variant,gain,0)
            result.update(run=HOLDOUT,model=f'pp_probe_angular_tau_x{multiplier}',variant=variant,gain=gain,seed=0,noise=False)
            records.append(result)
    for a in records:
        if a['variant']!='bounded_zero':continue
        original=next(r for r in records if r['run']==a['run'] and r['model']==a['model'] and r['seed']==a['seed'] and r['variant']=='original')
        if abs(a['rmse_cm']-original['rmse_cm'])>1e-10:raise ValueError('K=0的消融未还原原版，比较被其他变化混淆')
    sources={str(p.relative_to(ROOT)):sha(p) for p in (Path(__file__),HERE/'pp_curvature_lookahead.py',
        HERE/'four_controler/four_controler.py',HERE/'simulate.py',HERE/'campaign_model.py',HERE/'campaign_data.py',HERE/'ground_model.py',HERE/'observation.py')}
    raw={**train_data.metadata['source_sha256'],**holdout_data.metadata['source_sha256'],**first['source_sha256'],**second['source_sha256']}
    report=dict(selection=selection,formula='clip((L0 + Tv*abs(v))/(1+K*abs(reference_curvature)), .65, 1.05)',
        curvature='tangent change over fixed 1.0m forward reference arc-length window; independent of steering and variable lookahead',
        speed_policy='same 0.5m/s upper request; existing angular/endpoint speed constraints preserved; no new speed planner',
        source_sha256=raw,implementation_sha256=sources,base_profile_source=profile_source,model_velocity_validation=velocity,
        cases=[dict(name=c['name'],initial_pose=c['initial'],initial_progress=c['progress'],parameters=c['mission']['parameters']) for c in (first,second)],
        sensitivity=sensitivity,paired_results=records,
        limitations=['Simulation-only evidence, no new real-vehicle run or independent truth',
            'PP probe fits angular gain/tau to first run only; linear/in-place/lateral/timing parameters inherited from LQR-based v2',
            'Two PP records have same path; second case is a different initial state, not an independent path family',
            'No closed-loop trajectory replay, waypoint snapping or measured-state resets',
            'Unknown timing/slip/GNSS effects remain; default K=.25 was an empirical hypothesis, not a published proven law',
            'Noise and tau multipliers are scenario stresses, not measured noise or statistical confidence intervals'])
    (output/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    fields=['run','model','variant','gain','seed','state','rmse_cm','common_progress_rmse_cm','p95_cm','max_cm','heading_rmse_deg','angular_limit_fraction','angular_command_total_variation_per_s','command_v_mean_m_s','duration_s']
    with (output/'comparison.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fields,extrasaction='ignore');writer.writeheader();writer.writerows(records)
    draw(output,report,examples,first,second)
    for path,digest in raw.items():
        if sha(ROOT/path)!=digest:raise ValueError('分析期间原始输入改变')
    return report


def draw(output,report,examples,first,second):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(16,8),constrained_layout=True)
    for i,c in enumerate((first,second)):
        a=np.array([[float(r[k]) for k in ('progress','cross_track')] for r in c['real']])
        for j,model in enumerate(('ideal','frozen_v2','pp_probe')):
            ax=axes[i,j];ax.plot(a[:,0],a[:,1]*100,label='Recorded real PP (reference only)',alpha=.45,color='grey')
            for variant in ('original','proposed','selected'):
                rows=examples.get((c['name'],model,variant))
                if rows is None:continue
                rows=[r for r in rows if r['phase']=='TRACKING']
                ax.plot([r['progress_m'] for r in rows],[r['cross_track_true_m']*100 for r in rows],label=variant,lw=.8)
            ax.set(title=c['name'][9:15]+' | '+model,xlabel='Reference progress (m)',ylabel='Lateral error (cm)');ax.grid(alpha=.25);ax.legend(fontsize=7)
    fig.suptitle('Free closed-loop PP lookahead comparison | seed 0 | simulation is not real-vehicle truth')
    fig.savefig(output/'paired_error_comparison.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,5),constrained_layout=True)
    for seed in (0,1):
        data=[r for r in report['sensitivity'] if r['seed']==seed]
        ax.plot([r['gain'] for r in data],[r['common_progress_rmse_cm'] for r in data],'-o',label='first case, seed '+str(seed))
    ax.axvline(.25,color='grey',ls='--',label='proposed K=0.25');ax.set(xlabel='Curvature gain K (m)',ylabel='Common-progress lateral RMSE (cm)',title='Sensitivity on selection case only | second case excluded');ax.grid(alpha=.25);ax.legend()
    fig.savefig(output/'gain_sensitivity.png',dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--seeds',type=int,nargs='+',default=[0,1,2])
    args=parser.parse_args()
    if not args.seeds or len(set(args.seeds))!=len(args.seeds):parser.error('种子需非空且不能重复')
    output=args.output or HERE/'results'/('pp_lookahead_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S'))
    run(output,args.seeds);print('结果：'+str(output),flush=True)


if __name__=='__main__':main()
