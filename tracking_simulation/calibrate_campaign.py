#!/usr/bin/env python3
"""从11月27日地面数据校准仿真；留出实验不参与参数拟合。无ROS/硬件操作。"""
import argparse
from copy import deepcopy
from dataclasses import replace
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
from campaign_data import (ROOT, BASIC, TRAIN_NAMES, HOLDOUT_NAMES, controller_timing,
                           csv_rows, read_campaign_run, fingerprint)
from campaign_model import (estimate_delivery_distribution, fit_moving_points, fit_response,
                            replay_response, validate_campaign_profile)
from ground_model import load_profile, pose_window_check, replay_channel
from simulate import SimulationConfig, load_path, simulate, write_outputs
from run_tracking import read_config
from four_controler.four_controler import build_controller


def initial_profile():
    return dict(schema_version=2,kind='X3_GROUND_EQUIVALENT_RESPONSE',name='X3 hard ground navigation-equivalent v2',
        linear=dict(gain=.93,limit=1.,tau_s=.4,delay_s=.1),
        angular=dict(gain=.85,limit=1.5,tau_s=.8,delay_s=.05),
        in_place_angular=dict(gain=.9,limit=1.5,tau_s=.7,delay_s=.8),in_place_order=2,
        stop_tau_s=[.04,.04],lateral_gain_s=0.,feedback_period_s=.1,feedback_timeout_s=.5,
        observation_delay_samples_s=[.01],control_interval_samples_s=[.05],controller_start_phase_s=.025)


def identify(train_paths,validation_paths,basic_path=BASIC):
    train_paths=[Path(p).resolve() for p in train_paths];validation_paths=[Path(p).resolve() for p in validation_paths]
    if not train_paths or not validation_paths or set(train_paths)&set(validation_paths):
        raise ValueError('辨识集/验证集必须非空且互不重合')
    if len(set(train_paths+validation_paths))!=len(train_paths+validation_paths):raise ValueError('重复实验')
    if Path(basic_path).resolve() in set(train_paths+validation_paths):raise ValueError('响应记录不能冒充独立覆盖验证')
    train=[read_campaign_run(p) for p in train_paths]
    basic=read_campaign_run(basic_path,allow_basic=True)
    if any(r.metadata['projection']!=train[0].metadata['projection'] for r in train+[basic]):raise ValueError('原点不一致')
    # No held-out observations are loaded before all parameters are frozen.
    p=fit_response(train+[basic],initial_profile())
    p['moving_angular_points']=fit_moving_points(train,p)
    phases=[]
    with (Path(basic_path)/'telemetry/status.jsonl').open() as stream:
        for line in stream:
            e=json.loads(line)
            if e['topic']=='/experiment/phase':phases.append(e)
    ratios=[]
    for phase in phases:
        command=phase['data'];w=command['angular']
        if not w or command['linear']:continue
        end=(phase['received_ros_ns']-basic.source_origin_ns)*1e-9+command['duration_s']
        mask=(basic.time>=end-.5)&(basic.time<end)
        ratios.extend((basic.measured[mask,1]/w).tolist())
    p['in_place_angular']['gain']=float(np.median(ratios))
    p['in_place_gain_note']='训练短平台末0.5s的表观增益固定为先验；4s平台不足以独立确认DC增益，不联合任意放大增益/时间常数'
    p=fit_response([basic],p,in_place=True)
    x=np.concatenate([(r.measured[:,0]*r.measured[:,1])[r.valid] for r in train])
    y=np.concatenate([r.lateral[r.valid] for r in train])
    p['lateral_gain_s']=float(np.clip(x@y/max(x@x,1e-12),-2,2))
    timing=[controller_timing(path) for path in train_paths]
    p['observation_delay_samples_s'],p['timing_identification']=estimate_delivery_distribution(timing)
    p['controller_start_phase_s']=float(np.median(np.concatenate([r['sample_to_next_tick_s'] for r in timing])))
    p['control_interval_samples_s']=np.concatenate([np.quantile(r['control_intervals_s'],np.linspace(.005,.995,51)) for r in timing]).tolist()
    p['feedback_period_s']=float(np.median([r.metadata['feedback_period_median_s'] for r in train]))
    p['training_runs']=[r.name for r in train]+[basic.name]
    p['validation_runs']=[path.name for path in validation_paths]
    p['training_source_sha256']={r.name:r.metadata['source_sha256'] for r in train+[basic]}
    p['timing_matching']=[dict(run=r.name,matched=t['matched'],total=t['total']) for r,t in zip(train,timing)]
    p['model_scope']=dict(surface='X3 outdoor hard ground',speed_m_s=[.15,.5],
        moving_turn_speed_knots=[x['speed'] for x in p['moving_angular_points']],max_angular_command=1.,
        limit_identified=False,observation='MGX navigation-equivalent; no independent truth')
    p['limits']=[
        '模型预测的是MGX导航观测下的等效运动，不是已分离的真实车体/轮胎/GNSS动力学。',
        '转向反馈采用航向的后向差分，辨识显式比较区间平均角速度；仿真积分使用瞬时模型输出。',
        '指令优先采用真实串口TX主机时间，早期记录回退到headerless驱动话题接收时间；TX不等于MCU确认。',
        '定位交付由唯一LQR输出匹配推断可用时间区间，区间内分布不唯一；1ms网格不是1ms测量精度。',
        '归零、原地转向与行进转向分开；原地表观增益来自有限短平台，不称稳态标定。',
        '横向项是导航观测下的等效速度关联，不可直接解释为物理滑移或杆臂标定。',
        '限幅仅为模型数值边界，没有充分饱和激励，不称机械速度上限。',
        '不支持已验证后退、横移、土壤、更换底盘/载荷、电池状态外推或0.5m/s以上速度。',
        '人工干预且收尾FAULT的长转向组整体不参加本轮辨识或主要验证。',
        '单次仿真或算法排名不保证实车精度；改善须跨工况、种子和留出数据检验后再实车验证。']
    validate_campaign_profile(p)
    validation=[read_campaign_run(path) for path in validation_paths]
    if any(r.metadata['projection']!=train[0].metadata['projection'] for r in validation):raise ValueError('验证原点不一致')
    return p,train,validation,basic,timing


def rms(a):return float(np.sqrt(np.mean(np.asarray(a)**2)))


def compare_closed_loop(directory,p,seeds,*,output=None):
    d=Path(directory);params=json.loads((d/'config/mission.json').read_text())['parameters']
    poses,source=load_path(next((d/'paths').glob('coverage_*.json')))
    real=csv_rows(next((d/'tracks').glob('track_*.csv')))
    initial=tuple(float(real[0][k]) for k in ('x','y','yaw'))
    base=SimulationConfig(**{k:params[k] for k in ('lookahead','speed','max_angular','completion_tolerance','search_window')},
        feedback_period=.1,initial_lateral=0.,initial_heading_deg=0.)
    tuning=read_config(HERE/'config.yaml')[0]['tuning']
    snapshot=json.loads((d/'config/tracking_controller.json').read_text())
    if snapshot['algorithm']!='lqr':raise ValueError('本次闭环对照只接受冻结LQR记录')
    tuning['lqr']=snapshot['tuning']
    v1,_=load_profile(HERE/'profiles/x3_hard_ground_v1.json')
    stations=np.arange(.5,61.2,.1)
    def sample_progress(progress,values):
        progress=np.asarray(progress);keep=np.r_[np.diff(progress)>1e-8,True]
        return np.interp(stations,progress[keep],np.asarray(values)[keep])
    actual_lat=np.array([float(r['cross_track']) for r in real])
    actual_s=np.array([float(r['progress']) for r in real])
    actual_uniform=sample_progress(actual_s,actual_lat)
    results=[];plot_rows={}
    for label,profile,seedlist in (('ideal',None,[0]),('v1',v1,[0]),('v2',p,seeds)):
        for seed in seedlist:
            cfg=replace(base,seed=seed,feedback_period=0. if label=='ideal' else p['feedback_period_s'])
            rows,m=simulate(poses,cfg,initial_pose=initial,initial_progress=float(real[0]['progress']),
                controller_factory=build_controller('lqr',dt=cfg.dt,tuning=tuning),model_profile=profile)
            formal=[r for r in rows if r['phase']=='TRACKING']
            sampled=sample_progress([r['progress_m'] for r in formal],[r['cross_track_true_m'] for r in formal])
            result=dict(model=label,seed=seed,state=m['state'],lateral_rmse_cm=m['cross_track_true_m']['rmse']*100,
                common_progress_rmse_cm=rms(sampled)*100,common_progress_error_trace_gap_cm=rms(sampled-actual_uniform)*100,
                real_lateral_rmse_cm=rms(actual_lat)*100,real_common_progress_rmse_cm=rms(actual_uniform)*100)
            result['rmse_gap_cm']=abs(result['lateral_rmse_cm']-result['real_lateral_rmse_cm'])
            results.append(result)
            if seed==seedlist[0]:plot_rows[label]=(rows,m)
            if output is not None and label=='v2' and seed==seedlist[0]:
                m['source']=source;write_outputs(Path(output)/'closed_loop',poses,rows,m)
    return results,plot_rows,real,poses


def export(output,p,train,validation,basic,timing,seeds):
    output=Path(output).resolve()
    if not output.is_relative_to((HERE/'results').resolve()) or output==(HERE/'results').resolve():
        raise ValueError('产物只能写入tracking_simulation/results独立子目录')
    if output.exists() and any(output.iterdir()):raise ValueError('输出目录非空，拒绝覆盖')
    output.mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    v1,_=load_profile(HERE/'profiles/x3_hard_ground_v1.json')
    scores=[]
    for split,runs in (('training',train),('validation',validation),('response_training',[basic])):
        for r in runs:
            directory=output/r.name;directory.mkdir()
            pred=replay_response(r,p);instant=replay_response(r,p,observed=False)
            old=np.column_stack([replay_channel(r,i,v1[k]) for i,k in enumerate(('linear','angular'))])
            idx=np.maximum(0,np.searchsorted(r.command_time,r.time,side='right')-1);ideal=r.commands[idx]
            metrics={}
            for i,k in enumerate(('linear','angular')):
                metrics[k]={label:rms(a[r.valid,i]-r.measured[r.valid,i]) for label,a in [('ideal',ideal),('v1',old),('v2',pred)]}
            lateral=p['lateral_gain_s']*instant[:,0]*instant[:,1]
            pose,_=pose_window_check(r,instant,ideal,lateral_prediction=lateral)
            oldpose,_=pose_window_check(r,old,ideal)
            result=dict(run=r.name,split=split,velocity_rmse=metrics,pose_window_v2=pose,pose_window_v1=oldpose,metadata=r.metadata)
            fig,axes=plt.subplots(2,1,figsize=(13,7),constrained_layout=True)
            for i,ax in enumerate(axes):
                ax.plot(r.time,r.measured[:,i],label='MGX body v' if i==0 else 'MGX heading difference',lw=1.)
                ax.plot(r.time,old[:,i],label='Previous v1',alpha=.6,lw=.8)
                ax.plot(r.time,pred[:,i],label='New v2 (observed equivalent)',alpha=.8,lw=.8)
                ax.set(xlabel='Host source time (s)',ylabel='m/s' if i==0 else 'rad/s');ax.legend();ax.grid(alpha=.25)
            fig.suptitle(split+' | '+r.name+' | open-loop replay, no state resets')
            fig.savefig(directory/'response_replay.png',dpi=150);plt.close(fig)
            if split!='response_training':
                results,simrows,real,poses=compare_closed_loop(ROOT/'data/experiments'/r.name,p,seeds,output=directory)
                result['closed_loop']=results
                fig,axes=plt.subplots(1,2,figsize=(13,6),constrained_layout=True)
                axes[0].plot([a[0] for a in poses],[a[1] for a in poses],'k--',label='Reference')
                axes[0].plot([float(x['x']) for x in real],[float(x['y']) for x in real],label='Real MGX',lw=1.)
                axes[1].plot([float(x['progress']) for x in real],[100*float(x['cross_track']) for x in real],label='Real MGX',lw=1.)
                for label,(rows,_) in simrows.items():
                    rows=[x for x in rows if x['phase']=='TRACKING']
                    axes[0].plot([x['x_true_m'] for x in rows],[x['y_true_m'] for x in rows],label=label,lw=.8)
                    axes[1].plot([x['progress_m'] for x in rows],[100*x['cross_track_true_m'] for x in rows],label=label,lw=.8)
                axes[0].set_aspect('equal');axes[0].set(xlabel='Map X (m)',ylabel='Map Y (m)')
                axes[1].set(xlabel='Progress (m)',ylabel='Lateral error (cm)')
                for ax in axes:ax.legend();ax.grid(alpha=.25)
                fig.suptitle(split+' | '+r.name+' | free closed loop; same initial pose only')
                fig.savefig(directory/'closed_loop_comparison.png',dpi=150);plt.close(fig)
            scores.append(result)
            print(split,r.name,metrics,flush=True)
    (output/'profile.json').write_text(json.dumps(p,ensure_ascii=False,indent=2,allow_nan=False))
    report=dict(profile=p,results=scores,comparison_seeds=seeds,
        timing=[dict(run=r.name,**t) for r,t in zip(train,timing)],
        implementation_sha256={str(x.relative_to(ROOT)):fingerprint(x) for x in (
            Path(__file__),HERE/'campaign_data.py',HERE/'campaign_model.py',HERE/'ground_model.py',
            HERE/'simulate.py',HERE/'observation.py',HERE/'four_controler/four_controler.py',
            ROOT/'agri_robot_ws/src/agri_path_tracking/agri_path_tracking/controller.py',
            ROOT/'agri_robot_ws/src/agri_path_tracking/agri_path_tracking/coverage.py',
            ROOT/'agri_robot_ws/src/agri_base/agri_base/coordinates.py',
            ROOT/'agri_robot_ws/src/agri_base/agri_base/path_geometry.py',ROOT/'agri_backend/experiment_analysis.py')})
    (output/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    lines=['# X3实车数据校准与留出验证','',
        '只读历史数据；未启ROS/设备、修改实车控制器或升级系统。训练与验证实验互不重合，验证误差没有进入参数拟合。','',
        '辨识：'+', '.join(p['training_runs']), '留出：'+', '.join(p['validation_runs']),'',
        '## 速度响应与5秒局部位姿一致性','',
        '|集合/实验|v RMSE：v1→v2 m/s|ω RMSE：v1→v2 rad/s|5s端点RMSE：v1→v2 m|',
        '|---|---:|---:|---:|']
    for row in scores:
        v,w=row['velocity_rmse']['linear'],row['velocity_rmse']['angular']
        a,b=row['pose_window_v1']['model_endpoint_rmse_m'],row['pose_window_v2']['model_endpoint_rmse_m']
        lines.append(f"|{row['split']}/{row['run']}|{v['v1']:.4f}→{v['v2']:.4f}|{w['v1']:.4f}→{w['v2']:.4f}|{a}→{b}|")
    lines+=['','## 自由闭环跟踪','',
        '只从实际首个正式跟踪样本取得初始位姿/进度，之后状态完全由模型和本地LQR闭环生成；不输入后续实测轨迹、不吸附路径、不重置模型状态。',
        '表中范围是指定种子的场景结果范围，不是统计置信区间。原始RMSE采样频率有差异，另列共同0.1m进度采样与误差曲线差距于validation.json。','',
        '|集合/实验|实测RMSE cm|理想 cm|v1 cm|v2场景范围 cm|', '|---|---:|---:|---:|---:|']
    for row in scores:
        if 'closed_loop' not in row:continue
        c=row['closed_loop'];new=[x['lateral_rmse_cm'] for x in c if x['model']=='v2']
        get=lambda name:next(x['lateral_rmse_cm'] for x in c if x['model']==name)
        lines.append(f"|{row['split']}/{row['run']}|{c[0]['real_lateral_rmse_cm']:.3f}|{get('ideal'):.3f}|{get('v1'):.3f}|{min(new):.3f}～{max(new):.3f}|")
    lines+=['','不能凭一个RMSE接近就声称复现了实车。须同时看速度回放、局部端点、共同进度误差曲线、直线/弯道及完成状态。',
        '来源/模型边界：']+['- '+s for s in p['limits']]
    (output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    for r in train+validation+[basic]:
        for path,digest in r.metadata['source_sha256'].items():
            if fingerprint(ROOT/path)!=digest:raise ValueError('分析期间原始文件改变')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--seeds',type=int,nargs='+',default=[0,1,2])
    args=parser.parse_args()
    output=args.output or HERE/'results'/('campaign_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S'))
    p,train,validation,basic,timing=identify([ROOT/'data/experiments'/n for n in TRAIN_NAMES],
        [ROOT/'data/experiments'/n for n in HOLDOUT_NAMES])
    export(output,p,train,validation,basic,timing,args.seeds)
    print('结果：'+str(output),flush=True)


if __name__=='__main__':main()
