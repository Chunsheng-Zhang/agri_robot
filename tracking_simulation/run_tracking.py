#!/usr/bin/env python3
"""Run one or all robot tracking algorithms on paths/ and export to results/."""
import argparse
import csv
from dataclasses import asdict, replace
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path

HERE=Path(__file__).resolve().parent
for variable,relative in (('TMPDIR','.cache/tmp'),('MPLCONFIGDIR','.cache/matplotlib')):
    os.environ.setdefault(variable,str(HERE/relative))
    Path(os.environ[variable]).mkdir(parents=True,exist_ok=True)

import yaml

from simulate import (ROOT, SimulationConfig, directed_poses, load_path,
                      point_turn_indices, simulate, write_outputs)
from four_controler.four_controler import LABELS, NAMES, PP_DEFAULTS, build_controller
from reports import save_comparison


def read_config(filename):
    filename=Path(filename).resolve()
    data=yaml.safe_load(filename.read_text(encoding='utf-8'))
    if not isinstance(data,dict):raise ValueError('配置必须为YAML对象')
    allowed={'path','controllers','simulation','tuning','animation','model_profile','experiment','experiment_overrides'}
    if set(data)-allowed:raise ValueError('未知配置项：'+str(sorted(set(data)-allowed)))
    for key in ('model_profile','experiment'):
        if key in data and (not isinstance(data[key],str) or not data[key].strip()):
            raise ValueError(key+'需要非空路径字符串')
    if 'experiment_overrides' in data:
        overrides=data['experiment_overrides']
        if not isinstance(overrides,dict) or set(overrides)-{'lookahead','speed','max_angular','completion_tolerance','search_window'}:
            raise ValueError('experiment_overrides仅支持五项任务跟踪参数')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in overrides.values()):
            raise ValueError('experiment_overrides参数必须为有限正数')
    sim=SimulationConfig(**data['simulation'])
    for key,value in asdict(sim).items():
        if key=='duration' and value is None:continue
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError('simulation.'+key+'必须是有限数值')
    if type(sim.seed) is not int:raise ValueError('simulation.seed必须是整数')
    for key in ('position_noise', 'heading_noise_deg', 'feedback_period', 'actuator_linear_tau', 'actuator_angular_tau'):
        if getattr(sim, key) < 0:raise ValueError('simulation.'+key+'必须非负')
    controllers=data['controllers']
    if not isinstance(controllers,list) or not controllers or len(set(controllers))!=len(controllers) or any(n not in NAMES for n in controllers):
        raise ValueError('controllers需要不重复的pp/stanley/lqr/mpc列表')
    tuning=data['tuning']
    tuning['pp'] = dict(PP_DEFAULTS, **tuning.get('pp', {}))
    for name,keys in {'stanley':('cross_track_gain','heading_gain','softening_speed'),
                      'lqr':('lateral_weight','heading_weight','angular_weight'),
                      'mpc':('lateral_weight','heading_weight','angular_weight','terminal_multiplier')}.items():
        for key in keys:
            value=tuning[name][key]
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError(f'{name}.{key}必须为有限正数')
    for key,low,high in (('horizon',2,60),('max_iterations',1,2000)):
        value=tuning['mpc'][key]
        if type(value) is not int or not low<=value<=high:raise ValueError(f'mpc.{key}需在{low}～{high}之间')
    animation=data['animation']
    if type(animation['enabled']) is not bool:raise ValueError('animation.enabled必须为布尔值')
    for key,low,high in (('max_frames',2,200),('fps',1,30)):
        value=animation[key]
        if type(value) is not int or not low<=value<=high:raise ValueError(f'animation.{key}需在{low}～{high}之间')
    path=Path(data['path'])
    if not path.is_absolute():path=filename.parent/path
    return data,sim,controllers,path.resolve()


def run(config_file, *, path_file=None, controller='all', speed=None, lookahead=None,
        duration=None, output=None, no_animation=False, ideal=False, model_profile=None,experiment=None,seed=None):
    data,settings,names,default_path=read_config(config_file)
    config_dir=Path(config_file).resolve().parent
    if model_profile is None and data.get('model_profile') and not ideal:
        model_profile=Path(data['model_profile'])
        if not model_profile.is_absolute():model_profile=config_dir/model_profile
    if experiment is None and data.get('experiment'):
        experiment=Path(data['experiment'])
        if not experiment.is_absolute():experiment=config_dir/experiment
    case=None;initial_pose=None;initial_progress=0.
    if experiment is not None:
        directory=Path(experiment).resolve()
        if path_file is not None:raise ValueError('--experiment与--path-file不能同时指定')
        mission_file=directory/'config/mission.json'
        mission=json.loads(mission_file.read_text())
        plans=list((directory/'paths').glob('coverage_*.json'));tracks=list((directory/'tracks').glob('track_*.csv'))
        if len(plans)!=1 or len(tracks)!=1:raise ValueError('实验需要唯一覆盖路径和正式跟踪CSV')
        path_file=plans[0]
        with tracks[0].open(encoding='utf-8-sig') as stream:first=next(csv.DictReader(stream),None)
        if not first:raise ValueError('实验没有正式跟踪起始位姿')
        initial_pose=tuple(float(first[k]) for k in ('x','y','yaw'));initial_progress=float(first['progress'])
        settings=replace(settings,**{k:mission['parameters'][k] for k in
            ('lookahead','speed','max_angular','completion_tolerance','search_window')})
        case=dict(directory=str(directory),initial_pose=list(initial_pose),initial_progress_m=initial_progress,
            mission=mission,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (mission_file,plans[0],tracks[0])},
            note='Only first recorded pose/progress and frozen task/reference used; no later measured poses feed simulation; tuning remains current simulation YAML')
        settings=replace(settings,**data.get('experiment_overrides',{}))
    elif data.get('experiment_overrides'):
        raise ValueError('experiment_overrides需要同时指定实验')
    profile = profile_source = None
    if model_profile is not None:
        if ideal:raise ValueError('--ideal和--model-profile不能同时使用')
        from ground_model import load_profile
        profile, profile_source = load_profile(model_profile)
        settings=replace(settings, feedback_period=profile['feedback_period_s'],
                         actuator_linear_tau=profile['linear']['tau_s'],
                         actuator_angular_tau=profile['angular']['tau_s'])
    if ideal:
        settings=replace(settings, position_noise=0., heading_noise_deg=0.,
                         actuator_linear_tau=0., actuator_angular_tau=0., feedback_period=0.)
    if controller!='all':
        if controller not in NAMES:raise ValueError('未知控制器：'+controller)
        names=[controller]
    if speed is not None:settings=replace(settings,speed=speed)
    if lookahead is not None:settings=replace(settings,lookahead=lookahead)
    if duration is not None:settings=replace(settings,duration=duration)
    if seed is not None:settings=replace(settings,seed=seed)
    path_file=Path(path_file).resolve() if path_file else default_path
    poses,origin=load_path(path_file)
    _,directions=directed_poses(poses)
    if any(n!='pp' for n in names) and (-1 in directions or point_turn_indices(poses)):
        raise ValueError('四算法对比当前支持普通前进路径；点转/倒车路径请使用--controller pp')
    output=Path(output).resolve() if output else HERE/'results'/(('ideal_' if ideal else 'ground_' if profile else '')+path_file.stem+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    source_directory=(HERE/'paths').resolve()
    if output==source_directory or source_directory in output.parents or output==path_file or output==ROOT/'data' or (ROOT/'data') in output.parents:
        raise ValueError('结果不能写入路径源目录或原始data目录')
    if output.exists() and any(output.iterdir()):raise ValueError('输出目录非空，拒绝覆盖')
    paths=[Path(__file__),HERE/'simulate.py',HERE/'disturbances.py',HERE/'ground_model.py',HERE/'campaign_model.py',HERE/'observation.py',HERE/'reports.py',HERE/'four_controler/four_controler.py',
           ROOT/'agri_robot_ws/src/agri_path_tracking/agri_path_tracking/controller.py',
           ROOT/'agri_robot_ws/src/agri_path_tracking/agri_path_tracking/coverage.py',
           ROOT/'agri_robot_ws/src/agri_base/agri_base/coordinates.py',
           ROOT/'agri_robot_ws/src/agri_base/agri_base/path_geometry.py',
           ROOT/'agri_robot_ws/src/agri_waypoint/agri_waypoint/controller.py',
           ROOT/'agri_backend/experiment_analysis.py']
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    animation=dict(data['animation'])
    if no_animation:animation['enabled']=False
    results={}
    for name in names:
        factory=build_controller(name,dt=settings.dt,tuning=data['tuning'])
        rows,metrics=simulate(poses,settings,controller_factory=factory,model_profile=profile,
                              initial_pose=initial_pose,initial_progress=initial_progress)
        metrics.update(algorithm=name,source=origin,implementation_sha256=hashes,
                       controller_settings=data['tuning'].get(name,{}))
        metrics['notes'].append('平面等效速度模型，没有Gazebo车体动力学；仿真误差不是实车精度。')
        write_outputs(output/name,poses,rows,metrics)
        results[name]=(rows,metrics)
        rmse=metrics['cross_track_true_m']['rmse']
        print(f"{LABELS[name]}: {metrics['state']}, 横向RMSE="+(f'{100*rmse:.3f} cm' if rmse is not None else '无跟踪样本'),flush=True)
    save_comparison(output,poses,results,animation)
    summary={name:dict(state=m['state'],lateral_rmse_cm=None if m['cross_track_true_m']['rmse'] is None else 100*m['cross_track_true_m']['rmse'],
                       heading_rmse_deg=m['heading_error_true_deg']['rmse'],duration_s=m['elapsed_simulation_s'],
                       endpoint_distance_m=m['final_endpoint_distance_m'],reference_regions=m['reference_regions'],optimizer=m['optimizer']) for name,(_,m) in results.items()}
    snapshot=dict(schema_version=1,kind='ROBOT_TRACKING_SIMULATION',source=origin,ideal=ideal,
                  simulation=asdict(settings),tuning=data['tuning'],animation=animation,
                  algorithms=summary,implementation_sha256=hashes)
    if profile is not None:
        snapshot.update(model_profile=profile, model_profile_source=profile_source)
        (output/'model_profile.json').write_text(json.dumps(profile,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    if case is not None:
        snapshot['experiment_case']=case
        (output/'experiment_case.json').write_text(json.dumps(case,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (output/'summary.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    saved_config=dict(data,simulation=asdict(settings),path=str(path_file),controllers=names,animation=animation)
    if profile is not None:saved_config['model_profile']=profile_source['file']
    else:saved_config.pop('model_profile',None)
    if case is not None:
        saved_config['experiment']=case['directory']
        saved_config['experiment_overrides']={k:getattr(settings,k) for k in
            ('lookahead','speed','max_angular','completion_tolerance','search_window')}
    (output/'config.yaml').write_text(yaml.safe_dump(saved_config,allow_unicode=True,sort_keys=False),encoding='utf-8')
    if hashlib.sha256(path_file.read_bytes()).hexdigest()!=origin['sha256']:
        raise ValueError('仿真期间路径文件发生变化，结果不能视为冻结路径对照')
    if case is not None:
        for filename,digest in case['source_sha256'].items():
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest()!=digest:
                raise ValueError('仿真期间实验输入发生变化，不能视为冻结工况对照')
    lines=['# 路径跟踪仿真结果','',f"参考路径：`{path_file}`。保持原map坐标及投影。",'',
           '本次工况：'+('理想条件，无附加噪声/执行响应滞后，定位每控制步更新。' if ideal else '按配置的噪声、采样和执行响应条件运行。'),'',
           '| 控制器 | 状态 | 横向RMSE cm | 航向RMSE ° | 模拟用时 s | MPC未收敛次数 |',
           '| --- | --- | ---: | ---: | ---: | ---: |']
    for name,m in summary.items():
        lateral='无样本' if m['lateral_rmse_cm'] is None else f"{m['lateral_rmse_cm']:.4f}"
        heading='无样本' if m['heading_rmse_deg'] is None else f"{m['heading_rmse_deg']:.4f}"
        lines.append(f"| {LABELS[name]} | {m['state']} | {lateral} | {heading} | {m['duration_s']:.3f} | {m['optimizer']['unconverged']} |")
    lines+=['','![轨迹与误差](controller_comparison.png)','', '![速度指令](controller_commands.png)','',
            '每个控制器子目录包含逐周期tracking.csv、metrics.json、参考路径及四合一图。',
            '包含初始收敛；阶段、采样周期、初始位姿、速度上限及噪声种子共用。',
            '四种仿真控制律均在four_controler/four_controler.py；实车迁移实现另在agri_path_tracking/algorithms.py，并非本脚本调用链。Stanley为角速度适配，LQR/MPC为两状态模型；不代表实车算法排名。',
            'MPC在预测中约束角速度，求解未达到容差的次数单列，不能把未收敛或未完成当作验证通过。',
            f'位置噪声标准差：{settings.position_noise} m（x/y独立）；航向标准差：{settings.heading_noise_deg}°。',
            (f'附加一阶响应时间常数：线速度{settings.actuator_linear_tau} s、角速度{settings.actuator_angular_tau} s；0关闭。' if profile is None else
             '执行响应由model_profile.json提供，不额外叠加YAML的一阶时间常数；分工况参数以模型快照为准。'),
            '车辆按执行响应输出积分；没有刚体/土壤动力学，不将仿真误差当作实车精度。',
            '每种算法的actuator_response.png对比控制请求、执行器输出和模型实际速度。']
    if profile is not None:
        lines += ['', '本轮显式加载X3室外硬地等效响应：`'+profile_source['file']+'`。',
                  '模型快照见model_profile.json。响应为增益/限幅/等效延迟/一阶惯性，不叠加原YAML的执行器时间常数。',
                  '定位采样周期来自辨识集；噪声继续使用本轮YAML，没有从残差虚构GNSS真值。',
                  'delay包含MGX测量响应及归档时间对齐，不能称为MCU单程延迟。高速度/转向范围外的排名没有验证依据。']
        if profile['schema_version']==2:
            lines += ['', 'schema2模型另包含行进速度工况、原地二阶响应、独立归零、横向等效关联及因果定位交付/控制时序。',
                      '角速度辨识已计入航向后向差分的区间平均，不把该平均再当成额外机械延迟。',
                      '交付时序来自训练记录的唯一LQR输出匹配与区间估计，不等同于记录器排队、串口RTT或直接网络延迟。']
    if case is not None:
        lines += ['', '本轮复用实验：`'+case['directory']+'`，参考路径与首个正式跟踪位姿已冻结；任务参数及本轮覆盖值见config.yaml。',
                  '后续实测位置没有输入模型；算法权重来自本轮仿真YAML，可用于离线改进对照。']
    lines += ['', '直线/转弯分区按参考路径0.5m窗口的航向变化计算，|曲率|≥0.1rad/m归为转弯；不按控制器是否摆动划分。',
              '各区包含样本数、横向及航向的RMSE/P95/最大误差，见summary.json中的reference_regions。', '',
              '| 控制器 | 直线横向RMSE cm | 转弯横向RMSE cm |', '| --- | ---: | ---: |']
    for name, m in summary.items():
        values = [m['reference_regions'][region]['cross_track_m']['rmse'] for region in ('straight', 'turn')]
        rendered = ['无样本' if value is None else f'{value*100:.4f}' for value in values]
        lines.append(f'| {LABELS[name]} | {rendered[0]} | {rendered[1]} |')
    if animation['enabled']:lines+=['','动画见[controller_compare.gif](controller_compare.gif)，为加速播放，画面显示模拟时间。']
    (output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return output,summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=HERE/'config.yaml')
    parser.add_argument('--path-file',type=Path)
    parser.add_argument('--controller',choices=('all',)+NAMES,default='all')
    parser.add_argument('--speed',type=float)
    parser.add_argument('--lookahead',type=float)
    parser.add_argument('--duration',type=float)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--no-animation',action='store_true')
    parser.add_argument('--ideal',action='store_true',help='仅本次关闭位置/航向噪声与执行器响应滞后，并在每个控制步更新定位；保留配置的初始误差')
    parser.add_argument('--model-profile',type=Path,help='显式加载历史实验辨识的地面等效响应JSON；不能与--ideal同用')
    parser.add_argument('--experiment',type=Path,help='只读实验任务、参考路径与首个正式跟踪位姿；其后自由闭环')
    parser.add_argument('--seed',type=int,help='本轮噪声/交付时序种子，不改YAML')
    args=parser.parse_args()
    try:
        output,summary=run(args.config,path_file=args.path_file,controller=args.controller,speed=args.speed,
                           lookahead=args.lookahead,duration=args.duration,output=args.output,no_animation=args.no_animation,ideal=args.ideal,model_profile=args.model_profile,experiment=args.experiment,seed=args.seed)
    except (ValueError,TypeError,KeyError,OSError) as exc:parser.exit(1,str(exc)+'\n')
    print('结果目录：'+str(output))
    return 0 if all(m['state']=='COMPLETED' and m['optimizer']['unconverged']==0 for m in summary.values()) else 2


if __name__=='__main__':raise SystemExit(main())
