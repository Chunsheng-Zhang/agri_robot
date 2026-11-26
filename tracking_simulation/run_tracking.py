#!/usr/bin/env python3
"""Run one or all robot tracking algorithms on paths/ and export to results/."""
import argparse
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
    allowed={'path','controllers','simulation','tuning','animation'}
    if set(data)-allowed:raise ValueError('未知配置项：'+str(sorted(set(data)-allowed)))
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
        duration=None, output=None, no_animation=False, ideal=False):
    data,settings,names,default_path=read_config(config_file)
    if ideal:
        settings=replace(settings, position_noise=0., heading_noise_deg=0.,
                         actuator_linear_tau=0., actuator_angular_tau=0., feedback_period=0.)
    if controller!='all':
        if controller not in NAMES:raise ValueError('未知控制器：'+controller)
        names=[controller]
    if speed is not None:settings=replace(settings,speed=speed)
    if lookahead is not None:settings=replace(settings,lookahead=lookahead)
    if duration is not None:settings=replace(settings,duration=duration)
    path_file=Path(path_file).resolve() if path_file else default_path
    poses,origin=load_path(path_file)
    _,directions=directed_poses(poses)
    if any(n!='pp' for n in names) and (-1 in directions or point_turn_indices(poses)):
        raise ValueError('四算法对比当前支持普通前进路径；点转/倒车路径请使用--controller pp')
    output=Path(output).resolve() if output else HERE/'results'/(('ideal_' if ideal else '')+path_file.stem+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    source_directory=(HERE/'paths').resolve()
    if output==source_directory or source_directory in output.parents or output==path_file or output==ROOT/'data' or (ROOT/'data') in output.parents:
        raise ValueError('结果不能写入路径源目录或原始data目录')
    if output.exists() and any(output.iterdir()):raise ValueError('输出目录非空，拒绝覆盖')
    paths=[Path(__file__),HERE/'simulate.py',HERE/'disturbances.py',HERE/'reports.py',HERE/'four_controler/four_controler.py',
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
        rows,metrics=simulate(poses,settings,controller_factory=factory)
        metrics.update(algorithm=name,source=origin,implementation_sha256=hashes,
                       controller_settings=data['tuning'].get(name,{}))
        metrics['notes'].append('平面速度模型，含配置的一阶执行响应；没有Gazebo车体动力学，不是已标定实车结果。')
        write_outputs(output/name,poses,rows,metrics)
        results[name]=(rows,metrics)
        rmse=metrics['cross_track_true_m']['rmse']
        print(f"{LABELS[name]}: {metrics['state']}, 横向RMSE="+(f'{100*rmse:.3f} cm' if rmse is not None else '无跟踪样本'),flush=True)
    save_comparison(output,poses,results,animation)
    summary={name:dict(state=m['state'],lateral_rmse_cm=None if m['cross_track_true_m']['rmse'] is None else 100*m['cross_track_true_m']['rmse'],
                       heading_rmse_deg=m['heading_error_true_deg']['rmse'],duration_s=m['elapsed_simulation_s'],
                       endpoint_distance_m=m['final_endpoint_distance_m'],optimizer=m['optimizer']) for name,(_,m) in results.items()}
    snapshot=dict(schema_version=1,kind='ROBOT_TRACKING_SIMULATION',source=origin,ideal=ideal,
                  simulation=asdict(settings),tuning=data['tuning'],animation=animation,
                  algorithms=summary,implementation_sha256=hashes)
    (output/'summary.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (output/'config.yaml').write_text(yaml.safe_dump(dict(data,simulation=asdict(settings),path=str(path_file),
                                                          controllers=names,animation=animation),allow_unicode=True,sort_keys=False),encoding='utf-8')
    if hashlib.sha256(path_file.read_bytes()).hexdigest()!=origin['sha256']:
        raise ValueError('仿真期间路径文件发生变化，结果不能视为冻结路径对照')
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
            '四种控制律均在four_controler/four_controler.py；PP迁移参考脚本的速度相关前视与转角公式并适配机器人v/ω，未修改实车PP。Stanley为角速度适配，LQR/MPC为两状态模型；不代表实车算法排名。',
            'MPC在预测中约束角速度，求解未达到容差的次数单列，不能把未收敛或未完成当作验证通过。',
            f'位置噪声标准差：{settings.position_noise} m（x/y独立）；航向标准差：{settings.heading_noise_deg}°。',
            f'附加一阶响应时间常数：线速度{settings.actuator_linear_tau} s、角速度{settings.actuator_angular_tau} s；0关闭。',
            '车辆按一阶响应输出积分；没有刚体/土壤动力学，也未实车标定，不将误差当作实车精度。',
            '每种算法的actuator_response.png对比控制请求、执行器输出和模型实际速度。']
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
    args=parser.parse_args()
    try:
        output,summary=run(args.config,path_file=args.path_file,controller=args.controller,speed=args.speed,
                           lookahead=args.lookahead,duration=args.duration,output=args.output,no_animation=args.no_animation,ideal=args.ideal)
    except (ValueError,TypeError,KeyError,OSError) as exc:parser.exit(1,str(exc)+'\n')
    print('结果目录：'+str(output))
    return 0 if all(m['state']=='COMPLETED' and m['optimizer']['unconverged']==0 for m in summary.values()) else 2


if __name__=='__main__':raise SystemExit(main())
