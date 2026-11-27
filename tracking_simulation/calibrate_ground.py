#!/usr/bin/env python3
"""用只读历史实验辨识X3地面等效响应，并用独立实验回放验证。"""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for key, relative in (('TMPDIR', '.cache/tmp'), ('MPLCONFIGDIR', '.cache/matplotlib')):
    os.environ.setdefault(key, str(HERE/relative))
    Path(os.environ[key]).mkdir(parents=True, exist_ok=True)

import numpy as np
from ground_model import evaluate_run, fit_channel, pose_window_check, read_ground_run

DEFAULT_TRAIN = ['20261105_125347_323692_web_d320a929', '20261105_141817_620046_web_4aa4bbfc']
DEFAULT_VALIDATE = ['20261105_130800_067216_web_aff105d9', '20261105_141044_857775_web_8228aa11']


def identify(train_paths, validation_paths):
    train_paths = [Path(p).resolve() for p in train_paths]
    validation_paths = [Path(p).resolve() for p in validation_paths]
    if not train_paths or not validation_paths or set(train_paths) & set(validation_paths):
        raise ValueError('必须提供非空、互不重合的辨识集和验证集')
    if len(set(train_paths+validation_paths)) != len(train_paths+validation_paths):
        raise ValueError('不允许重复实验')
    train = [read_ground_run(p) for p in train_paths]
    validation = [read_ground_run(p) for p in validation_paths]
    if any(r.metadata['projection'] != train[0].metadata['projection'] for r in train+validation):
        raise ValueError('实验坐标系/固定原点不一致')
    profile = dict(schema_version=1, kind='X3_GROUND_EQUIVALENT_RESPONSE',
                   name='X3 outdoor hard ground, provisional equivalent model',
                   linear=fit_channel(train, 0), angular=fit_channel(train, 1),
                   feedback_period_s=float(np.median([r.metadata['feedback_period_median_s'] for r in train])),
                   training_runs=[r.name for r in train], validation_runs=[r.name for r in validation],
                   identification=dict(integration_dt_s=.01, delay_grid_s=.025,
                       loss='equal weight per run, squared body-velocity prediction error',
                       command_topic='/base/driver_cmd_vel',
                       linear_unit='m/s', angular_unit='rad/s'),
                   limits=[
                       '仅为当前X3室外硬质地面的历史记录等效模型，不是精确轮胎/麦轮动力学。',
                       '等效延迟包含时间戳对齐和MGX测量响应，不能解释为MCU单程延迟。',
                       '限幅是本模型拟合参数，不是已确认的底盘机械速度上限。',
                       '没有独立定位真值，不能从记录分离GNSS误差、侧滑、横移和机械扰动。',
                       '未标定后退、横移、不同地面/载荷/电池电压；训练未覆盖的速度和转向不保证。',
                       '验证实验没有独立1m/s数据，高速外推尚未验证；不将仿真排名当成实车结论。',
                       '历史实验使用旧PP，模型用指令开环回放验证，不声称复现新版四算法实车闭环。'])
    profile['training_source_sha256'] = {r.name: r.metadata['source_sha256'] for r in train}
    profile['training_excitation'] = {}
    for i, name in enumerate(('linear', 'angular')):
        maximum = max(r.metadata['max_abs_command'][i] for r in train)
        p = profile[name]
        clipped = sum(int(np.sum(p['gain']*np.abs(r.commands[:, i]) >= p['limit'])) for r in train)
        total = sum(len(r.commands) for r in train)
        profile['training_excitation'][name] = dict(max_abs_command=maximum,
            fitted_clip_fraction_of_commands=clipped/total,
            limit_excited=bool(clipped),
            limit_note='拟合限幅有被触发' if clipped else '训练指令没有触发此限幅，限幅值不可充分辨识')
    return profile, train, validation


def export(output, profile, train, validation):
    output = Path(output).resolve()
    results_root = (HERE/'results').resolve()
    if not output.is_relative_to(results_root) or output == results_root:
        raise ValueError('辨识产物必须位于tracking_simulation/results/的独立子目录')
    if output.exists() and any(output.iterdir()):
        raise ValueError('输出目录非空，拒绝覆盖')
    output.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    report = ['# X3室外硬地等效模型辨识与独立回放验证', '',
              '只读取历史实验，不启动ROS、Gazebo、实车或网页，不修改原始实验和实车参数。', '',
              '模型：`tau * dy/dt + y = clip(gain * command(t-delay), -limit, limit)`；v和ω独立。',
              '目标是给算法筛选增加实测约束；尚不具备精确预测实车厘米级误差的依据。', '',
              '辨识集：'+', '.join(r.name for r in train),
              '独立验证集：'+', '.join(r.name for r in validation), '',
              '| 通道 | gain | limit | tau s | 等效delay s |', '| --- | ---: | ---: | ---: | ---: |']
    for name in ('linear', 'angular'):
        p = profile[name]
        report.append(f"| {name} | {p['gain']:.6f} | {p['limit']:.6f} | {p['tau_s']:.6f} | {p['delay_s']:.3f} |")
    report += ['', 'limit的单位分别为m/s、rad/s。限幅与增益可能互相补偿；缺少饱和激励时限幅不可充分辨识。',
               '延迟按25ms网格搜索，数值回放步长10ms，不能把结果精确到微秒。', '',
               '| 数据集 | 实验 | v RMSE m/s：理想→拟合 | ω RMSE rad/s：理想→拟合 |',
               '| --- | --- | ---: | ---: |']
    results = []
    for split, runs in (('training', train), ('validation', validation)):
        for run in runs:
            predicted, baseline, metrics = evaluate_run(run, profile)
            pose_metrics, windows = pose_window_check(run, predicted, baseline)
            results.append(dict(split=split, run=run.name, metrics=metrics, pose_window_check=pose_metrics, metadata=run.metadata))
            v, w = metrics['linear'], metrics['angular']
            report.append(f"| {split} | {run.name} | {v['ideal_rmse']:.5f} → {v['rmse']:.5f} | {w['ideal_rmse']:.5f} → {w['rmse']:.5f} |")
            folder = output/run.name
            folder.mkdir()
            (folder/'pose_windows.json').write_text(json.dumps(dict(summary=pose_metrics, windows=windows), ensure_ascii=False, indent=2, allow_nan=False)+'\n')
            with (folder/'replay.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['time_s', 'score_valid', 'v_mgx_body_m_s', 'v_model_m_s', 'v_ideal_m_s',
                                 'w_mgx_derived_rad_s', 'w_model_rad_s', 'w_ideal_rad_s', 'x_mgx_m', 'y_mgx_m', 'yaw_mgx_rad'])
                for i, time in enumerate(run.time):
                    writer.writerow([time, int(run.valid[i]), run.measured[i, 0], predicted[i, 0], baseline[i, 0],
                                     run.measured[i, 1], predicted[i, 1], baseline[i, 1], *run.pose[i]])
            fig, axes = plt.subplots(3, 1, figsize=(14, 9), constrained_layout=True)
            try:
                for i, axis in enumerate(axes[:2]):
                    axis.plot(run.time, baseline[:, i], '--', alpha=.6, label='Driver command / ideal model')
                    axis.plot(run.time, run.measured[:, i], lw=1, label='MGX body speed' if i == 0 else 'MGX heading derivative')
                    axis.plot(run.time, predicted[:, i], lw=1, label='Fitted equivalent model')
                    axis.set_ylabel('v (m/s)' if i == 0 else 'omega (rad/s)')
                    axis.legend(fontsize=8)
                for i, label in enumerate(('v error (m/s)', 'omega error (rad/s)')):
                    axes[2].plot(run.time, np.where(run.valid, predicted[:, i]-run.measured[:, i], np.nan), label=label, lw=.8)
                axes[2].set(xlabel='Host clock relative time (s)', ylabel='Model - measurement')
                axes[2].legend(fontsize=8)
                for axis in axes:
                    axis.grid(alpha=.25)
                fig.suptitle(f'{split.upper()} | {run.name} | velocity replay, no feedback resets')
                fig.savefig(folder/'response_replay.png', dpi=150)
            finally:
                plt.close(fig)
    improved = all(r['metrics'][c]['rmse'] < r['metrics'][c]['ideal_rmse']
                   for r in results if r['split'] == 'validation' for c in ('linear', 'angular'))
    profile['validation_status'] = ('lower_velocity_rmse_than_ideal_on_both_holdouts' if improved
                                    else 'mixed_holdout_results_provisional')
    report += ['', f"独立验证：{'两次验证的两个速度通道RMSE均低于理想执行模型' if improved else '存在未改善的验证通道，不能声称模型普遍更准确'}。",
               '这个判断只比较速度响应，不等价于通过路径跟踪精度验证。', '',
               '## 5秒局部开环位姿一致性', '',
               '每窗只在起点取一次MGX位姿，随后按预测速度积分，不在窗内吸附参考轨迹；端点与MGX比较。它衡量运动学一致性，没有独立定位真值。', '',
               '| 数据集 | 实验 | 窗数 | 端点RMSE m：理想→拟合 |', '| --- | --- | ---: | ---: |']
    for row in results:
        p = row['pose_window_check']
        report.append(f"| {row['split']} | {row['run']} | {p['windows']} | {p['ideal_endpoint_rmse_m']} → {p['model_endpoint_rmse_m']} |")
    report += ['', '不可充分辨识的参数：'+', '.join(name+'.limit' for name, item in profile['training_excitation'].items() if not item['limit_excited']), '',
               '## 数据与统计约定', '',
               '输入为实际驱动话题，反馈为同一source_stamp精确配对的MGX速度与定位航向；NED速度转UTM网格再投影到车体，包含配置的天线杆臂修正。',
               '时间使用同一主机ROS时钟：指令接收时间对MGX主机source_stamp。记录器传输排队、MGX滤波等不能单独分离。',
               '剔除初始3s、指令超过250ms未更新，以及指令/定位间断后3s；每次实验等权参与辨识，验证集不参与参数拟合。',
               '不把无回波/缺失数据当成0，不从已拟合残差反推GNSS白噪声，不混入架空台架实验。', '',
               '## 使用范围与尚未确定的因素', '']
    report += ['- '+note for note in profile['limits']]
    report += ['', '每个实验目录保存`replay.csv`和`response_replay.png`；`validation.json`保存数据指纹与评价。',
               '辨识与验证样本均为旧实验；今后的独立实车数据应继续用于验证，特别是1m/s及急转弯。']
    (output/'profile.json').write_text(json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    summary = dict(profile=profile, results=results,
                   implementation_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in (Path(__file__), HERE/'ground_model.py', HERE/'disturbances.py')})
    (output/'validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    (output/'report.md').write_text('\n'.join(report)+'\n')
    # Verify source snapshots stayed unchanged throughout identification/export.
    for run in train+validation:
        for name, fingerprint in run.metadata['source_sha256'].items():
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != fingerprint:
                raise ValueError('辨识期间原始实验文件发生变化')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, nargs='+')
    parser.add_argument('--validate', type=Path, nargs='+')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if bool(args.train) != bool(args.validate):
        parser.error('--train和--validate需要同时提供')
    train = args.train or [ROOT/'data/experiments'/name for name in DEFAULT_TRAIN]
    validation = args.validate or [ROOT/'data/experiments'/name for name in DEFAULT_VALIDATE]
    output = args.output or HERE/'results'/('ground_fit_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    try:
        profile, training, holdout = identify(train, validation)
        summary = export(output, profile, training, holdout)
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(1, str(exc)+'\n')
    print(json.dumps(dict(output=str(output), profile=profile, metrics=[dict(run=r['run'], split=r['split'], metrics=r['metrics'])
                                                                      for r in summary['results']]), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
