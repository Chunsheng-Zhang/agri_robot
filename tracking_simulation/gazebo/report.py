"""Plots distinguish ODE body velocity from commands and localization estimates."""
import math
import csv


def physical_report(output, poses, rows, metrics):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    times = [r['time_s'] for r in rows]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    try:
        fig.suptitle('Gazebo ODE | '+metrics['controller']+' | '+metrics['state'])
        ax = axes[0, 0]
        ax.plot([p[0] for p in poses], [p[1] for p in poses], '--', label='Reference')
        ax.plot([r['x_true_m'] for r in rows], [r['y_true_m'] for r in rows], label='ODE ground truth')
        ax.set(title='Trajectory in original map coordinates', xlabel='x (m)', ylabel='y (m)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend()
        for key, label in (('cross_track_true_m', 'Physical truth'), ('cross_track_observed_m', 'Controller observation')):
            axes[0, 1].plot(times, [100*r[key] if r['phase'] == 'TRACKING' else math.nan for r in rows], label=label)
        axes[0, 1].set(title='Lateral error (tracking only)', xlabel='Simulation time (s)', ylabel='Error (cm)')
        axes[0, 1].legend()
        axes[1, 0].plot(times, [math.degrees(r['heading_error_true_rad']) if r['phase'] == 'TRACKING' else math.nan for r in rows], label='Physical truth')
        axes[1, 0].plot(times, [math.degrees(r['heading_error_observed_rad']) if r['phase'] == 'TRACKING' else math.nan for r in rows], alpha=.5, label='Controller observation')
        axes[1, 0].legend(fontsize=8)
        axes[1, 0].set(title='Physical body-heading error', xlabel='Simulation time (s)', ylabel='Error (deg)')
        ax = axes[1, 1]
        ax.plot(times, [r['v_command_m_s'] for r in rows], '--', color='tab:blue', label='v command')
        ax.plot(times, [r['v_actual_body_m_s'] for r in rows], color='tab:blue', label='v physical')
        angular = ax.twinx()
        angular.plot(times, [r['w_command_rad_s'] for r in rows], '--', color='tab:orange', alpha=.6, label='w command')
        angular.plot(times, [r['w_actual_rad_s'] for r in rows], color='tab:orange', alpha=.6, label='w physical')
        ax.set(title='Commands and physical response', xlabel='Simulation time (s)', ylabel='v (m/s)')
        angular.set_ylabel('w (rad/s)')
        lines = ax.lines+angular.lines
        ax.legend(lines, [line.get_label() for line in lines], fontsize=8)
        for ax in axes.flat:
            ax.grid(True, alpha=.25)
        fig.supxlabel('Uncalibrated four-wheel differential equivalent. Simulation truth is not field measurement.', fontsize=9)
        fig.savefig(output/'tracking_simulation.png', dpi=160)
    finally:
        plt.close(fig)
    # The control loop ends when completion is reported. Use the independent
    # drive record for the response figure so the final braking is visible too.
    drive_file = output.parent/'actuator.csv'
    if rows and drive_file.exists():
        from reports import save_actuator_response
        begin = rows[0]['gazebo_time_s']
        with drive_file.open(encoding='utf-8') as stream:
            response = []
            for record in csv.DictReader(stream):
                stamp = float(record['gazebo_time_s'])
                if stamp >= begin:
                    response.append(dict(time_s=stamp-begin, **{key: float(record[key]) for key in
                        ('v_command_m_s', 'w_command_rad_s', 'v_actuator_m_s', 'w_actuator_rad_s',
                         'v_actual_body_m_s', 'w_actual_rad_s')}))
        save_actuator_response(output, response)
    rmse = metrics['cross_track_true_m']['rmse']
    lines = ['# Gazebo 路径跟踪结果', '',
             f"控制器：{metrics['controller']}；状态：{metrics['state']}；原因：{metrics['reason']}。", '',
             f"横向真值RMSE：{None if rmse is None else round(100*rmse, 4)} cm。", '',
             '![轨迹、误差与物理速度](tracking_simulation.png)', '',
             '![指令、执行器响应与实际速度](actuator_response.png)', '',
             '`tracking.csv`包含控制周期真值/观测、采样年龄、进度、指令和实际刚体速度。',
             '上级目录`ground_truth.csv`与`actuator.csv`记录物理状态及实际下发的滞后输出，包含完成后至少1 s、至少5倍时间常数的制动过程。',
             f"位置标准差={metrics['parameters']['position_noise']} m（x/y独立）；航向标准差={metrics['parameters']['heading_noise_deg']}°。",
             f"附加一阶响应时间常数：线速度={metrics['parameters']['actuator_linear_tau']} s，角速度={metrics['parameters']['actuator_angular_tau']} s；0表示关闭。",
             '图中虚线为指令，实线为物理响应；角速度单位rad/s，不是转角。',
             '四种跟踪器均来自four_controler/four_controler.py；PP为参考脚本迁移适配版，实车PP未修改。',
             '仿真模型为四轮差速等效模型，复用X3外观和轮距；质量/轮径/摩擦/电机仍为工程初值。',
             '模拟定位不是MGX23真实设备，也没有伪造RTK FIX状态；误差不代表实车精度。',
             '控制按模拟时间触发，CSV保留实际控制间隔；不保证加速仿真与实车实时调度完全一致。',
             f"MPC调用/未收敛：{metrics['optimizer']['calls']}/{metrics['optimizer']['unconverged']}。", '']
    (output/'report.md').write_text('\n'.join(lines), encoding='utf-8')
