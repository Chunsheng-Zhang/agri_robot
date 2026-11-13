"""Offline task report. Reads finished files only; no ROS imports or control calls."""
import argparse
import csv
import json
import math
import os
from collections import Counter
from pathlib import Path
from threading import Lock

from .missions import ROOT, atomic_json, now

PLOT_LOCK = Lock()


def read_csv(path):
    if not path.exists():
        return []
    with path.open(encoding='utf-8', newline='') as file:
        return list(csv.DictReader(file))


def number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def statistics(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return dict(samples=0, **{k:None for k in ('mean','mae','rmse','std','abs_median','abs_p95','abs_p99','abs_max')})
    absolute = sorted(abs(v) for v in values)
    def percentile(p):
        position = p * (len(values) - 1)
        lo = int(position)
        return absolute[lo] + (absolute[min(lo + 1, len(values) - 1)] - absolute[lo]) * (position - lo)
    mean=sum(values)/len(values)
    return {'samples': len(values), 'mean': mean, 'std':math.sqrt(sum((v-mean)**2 for v in values)/len(values)),
            'mae': sum(absolute) / len(values), 'rmse': math.sqrt(sum(v*v for v in values) / len(values)),
            'abs_median':percentile(.5), 'abs_p95': percentile(.95), 'abs_p99':percentile(.99), 'abs_max': absolute[-1]}


def trajectory(rows):
    """Exclude nonfinite rows, report timestamps rather than silently reordering them."""
    valid = []
    for row in rows:
        parsed = {k: number(v) for k, v in row.items()}
        if all(parsed.get(k) is not None for k in ('stamp', 'x', 'y', 'yaw')):
            valid.append(parsed)
    return valid


def trajectory_metrics(rows):
    stamps = [r['stamp'] for r in rows]
    intervals = [b-a for a, b in zip(stamps, stamps[1:])]
    return {'samples': len(rows), 'first_ros_s': stamps[0] if rows else None,
            'last_ros_s': stamps[-1] if rows else None,
            'recorded_span_s': stamps[-1]-stamps[0] if rows else None,
            'non_increasing_timestamps': sum(dt <= 0 for dt in intervals),
            'max_sample_interval_s': max(intervals, default=None),
            'recorded_polyline_length_m': sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(rows, rows[1:])),
            'cross_track_m': statistics([r.get('cross_track') for r in rows]),
            'target_distance_m': statistics([r.get('distance') for r in rows]),
            'heading_error_rad': statistics([r.get('heading_error') for r in rows]),
            'heading_error_deg': statistics([math.degrees(r['heading_error']) if r.get('heading_error') is not None else None for r in rows])}


def analyze_run(directory):
    directory = Path(directory).resolve()
    output = directory / 'analysis'
    output.mkdir(exist_ok=True)
    run = json.loads((directory / 'run.json').read_text(encoding='utf-8'))
    capture_path = directory / 'telemetry/recording.json'
    capture = json.loads(capture_path.read_text()) if capture_path.exists() else {}
    terminal = capture.get('terminal') or {}
    if capture.get('state') == 'RECORDING' or (not terminal and run['state'] not in ('COMPLETED', 'CANCELED', 'FAULT', 'FAILED')):
        raise ValueError('任务或数据采集尚未结束，不能分析未关闭文件')
    atomic_json(output / 'status.json', {'state': 'PROCESSING', 'updated_at': now()})
    mission = json.loads((directory / 'config/mission.json').read_text())
    notes = [
        'Errors use the same MGX23 localization as control, not independent ground truth.',
        'Statistics use recorded samples, not distance-resampled or time-weighted samples; pauses may create gaps.',
        'Approach, explicit alignment and coverage tracking are evaluated separately when separate recordings exist; legacy tracking may include initial alignment.',
        'Completion is the controller result, not proof of whole-field implement coverage.',
        'Motion series retain their own source/receipt timestamps; no speed is synthesized from position differences.',
        'MGX23 horizontal speed is unsigned hypot(N,E); chassis/cmd linear_x is signed body-forward speed.',
    ]
    sources = {}
    phase_rows = {}
    track_files = sorted((directory / 'tracks').glob('*.csv'))
    for file in track_files:
        phase = ('approach' if file.name.startswith('approach_') else 'alignment' if file.name.startswith('alignment_')
                 else 'tracking' if file.name.startswith('track_') else 'waypoint')
        if phase in phase_rows:
            raise ValueError(f'Multiple {phase} recordings; cannot silently mix attempts')
        sidecar = json.loads(file.with_suffix('.json').read_text())
        if sidecar.get('projection') != run['projection'] or sidecar.get('frame_id') != 'map':
            raise ValueError('Trajectory coordinate origin/frame mismatch')
        rows = trajectory(read_csv(file))
        if rows and any(b['stamp'] <= a['stamp'] for a,b in zip(rows,rows[1:])):
            notes.append(f'{phase}: non-increasing ROS timestamps; time-based comparisons need review.')
        phase_rows[phase], sources[phase] = rows, str(file.relative_to(directory))
    reference = []
    if run['mode'] == 'coverage':
        paths = sorted((directory / 'paths').glob('coverage_*.json'))
        if len(paths) != 1:
            raise ValueError('Coverage report requires exactly one saved reference path')
        document = json.loads(paths[0].read_text())
        if document.get('projection') != run['projection'] or document.get('frame_id') != 'map':
            raise ValueError('Reference coordinate origin/frame mismatch')
        reference = [(float(p['x']), float(p['y'])) for p in document['points']]
        sources['reference'] = str(paths[0].relative_to(directory))
        if phase_rows.get('tracking'):
            sidecar = json.loads((directory / sources['tracking']).with_suffix('.json').read_text())
            recorded = [(p['x'], p['y']) for p in sidecar['reference_path']['points']]
            if recorded != reference:
                raise ValueError('Recorded reference differs from planned reference')
    else:
        reference = [(p['x'], p['y']) for p in mission.get('preview', {}).get('points', [])]
        notes.append('Waypoint distance is distance to the active target, not lateral path-tracking error.')
    if not all(math.isfinite(x+y) for x,y in reference):
        raise ValueError('Nonfinite reference path')
    primary = 'tracking' if run['mode'] == 'coverage' else 'waypoint'
    main_rows = phase_rows.get(primary, [])
    t0 = main_rows[0]['stamp'] if main_rows else min(
        (r[0]['stamp'] for r in phase_rows.values() if r), default=None)
    phases = {name: trajectory_metrics(rows) for name,rows in phase_rows.items()}
    if main_rows and reference:
        phases[primary]['final_endpoint_distance_m'] = math.hypot(main_rows[-1]['x']-reference[-1][0], main_rows[-1]['y']-reference[-1][1])
        phases[primary]['final_progress_m'] = main_rows[-1].get('progress')
    if phase_rows.get('approach'):
        phases['approach']['final_target_distance_m'] = phase_rows['approach'][-1].get('distance')
    motion = read_csv(directory / 'telemetry/motion.csv')
    mgx = read_csv(directory / 'telemetry/mgx23.csv')
    if t0 is None:
        stamps = [number(r.get('received_ros_ns')) for r in motion + mgx]
        t0 = min((t / 1e9 for t in stamps if t is not None), default=None)
    speeds = {}
    for topic in sorted({r['topic'] for r in motion}):
        selected = [r for r in motion if r['topic'] == topic]
        speeds[topic] = {'samples': len(selected),
            'linear_x_m_s': statistics([number(r['linear_x']) for r in selected]),
            'angular_z_rad_s': statistics([number(r['angular_z']) for r in selected])}
    counts = capture.get('samples_by_topic', {})
    expected = ['/mgx23/raw', '/mgx23/position', '/mgx23/attitude_stamped', '/mgx23/velocity_stamped',
                '/mgx23/status', '/mgx23/accuracy', '/odom', '/cmd_vel', '/base/driver_cmd_vel',
                '/vel_raw', f'/control/{"tracking" if run["mode"] == "coverage" else "waypoint"}/cmd_vel']
    missing = [topic for topic in expected if not counts.get(topic)]
    if missing:
        notes.append('Missing recorded topics: ' + ', '.join(missing))
    if capture.get('queue_drops_by_topic') or capture.get('errors') or capture.get('raw_sequence_gaps'):
        notes.append('Capture reports drops/errors/raw sequence gaps; inspect telemetry/recording.json.')
    from .experiment_communication import analyze_communication
    from .experiment_summary import tracking_assessment, write_summary
    from .communication_plots import build_plot_data, plot_communication
    try:
        communication=analyze_communication(directory,capture)
    except (OSError, ValueError, KeyError) as exc:
        communication={'available':False,'reason':f'通信分析失败，不能宣称零丢包/零延时：{exc}'}
        notes.append(communication['reason'])
    assessment=tracking_assessment(directory,mission,run,reference,phase_rows,motion,mgx,statistics)
    plot_data=build_plot_data(directory,communication,capture,t0,run['mode'],primary if main_rows else None)
    communication['plots']={'loss':'communication_loss.png','latency':'communication_latency.png',
        'data':'communication_plot_data.json','time_origin_ros_s':plot_data['time_origin_ros_s'],
        'time_origin_reference':plot_data['time_origin_reference'],
        'loss_window_s':plot_data['loss_window_s'],'latency_window_s':plot_data['latency_window_s']}
    metrics = {'schema_version': 2, 'run_id': run['id'], 'generated_at': now(),
        'task_state': terminal.get('state', run['state']), 'mode': run['mode'],
        'projection': run['projection'], 'position_reference': 'base_link', 'sources': sources,
        'reference_points': len(reference),
        'reference_polyline_length_m': sum(math.dist(a,b) for a,b in zip(reference,reference[1:])),
        'phases': phases, 'motion_by_topic': speeds,
        'telemetry_counts': dict(Counter(r['topic'] for r in mgx)),
        'missing_topics': missing, 'capture': capture, 'communication':communication,
        'tracking_assessment':assessment, 'notes': notes}
    if run.get('coverage_quality_version') == 1 and run['mode'] == 'coverage':
        try:
            from .coverage_quality import completed_quality
            quality = completed_quality(directory, metrics)
            metrics['coverage_quality'] = {k:quality[k] for k in ('planned','actual')}
            atomic_json(output/'coverage_quality_status.json',{'state':'SUCCEEDED','updated_at':now()})
        except Exception as exc:
            notes.append(f'覆盖作业质量评价失败，不能作为零漏作/零重叠结论：{exc}')
            atomic_json(output/'coverage_quality_status.json',{'state':'FAILED','reason':str(exc),'updated_at':now()})
    atomic_json(output / 'metrics.json', metrics)
    atomic_json(output / 'communication.json', communication)
    atomic_json(output / 'communication_plot_data.json', plot_data)
    if communication['available']:
        atomic_json(directory / 'communication/summary.json', communication)
    with (output / 'metrics.csv').open('w', encoding='utf-8', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['phase', 'indicator', 'unit', 'samples', 'mean', 'mae', 'rmse', 'abs_p95', 'abs_max','std','abs_median','abs_p99'])
        for phase, values in phases.items():
            for name, unit in (('cross_track_m', 'm'), ('target_distance_m', 'm'), ('heading_error_rad', 'rad'), ('heading_error_deg', 'deg')):
                item = values[name]
                if item['samples']:
                    writer.writerow([phase, name, unit] + [item[k] for k in ('samples','mean','mae','rmse','abs_p95','abs_max','std','abs_median','abs_p99')])
    # Matplotlib is used only in this background/offline worker; never on the control path.
    with PLOT_LOCK:
        plot_report(output, mission, {**run, 'state': metrics['task_state']}, reference, phase_rows, motion, mgx, t0, primary)
        plot_communication(output,plot_data,communication,run['id'])
    write_summary(directory,mission,metrics)
    if (output/'coverage_quality_report.md').exists():
        with (output/'report.md').open('a',encoding='utf-8') as stream:
            stream.write('\n## 全覆盖作业质量\n\n[规划与实际轨迹覆盖、漏作及重复作业评价](coverage_quality_report.md)\n')
    lines = ['# Task analysis', '', '[本轮实验结果汇总与跟踪精度评价](report.md)', '', f'Run: `{run["id"]}`', f'State: `{metrics["task_state"]}`', '',
             '![Trajectory, speed, lateral and heading error](trajectory_and_speed.png)', '',
             '![Observed message missing ratios](communication_loss.png)', '',
             '![Message age and backend handling latency](communication_latency.png)', '',
             '## Data and limitations', '', *['- ' + note for note in notes], '',
             'Detailed per-phase statistics: `metrics.json` and `metrics.csv`.',
             'Raw capture integrity: `../telemetry/recording.json`.', '']
    (output / 'README.md').write_text('\n'.join(lines), encoding='utf-8')
    if (output/'coverage_quality_report.md').exists():
        with (output/'README.md').open('a',encoding='utf-8') as stream:
            stream.write('\n[全覆盖作业质量评价报告](coverage_quality_report.md)\n')
    atomic_json(output / 'status.json', {'state': 'SUCCEEDED', 'updated_at': now(),
        'missing_topics': missing, 'task_state': metrics['task_state']})
    return metrics


def plot_report(output, mission, run, reference, phases, motion, mgx, t0, primary):
    os.environ.setdefault('MPLCONFIGDIR', str(ROOT / '.diagnostics/matplotlib'))
    Path(os.environ['MPLCONFIGDIR']).mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def trajectory_plot(ax):
        for i, ring in enumerate(mission.get('preview', {}).get('rings', [])):
            ax.plot([p['x'] for p in ring], [p['y'] for p in ring], 'k-', lw=1, label='Field boundary' if i == 0 else 'Exclusion')
        if reference:
            ax.plot(*zip(*reference), '--', color='tab:orange', label='Reference')
            ax.scatter(*reference[0], c='green', s=25, label='Reference start')
            ax.scatter(*reference[-1], c='red', marker='x', s=25, label='Reference end')
        for phase, rows in phases.items():
            if rows:
                if phase == 'alignment':
                    # Multiple stationary alignments can occur far apart along
                    # the route; do not draw a fictitious joining trajectory.
                    ax.scatter([r['x'] for r in rows], [r['y'] for r in rows],
                               s=3, color='tab:purple', label='Alignment (separate)')
                    continue
                ax.plot([r['x'] for r in rows], [r['y'] for r in rows],
                    lw=.8, label='Approach (separate)' if phase == 'approach' else 'Recorded '+phase,
                    marker='.' if len(rows) < 3 else None,
                    color='gray' if phase == 'approach' else 'tab:blue',
                    linestyle=':' if phase == 'approach' else '-')
        ax.set(title='Reference and recorded trajectory', xlabel='Map east (m)', ylabel='Map north (m)')
        ax.set_aspect('equal', adjustable='datalim')
        if reference or any(phases.values()):
            ax.legend(fontsize=7)

    def speed_plot(ax):
        plotted = False
        missing_labels = []
        # Use receipt ROS time for unstamped commands; retain source stamps for MGX23.
        for topic, label in (('/cmd_vel', 'Safety command vx'), ('/base/driver_cmd_vel', 'Driver command vx'),
                             ('/vel_raw', 'Chassis feedback vx')):
            rows = [(number(r['received_ros_ns']), number(r['linear_x'])) for r in motion if r['topic'] == topic]
            rows = [(stamp/1e9-t0, v) for stamp,v in rows if stamp is not None and v is not None and t0 is not None]
            if rows:
                ax.plot(*zip(*rows), lw=.7, alpha=.8, label=label, marker='.' if len(rows) < 3 else None)
                plotted = True
            else:
                missing_labels.append(label)
        rows = []
        for r in mgx:
            if r['topic'] != '/mgx23/velocity_stamped' or t0 is None:
                continue
            stamp, n, e = (number(r[k]) for k in ('source_stamp_ns','v1','v2'))
            if None not in (stamp, n, e):
                rows.append((stamp/1e9-t0, math.hypot(n,e)))
        if rows:
            ax.plot(*zip(*rows), lw=.8, label='MGX23 measured horizontal speed', marker='.' if len(rows) < 3 else None)
            plotted = True
        else:
            missing_labels.append('MGX23 speed')
        target = number(mission.get('parameters', {}).get('speed', mission.get('parameters', {}).get('max_linear')))
        if target is not None:
            ax.axhline(target, c='red', linestyle=':', lw=.8, label='Task configured speed (reference)')
        if plotted:
            ax.legend(fontsize=7)
            if missing_labels:
                ax.text(.02,.02,'Missing: '+', '.join(missing_labels),fontsize=7,transform=ax.transAxes,wrap=True)
        else:
            ax.text(.5,.5,'No measured/command speed samples recorded',ha='center',transform=ax.transAxes,wrap=True)
        # Negative times are approach; do not imply the full plot is coverage-only.
        origin = primary+' start' if phases.get(primary) else 'first recorded sample'
        ax.set(title='Speed: commands and measurements', xlabel='Time relative to '+origin+' (s)', ylabel='Speed (m/s)')

    def error_plot(ax, field, unit, title):
        rows = phases.get(primary, [])
        points = [(r['stamp']-t0, math.degrees(r[field]) if unit == 'deg' else r[field])
                  for r in rows if r.get(field) is not None and t0 is not None]
        if points:
            ax.plot(*zip(*points), lw=.8, color='tab:blue')
            ax.axhline(0, c='gray', lw=.6)
        else:
            ax.text(.5,.5,'No '+field+' samples',ha='center',transform=ax.transAxes)
        ax.set(title=title+' ('+primary+' only)', xlabel='Elapsed (s)', ylabel='Error ('+unit+')')

    plotters = [trajectory_plot, speed_plot,
        lambda ax: error_plot(ax, 'cross_track' if primary == 'tracking' else 'distance', 'm',
                              'Signed cross-track error' if primary == 'tracking' else 'Distance to active waypoint'),
        lambda ax: error_plot(ax, 'heading_error', 'deg', 'Heading error')]
    names = ['trajectory', 'speed', 'cross_track_error' if primary == 'tracking' else 'target_distance', 'heading_error']
    fig, axes = plt.subplots(2,2,figsize=(15,11),constrained_layout=True)
    try:
        fig.suptitle(f'{run["id"]} - '+str(run.get('state','UNKNOWN')))
        for ax, draw in zip(axes.flat, plotters):
            draw(ax)
            ax.grid(True,alpha=.25)
        fig.savefig(output / 'trajectory_and_speed.png',dpi=160)
    finally:
        plt.close(fig)
    for name, draw in zip(names, plotters):
        fig, ax = plt.subplots(figsize=(9,6),constrained_layout=True)
        try:
            draw(ax)
            ax.grid(True,alpha=.25)
            fig.savefig(output / f'{name}.png',dpi=160)
        finally:
            plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    metrics = analyze_run(args.directory)
    print(json.dumps({'analysis': str(args.directory / 'analysis'), 'phases': metrics['phases']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
