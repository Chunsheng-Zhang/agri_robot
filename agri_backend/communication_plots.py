"""Offline communication plots with explicit sample denominators and one task clock."""
import bisect
import json
import math

from .experiment_communication import distribution, numeric, read_csv

PHASE_COLORS = {'APPROACHING': '#94a3b8', 'ALIGNING': '#b08ae6',
                'TRACKING': '#85c7a0', 'WAYPOINT': '#85c7a0', 'PAUSED': '#f0ca69'}
PHASE_LABELS = {'APPROACHING': 'Approach', 'ALIGNING': 'Align', 'TRACKING': 'Track',
                'WAYPOINT': 'Waypoint', 'PAUSED': 'Pause'}


def phase_intervals(directory, mode, origin):
    path = directory / 'telemetry/status.jsonl'
    if origin is None or not path.exists():
        return []
    topic = '/tracking/status' if mode == 'coverage' else '/waypoint/status'
    rows = []
    for line in path.read_text().splitlines():
        try:
            item = json.loads(line)
            data = item['data']
            stamp = numeric(item['received_ros_ns'])
            if item['topic'] != topic or data.get('run_directory') != str(directory / 'tracks') or stamp is None:
                continue
            phase = ('PAUSED' if data.get('state') == 'PAUSED' else
                     data.get('phase', 'WAYPOINT' if mode == 'waypoint' else None)
                     if data.get('state') == 'RUNNING' else None)
            rows.append((stamp / 1e9 - origin, phase))
        except (ValueError, TypeError, KeyError):
            continue
    intervals = []
    for i, (start, phase) in enumerate(rows):
        if phase not in PHASE_COLORS:
            continue
        # Do not paint an unknown long status outage as known continuous execution.
        end = min(rows[i+1][0], start+1) if i+1 < len(rows) else start+.05
        if end <= start:
            continue
        if intervals and intervals[-1]['phase'] == phase and abs(intervals[-1]['end_s']-start) < 1e-6:
            intervals[-1]['end_s'] = end
        else:
            intervals.append({'phase': phase, 'start_s': start, 'end_s': end})
    return intervals


def build_plot_data(directory, communication, capture, t0, mode, primary):
    timing = read_csv(directory / 'communication/ros_timing.csv')
    http = read_csv(directory / 'communication/http_server.csv')
    chunks = read_csv(directory / 'raw/chunks.csv')
    ros_times = [numeric(r['received_ros_ns']) for r in timing]
    ros_times = [stamp / 1e9 for stamp in ros_times if stamp is not None]
    origin = t0 if t0 is not None else min(ros_times, default=None)
    loss_window, latency_window = 10, 1
    reference = 'formal ' + primary + ' start' if primary else 'first recorded sample'
    if t0 is None:
        reference = 'first communication sample' if origin is not None else 'unavailable'
    anchors = sorted((float(r['received_monotonic_ns']), float(r['received_ros_ns']))
                     for r in timing if numeric(r['received_monotonic_ns']) is not None
                     and numeric(r['received_ros_ns']) is not None)
    anchor_times = [a[0] for a in anchors]

    def elapsed(stamp):
        value = numeric(stamp)
        return value / 1e9 - origin if value is not None and origin is not None else None

    def monotonic_elapsed(stamp):
        value = numeric(stamp)
        if value is None or not anchors or origin is None:
            return None
        index = bisect.bisect_left(anchor_times, value)
        options = [j for j in (index-1, index) if 0 <= j < len(anchors)]
        nearest = min(options, key=lambda j: abs(anchor_times[j]-value))
        mono, ros = anchors[nearest]
        if abs(mono-value) > 1e9:
            return None  # No reliable ROS/monotonic association across an outage.
        return (ros + value - mono) / 1e9 - origin

    extents = [value-origin for value in ros_times] if origin is not None else []
    raw_windows, previous, raw_unplaced = {}, {}, 0
    for row in chunks:
        session, sequence = row['session'], int(row['sequence'])
        old = previous.get(session)
        if old is not None and sequence <= old:
            continue
        expected = sequence-old if old is not None else 1
        previous[session] = sequence
        time = elapsed(row['received_ros_ns'])
        if time is None:
            raw_unplaced += 1
            continue
        index = math.floor(time/loss_window)
        item = raw_windows.setdefault(index, {'expected': 0, 'missing': 0, 'received': 0})
        item['expected'] += expected
        item['missing'] += expected-1
        item['received'] += 1
        extents.append(time)
    # Use the same exact-header comparison and one-second margins as the summary.
    positions = [r for r in timing if r['topic'] == '/mgx23/position'
                 and numeric(r['source_stamp_ns']) is not None and numeric(r['received_monotonic_ns']) is not None]
    receipt = [float(r['received_monotonic_ns']) for r in positions]
    lo, hi = (min(receipt)+1e9, max(receipt)-1e9) if receipt else (1, 0)
    references = {r['source_stamp_ns']: r for r in positions if lo <= float(r['received_monotonic_ns']) <= hi}
    matches = {}
    for topic in ('/mgx23/attitude_stamped', '/mgx23/velocity_stamped', '/odom'):
        stamps = {r['source_stamp_ns'] for r in timing if r['topic'] == topic}
        windows = {}
        for stamp, row in references.items():
            time = elapsed(row['received_ros_ns'])
            if time is None:
                continue
            item = windows.setdefault(math.floor(time/loss_window), {'expected': 0, 'missing': 0})
            item['expected'] += 1
            item['missing'] += stamp not in stamps
        matches[topic] = windows
    latency, server, drops = {}, {}, {}
    for row in timing:
        age, time = numeric(row['source_age_ms']), elapsed(row['received_ros_ns'])
        if age is not None and age >= 0 and time is not None:
            latency.setdefault(row['topic'], {}).setdefault(math.floor(time/latency_window), []).append(age)
    http_unplaced = 0
    for row in http:
        time = monotonic_elapsed(row['received_monotonic_ns'])
        duration = numeric(row['duration_ms'])
        if time is None:
            http_unplaced += 1
        elif duration is not None and duration >= 0:
            server.setdefault(row['path'], {}).setdefault(math.floor(time/latency_window), []).append(duration)
            extents.append(time)
    unplaced_drops = 0
    for event in capture.get('queue_drop_events', []):
        time = monotonic_elapsed(event['received_monotonic_ns'])
        if time is None:
            unplaced_drops += 1
        else:
            drops[math.floor(time/loss_window)] = drops.get(math.floor(time/loss_window), 0)+1
            extents.append(time)
    span = [min(extents), max(extents)] if extents else None

    def ratio_series(windows):
        return [{'start_s': index*loss_window, 'end_s': (index+1)*loss_window,
                 **item, 'percent': 100*item['missing']/item['expected']}
                for index, item in sorted(windows.items())]

    def latency_series(windows, threshold):
        return [{'start_s': index*latency_window, 'end_s': (index+1)*latency_window,
                 **distribution(values, threshold)} for index, values in sorted(windows.items())]

    metadata_path = directory / 'communication/metadata.json'
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    ros_threshold, http_threshold = metadata.get('latency_reference_ms', 100), metadata.get('http_reference_ms', 2000)
    return {'schema_version': 1, 'available': communication.get('available', False),
            'time_origin_ros_s': origin, 'time_origin_reference': reference, 'observed_extent_s': span,
            'loss_window_s': loss_window, 'latency_window_s': latency_window,
            'phases': phase_intervals(directory, mode, origin),
            'raw_missing': ratio_series(raw_windows),
            'matched_missing': {topic: ratio_series(windows) for topic, windows in matches.items()},
            'ros_latency': {topic: latency_series(windows, ros_threshold) for topic, windows in latency.items()},
            'http_latency': {path: latency_series(windows, http_threshold) for path, windows in server.items()},
            'queue_drop_windows': [{'start_s': i*loss_window, 'end_s': (i+1)*loss_window, 'count': n} for i, n in sorted(drops.items())],
            'queue_drop_total': sum(capture['queue_drops_by_topic'].values()) if 'queue_drops_by_topic' in capture else None,
            'queue_drop_events_unplaced': unplaced_drops,
            'queue_drop_events_truncated': capture.get('queue_drop_events_truncated', 0),
            'raw_samples_unplaced': raw_unplaced, 'http_samples_unplaced': http_unplaced,
            'ros_threshold_ms': ros_threshold, 'http_threshold_ms': http_threshold,
            'notes': ['Raw gaps and the corresponding sequence span are assigned to the NEXT received chunk window; missing emission times are unknown.',
                      'No-data windows stay empty; session changes are not losses; all matching uses the complete capture.',
                      'HTTP/drop monotonic times map to the nearest ROS receipt anchor within 1s; Unix time is not assumed to equal ROS time.',
                      'Queue drop events are bounded to 1000; aggregate counts retain the full reported total.',
                      'Phase shading uses matching run status and never bridges status gaps longer than 1s.',
                      'Missing ratios are observed delivery diagnostics, not IP packet loss; delays exclude browser RTT and actuation.']}


def plot_communication(output, data, communication, run_id):
    # Caller configures the Agg backend/cache and holds the existing plot lock.
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    def windows(ax, rows, field, label, **style):
        # Plot each measured bin independently: never join across an empty bin.
        x, y = [], []
        for row in rows:
            x += [row['start_s'], row['end_s'], float('nan')]
            y += [row[field], row[field], float('nan')]
        if x:
            ax.plot(x, y, label=label, **style)
        return bool(x)

    def decorate(axes):
        for ax in axes:
            for interval in data['phases']:
                ax.axvspan(interval['start_s'], interval['end_s'], color=PHASE_COLORS[interval['phase']], alpha=.13, zorder=0)
            if data['observed_extent_s']:
                lo, hi = data['observed_extent_s']
                ax.set_xlim(lo-.2, max(lo+1, hi)+.2)
            ax.grid(True, alpha=.2)
            ax.set_xlabel('Time relative to '+data['time_origin_reference']+' (s)')

    def legend(ax):
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(handles, labels, fontsize=8, loc='upper left', ncol=2)

    phase_names = list(dict.fromkeys(p['phase'] for p in data['phases']))
    phase_handles = [Patch(facecolor=PHASE_COLORS[p], alpha=.4, label=PHASE_LABELS[p]) for p in phase_names]
    def finish(fig, filename):
        if phase_handles:
            ax=fig.axes[0]
            existing=ax.get_legend()
            if existing is not None:
                ax.add_artist(existing)
            ax.legend(handles=phase_handles, loc='upper right', ncol=len(phase_handles), fontsize=8)
        fig.savefig(output / filename, dpi=150)
        plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), constrained_layout=True)
    try:
        fig.suptitle(str(run_id)+' | Observed message missing ratios (NOT IP packet loss)', fontsize=12)
        if not windows(axes[0], data['raw_missing'], 'percent', 'MGX23 raw sequence missing', lw=1.5):
            axes[0].text(.5, .5, 'Not measured: no raw sequence observations', ha='center', transform=axes[0].transAxes)
        axes[0].set(title='Raw chunk missing ratio | 10s bins; gap charged to next received chunk', ylabel='Missing / sequence span (%)')
        measured = False
        for topic, rows in data['matched_missing'].items():
            measured |= windows(axes[1], rows, 'percent', topic, lw=1.3)
        if not measured:
            axes[1].text(.5, .5, 'Not measured: insufficient shared-header reference samples', ha='center', transform=axes[1].transAxes)
        axes[1].set(title='Shared-header unmatched ratio | position references; 1s edge margins', ylabel='Unmatched / references (%)')
        drops = data['queue_drop_windows']
        if drops:
            axes[2].bar([(r['start_s']+r['end_s'])/2 for r in drops], [r['count'] for r in drops], width=8, color='tab:red')
        else:
            text = ('No recorder queue drops reported (total = 0)' if data['queue_drop_total'] == 0 else
                    'Not measured: no recorder counters' if data['queue_drop_total'] is None else
                    'Drop times unavailable; see aggregate count')
            axes[2].text(.5, .5, text, ha='center', transform=axes[2].transAxes)
        axes[2].set(title=f'Recorder queue drops | total={data["queue_drop_total"]}; untimed={data["queue_drop_events_unplaced"]}; truncated={data["queue_drop_events_truncated"]}', ylabel='Observed drop events / 10s')
        for ax, series in ((axes[0], [data['raw_missing']]), (axes[1], list(data['matched_missing'].values()))):
            maximum = max((row['percent'] for rows in series for row in rows), default=0)
            ax.set_ylim(-.05*max(1,maximum), max(1,min(100,maximum*1.2)))
            legend(ax)
        decorate(axes)
        fig.supxlabel('Empty bins are unmeasured. Queue drops can overlap raw gaps; do not add them.', fontsize=9)
        finish(fig, 'communication_loss.png')
    except Exception:
        plt.close(fig)
        raise

    fig, axes = plt.subplots(3, 1, figsize=(14, 12), gridspec_kw={'height_ratios': [3, 2, 3]}, constrained_layout=True)
    try:
        fig.suptitle(str(run_id)+' | Message age and backend handling (NOT browser RTT)', fontsize=12)
        for ax, series, threshold, title in (
                (axes[0], data['ros_latency'], data['ros_threshold_ms'], 'ROS source stamp to recorder | 1s P95 solid; max dotted'),
                (axes[1], data['http_latency'], data['http_threshold_ms'], 'Backend HTTP handling | 1s P95 solid; max dotted')):
            for index, (name, rows) in enumerate(series.items()):
                color = f'C{index%10}'
                windows(ax, rows, 'p95', name, lw=1.2, color=color)
                windows(ax, rows, 'max', '_nolegend_', lw=.8, alpha=.5, linestyle=':', color=color)
            if not series:
                ax.text(.5, .5, 'Not measured: no timed samples', ha='center', transform=ax.transAxes)
            else:
                ax.axhline(threshold, color='tab:red', linestyle='--', lw=.8, label=f'Reference {threshold:g} ms')
            ax.set(title=title, ylabel='Milliseconds')
            ax.set_ylim(bottom=0)
            legend(ax)
        decorate(axes[:2])
        table_rows = []
        for entries, field in ((communication.get('topics', {}), 'source_to_recorder_ms'),
                               (communication.get('http_server', {}), 'server_duration_ms')):
            for name, item in entries.items():
                values = item[field]
                if values['samples']:
                    table_rows.append([name, str(values['samples'])]+[f'{values[k]:.2f}' for k in ('mean', 'p95', 'p99', 'max', 'over_threshold_percent')])
        axes[2].axis('off')
        if table_rows:
            table = axes[2].table(cellText=table_rows, colLabels=['Stream / endpoint', 'N', 'Mean ms', 'P95 ms', 'P99 ms', 'Max ms', 'Over ref %'],
                                  colWidths=[.34, .07, .115, .115, .115, .115, .12], loc='center')
            table.auto_set_font_size(False)
            table.set_fontsize(8)
            table.scale(1, 1.25)
        else:
            axes[2].text(.5, .5, 'Not measured: no latency distributions', ha='center')
        axes[2].set_title(f'Whole-capture statistics | ROS reference {data["ros_threshold_ms"]:g}ms; HTTP reference {data["http_threshold_ms"]:g}ms (diagnostic only)', fontsize=10)
        fig.supxlabel('Unstamped/negative ages are excluded; scan age includes acquisition. Empty bins stay empty.\n'
                      'Phase shading unavailable.' if not phase_names else
                      'Unstamped/negative ages are excluded; scan age includes acquisition. Empty bins stay empty.', fontsize=9)
        finish(fig, 'communication_latency.png')
    except Exception:
        plt.close(fig)
        raise
