"""Offline figures from the exact 50-point formula inputs, without re-filtering."""
from collections import defaultdict
import math
import os
from pathlib import Path
import textwrap

from .missions import ROOT
from .standard_accuracy import N, PLOT_FILES, TITLES


def plot_series(result, samples):
    """One independent distance origin per observed section; never join gaps."""
    grouped = defaultdict(list)
    for sample in samples:
        grouped[(sample['indicator'], sample['section_id'])].append(sample)
    series = {kind: [] for kind in TITLES}
    for kind in TITLES:
        for number, section in enumerate(result[kind]['sections'], 1):
            if section['state'] != 'AVAILABLE':
                continue
            points = sorted(grouped[(kind, section['id'])], key=lambda p: p['sample_index'])
            if len(points) != N or [p['sample_index'] for p in points] != list(range(1, N+1)):
                raise ValueError(f'绘图输入不是完整50点：{section["id"]}')
            stations = [float(p['station_m']) for p in points]
            errors = [float(p['error_cm']) for p in points]
            if (not all(math.isfinite(v) for v in stations+errors)
                    or any(b <= a for a, b in zip(stations, stations[1:]))):
                raise ValueError(f'绘图站位或误差无效：{section["id"]}')
            series[kind].append({'id': section['id'], 'number': number,
                'distance_m': [s-stations[0] for s in stations], 'error_cm': errors,
                'precision_cm': section['precision_cm'], 'mean_input_cm': section['mean_input_cm']})
    return series


def plot_standard_accuracy(output, result, samples, run_id):
    # Only called by the finished-task analysis worker under its plot lock.
    os.environ.setdefault('MPLCONFIGDIR', str(ROOT/'.diagnostics/matplotlib'))
    Path(os.environ['MPLCONFIGDIR']).mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.font_manager import fontManager
    from matplotlib.ticker import MaxNLocator

    data = plot_series(result, samples)
    names = {font.name for font in fontManager.ttflist}
    font = next((name for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'WenQuanYi Zen Hei',
                                  'Droid Sans Fallback') if name in names), None)
    chinese = font is not None
    settings = {'font.family': font or 'DejaVu Sans', 'axes.unicode_minus': False}
    english = {'straight': 'Straight work accuracy', 'handover': 'Adjacent-row accuracy',
               'curve': 'Curved work accuracy'}

    def label(zh, en):
        return zh if chinese else en

    with plt.rc_context(settings):
        for kind, filename in PLOT_FILES.items():
            item, groups = result[kind], data[kind]
            title = TITLES[kind] if chinese else english[kind]
            formula = 'P2' if kind == 'handover' else 'P1'
            fig, (ax, scores) = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True,
                                            gridspec_kw={'height_ratios': [3, 1.4]})
            try:
                fig.suptitle(f'{title} | {run_id}', fontsize=13)
                ax.set_xlabel(label('本观测段起点起算距离 (m)', 'Distance from observed section start (m)'))
                ylabel = {'straight': ('有符号横向偏差 (cm)', 'Signed lateral offset (cm)'),
                          'handover': ('交接行间距误差 |h-H| (cm)', 'Absolute spacing error |h-H| (cm)'),
                          'curve': ('到对应参考曲线的最小距离 (cm)', 'Minimum distance to reference curve (cm)')}
                ax.set_ylabel(label(*ylabel[kind]))
                scores.set_xlabel(label('分段序号（对应CSV中本项的分段顺序）', 'Section number (same per-metric order as CSV)'))
                scores.set_ylabel(f'{formula} (cm)')
                for axis in (ax, scores):
                    axis.grid(True, alpha=.25)
                if groups:
                    worst = max(groups, key=lambda group: group['precision_cm'])
                    others = [list(zip(group['distance_m'], group['error_cm']))
                              for group in groups if group is not worst]
                    if others:
                        ax.add_collection(LineCollection(others, colors='0.65', linewidths=.7, alpha=.6,
                            label=label('其他有效段（分别起算）', 'Other sections (independent origins)')))
                    ax.plot(worst['distance_m'], worst['error_cm'], '.-', lw=1.3, markersize=4,
                            color='tab:blue', label=label('最大值所在段：', 'Worst section: ')+worst['id'])
                    ax.axhline(0, color='0.35', lw=.7, linestyle=':')
                    ax.axhline(worst['mean_input_cm'], color='tab:orange', lw=.9, linestyle='--',
                               label=label('该段输入均值', 'Mean input of worst section'))
                    ax.autoscale_view()
                    ax.legend(fontsize=8, loc='best')
                    ax.set_title(label('50点原值；未再次滤波或合并观测段',
                                       'Exact 50-point inputs; no additional filtering or joined sections'), fontsize=10)
                    scores.scatter([g['number'] for g in groups], [g['precision_cm'] for g in groups],
                                   s=15, color='0.5')
                    scores.scatter([worst['number']], [worst['precision_cm']], s=45, color='tab:red', zorder=3)
                    scores.xaxis.set_major_locator(MaxNLocator(integer=True))
                    scores.set_ylim(bottom=0)
                    scores.set_title(label(
                        f'有效段 {len(groups)}；最大 {formula}={worst["precision_cm"]:.6f} cm；该段输入均值={worst["mean_input_cm"]:.6f} cm',
                        f'{len(groups)} valid sections; max {formula}={worst["precision_cm"]:.6f} cm; mean input={worst["mean_input_cm"]:.6f} cm'), fontsize=10)
                else:
                    reason = item.get('reason') or label('没有适用观测段', 'No applicable observations')
                    message = (f'未测 / 不适用：{item["state"]}\n'+textwrap.fill(reason, 46) if chinese else
                               f'Not measured / not applicable: {item["state"]}\nSee the report for the recorded reason.')
                    ax.text(.5, .5, message, ha='center', va='center', transform=ax.transAxes, fontsize=11)
                    scores.text(.5, .5, label('无有效分段；未以0替代', 'No valid sections; missing values are not zero'),
                                ha='center', va='center', transform=scores.transAxes)
                    for axis in (ax, scores):
                        axis.set_xticks([]); axis.set_yticks([])
                fig.supxlabel(label(
                    '每段N=50，公式分母N-1；同一MGX23记录的工程统计，未核验完整国标工况。恒定偏差的标准差可能为0。',
                    'N=50 per section, denominator N-1. Same-MGX23 engineering statistics, not full standard conformance. Constant bias can have zero SD.'), fontsize=9)
                fig.savefig(Path(output)/filename, dpi=160)
            finally:
                plt.close(fig)
