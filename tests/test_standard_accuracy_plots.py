"""Plot inputs must reproduce the stored standard-formula samples exactly."""
import copy
import math

import pytest

from agri_backend.standard_accuracy import N, PLOT_FILES, empty_result, formula_1_cm, write_artifacts
from agri_backend.standard_accuracy_plots import plot_series, plot_standard_accuracy


def measured_result():
    result = empty_result('synthetic')
    samples = []
    for kind in PLOT_FILES:
        sections = []
        for number in (1, 2):
            errors = [(number * math.sin(i*.2) if kind == 'straight' else
                       number * (1 + math.sin(i*.2))) for i in range(N)]
            identity = f'{kind}_window_{number}'
            sections.append({'id':identity, 'state':'AVAILABLE', 'samples':N,
                             'precision_cm':formula_1_cm([x/100 for x in errors]),
                             'mean_input_cm':sum(errors)/N})
            # Deliberately shuffled input and widely separated absolute stations.
            samples.extend({'indicator':kind, 'section_id':identity, 'sample_index':i+1,
                            'station_m':100*number+i*.2, 'error_cm':error}
                           for i, error in reversed(list(enumerate(errors))))
        result[kind] = {'available':True, 'state':'AVAILABLE', 'reason':'',
                        'value_cm':max(s['precision_cm'] for s in sections),
                        'valid_sections':len(sections), 'sections':sections}
    return result, samples


def test_plot_inputs_keep_exact_errors_and_independent_section_origins():
    result, samples = measured_result()
    before = copy.deepcopy((result, samples))
    data = plot_series(result, samples)
    for kind, groups in data.items():
        assert len(groups) == 2
        for group in groups:
            source = sorted((p for p in samples if p['indicator']==kind and p['section_id']==group['id']),
                            key=lambda p:p['sample_index'])
            assert group['distance_m'] == pytest.approx([i*.2 for i in range(N)])
            assert group['error_cm'] == [p['error_cm'] for p in source]
            assert group['precision_cm'] == result[kind]['sections'][group['number']-1]['precision_cm']
    assert (result, samples) == before


@pytest.mark.parametrize('damage', ['missing', 'duplicate', 'nan', 'reversed_station'])
def test_damaged_fifty_point_plot_inputs_are_not_silently_rendered(damage):
    result, samples = measured_result()
    if damage == 'missing':samples.pop()
    elif damage == 'duplicate':samples[0]['sample_index'] = samples[1]['sample_index']
    elif damage == 'nan':samples[0]['error_cm'] = float('nan')
    else:samples[0]['station_m'] = -1
    with pytest.raises(ValueError):plot_series(result, samples)


def test_three_figures_show_raw_errors_precision_values_and_all_sections(tmp_path, monkeypatch):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.figure import Figure
    from matplotlib.collections import LineCollection
    result, samples = measured_result()
    captured = {}
    savefig = Figure.savefig

    def inspect(fig, path, **kwargs):
        kind = next(key for key, name in PLOT_FILES.items() if name == path.name)
        ax, scores = fig.axes
        curve = next(line for line in ax.lines if len(line.get_xdata()) == N)
        expected = sorted((p for p in samples if p['indicator']==kind and p['section_id']==f'{kind}_window_2'),
                          key=lambda p:p['sample_index'])
        assert list(curve.get_ydata()) == [p['error_cm'] for p in expected]
        assert list(curve.get_xdata()) == pytest.approx([i*.2 for i in range(N)])
        other = next(c for c in ax.collections if isinstance(c, LineCollection))
        assert len(other.get_segments()) == 1
        assert len(other.get_segments()[0]) == N
        assert scores.collections[0].get_offsets()[:, 1].tolist() == [s['precision_cm'] for s in result[kind]['sections']]
        assert f'{result[kind]["value_cm"]:.6f}' in scores.get_title()
        captured[kind] = True
        return savefig(fig, path, **kwargs)

    monkeypatch.setattr(Figure, 'savefig', inspect)
    before = copy.deepcopy((result, samples))
    plot_standard_accuracy(tmp_path, result, samples, 'synthetic-three-metrics')
    assert set(captured) == set(PLOT_FILES)
    assert (result, samples) == before
    assert not plt.get_fignums()
    for name in PLOT_FILES.values():
        assert (tmp_path/name).read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
        assert (tmp_path/name).stat().st_size > 10000


def test_missing_metrics_draw_explanations_without_zero_curves_and_reports_embed_images(tmp_path, monkeypatch):
    from matplotlib.figure import Figure
    result = empty_result('有效移动样本不足50个')
    (tmp_path/'analysis').mkdir()
    write_artifacts(tmp_path, result, [])
    savefig = Figure.savefig

    def inspect(fig, path, **kwargs):
        assert not any(ax.lines or ax.collections for ax in fig.axes)
        assert any('UNAVAILABLE' in text.get_text() for text in fig.axes[0].texts)
        return savefig(fig, path, **kwargs)

    monkeypatch.setattr(Figure, 'savefig', inspect)
    plot_standard_accuracy(tmp_path/'analysis', result, [], 'missing-observations')
    report = (tmp_path/'analysis/standard_accuracy_report.md').read_text()
    for name in PLOT_FILES.values():
        assert f']({name})' in report
        assert (tmp_path/'analysis'/name).stat().st_size > 10000


def test_failed_save_closes_the_figure(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    from matplotlib.figure import Figure
    def fail(*args, **kwargs):raise OSError('synthetic disk write failure')
    monkeypatch.setattr(Figure, 'savefig', fail)
    with pytest.raises(OSError, match='disk write'):
        plot_standard_accuracy(tmp_path, empty_result('not observed'), [], 'synthetic')
    assert not plt.get_fignums()
