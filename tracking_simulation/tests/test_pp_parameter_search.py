"""Search acceptance must reject incomplete or misleading improvements."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tune_pp import policy_check
from genetic_search import minimize


def baseline():
    return dict(state='COMPLETED', common_progress_rmse_cm=3., p95_cm=6., max_cm=12.,
                regions={'core': {'rmse_cm': 1.}}, angular_command_total_variation_per_s=.1,
                command_v_mean_m_s=.49, duration_s=140.)


def test_error_improvement_cannot_hide_incomplete_run():
    reference=baseline(); result=deepcopy(reference)
    result.update(state='TIME_LIMIT', common_progress_rmse_cm=.01)
    assert 'not_completed' in policy_check(result,reference)


@pytest.mark.parametrize('changes,reason',[
    ({'command_v_mean_m_s': .4}, 'slower'),
    ({'duration_s': 150.}, 'duration'),
    ({'angular_command_total_variation_per_s': .13}, 'angular_variation'),
    ({'p95_cm': 7.}, 'p95_cm'),
    ({'max_cm': 14.}, 'max_cm'),
    ({'regions': {'core': {'rmse_cm': 1.2}}}, 'core_rmse'),
])
def test_overall_rmse_cannot_hide_region_smoothness_or_speed_regression(changes,reason):
    reference=baseline(); result=deepcopy(reference)
    result.update(common_progress_rmse_cm=2., **changes)
    assert reason in policy_check(result,reference)


def test_unchanged_candidate_and_small_core_tradeoff_are_explicitly_allowed():
    reference=baseline()
    assert policy_check(reference,reference)==[]
    result=deepcopy(reference)
    result.update(common_progress_rmse_cm=2.5,regions={'core': {'rmse_cm': 1.1}})
    assert policy_check(result,reference)==[]


def test_ga_evaluates_new_population_and_preserves_best_without_mutating_initial():
    initial=np.array([.95,.95]);saved=initial.copy();seen=[]
    def objective(genes):
        assert all(0.<=x<=1. for x in genes)
        seen.append(genes)
        return sum((x-.35)**2 for x in genes)
    best,history=minimize(objective,[(0.,1.),(0.,1.)],initial=[initial],seed=19,
                          population_size=12,generations=10)
    assert np.array_equal(initial,saved)
    assert best.cost==pytest.approx(objective(best.genes))
    assert best.cost<.01
    assert all(b['best_cost']<=a['best_cost'] for a,b in zip(history,history[1:]))
    assert len(seen)==121
    assert all(r['distinct_population']>2 for r in history)


def test_ga_seed_is_reproducible_and_initial_bounds_are_enforced():
    evaluate=lambda g:sum(x*x for x in g)
    a=minimize(evaluate,[(-1.,1.)]*3,seed=5)
    b=minimize(evaluate,[(-1.,1.)]*3,seed=5)
    assert a==b
    with pytest.raises(ValueError):minimize(evaluate,[(0.,1.)],initial=[(1.1,)])


def test_nondefault_tuning_gives_identical_local_and_production_motion():
    from four_controler.four_controler import PurePursuit as LocalPP
    from agri_path_tracking.algorithms import PurePursuit as ProductionPP
    from simulate import simulate,SimulationConfig,scenario
    tuning=dict(lookahead_gain=.44,curvature_gain=.18)
    config=SimulationConfig(lookahead=.76,speed=.5,duration=120.,initial_lateral=.1,
                            initial_heading_deg=3.,feedback_period=.1,
                            actuator_linear_tau=.15,actuator_angular_tau=.3)
    results=[]
    for cls in (LocalPP,ProductionPP):
        rows,m=simulate(scenario('sine'),config,
            controller_factory=lambda p,**kw:cls(p,tuning=tuning,**kw))
        assert m['state']=='COMPLETED'
        results.append([{k:v for k,v in row.items() if k!='compute_ms'} for row in rows])
    assert results[0]==results[1]


def test_p1_station_sampling_uses_first_forward_crossing_and_rejects_time_gaps():
    from report_pp_ga import first_crossings
    u=np.array([0.,1.,.2,1.2]);e=np.array([2.,2.,20.,20.]);t=np.array([0.,.1,.2,.3])
    sampled=first_crossings(u,e,t,[.4,.6,.8])
    assert sampled==pytest.approx([2.,2.,2.])
    assert np.std(sampled,ddof=1)==0.  # constant offset is not tracking precision
    assert np.sqrt(np.mean(sampled**2))==2.
    assert first_crossings(np.array([0.,1.]),np.array([0.,1.]),np.array([0.,1.]),[.5]) is None
