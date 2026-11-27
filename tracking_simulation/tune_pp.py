#!/usr/bin/env python3
"""Bounded offline GA search of PP L0/T/K; never writes live settings.

Models freeze before search. Only first PP initial condition is used to rank
candidates; the second recorded initial condition and stress cases are evaluated
after selection. A GA candidate is not a real-vehicle or global optimum.
"""
import argparse
from copy import deepcopy
import csv
from dataclasses import asdict
import hashlib
from itertools import product
import json
import math
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
for key, relative in (('TMPDIR', '.cache/tmp'), ('MPLCONFIGDIR', '.cache/matplotlib'),
                      ('ROS_LOG_DIR', '.cache/ros')):
    os.environ.setdefault(key, str(HERE / relative))
    Path(os.environ[key]).mkdir(parents=True, exist_ok=True)
sys.dont_write_bytecode = True

import numpy as np
import yaml
from four_controler.four_controler import PurePursuit, PP_DEFAULTS
from ground_model import load_profile
from genetic_search import minimize
from simulate import SimulationConfig, simulate, write_outputs
from validate_pp_lookahead import load_case, score, TRAIN, HOLDOUT, ROOT


PARAMETERS = ('lookahead', 'lookahead_gain', 'curvature_gain', 'lookahead_min',
              'lookahead_max', 'curvature_window')
BASELINE = dict(lookahead=.8, **{k: PP_DEFAULTS[k] for k in PARAMETERS[1:]})
GENES = ('lookahead', 'lookahead_gain', 'curvature_gain')
BOUNDS = ((.45, 1.2), (0., 1.), (0., 1.))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity(parameters):
    return tuple(round(float(parameters[k]), 8) for k in PARAMETERS)


def policy_check(result, baseline):
    """Predeclared screening tolerances, engineering choices, not standards."""
    reasons = []
    if result['state'] != 'COMPLETED': reasons.append('not_completed')
    for key, ratio in (('common_progress_rmse_cm', 1.05), ('p95_cm', 1.10), ('max_cm', 1.10)):
        if result[key] > baseline[key] * ratio + 1e-9: reasons.append(key)
    core = result['regions']['core']['rmse_cm']
    base_core = baseline['regions']['core']['rmse_cm']
    if core is None or core > max(base_core * 1.15, base_core + .15): reasons.append('core_rmse')
    if result['angular_command_total_variation_per_s'] > baseline['angular_command_total_variation_per_s'] * 1.25:
        reasons.append('angular_variation')
    if result['command_v_mean_m_s'] < baseline['command_v_mean_m_s'] * .98: reasons.append('slower')
    if result['duration_s'] > baseline['duration_s'] * 1.05: reasons.append('duration')
    return reasons


def evaluate(case, profile, parameters, speed, seed=0, noise=False, output=None):
    settings = {k: case['mission']['parameters'][k] for k in
                ('lookahead', 'speed', 'max_angular', 'completion_tolerance', 'search_window')}
    settings.update(lookahead=parameters['lookahead'], speed=speed)
    cfg = SimulationConfig(**settings, dt=.05, duration=400., initial_lateral=0.,
                           initial_heading_deg=0., feedback_period=profile['feedback_period_s'] if profile else 0.,
                           position_noise=.03 if noise else 0., heading_noise_deg=.5 if noise else 0., seed=seed)
    tuning = dict(PP_DEFAULTS, **{k: parameters[k] for k in PARAMETERS[1:]})
    holder = {}
    def factory(path, **kwargs):
        holder['tracker'] = PurePursuit(path, tuning=tuning, **kwargs)
        return holder['tracker']
    factory.supports_segmented = True
    rows, metrics = simulate(case['poses'], cfg, initial_pose=case['initial'],
                             initial_progress=case['progress'], controller_factory=factory, model_profile=profile)
    result = score(rows, metrics, case)
    # score() uses recorded max_angular (held fixed), and segments common to all candidates.
    formal = [r for r in rows if r['phase'] == 'TRACKING']
    result['reference_length_m'] = metrics['reference_length_m']
    if output:
        output.mkdir(parents=True, exist_ok=False)
        metrics.update(source=case['source'], pp_parameters=parameters, simulation=asdict(cfg))
        write_outputs(output, case['poses'], rows, metrics)
    return result, formal


def run(output):
    output = Path(output).resolve()
    if output == HERE/'results' or not output.is_relative_to(HERE/'results'):
        raise ValueError('Output must be a new subdirectory of tracking_simulation/results')
    if output.exists(): raise ValueError('Refuse to overwrite any existing result directory')
    output.mkdir(parents=True)
    first = load_case(TRAIN)
    v2, _ = load_profile(HERE/'profiles/x3_hard_ground_v2.json')
    probe_file = HERE/'profiles/x3_pp_response_probe_20261127.json'
    probe, _ = load_profile(probe_file)
    # No refit here: both existing models are frozen, independently of the candidate parameters.
    sources = dict(first['source_sha256'])
    for path in (HERE/'profiles/x3_hard_ground_v2.json', probe_file, Path(__file__),
                 HERE/'simulate.py', HERE/'four_controler/four_controler.py',
                 HERE/'ground_model.py', HERE/'campaign_model.py', HERE/'observation.py',
                 HERE/'genetic_search.py',
                 HERE/'validate_pp_lookahead.py', HERE/'../agri_robot_ws/src/agri_path_tracking/agri_path_tracking/controller.py'):
        sources[str(path.resolve().relative_to(ROOT))] = digest(path)
    protected = [ROOT/'config/jetson_system.yaml', HERE/'config.yaml', HERE/'configs/x3_hard_ground.yaml',
                 ROOT/'agri_robot_ws/src/agri_path_tracking/agri_path_tracking/algorithms.py']
    protected_sha = {str(p.relative_to(ROOT)): digest(p) for p in protected}
    train = [('v2_050', v2, .5), ('probe_050', probe, .5), ('v2_030', v2, .3)]
    protocol = dict(baseline=BASELINE, training_case=TRAIN, validation_case=HOLDOUT,
        training_scenarios=[dict(name=n, speed=v, seed=0) for n,p,v in train],
        objective='minimum equal-scenario mean of candidate/baseline common-progress lateral RMSE',
        constraints=dict(completion=True, per_scenario_rmse_ratio_max=1.05, p95_ratio_max=1.10,
            peak_ratio_max=1.10, core_rmse_max='max(baseline*1.15,baseline+0.15cm)',
            angular_total_variation_ratio_max=1.25, command_speed_ratio_min=.98, duration_ratio_max=1.05),
        fixed=dict(wheelbase=.16, rear_offset=0., max_steer_deg=35., max_angular=1.,
                   control_dt=.05, speed_planner='unchanged'),
        optimizer=dict(method='real-coded genetic algorithm', genes=GENES, bounds=BOUNDS,
            population_size=12,generations=7,seeds=[27,2811],elites=2,tournament_size=3,
            mutation_probability=.35,immigrants=1,
            note='Two independent seeded searches; cached duplicate evaluations shared'),
        fixed_lookahead=dict(minimum_m=.65,maximum_m=1.05,curvature_window_m=1.),
        selection='Frozen before validation; no retuning based on validation',
        limitations=['Same recorded path, not a new-path field trial',
                     '0.30m/s PP is a model scenario, not a recorded PP trial',
                     'Stress noise and tau multipliers are engineering scenarios, not confidence intervals',
                     'Frozen models do not reproduce all real-vehicle behaviour; no real-vehicle optimum claim'])
    (output/'protocol.json').write_text(json.dumps(protocol, ensure_ascii=False, indent=2))
    (output/'models').mkdir()
    for name,p in [('v2',v2),('probe',probe)]:
        (output/'models'/f'{name}.json').write_text(json.dumps(p, ensure_ascii=False, indent=2))
    baselines = []
    for name,p,v in train:
        r,_ = evaluate(first,p,BASELINE,v)
        if r['state'] != 'COMPLETED': raise ValueError('Baseline did not complete: '+name)
        baselines.append(r)
    cache = {}; records = []
    stream = (output/'search.jsonl').open('w')
    def candidate(parameters, stage):
        key = identity(parameters)
        if key in cache: return cache[key]
        results=[]; reasons=[]
        for (name,p,v), base in zip(train,baselines):
            r,_ = evaluate(first,p,parameters,v)
            r['scenario'] = name
            results.append(r)
            reasons.extend(name+':'+x for x in policy_check(r,base))
        ratios=[r['common_progress_rmse_cm']/b['common_progress_rmse_cm'] for r,b in zip(results,baselines)]
        entry=dict(index=len(records),stage=stage,parameters=parameters,objective=float(np.mean(ratios)),
                   worst_ratio=max(ratios),eligible=not reasons,rejected_by=reasons,results=results)
        records.append(entry);cache[key]=entry
        stream.write(json.dumps(entry,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
        if len(records)%5==0 or entry['index']==0:
            best=min((r for r in records if r['eligible']),key=lambda r:r['objective'],default=None)
            print('search',len(records),stage,'best',None if best is None else
                  (best['index'],round(best['objective'],5),best['parameters']),flush=True)
        return entry
    candidate(dict(BASELINE),'baseline')
    histories=[]
    for ga_seed in protocol['optimizer']['seeds']:
        def objective(genes):
            p=dict(BASELINE,**{k:round(float(v),4) for k,v in zip(GENES,genes)})
            r=candidate(p,f'GA_seed_{ga_seed}')
            return r['objective']+(1000.+len(r['rejected_by']) if not r['eligible'] else 0.)
        def progress(item):
            print('generation',ga_seed,item,flush=True)
        winner,history=minimize(objective,BOUNDS,initial=[tuple(BASELINE[k] for k in GENES)],
            population_size=protocol['optimizer']['population_size'],
            generations=protocol['optimizer']['generations'],seed=ga_seed,on_generation=progress)
        histories.append(dict(seed=ga_seed,history=history,winning_genes=winner.genes,cost=winner.cost))
        (output/'ga_history.json').write_text(json.dumps(histories,ensure_ascii=False,indent=2))
    selected=min((r for r in records if r['eligible']),key=lambda r:r['objective'])
    stream.close()
    (output/'selection.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2))
    print('SELECTION FROZEN',selected['parameters'],selected['objective'],flush=True)
    # Validation starts only after selection.json is written. Never change selection below.
    second=load_case(HOLDOUT);sources.update(second['source_sha256'])
    validation=[]; examples={}
    validation_stream=(output/'validation.jsonl').open('w')
    for model_name,model in [('v2',v2),('probe',probe)]:
        for speed,seed in product((.3,.5),(1,2)):
            validation.append((f'{model_name}_{speed}_{seed}',model,speed,seed,False))
    for seed in (10,11,12):
        validation.append((f'noise_{seed}',probe,.5,seed,True))
    for multiplier in (.8,1.2):
        model=deepcopy(probe)
        for p in model['moving_angular_points']:p['tau_s']*=multiplier
        validation.append((f'tau_{multiplier}',model,.5,3,False))
    validation.append(('ideal_050',None,.5,0,False))
    evaluated=[]
    for index,(name,model,speed,seed,noise) in enumerate(validation):
        pair={}
        for variant,parameters in [('baseline',BASELINE),('selected',selected['parameters'])]:
            save=output/'examples'/name/variant if name in ('v2_0.5_1','probe_0.5_1','noise_10','ideal_050') else None
            r,rows=evaluate(second,model,parameters,speed,seed,noise,save)
            pair[variant]=r
            if save:examples[(name,variant)]=rows
        pair.update(scenario=name,speed=speed,seed=seed,noise=noise,
                    passed=not policy_check(pair['selected'],pair['baseline']),
                    rejected_by=policy_check(pair['selected'],pair['baseline']))
        evaluated.append(pair)
        validation_stream.write(json.dumps(pair,ensure_ascii=False)+'\n');validation_stream.flush()
        print('validation',index+1,name,round(pair['baseline']['rmse_cm'],3),
              round(pair['selected']['rmse_cm'],3),pair['passed'],flush=True)
    validation_stream.close()
    # A runnable candidate YAML preserves all other config and uses frozen v2 for ordinary runs.
    template=yaml.safe_load((HERE/'configs/x3_hard_ground.yaml').read_text())
    template.update(path=str(HERE/'paths/x3_20261127_coverage.json'),controllers=['pp'],
                    model_profile=str(output/'models/v2.json'),experiment=str(second['directory']),
                    experiment_overrides=dict(lookahead=selected['parameters']['lookahead'],speed=.5))
    template['tuning']['pp'].update({k:v for k,v in selected['parameters'].items() if k!='lookahead'})
    (output/'candidate.yaml').write_text(yaml.safe_dump(template,allow_unicode=True,sort_keys=False))
    with (output/'search_summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
        fields=['index','stage','objective','worst_ratio','eligible',*PARAMETERS,'rejected_by']
        w=csv.DictWriter(f,fields);w.writeheader()
        for r in records:w.writerow({**{k:r[k] for k in fields if k not in PARAMETERS},**r['parameters']})
    report=dict(protocol=protocol,selection=selected,training_baselines=baselines,
                ga_history=histories,
                candidates=len(records),training_simulations=3*(1+len(records)),
                validation_simulations=2*len(evaluated),validation=evaluated,
                source_sha256=sources,protected_sha256=protected_sha)
    for path,h in {**sources,**protected_sha}.items():
        if digest(ROOT/path)!=h:raise ValueError('Input/default changed during search: '+path)
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    draw(output,report,examples)
    print('DONE',output,flush=True)
    return report


def draw(output, report, examples):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,2,figsize=(13,12),constrained_layout=True)
    for row,name in enumerate(('v2_0.5_1','probe_0.5_1','noise_10','ideal_050')):
        for variant in ('baseline','selected'):
            data=examples[(name,variant)]
            s=[r['progress_m'] for r in data]
            axes[row,0].plot(s,[r['cross_track_true_m']*100 for r in data],label=variant,lw=.85)
            axes[row,1].plot(s,[r['w_command_rad_s'] for r in data],label=variant,lw=.7)
        for col,unit in enumerate(('Lateral error (cm)','Command yaw rate (rad/s)')):
            axes[row,col].set(title=name,xlabel='Path progress (m)',ylabel=unit)
            axes[row,col].grid(alpha=.25);axes[row,col].legend()
    fig.savefig(output/'validation_comparison.png',dpi=160);plt.close(fig)
    rows=report['validation'];x=np.arange(len(rows))
    fig,ax=plt.subplots(figsize=(13,5),constrained_layout=True)
    for offset,key in ((-.18,'baseline'),(.18,'selected')):
        ax.bar(x+offset,[r[key]['rmse_cm'] for r in rows],.36,label=key)
    ax.set_xticks(x,[r['scenario'] for r in rows],rotation=45,ha='right')
    ax.set(ylabel='Lateral RMSE (cm)',title='Frozen selection: validation scenarios')
    ax.grid(axis='y',alpha=.25);ax.legend();fig.savefig(output/'validation_rmse.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    run(args.output)
