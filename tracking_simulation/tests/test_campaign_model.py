import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

SIM=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM))
from calibrate_campaign import initial_profile
from campaign_model import CampaignActuator, estimate_delivery_distribution, replay_response, validate_campaign_profile
from campaign_data import read_campaign_run
from disturbances import GaussianPoseNoise
from ground_model import GroundRun, load_profile
from observation import DelayedPoseSensor
from simulate import SimulationConfig, forward_poses, simulate


def profile():
    p=initial_profile()
    for name in ('linear','angular','in_place_angular'):
        p[name]=dict(gain=1.,limit=1.5,tau_s=.5,delay_s=.2)
    p['stop_tau_s']=[.05,.05]
    p['observation_delay_samples_s']=[.02]
    p['controller_start_phase_s']=.025
    return p


def test_in_place_two_stage_step_and_faster_stop_are_analytic():
    p=profile();a=CampaignActuator(p)
    a.command(0.,1.,0.)
    assert a.advance(.199)==(0.,0.)
    value=a.advance(.7)[1]
    assert value==pytest.approx(1-2*math.exp(-1))
    a.command(0.,0.,.7)
    assert a.advance(.8)[1]==pytest.approx(value*math.exp(-2))


def test_reset_and_mode_change_cancel_old_delayed_targets():
    a=CampaignActuator(profile());a.command(0.,1.,0.)
    a.command(.3,-.2,.1)
    assert a.advance(.25)==(0.,0.)  # old in-place target at .2 was cancelled
    a.reset(.26)
    assert a.advance(3)==(0.,0.)


def test_partition_invariance_and_continuous_mode_switch():
    left,right=CampaignActuator(profile()),CampaignActuator(profile())
    for a in (left,right):a.command(0.,.3,0.)
    for t in np.arange(.001,.701,.001):right.advance(float(t))
    assert left.advance(.7)==pytest.approx(right.output,abs=1e-12)
    previous=left.output[1];left.command(.3,-.2,.7)
    assert left.output[1]==pytest.approx(previous,abs=1e-14)  # no velocity snap


def synthetic_run():
    t=np.arange(0.,12.01,.1)
    ct=np.arange(0.,12.001,.05)
    v=np.where(ct<2,0.,np.where(ct<5,.3,np.where(ct<8,.5,0.)))
    w=np.where(ct<1,.0,np.where(ct<4,.3,np.where(ct<8,-.2,0.)))
    return GroundRun('synthetic',ct,np.column_stack((v,w)),t,np.zeros((len(t),2)),
                     np.zeros((len(t),3)),t>0,{})


def test_free_replay_agrees_with_runtime_causal_response():
    p=profile();p['moving_angular_points']=[dict(speed=.3,gain=.8,tau_s=.4),dict(speed=.5,gain=.9,tau_s=.6)]
    run=synthetic_run();prediction=replay_response(run,p,dt=.001,observed=False)
    a=CampaignActuator(p);values=[];j=0
    for t in run.time:
        while j<len(run.command_time) and run.command_time[j]<=t+1e-9:
            a.command(*run.commands[j],float(run.command_time[j]));j+=1
        values.append(a.advance(float(t)))
    assert np.max(abs(np.asarray(values)-prediction))<.002
    run.measured[:]=100
    assert replay_response(run,p,dt=.001,observed=False)==pytest.approx(prediction)


def test_heading_difference_observation_is_interval_mean_not_instant_rate():
    p=profile();p['in_place_order']=1;p['in_place_angular']['delay_s']=0.
    t=np.arange(0.,1.01,.1)
    run=GroundRun('step',np.array([0.]),np.array([[0.,1.]]),t,np.zeros((len(t),2)),np.zeros((len(t),3)),t>0,{})
    observed=replay_response(run,p,dt=.0001)
    instant=replay_response(run,p,dt=.0001,observed=False)
    expected=1-.5/.1*(1-math.exp(-.1/.5))
    assert observed[1,1]==pytest.approx(expected,abs=1e-6)
    assert instant[1,1]>observed[1,1]


def test_sensor_never_delivers_future_or_overwrites_with_late_old_pose():
    sensor=DelayedPoseSensor(.1,[.3],GaussianPoseNoise(),seed=0)
    sensor.sample(0.,(0.,0.,0.),0.,initial=True)
    sensor.sample(.1,(1.,0.,0.),.3)
    assert sensor.deliver(.2)['source_time_s']==0
    sensor.delays=(0.,)
    sensor.sample(.2,(2.,0.,0.),.3)
    assert sensor.deliver(.2)['value'][0]==2
    assert sensor.deliver(.5)['value'][0]==2


def test_simulated_source_phase_delivery_age_and_timeout_stop():
    p=profile()
    cfg=SimulationConfig(duration=1.,speed=.3,feedback_period=.1,initial_lateral=0.,initial_heading_deg=0.)
    rows,m=simulate(forward_poses([(0.,0.),(20.,0.)]),cfg,model_profile=p)
    assert rows[0]['time_s']==pytest.approx(.025)
    assert all(r['feedback_time_s']<=r['feedback_delivery_time_s']<=r['time_s']+1e-9 for r in rows)
    fresh=[r for r in rows if r['feedback_fresh'] and r['time_s']<1.]
    assert all(r['feedback_age_s']==pytest.approx(.025,abs=1e-7) for r in fresh)
    p['observation_delay_samples_s']=[.5];p['feedback_timeout_s']=.2
    rows,m=simulate(forward_poses([(0.,0.),(20.,0.)]),cfg,model_profile=p)
    assert m['state']=='LOCALIZATION_TIMEOUT'
    assert rows[-1]['v_actual_body_m_s']==0 and rows[-1]['w_actual_rad_s']==0
    assert rows[-1]['phase']=='FAULT'


@pytest.mark.parametrize('value',[-1,float('nan'),float('inf'),True])
def test_profile_rejects_bad_response(value):
    p=profile();p['angular']['gain']=value
    with pytest.raises(ValueError):validate_campaign_profile(p)


def test_profile_file_and_legacy_format_are_separate(tmp_path):
    p=tmp_path/'profile.json';p.write_text(json.dumps(profile()))
    loaded,source=load_profile(p)
    assert loaded['schema_version']==2 and len(source['sha256'])==64
    legacy,_=load_profile(SIM/'profiles/x3_hard_ground_v1.json')
    assert legacy['schema_version']==1


def test_fault_or_manual_intervention_response_is_not_accepted(tmp_path):
    (tmp_path/'config').mkdir();(tmp_path/'telemetry').mkdir()
    (tmp_path/'config/protocol.json').write_text('{}')
    (tmp_path/'telemetry/recording.json').write_text(json.dumps({'terminal':{'state':'FAULT'}}))
    with pytest.raises(ValueError,match='故障或人工干预'):read_campaign_run(tmp_path,allow_basic=True)


def test_interval_censored_delivery_estimator_uses_only_admissible_bins():
    data=[dict(delivery_intervals_s=[[.005,.01]]*30+[[0.,.008]]*20)]
    samples,meta=estimate_delivery_distribution(data)
    assert min(samples)>=.005 and max(samples)<=.008
    assert meta['mean_s']>=.005 and meta['mean_s']<=.008


def test_training_and_holdout_overlap_rejected_before_reading(tmp_path):
    from calibrate_campaign import identify
    with pytest.raises(ValueError,match='互不重合'):identify([tmp_path/'same'],[tmp_path/'same'])


def test_calibrated_clock_rejects_a_different_controller_period():
    with pytest.raises(ValueError,match='20Hz'):
        simulate(forward_poses([(0.,0.),(5.,0.)]),SimulationConfig(dt=.1),model_profile=profile())


def test_controller_input_matching_uses_source_headers_not_recorder_arrival(tmp_path,monkeypatch):
    import campaign_data
    import simulate as simulation_module
    from agri_path_tracking.controller import Polyline
    from four_controler.four_controler import build_controller
    params=dict(speed=.3,lookahead=.8,max_angular=1.,completion_tolerance=.2,search_window=1.)
    tuning=dict(lateral_weight=8.,heading_weight=3.,angular_weight=.5)
    for name in ('config','tracks','paths','telemetry'):(tmp_path/name).mkdir()
    (tmp_path/'config/mission.json').write_text(json.dumps(dict(parameters=params)))
    (tmp_path/'config/tracking_controller.json').write_text(json.dumps(dict(algorithm='lqr',tuning=tuning)))
    (tmp_path/'paths/coverage_test.json').touch();(tmp_path/'tracks/track_test.csv').touch()
    poses=forward_poses([(0.,0.),(20.,0.)])
    monkeypatch.setattr(simulation_module,'load_path',lambda _: (poses,{}))
    controller=build_controller('lqr',dt=.05,tuning={'lqr':tuning})(Polyline([p[:2] for p in poses]),**params)
    track=[];commands=[];previous=(0.,0.)
    for i in range(180):
        stamp=1000+i*.1
        row=dict(stamp=stamp,x=i*.03,y=.005*math.sin(i*.07),yaw=.01*math.sin(i*.13),progress=i*.03)
        controller.progress=row['progress'];out=controller.command(row['x'],row['y'],row['yaw'])[:2]
        track.append(row)
        for offset,value in ((.01,previous if i%2 else out),(.06,out)):
            commands.append(dict(topic='/control/tracking/cmd_vel',source_stamp_ns=round((stamp+offset)*1e9),
                received_ros_ns=round((stamp+offset+5.)*1e9),linear_x=value[0],angular_z=value[1]))
        previous=out
    monkeypatch.setattr(campaign_data,'csv_rows',lambda f: commands if Path(f).name=='motion.csv' else track)
    timing=campaign_data.controller_timing(tmp_path)
    assert timing['matched']>=175
    assert max(timing['first_use_age_s'])<=.061
    assert np.median(timing['sample_to_next_tick_s'])==pytest.approx(.01,abs=1e-6)
    assert 0 in timing['missed_control_ticks'] and 1 in timing['missed_control_ticks']


def test_experiment_preset_is_free_closed_loop_and_snapshot_is_reusable(tmp_path,monkeypatch):
    import run_tracking
    import yaml
    directory=tmp_path/'experiment'
    for name in ('config','tracks','paths'):(directory/name).mkdir(parents=True)
    params=dict(speed=.3,lookahead=.8,max_angular=1.,completion_tolerance=.2,search_window=1.)
    (directory/'config/mission.json').write_text(json.dumps(dict(parameters=params)))
    (directory/'paths/coverage_test.json').write_text(json.dumps({'points':[{'x':0.,'y':0.},{'x':10.,'y':0.}]}))
    track=directory/'tracks/track_test.csv'
    track.write_text('stamp,x,y,yaw,progress\n0,0,0.01,0,0\n1,500,500,1,3\n')
    model=tmp_path/'model.json';model.write_text(json.dumps(profile()))
    config=yaml.safe_load((SIM/'config.yaml').read_text())
    config.update(model_profile='model.json',experiment='experiment')
    configfile=tmp_path/'test.yaml';configfile.write_text(yaml.safe_dump(config))
    captured=[];original=run_tracking.simulate
    def capture(*args,**kwargs):
        rows,metrics=original(*args,**kwargs);captured.append(rows);return rows,metrics
    monkeypatch.setattr(run_tracking,'simulate',capture)
    monkeypatch.setattr(run_tracking,'write_outputs',lambda output,*args: output.mkdir(parents=True,exist_ok=True))
    monkeypatch.setattr(run_tracking,'save_comparison',lambda *args: None)
    output,_=run_tracking.run(configfile,controller='lqr',speed=.4,duration=.4,output=tmp_path/'first',no_animation=True)
    saved=yaml.safe_load((output/'config.yaml').read_text())
    assert saved['model_profile']==str(model)
    assert saved['simulation']['speed']==.4
    assert saved['experiment_overrides']['speed']==.4
    track.write_text('stamp,x,y,yaw,progress\n0,0,0.01,0,0\n1,-500,-500,-1,9\n')
    run_tracking.run(output/'config.yaml',duration=.4,output=tmp_path/'second',no_animation=True)
    states=lambda rows:[{k:v for k,v in r.items() if k!='compute_ms'} for r in rows]
    assert states(captured[0])==states(captured[1])  # only wall-clock calculation timing can differ
    run_tracking.run(configfile,controller='lqr',duration=.4,output=tmp_path/'ideal',no_animation=True,ideal=True)
    assert 'model_profile' not in yaml.safe_load((tmp_path/'ideal/config.yaml').read_text())
