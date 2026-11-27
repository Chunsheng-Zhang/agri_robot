"""Numerical law, bounded reference curvature and baseline isolation."""
import math
from pathlib import Path
import sys

import pytest

SIM=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM))
from pp_curvature_lookahead import CurvaturePursuit,LookaheadRule,adaptive_distance,SpeedPursuitBaseline
from four_controler.four_controler import PurePursuit
from agri_path_tracking.controller import Polyline
from simulate import SimulationConfig,forward_poses,simulate


def test_proposed_example_and_left_right_symmetry():
    rule=LookaheadRule()
    assert adaptive_distance(.8,.46,.35,0.,rule)==pytest.approx(.961)
    assert adaptive_distance(.8,.46,.35,1.,rule)==pytest.approx(.7688)
    assert adaptive_distance(.8,.46,.35,-1.,rule)==pytest.approx(.7688)
    assert adaptive_distance(.8,-.46,.35,1.,rule)==pytest.approx(.7688)


def test_clip_is_applied_after_both_speed_and_curvature_terms():
    rule=LookaheadRule()
    assert adaptive_distance(.8,10.,.35,0.,rule)==rule.maximum_m
    assert adaptive_distance(.8,0.,.35,100.,rule)==rule.minimum_m


@pytest.mark.parametrize('changes',[
    {'curvature_gain_m':-.1},{'curvature_gain_m':float('nan')},
    {'minimum_m':0.},{'minimum_m':1.2,'maximum_m':1.},
    {'maximum_m':True},{'curvature_window_m':0.},
])
def test_invalid_rule_parameters_are_rejected(changes):
    with pytest.raises(ValueError):LookaheadRule(**changes)


def test_curvature_comes_from_reference_geometry_not_command_or_lookahead():
    theta=[i*.02 for i in range(120)]
    path=Polyline([(2*math.cos(t),2*math.sin(t)) for t in theta])
    a=CurvaturePursuit(path,rule=LookaheadRule(curvature_gain_m=.25),lookahead=.8,speed=.5)
    b=CurvaturePursuit(path,rule=LookaheadRule(curvature_gain_m=.6),lookahead=.5,speed=.3)
    for controller in (a,b):
        controller.set_speed_feedback(.46);controller.lookahead_distance(.5,0.,path.total)
    assert a.preview_curvature==pytest.approx(.5,abs=.03)
    assert a.preview_curvature==b.preview_curvature
    assert a.lookahead_history[-1][1]!=b.lookahead_history[-1][1]


def test_tangent_wrap_does_not_create_a_spurious_two_pi_curvature():
    theta=[2.5+i*.02 for i in range(120)]
    path=Polyline([(math.cos(t),math.sin(t)) for t in theta])
    controller=CurvaturePursuit(path,lookahead=.8,speed=.5)
    controller.lookahead_distance(.2,0.,path.total)
    assert controller.preview_curvature==pytest.approx(1.,abs=.03)


def test_zero_curvature_gain_restores_entire_original_free_closed_loop():
    poses=forward_poses([(0.,0.),(3.,0.),(4.,1.),(4.,4.)])
    config=SimulationConfig(speed=.5,duration=15.,feedback_period=.1)
    original,_=simulate(poses,config,controller_factory=lambda p,**kw:SpeedPursuitBaseline(p,**kw))
    zero,_=simulate(poses,config,controller_factory=lambda p,**kw:CurvaturePursuit(p,rule=LookaheadRule(curvature_gain_m=0.),**kw))
    states=lambda rows:[{k:v for k,v in r.items() if k!='compute_ms'} for r in rows]
    assert states(original)==states(zero)


def test_reference_curvature_window_is_bounded_at_path_endpoint():
    path=Polyline([(0.,0.),(1.,0.)]);controller=CurvaturePursuit(path)
    assert controller.lookahead_distance(1.,0.,1.)==.8
    assert controller.preview_curvature==0.


def test_default_pp_now_matches_validated_formula_in_free_closed_loop():
    poses=forward_poses([(0.,0.),(3.,0.),(4.,1.),(4.,4.)])
    config=SimulationConfig(speed=.5,duration=15.,feedback_period=.1)
    default,_=simulate(poses,config,controller_factory=lambda p,**kw:PurePursuit(p,**kw))
    proposed,_=simulate(poses,config,controller_factory=lambda p,**kw:CurvaturePursuit(p,**kw))
    states=lambda rows:[{k:v for k,v in r.items() if k!='compute_ms'} for r in rows]
    assert states(default)==states(proposed)
