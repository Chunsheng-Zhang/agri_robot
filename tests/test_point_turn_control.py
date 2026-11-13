"""Stopped-turn boundaries, settling and saved motion integrity without hardware."""
import copy
import json
import math
import pytest
from shapely.geometry import box
from agri_base.path_geometry import directed_poses,point_turn_indices,rotation_envelope,validate_point_turn_path,motion_signature
from agri_base.coordinates import LocalFrame
from agri_path_tracking.controller import Polyline
from agri_path_tracking.coverage import CoverageController
from agri_path_planning.saved_path import read_saved_path

POSES=[(1.,1.,0.),(3.,1.,0.),(3.,1.,math.pi/2),(3.,3.,math.pi/2),
       (3.,3.,math.pi),(1.,3.,math.pi)]


def controller(**kw):
    clean,directions=directed_poses(POSES)
    return CoverageController(Polyline([p[:2] for p in clean],directions,point_turn_indices(POSES)),
        clean[0][:2],**kw)


def at_first_turn(c):
    c.command(1,1,0,now=0,sample_stamp=0)
    c.command(1,1,0,now=.5,sample_stamp=1)
    c.tracker.progress=1.99
    assert c.command(3,1,0,now=1,sample_stamp=2)[:2]==(0,0)
    assert c.phase=='ALIGNING' and c.alignment_reason=='point_turn'


def test_large_lookahead_and_search_never_cross_an_uncompleted_turn():
    c=controller(lookahead=20,search_window=20)
    assert c.path.legs==[(0.,2.,1),(2.,4.,1),(4.,6.,1)]
    c.phase='TRACKING'
    # The next row is directly beside the car. Match only the active first row.
    c.command(1,2.9,0,now=0,sample_stamp=1)
    assert c.progress<=2 and c.tracker.leg_index==0
    assert c.tracker.error(2.9,1)[3]==pytest.approx(0)


def test_turn_stops_before_rotation_requires_new_samples_and_resume_retains_leg():
    c=controller();at_first_turn(c)
    assert c.command(3,1,0,now=1.1,sample_stamp=3)[:2]==(0,0)
    assert c.command(3,1,0,now=2,sample_stamp=3)[:2]==(0,0)
    assert c.pre_turn_settling
    c.align_for_resume()
    assert c.progress==2 and c.tracker.leg_index==1
    assert c.command(3,1,0,now=3,sample_stamp=4)[:2]==(0,0)
    assert c.command(3,1,0,now=3.4,sample_stamp=5)[:2]==(0,0)
    assert c.pre_turn_settling
    assert c.command(3,1,0,now=3.51,sample_stamp=6)[:2]==(0,0)
    assert not c.pre_turn_settling
    v,w,*_=c.command(3,1,0,now=3.6,sample_stamp=7)
    assert v==0 and 0<w<=.3
    assert c.command(3,1,math.pi/2,now=4,sample_stamp=8)[:2]==(0,0)
    c.command(3,1,math.pi/2,now=5,sample_stamp=8)
    assert c.phase=='ALIGNING'
    c.command(3,1,math.pi/2,now=5.1,sample_stamp=9)
    assert c.phase=='TRACKING'
    assert c.command(3,1,math.pi/2,now=5.2,sample_stamp=10)[0]>0


def test_drifting_position_cannot_complete_pre_turn_wait():
    c=controller();at_first_turn(c)
    for i in range(1,9):
        assert c.command(3+i*.03,1,0,now=1+i*.2,sample_stamp=2+i)[:2]==(0,0)
        assert c.pre_turn_settling


def test_full_forward_loop_stops_and_aligns_at_every_corner():
    c=controller(lookahead=.3,speed=.3,completion_tolerance=.05)
    x,y,yaw=POSES[0];visited=set();turns=set()
    for step in range(5000):
        v,w,*_,done=c.command(x,y,yaw,now=step*.05,sample_stamp=step)
        visited.add(c.tracker.leg_index)
        if c.phase=='ALIGNING':
            assert v==0
            if c.alignment_reason=='point_turn':turns.add(c.tracker.leg_index)
        if done:break
        x+=v*math.cos(yaw)*.05;y+=v*math.sin(yaw)*.05;yaw+=w*.05
    assert done and visited=={0,1,2} and turns=={1,2}
    assert math.dist((x,y),POSES[-1][:2])<=.05


def test_sweep_and_missing_turn_markers_are_rejected():
    envelope=rotation_envelope(.45,.2,.05)
    validate_point_turn_path(POSES,box(0,0,5,5),envelope)
    clean,_=directed_poses(POSES)
    with pytest.raises(ValueError,match='停车标记'):validate_point_turn_path(clean,box(0,0,5,5),envelope)
    outside=[(x-.9,y,yaw) for x,y,yaw in POSES]
    with pytest.raises(ValueError,match='净空'):validate_point_turn_path(outside,box(0,0,5,5),envelope)


def test_saved_turns_survive_reload_and_changed_geometry_or_yaw_is_rejected(tmp_path):
    frame=LocalFrame(36.6468527633281,117.021433066391)
    env=rotation_envelope(.45,.2,.05)
    data={'frame_id':'map','projection':frame.identity,
        'points':[dict(x=x,y=y,yaw=yaw) for x,y,yaw in POSES],
        'is_backward':[False]*len(POSES),'velocities':[],
        'motion_signature':motion_signature(POSES),'planning':{'turn_mode':'POINT_TURN','rotation_envelope':env}}
    path=tmp_path/'path.json';path.write_text(json.dumps(data))
    _,points=read_saved_path(path,frame,box(0,0,5,5),.2,False,env)
    assert point_turn_indices(points)==[1,2] and points==POSES
    with pytest.raises(ValueError,match='尺寸/安全余量'):
        read_saved_path(path,frame,box(0,0,5,5),.2,False,rotation_envelope(.5,.2,.05))
    changed=copy.deepcopy(data);changed['points'][1]['yaw']+=.01;path.write_text(json.dumps(changed))
    with pytest.raises(ValueError,match='指纹'):read_saved_path(path,frame,box(0,0,5,5),.2,False,env)
