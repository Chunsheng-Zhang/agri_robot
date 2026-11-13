"""Native curves, opt-in reverse and cusp handling without ROS or hardware."""
import math
import pytest
from shapely.geometry import box
from agri_base.path_geometry import directed_poses
from agri_path_tracking.controller import Polyline,PurePursuit
from agri_path_tracking.coverage import CoverageController
from agri_path_planning.contour import Builder
from agri_path_planning.curves import CURVES
from agri_path_planning.turn_library import NativeTurns
from agri_path_planning.planning_options import options
from agri_backend.missions import CoverageParameters


def native(name,allowed=False):
    b=Builder(box(-10,-10,20,20),1,2,1,.05,.35,name,allow_reverse=allowed)
    return NativeTurns(b,options({}),allowed)


@pytest.mark.parametrize('name',list(CURVES))
def test_four_native_generators_keep_radius_and_reverse_permission(name):
    lib=native(name)
    a=(0.,0.,0.);b=(0.,.4,math.pi)
    value=lib.connection(a,b)
    assert value and value['type']==name and value['radius_m']==1
    assert lib.total==1
    assert lib.connection(a,b) is value and lib.total==1
    reverse=any(p[3]<0 for p in value['points'])
    if reverse:
        assert not value['executable']
        with pytest.raises(ValueError,match='允许倒车'):lib.connect(a,b)
        enabled=native(name,True);points=enabled.connect(a,b)
        assert any(p[3]<0 for p in points)
    else:
        assert value['executable']
        assert all(p[3]>0 for p in lib.connect(a,b))


def test_reverse_option_is_strict_false_by_default_and_legacy_values_are_removed():
    assert CoverageParameters().allow_reverse is False
    assert CoverageParameters().min_turning_radius==1
    for name in CURVES:assert CoverageParameters(path_continuity=name,allow_reverse=True).path_continuity==name
    for value in (1,'false','true',None):
        with pytest.raises(ValueError):CoverageParameters(allow_reverse=value)
    old=CoverageParameters(path_continuity='CONTINUOUS',optimization_turns='OMEGA').model_dump()
    assert old['path_continuity']=='DUBINS_CC' and 'optimization_turns' not in old
    assert CoverageParameters(path_continuity='DISCONTINUOUS').path_continuity=='DUBINS'


def test_reverse_pursuit_corrects_lateral_error_without_turning_the_vehicle_around():
    c=PurePursuit(Polyline([(0,0),(-3,0)],[-1]),lookahead=.3)
    x,y,yaw=0,.12,0
    for _ in range(2000):
        v,w,_,_,heading,done=c.command(x,y,yaw)
        if done:break
        assert v<=0 and abs(w)<=1
        x+=v*math.cos(yaw)*.05;y+=v*math.sin(yaw)*.05;yaw+=w*.05
    assert done and abs(y)<.04 and abs(yaw)<.1


def test_cusps_bound_projection_and_lookahead_then_require_fresh_stationary_dwell():
    c=CoverageController(Polyline([(0,0),(2,0),(0,0),(1,0)],[1,-1,1]),(0,0),lookahead=.3)
    c.command(0,0,0,now=0,sample_stamp=1);c.command(0,0,0,now=.5,sample_stamp=2)
    assert c.phase=='TRACKING'
    c.tracker.progress=1.99
    assert c.command(2,0,0,now=1,sample_stamp=3)[:2]==(0,0)
    assert c.phase=='ALIGNING' and c.tracker.leg_index==1
    c.command(2,0,0,now=1.1,sample_stamp=4)
    c.command(2,0,0,now=2,sample_stamp=4)
    assert c.phase=='ALIGNING'
    c.command(2,0,0,now=2.1,sample_stamp=5)
    assert c.phase=='TRACKING'
    assert c.command(2,0,0,now=2.2,sample_stamp=6)[0]<0
    assert 2<=c.progress<3  # Cannot project onto the later overlapping forward leg.
    progress=c.progress;c.align_for_resume()
    assert c.command(1.9,0,0,now=4,sample_stamp=7)[:2]==(0,0)
    assert c.alignment_yaw==pytest.approx(0,abs=1e-8) and c.progress==progress


def test_complete_forward_reverse_forward_loop_never_skips_a_leg():
    c=CoverageController(Polyline([(0,0),(2,0),(0,0),(1,0)],[1,-1,1]),(0,0),lookahead=.25)
    x=y=yaw=0.;signs=[]
    for step in range(4000):
        v,w,*_,done=c.command(x,y,yaw,now=step*.05,sample_stamp=step)
        if done:break
        if v and (not signs or signs[-1]!=math.copysign(1,v)):signs.append(math.copysign(1,v))
        x+=v*math.cos(yaw)*.05;y+=v*math.sin(yaw)*.05;yaw+=w*.05
    assert done and signs==[1,-1,1] and c.progress>4.7


@pytest.mark.parametrize('name',['REEDS_SHEPP','REEDS_SHEPP_HC'])
def test_real_reeds_shepp_curve_tracks_in_synthetic_kinematics(name):
    points=native(name,True).connect((0.,0.,0.),(0.,.4,math.pi))
    poses,directions=directed_poses(points)
    assert -1 in directions
    c=CoverageController(Polyline([p[:2] for p in poses],directions),poses[0][:2],
                         lookahead=.15,speed=.15,completion_tolerance=.05,search_window=.4)
    x,y,yaw=poses[0];negative=False
    for step in range(10000):
        v,w,*_,done=c.command(x,y,yaw,now=step*.05,sample_stamp=step)
        if done:break
        negative|=v<0
        x+=v*math.cos(yaw)*.05;y+=v*math.sin(yaw)*.05;yaw+=w*.05
    assert done and negative and math.dist((x,y),poses[-1][:2])<=.05


def test_unstable_position_cannot_complete_direction_change_dwell():
    c=CoverageController(Polyline([(0,0),(2,0),(0,0)],[1,-1]),(0,0))
    c.command(0,0,0,now=0,sample_stamp=0);c.command(0,0,0,now=.5,sample_stamp=1)
    c.tracker.progress=1.99;c.command(2,0,0,now=1,sample_stamp=2)
    for i in range(1,7):
        assert c.command(2+i*.03,0,0,now=1+i*.1,sample_stamp=i+2)[0]==0
        assert c.phase=='ALIGNING'
