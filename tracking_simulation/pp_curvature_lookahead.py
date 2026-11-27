"""Experimental lookahead law for offline validation only; no ROS or device IO."""
from dataclasses import dataclass
import math

from four_controler.four_controler import PurePursuit
from agri_base.coordinates import wrap_angle


class SpeedPursuitBaseline(PurePursuit):
    """Historical single-speed lookahead, only for explicit offline comparisons."""
    name='Historical PP speed-only baseline (offline comparison)'

    def lookahead_distance(self,progress,lower,upper):
        return self.lookahead+self.tuning['lookahead_gain']*abs(self.measured_speed)


@dataclass(frozen=True)
class LookaheadRule:
    curvature_gain_m: float = .25
    minimum_m: float = .65
    maximum_m: float = 1.05
    curvature_window_m: float = 1.

    def __post_init__(self):
        for value in vars(self).values():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('前视实验参数必须为有限数值')
        if not 0<=self.curvature_gain_m<=2:raise ValueError('曲率增益超出实验范围')
        if not 0<self.minimum_m<=self.maximum_m<=3:raise ValueError('前视上下限无效')
        if not .1<=self.curvature_window_m<=1.5:raise ValueError('曲率窗口需在0.1～1.5米')


def adaptive_distance(base_m,speed_m_s,speed_gain_s,curvature_rad_m,rule):
    if not all(math.isfinite(v) for v in (base_m,speed_m_s,speed_gain_s,curvature_rad_m)):
        raise ValueError('前视计算输入无效')
    if base_m<=0 or speed_gain_s<0:raise ValueError('基础前视须为正，速度系数不能为负')
    requested=(base_m+speed_gain_s*abs(speed_m_s))/(1+rule.curvature_gain_m*abs(curvature_rad_m))
    return max(rule.minimum_m,min(rule.maximum_m,requested))


class CurvaturePursuit(PurePursuit):
    """Change only the lookahead hook; preserve target vertices and v/omega law.

    Curvature is the reference polyline tangent change over a fixed forward
    arc-length window, independent of current steering output and lookahead.
    This experiment is scoped to ordinary forward paths, not cusp acquisition.
    """
    name='PP with experimental speed/curvature lookahead'

    def __init__(self,path,*,rule=LookaheadRule(),**kwargs):
        super().__init__(path,**kwargs)
        self.rule=rule;self.preview_curvature=0.
        self.lookahead_history=[]

    def lookahead_distance(self,progress,lower,upper):
        start=max(lower,min(upper,progress));end=min(upper,start+self.rule.curvature_window_m)
        tangent_end=max(lower,upper-1e-8)
        self.preview_curvature=(wrap_angle(self.path.at(min(end,tangent_end))[2]-self.path.at(min(start,tangent_end))[2])/(end-start)
            if end-start>1e-6 else 0.)
        result=adaptive_distance(self.lookahead,self.measured_speed,self.tuning['lookahead_gain'],self.preview_curvature,self.rule)
        self.lookahead_history.append((progress,result,self.preview_curvature))
        return result
