"""Shared observation noise and causal actuator lag for simulation only."""
import math
import random


class GaussianPoseNoise:
    """Independent zero-mean x/y errors (metres) and heading error (degrees)."""
    def __init__(self, position_std=0., heading_std_deg=0., seed=0):
        if any(isinstance(v, bool) or not math.isfinite(v) or v < 0
               for v in (position_std, heading_std_deg)):
            raise ValueError('位置/航向噪声标准差必须为有限非负数')
        self.position_std = position_std
        self.heading_std = math.radians(heading_std_deg)
        self.rng = random.Random(seed)

    def sample(self, x, y, yaw):
        x += self.rng.gauss(0, self.position_std)
        y += self.rng.gauss(0, self.position_std)
        yaw += self.rng.gauss(0, self.heading_std)
        return x, y, math.atan2(math.sin(yaw), math.cos(yaw))


class FirstOrderActuator:
    """tau*du/dt + u = target, independently for body v and omega.

    Commands take effect from their timestamp, never retroactively. Between
    commands each channel uses its exact zero-order-hold exponential response.
    Zero tau is direct transmission. reset() is the explicit fault-stop bypass.
    """
    def __init__(self, linear_tau=0., angular_tau=0.):
        for value in (linear_tau, angular_tau):
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError('执行器响应时间常数必须为有限非负数')
        self.taus = (linear_tau, angular_tau)
        self.target = (0., 0.)
        self.output = (0., 0.)
        self.stamp = None

    def advance(self, stamp):
        if not math.isfinite(stamp) or (self.stamp is not None and stamp < self.stamp-1e-9):
            raise ValueError('执行器时间必须有限且不能倒退')
        dt = 0. if self.stamp is None else max(0., stamp-self.stamp)
        self.output = tuple(target if tau == 0 else value+(target-value)*(-math.expm1(-dt/tau))
                            for value, target, tau in zip(self.output, self.target, self.taus))
        self.stamp = stamp
        return self.output

    def command(self, linear, angular, stamp):
        if not all(math.isfinite(v) for v in (linear, angular)):
            raise ValueError('执行器输入指令必须为有限数值')
        self.advance(stamp)
        self.target = (float(linear), float(angular))
        self.output = tuple(target if tau == 0 else value
                            for value, target, tau in zip(self.output, self.target, self.taus))
        return self.output

    def reset(self, stamp):
        self.advance(stamp)
        self.output = self.target = (0., 0.)
        return self.output
