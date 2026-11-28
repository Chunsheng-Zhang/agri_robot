"""Robot controllers ported from tracking_simulation/four_controler.

Production additionally bounds control to each leg, preserves stopped turns and
reverse direction, and faults on unconverged MPC. No ROS or hardware IO here.
"""
import math
import bisect
from copy import deepcopy

import numpy as np

from agri_base.coordinates import wrap_angle
from .controller import PurePursuit as PathTrackingState

NAMES = ('pp', 'pp_speed', 'stanley', 'lqr', 'mpc')
LABELS = {'pp': 'PP (speed/curvature, GA)', 'pp_speed': 'PP (speed-only, 2026-11-27)',
          'stanley': 'Stanley (yaw-rate)', 'lqr': 'LQR', 'mpc': 'MPC'}
PP_DEFAULTS = dict(lookahead_gain=.8388, wheelbase=.16, rear_offset=0., max_steer_deg=35.,
                   curvature_gain=.4076, lookahead_min=.65, lookahead_max=1.05, curvature_window=1.)
PP_SPEED_DEFAULTS = dict(lookahead_gain=.35, wheelbase=.16, rear_offset=0., max_steer_deg=35.)
DEFAULT_TUNING = {
    'pp': PP_DEFAULTS,
    'pp_speed': PP_SPEED_DEFAULTS,
    'stanley': dict(cross_track_gain=1., heading_gain=2., softening_speed=.1),
    'lqr': dict(lateral_weight=8., heading_weight=3., angular_weight=.5),
    'mpc': dict(lateral_weight=8., heading_weight=3., angular_weight=.2,
                terminal_multiplier=2., horizon=20, max_iterations=200),
}
ALGORITHMS = NAMES


def validate_algorithm(value):
    if value not in ALGORITHMS:
        raise ValueError('Unknown tracking controller: ' + str(value))
    return value


class ControllerFault(ValueError):
    """Invalid/unconverged controller outputs must stop the task."""


def make_tracker(path, algorithm='pp', dt=.05, **settings):
    validate_algorithm(algorithm)
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Controller period must be finite and positive')
    if algorithm == 'pp':
        tracker = PurePursuit(path, tuning=deepcopy(DEFAULT_TUNING['pp']), **settings)
    elif algorithm == 'pp_speed':
        tracker = SpeedAdaptivePursuit(path, tuning=deepcopy(DEFAULT_TUNING['pp_speed']), **settings)
    else:
        try:
            tracker = RobotController(path, algorithm, dt, deepcopy(DEFAULT_TUNING), **settings)
        except np.linalg.LinAlgError as exc:
            raise ControllerFault(f'Controller initialization failed: {exc}') from exc
    tracker.algorithm_id = algorithm
    return tracker


def controller_metadata(algorithm):
    validate_algorithm(algorithm)
    return {'algorithm': algorithm, 'label': LABELS[algorithm],
            **({'parameter_profile': 'ga_L0_T_K_20261127'} if algorithm == 'pp' else {}),
            'implementation': ('agri_path_tracking.pp_speed_only.20261127' if algorithm == 'pp_speed'
                               else 'agri_path_tracking.pp_speed_curvature.v2' if algorithm == 'pp'
                               else 'agri_path_tracking.algorithms.v1'),
            'tuning': deepcopy(DEFAULT_TUNING.get(algorithm, {}))}


def pure_pursuit_control(rear_x, rear_y, yaw, target_x, target_y, wheelbase, max_steer):
    """Reference script's rear-point geometry, returning a virtual steer angle.

    Source: four_controler/four_controler.py:pure_pursuit_control. The robot
    converts this angle to omega=v*tan(delta)/wheelbase; no physical steering
    actuator is assumed. Keep the actual target chord in the denominator.
    """
    actual_ld = max(math.hypot(target_x-rear_x, target_y-rear_y), 1e-6)
    alpha = wrap_angle(math.atan2(target_y-rear_y, target_x-rear_x)-yaw)
    steer = math.atan2(2.0*wheelbase*math.sin(alpha), actual_ld)
    return max(-max_steer, min(max_steer, steer))


class PurePursuit(PathTrackingState):
    """Speed/curvature-adaptive PP, with the existing segment-state interface."""
    supports_segmented = True
    name = 'Pure Pursuit (speed/curvature lookahead)'

    def __init__(self, path, tuning=None, **settings):
        super().__init__(path, **settings)
        self.tuning = dict(PP_DEFAULTS, **(tuning or {}))
        if set(self.tuning) != set(PP_DEFAULTS):
            raise ValueError('未知PP配置项')
        for key, value in self.tuning.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('pp.'+key+'必须为有限数值')
            if value < 0 or (key in ('wheelbase', 'max_steer_deg') and value == 0):
                raise ValueError('PP增益/偏移须非负，轴距和转角上限须为正')
        if self.tuning['max_steer_deg'] >= 90:
            raise ValueError('PP虚拟转角上限必须小于90度')
        if not 0 < self.tuning['lookahead_min'] <= self.tuning['lookahead_max'] <= 3.:
            raise ValueError('PP前视距离须满足0 < 下限 <= 上限 <= 3m')
        if not 0 <= self.tuning['curvature_gain'] <= 2.:
            raise ValueError('PP曲率增益须在0～2m之间')
        if not .1 <= self.tuning['curvature_window'] <= 1.5:
            raise ValueError('PP曲率窗口须在0.1～1.5m之间')
        self.measured_speed = 0.
        self.preview_curvature = 0.
        self.steering_angle = 0.
        self.effective_lookahead = self.lookahead
        self.target_point = path.points[0]

    def set_speed_feedback(self, speed):
        if not math.isfinite(speed):
            raise ValueError('PP速度反馈必须为有限数值')
        self.measured_speed = float(speed)

    def lookahead_distance(self, progress, lower, upper):
        """Forward reference tangent change; never preview across a stopped turn/cusp."""
        start = max(lower, min(upper, progress))
        end = min(upper, start+self.tuning['curvature_window'])
        # At an interior leg endpoint, Polyline.at(upper) belongs to the next leg.
        tangent_end = max(lower, upper-1e-8)
        self.preview_curvature = (
            wrap_angle(self.path.at(min(end, tangent_end))[2]
                       - self.path.at(min(start, tangent_end))[2])/(end-start)
            if end-start > 1e-6 else 0.)
        requested = (self.lookahead+self.tuning['lookahead_gain']*abs(self.measured_speed)) / (
            1.+self.tuning['curvature_gain']*abs(self.preview_curvature))
        return max(self.tuning['lookahead_min'], min(self.tuning['lookahead_max'], requested))

    def command(self, x, y, yaw):
        distance, s, lateral, body_yaw = self.error(x, y)
        self.progress = max(self.progress, s)
        lo, hi, direction = self.leg
        remaining = hi-self.progress
        endpoint = math.dist((x, y), self.path.at(hi)[:2])
        final = self.leg_index == len(self.path.legs)-1
        tolerance = self.tolerance if final else min(.05, self.tolerance, (hi-lo)/4)
        reached = remaining <= tolerance and endpoint <= tolerance
        done = final and reached
        if reached and not final:
            self.needs_switch = True
        heading = wrap_angle(body_yaw-yaw)
        if done or self.needs_switch:
            return 0., 0., distance, lateral, heading, done

        travel_yaw = wrap_angle(yaw+(math.pi if direction < 0 else 0.))
        rear_x = x-self.tuning['rear_offset']*math.cos(travel_yaw)
        rear_y = y-self.tuning['rear_offset']*math.sin(travel_yaw)
        # Preserve bounded matching at neighboring rows and direction changes.
        _, rear_s, _, _ = self.path.project(rear_x, rear_y, self.progress, self.window, lo, hi)
        self.effective_lookahead = self.lookahead_distance(rear_s,lo,hi)
        target_s = min(hi, rear_s+self.effective_lookahead)
        index = min(bisect.bisect_left(self.path.lengths, target_s),
                    bisect.bisect_left(self.path.lengths, hi), len(self.path.points)-1)
        tx, ty = self.target_point = self.path.points[index]
        self.steering_angle = pure_pursuit_control(rear_x, rear_y, travel_yaw, tx, ty,
            self.tuning['wheelbase'], math.radians(self.tuning['max_steer_deg']))
        curvature = math.tan(self.steering_angle)/self.tuning['wheelbase']
        # Robot-only acquisition and endpoint stop; the reference car had no
        # stopped turns/reverse legs or task completion state machine.
        if abs(heading) > .8:
            return 0., max(-self.max_angular, min(self.max_angular, heading)), distance, lateral, heading, False
        speed = min(self.speed, .5*max(endpoint, remaining),
                    self.max_angular/max(abs(curvature), 1e-9))
        return direction*speed, speed*curvature, distance, lateral, heading, False


class SpeedAdaptivePursuit(PurePursuit):
    """November 27 speed-only law, sharing geometry and segment safety states."""
    name = 'Pure Pursuit (speed-only, 2026-11-27)'

    def __init__(self, path, tuning=None, **settings):
        legacy = dict(PP_SPEED_DEFAULTS, **(tuning or {}))
        if set(legacy) != set(PP_SPEED_DEFAULTS):
            raise ValueError('未知速度自适应PP配置项')
        settings.setdefault('lookahead', .8)
        super().__init__(path, tuning=legacy, **settings)
        self.tuning = legacy

    def lookahead_distance(self, progress, lower, upper):
        # Historical law has neither a curvature denominator nor the new clip.
        self.preview_curvature = 0.
        return self.lookahead+self.tuning['lookahead_gain']*abs(self.measured_speed)


def discrete_model(speed, dt):
    """e=[lateral, body yaw minus reference yaw], input=body yaw rate."""
    return np.array([[1., speed*dt], [0., 1.]]), np.array([[.5*speed*dt*dt], [dt]])


def solve_discrete_lqr(a, b, q, r):
    """Discrete Riccati iteration, retaining the original example's method."""
    p = q.copy()
    for _ in range(10000):
        gain = np.linalg.solve(r+b.T@p@b, b.T@p@a)
        updated = a.T@p@a-a.T@p@b@gain+q
        if np.max(np.abs(updated-p)) < 1e-10:
            p = updated
            break
        p = updated
    else:
        raise ValueError('LQR Riccati iteration did not converge')
    return np.linalg.solve(r+b.T@p@b, b.T@p@a)


def build_lifted_matrices(a, b, horizon):
    """Stack x_1...x_N = Sx x_0 + Su u + reference disturbance."""
    sx = np.vstack([np.linalg.matrix_power(a, i+1) for i in range(horizon)])
    su = np.zeros((2*horizon, horizon))
    for i in range(horizon):
        for j in range(i+1):
            su[2*i:2*i+2, j:j+1] = np.linalg.matrix_power(a, i-j)@b
    return sx, su


def box_qp(h, g, limit, initial=None, tolerance=1e-6, max_iterations=200):
    """Solve min .5*u'Hu+g'u subject to |u|<=limit.

    SPD quadratic: unconstrained solve when feasible, otherwise projected
    coordinate descent, with a projected-gradient optimality check. Reports
    nonconvergence instead of silently calling a clipped unconstrained answer MPC.
    """
    exact = np.linalg.solve(h, -g)
    if np.max(np.abs(exact)) <= limit:
        return exact, True, 0
    u = np.clip(exact if initial is None else initial, -limit, limit)
    for iteration in range(1, max_iterations+1):
        for j in range(len(u)):
            u[j] = np.clip(u[j]-(h[j]@u+g[j])/h[j,j], -limit, limit)
        projected = u-np.clip(u-(h@u+g), -limit, limit)
        if np.max(np.abs(projected)) < tolerance:
            return u, True, iteration
    return u, False, max_iterations


class RobotController(PathTrackingState):
    """Reuse production matching/progress; expose the same command/error API."""
    def __init__(self, path, algorithm, dt, tuning, **settings):
        super().__init__(path, **settings)
        self.algorithm = algorithm
        self.name = LABELS[algorithm]
        self.dt = dt
        self.tuning = tuning
        self.optimization_calls = self.optimization_unconverged = 0
        self.optimization_iterations = 0
        self.a, self.b = discrete_model(self.speed, dt)
        if algorithm in ('lqr', 'mpc'):
            cfg = tuning[algorithm]
            self.q = np.diag([cfg['lateral_weight'], cfg['heading_weight']])
            self.r = np.array([[cfg['angular_weight']]])
        if algorithm == 'lqr':
            self.gain = solve_discrete_lqr(self.a, self.b, self.q, self.r)
        if algorithm == 'mpc':
            self.horizon = tuning['mpc']['horizon']
            self.previous_sequence = None
            self.model_cache = {}

    def reference_yaw(self, progress):
        lo, hi, _ = self.leg
        return self.path.at(max(lo, min(hi-1e-8, progress)))[2]

    def curvature(self, progress):
        begin, end, _ = self.leg
        lo, hi = max(begin, progress-.1), min(end-1e-8, progress+.1)
        return wrap_angle(self.reference_yaw(hi)-self.reference_yaw(lo))/max(hi-lo, 1e-6)

    def predict_mpc(self, lateral, heading, speed):
        # v changes during approach to the endpoint or curvature speed limiting.
        key = round(speed, 6)
        if key not in self.model_cache:
            a,b = discrete_model(speed, self.dt)
            sx,su = build_lifted_matrices(a,b,self.horizon)
            weights = np.tile(np.diag(self.q), self.horizon)
            weights[-2:] *= self.tuning['mpc']['terminal_multiplier']
            h = su.T@(weights[:,None]*su)+self.r[0,0]*np.eye(self.horizon)
            if len(self.model_cache)>=128:self.model_cache.clear()
            self.model_cache[key] = a,b,sx,su,weights,h
        a,b,sx,su,weights,h = self.model_cache[key]
        x = np.array([lateral, -heading])
        drift = np.zeros(2)
        prediction=[];reference=[]
        for i in range(self.horizon):
            s0=min(self.leg[1]-1e-8,self.progress+i*speed*self.dt)
            s1=min(self.leg[1]-1e-8,self.progress+(i+1)*speed*self.dt)
            delta=wrap_angle(self.reference_yaw(s1)-self.reference_yaw(s0))
            drift=a@drift+np.array([-.5*speed*self.dt*delta,-delta])
            prediction.extend(drift)
            reference.append(delta/self.dt)
        free=sx@x+np.asarray(prediction)
        g=su.T@(weights*free)-self.r[0,0]*np.asarray(reference)
        initial = None if self.previous_sequence is None else np.r_[self.previous_sequence[1:],self.previous_sequence[-1]]
        try:
            sequence,converged,iterations=box_qp(h,g,self.max_angular,initial,
                                                 max_iterations=self.tuning['mpc']['max_iterations'])
        except np.linalg.LinAlgError as exc:
            raise ControllerFault(f'MPC numerical solve failed: {exc}') from exc
        self.previous_sequence=sequence
        self.optimization_calls+=1
        self.optimization_unconverged+=int(not converged)
        self.optimization_iterations+=iterations
        if not converged:
            raise ControllerFault('MPC did not converge within its iteration limit')
        return float(sequence[0])

    def advance_leg(self):
        super().advance_leg()
        if self.algorithm == 'mpc':
            self.previous_sequence = None

    def command(self, x, y, yaw):
        distance,s,lateral,body_yaw = self.error(x,y)
        self.progress=max(self.progress,s)
        lo, hi, direction = self.leg
        remaining=hi-self.progress
        endpoint=math.dist((x,y),self.path.at(hi)[:2])
        heading=wrap_angle(body_yaw-yaw)
        final=self.leg_index == len(self.path.legs)-1
        tolerance=self.tolerance if final else min(.05,self.tolerance,(hi-lo)/4)
        reached=remaining<=tolerance and endpoint<=tolerance
        done=final and reached
        if reached and not final:self.needs_switch=True
        if done or self.needs_switch:return 0.,0.,distance,lateral,heading,done
        # Robot heading acquisition, shared matching and independent control law.
        if abs(heading)>.8:
            return 0.,max(-self.max_angular,min(self.max_angular,heading)),distance,lateral,heading,False
        curvature=self.curvature(self.progress)
        speed=min(self.speed,.5*max(endpoint,remaining),
                  self.max_angular/max(abs(curvature),1e-9))
        feedforward=speed*curvature
        if self.algorithm=='stanley':
            cfg=self.tuning['stanley']
            correction=math.atan2(-cfg['cross_track_gain']*lateral,speed+cfg['softening_speed'])
            angular=feedforward+cfg['heading_gain']*(heading+correction)
        elif self.algorithm=='lqr':
            angular=feedforward-float((self.gain@np.array([lateral,-heading])).item())
        else:
            angular=self.predict_mpc(lateral,heading,speed)
        angular=max(-self.max_angular,min(self.max_angular,angular))
        if not all(math.isfinite(v) for v in (speed, angular, lateral, heading)):
            raise ControllerFault('Non-finite controller output')
        return direction*speed,angular,distance,lateral,heading,False
