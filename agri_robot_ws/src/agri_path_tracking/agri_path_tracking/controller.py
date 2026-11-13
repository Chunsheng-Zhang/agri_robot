"""Direction-aware Pure Pursuit with bounded progress and explicit cusp stops."""

import bisect
import math

from agri_base.coordinates import finite_positive, wrap_angle

MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0


def validate_velocity_limit(name, value):
    value = finite_positive(value, name)
    limit = {'speed': MAX_LINEAR_SPEED, 'max_angular': MAX_ANGULAR_SPEED}[name]
    if value > limit:
        raise ValueError(f'{name} must not exceed {limit}')
    return value


class Polyline:
    def __init__(self, points, directions=None, stop_indices=None):
        points=list(points)
        if directions is not None and (len(directions)!=len(points)-1 or
                any(type(d) is not int or d not in (-1,1) for d in directions)):
            raise ValueError('Require one +1/-1 direction for each path segment')
        self.points = []
        self.directions=[]
        for i,(x, y) in enumerate(points):
            if not math.isfinite(x + y):
                raise ValueError('Path contains non-finite coordinates')
            if not self.points or math.dist(self.points[-1], (x, y)) > 1e-6:
                if self.points:self.directions.append(directions[i-1] if directions is not None else 1)
                self.points.append((x, y))
        if len(self.points) < 2:
            raise ValueError('Path needs two distinct points')
        self.lengths = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.lengths.append(self.lengths[-1] + math.dist(a, b))
        self.total = self.lengths[-1]
        self.stop_indices=set(stop_indices or [])
        if any(type(i) is not int or not 0<i<len(self.points)-1 for i in self.stop_indices):
            raise ValueError('Invalid stopped-turn index')
        self.legs=[];start=0
        for i in range(1,len(self.directions)):
            if self.directions[i]!=self.directions[i-1] or i in self.stop_indices:
                self.legs.append((self.lengths[start],self.lengths[i],self.directions[i-1]));start=i
        self.legs.append((self.lengths[start],self.total,self.directions[-1]))

    def at(self, distance):
        distance = max(0.0, min(self.total, distance))
        i = min(len(self.points) - 2, max(0, bisect.bisect_right(self.lengths, distance) - 1))
        a, b = self.points[i], self.points[i + 1]
        ratio = (distance - self.lengths[i]) / (self.lengths[i + 1] - self.lengths[i])
        return a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])

    def project(self, x, y, progress, window, lower=0., upper=None):
        best = None
        lo, hi = max(lower, progress - 0.1), min(self.total if upper is None else upper, progress + window)
        first = max(0, bisect.bisect_right(self.lengths, lo) - 1)
        last = min(len(self.points) - 2, bisect.bisect_left(self.lengths, hi))
        for i in range(first, last + 1):
            a, b = self.points[i], self.points[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = self.lengths[i + 1] - self.lengths[i]
            t = ((x - a[0]) * dx + (y - a[1]) * dy) / length ** 2
            t = max(0.0, min(1.0, t))
            s = max(lo, min(hi, self.lengths[i] + t * length))
            px, py, tangent = self.at(s)
            distance = math.hypot(x - px, y - py)
            signed = (dx * (y - py) - dy * (x - px)) / length
            candidate = (distance, s, signed, tangent)
            if best is None or distance < best[0]:
                best = candidate
        return best


class PurePursuit:
    def __init__(self, path, lookahead=0.8, speed=0.2, max_angular=1.0,
                 completion_tolerance=0.2, search_window=1.0):
        for name in ('lookahead', 'speed', 'max_angular', 'completion_tolerance', 'search_window'):
            finite_positive(locals()[name], name)
        self.path = path
        self.lookahead = lookahead
        self.speed = validate_velocity_limit('speed', speed)
        self.max_angular = validate_velocity_limit('max_angular', max_angular)
        self.tolerance, self.window = completion_tolerance, search_window
        self.progress = 0.0
        self.leg_index=0
        self.needs_switch=False

    @property
    def leg(self):
        return self.path.legs[self.leg_index]

    def advance_leg(self):
        if not self.needs_switch:raise ValueError('No stopped direction change pending')
        self.progress=self.leg[1]
        self.leg_index+=1
        self.needs_switch=False

    def error(self,x,y):
        lo,hi,direction=self.leg
        distance,s,lateral,_=self.path.project(x,y,self.progress,self.window,lo,hi)
        # At a cusp Path.at(hi) belongs to the next, opposite travel segment.
        tangent=self.path.at(min(hi-1e-8,max(lo,s)))[2]
        return distance,s,lateral,wrap_angle(tangent+(math.pi if direction<0 else 0))

    def command(self, x, y, yaw):
        distance, s, lateral, tangent = self.error(x,y)
        self.progress = max(self.progress, s)
        lo,hi,direction=self.leg
        remaining = hi - self.progress
        end_distance = math.dist((x, y), self.path.at(hi)[:2])
        final=self.leg_index==len(self.path.legs)-1
        tolerance=self.tolerance if final else min(.05,self.tolerance,(hi-lo)/4)
        # A loop ending beside its start must not complete before traversing it.
        reached = remaining <= tolerance and end_distance <= tolerance
        done=final and reached
        if reached and not final:self.needs_switch=True
        tx, ty, _ = self.path.at(min(hi, self.progress + self.lookahead))
        dx, dy = tx - x, ty - y
        alpha = wrap_angle(math.atan2(dy, dx) - yaw - (math.pi if direction<0 else 0))
        heading_error = wrap_angle(tangent - yaw)
        if done or self.needs_switch:
            return 0.0, 0.0, distance, lateral, heading_error, done
        if abs(alpha) > 0.8:
            return 0.0, max(-self.max_angular, min(self.max_angular, alpha)), distance, lateral, heading_error, False
        curvature = 2 * math.sin(alpha) / max(0.05, math.hypot(dx, dy))
        # Reduce speed rather than clipping curvature at tight turns.
        v = min(self.speed, 0.5 * max(end_distance, remaining),
                self.max_angular / max(abs(curvature), 1e-9))
        return direction*v, v * curvature, distance, lateral, heading_error, False
