"""Forward Pure Pursuit with bounded, monotonic arc-length progress."""

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
    def __init__(self, points):
        self.points = []
        for x, y in points:
            if not math.isfinite(x + y):
                raise ValueError('Path contains non-finite coordinates')
            if not self.points or math.dist(self.points[-1], (x, y)) > 1e-6:
                self.points.append((x, y))
        if len(self.points) < 2:
            raise ValueError('Path needs two distinct points')
        self.lengths = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.lengths.append(self.lengths[-1] + math.dist(a, b))
        self.total = self.lengths[-1]

    def at(self, distance):
        distance = max(0.0, min(self.total, distance))
        i = min(len(self.points) - 2, max(0, bisect.bisect_right(self.lengths, distance) - 1))
        a, b = self.points[i], self.points[i + 1]
        ratio = (distance - self.lengths[i]) / (self.lengths[i + 1] - self.lengths[i])
        return a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])

    def project(self, x, y, progress, window):
        best = None
        lo, hi = max(0.0, progress - 0.1), min(self.total, progress + window)
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

    def command(self, x, y, yaw):
        distance, s, lateral, tangent = self.path.project(x, y, self.progress, self.window)
        self.progress = max(self.progress, s)
        remaining = self.path.total - self.progress
        end_distance = math.dist((x, y), self.path.points[-1])
        # A loop ending beside its start must not complete before traversing it.
        done = remaining <= self.tolerance and end_distance <= self.tolerance
        tx, ty, _ = self.path.at(min(self.path.total, self.progress + self.lookahead))
        dx, dy = tx - x, ty - y
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)
        heading_error = wrap_angle(tangent - yaw)
        if done:
            return 0.0, 0.0, distance, lateral, heading_error, True
        if abs(alpha) > 0.8:
            return 0.0, max(-self.max_angular, min(self.max_angular, alpha)), distance, lateral, heading_error, False
        curvature = 2 * math.sin(alpha) / max(0.05, math.hypot(dx, dy))
        # Reduce speed rather than clipping curvature at tight turns.
        v = min(self.speed, 0.5 * max(end_distance, remaining),
                self.max_angular / max(abs(curvature), 1e-9))
        return v, v * curvature, distance, lateral, heading_error, False
