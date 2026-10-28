"""Geometric waypoint controller, independent of coverage and Pure Pursuit."""

import math
from agri_base.coordinates import finite_positive, wrap_angle


class WaypointController:
    def __init__(self, max_linear=0.2, min_linear=0.03, max_angular=0.5,
                 arrival_tolerance=0.2, heading_kp=1.0, distance_kp=0.5,
                 rotate_threshold=1.0, align_threshold=0.2):
        for name, value in locals().copy().items():
            if name != 'self':
                finite_positive(value, name)
        if min_linear > max_linear or not align_threshold < rotate_threshold <= math.pi:
            raise ValueError('Invalid speed or heading thresholds')
        self.max_linear, self.min_linear = max_linear, min_linear
        self.max_angular, self.arrival = max_angular, arrival_tolerance
        self.heading_kp, self.distance_kp = heading_kp, distance_kp
        self.rotate_threshold, self.align_threshold = rotate_threshold, align_threshold
        self.rotating = False

    def command(self, x, y, yaw, target):
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        error = wrap_angle(math.atan2(dy, dx) - yaw)
        if distance <= self.arrival:
            self.rotating = False
            return 0.0, 0.0, distance, error, True
        if abs(error) >= self.rotate_threshold:
            self.rotating = True
        elif abs(error) <= self.align_threshold:
            self.rotating = False
        angular = max(-self.max_angular, min(self.max_angular, self.heading_kp * error))
        linear = 0.0 if self.rotating else min(self.max_linear,
                     max(self.min_linear, self.distance_kp * distance)) * max(0.0, math.cos(error))
        return linear, angular, distance, error, False
