"""Laser geometry in base_link and latched stopping. No motion generation."""

import math


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ScanGeometry:
    def __init__(self, laser_x=0.0435, laser_y=0.00005258, laser_yaw=0.0,
                 body_length=0.45, body_width=0.35, margin=0.05,
                 front_half_angle=math.pi / 3, rear_half_angle=math.pi / 3,
                 min_coverage_fraction=0.85, front_only=False):
        values = (laser_x, laser_y, laser_yaw, body_length, body_width, margin,
                  front_half_angle, rear_half_angle, min_coverage_fraction)
        if (not all(math.isfinite(v) for v in values) or min(body_length, body_width) <= 0
                or margin < 0 or not 0 < front_half_angle <= math.pi
                or not 0 < rear_half_angle <= math.pi
                or not 0 < min_coverage_fraction <= 1):
            raise ValueError('Invalid laser/body/coverage configuration')
        if not isinstance(front_only, bool):
            raise ValueError('front_only must be boolean')
        self.front_only = front_only
        self.x, self.y, self.yaw = laser_x, laser_y, laser_yaw
        self.half_length, self.half_width = body_length / 2, body_width / 2
        self.margin, self.front_angle, self.rear_angle = margin, front_half_angle, rear_half_angle
        if front_only:
            self.front_angle = math.pi / 2
        self.min_coverage = min_coverage_fraction
        self.points, self.beams = [], []
        self.step = 0.0
        self.global_distance = None

    def update(self, ranges, angle_min, angle_max, angle_increment, range_min, range_max):
        self.points, self.beams, self.global_distance = [], [], None
        if (len(ranges) < 3 or not all(math.isfinite(v) for v in
                (angle_min, angle_max, angle_increment, range_min, range_max))
                or range_min < 0 or range_max <= range_min or angle_increment == 0
                or abs(angle_increment) > math.pi / 8
                or abs(angle_max - angle_min - (len(ranges) - 1) * angle_increment) > abs(angle_increment) * 1.5
                or abs(angle_max - angle_min) > 2 * math.pi + abs(angle_increment) * 2):
            raise ValueError('Malformed LaserScan geometry')
        self.step = abs(angle_increment)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        for i, r in enumerate(ranges):
            a = angle_min + i * angle_increment
            bearing = angle_delta(a + self.yaw, 0)
            # Scope follows vehicle orientation about the laser origin. Excluded
            # beams are NOT free-space rays; retain the original resolution.
            if self.front_only and abs(bearing) > math.pi / 2:
                continue
            valid = math.isfinite(r) and 0 < r and range_min <= r <= range_max
            self.beams.append((bearing, valid))
            if not valid:
                continue
            lx, ly = r * math.cos(a), r * math.sin(a)
            self.points.append((self.x + c * lx - s * ly, self.y + s * lx + c * ly))
            self.global_distance = r if self.global_distance is None else min(self.global_distance, r)
        # No return is not an obstacle. Frame integrity/freshness is checked
        # separately; never manufacture a distance for an absent return.

    def sector_valid(self, center, half_angle):
        selected = [valid for angle, valid in self.beams
                    if abs(angle_delta(angle, center)) <= half_angle + self.step / 2]
        # Expected angular samples are independent of the received scan span.
        expected = max(1, int(2 * half_angle / self.step)) if self.step else 1
        return len(selected) >= expected * self.min_coverage

    def nearest_sector(self, center, half_angle):
        values = [math.hypot(x, y) for x, y in self.points
                  if abs(angle_delta(math.atan2(y, x), center)) <= half_angle]
        return min(values) if values else None

    def motion_clearances(self, vx, vy, omega):
        """One clearance per detected point in the motion corridor/turn disk.

        The denominator includes ALL received slots in the protection scope,
        including absent returns. A point in both swept regions is counted once.
        """
        if not all(math.isfinite(v) for v in (vx, vy, omega)):
            return False, [], 0
        total = len(self.beams)
        # Explicit partial protection: rear observations are neither required nor
        # checked, including during turns/reversing. Never claim rear coverage.
        if self.front_only and not self.sector_valid(0, math.pi / 2):
            return False, [], total
        translating, rotating = math.hypot(vx, vy) > 1e-4, abs(omega) > 1e-4
        if translating:
            direction = math.atan2(vy, vx)
            half_angle = self.rear_angle if vx < 0 and abs(vx) >= abs(vy) else self.front_angle
            if not self.front_only and not self.sector_valid(direction, half_angle):
                return False, [], total
            dx, dy = math.cos(direction), math.sin(direction)
            longitudinal = abs(dx) * self.half_length + abs(dy) * self.half_width
            lateral = abs(dy) * self.half_length + abs(dx) * self.half_width + self.margin
        if rotating and not self.front_only and not self.sector_valid(0, math.pi):
            return False, [], total
        radius = math.hypot(self.half_length, self.half_width) + self.margin
        values = []
        for x, y in self.points:
            clearance = math.inf
            if translating:
                along, across = dx * x + dy * y, -dy * x + dx * y
                if along >= -longitudinal and abs(across) <= lateral:
                    # Points already inside the footprint are always hazardous.
                    if along >= 0 or (abs(x) <= self.half_length and abs(y) <= self.half_width):
                        clearance = max(0.0, along - longitudinal)
            if rotating:
                clearance = min(clearance, max(0.0, math.hypot(x, y) - radius))
            if math.isfinite(clearance):
                values.append(clearance)
        return bool(total), values, total

    def clearance(self, vx, vy, omega):
        """Nearest detected body clearance; absence of returns is infinity."""
        valid, values, _ = self.motion_clearances(vx, vy, omega)
        return valid, min(values, default=math.inf) if valid else 0.0

    def occupancy(self, vx, vy, omega, stop_distance, resume_distance):
        valid, values, total = self.motion_clearances(vx, vy, omega)
        stop_count = sum(v < stop_distance for v in values)
        resume_count = sum(v <= resume_distance for v in values)
        return {'sector_observed': valid, 'total_beams': total,
                'stop_points': stop_count, 'stop_fraction': stop_count / total if total else None,
                'resume_points': resume_count, 'resume_fraction': resume_count / total if total else None}


class ObstacleLatch:
    def __init__(self, stop_distance=0.8, resume_distance=1.2, clear_duration=1.0):
        if (not all(math.isfinite(v) for v in (stop_distance, resume_distance, clear_duration))
                or not 0 < stop_distance < resume_distance or clear_duration <= 0):
            raise ValueError('Require 0 < stop distance < resume distance and positive clear duration')
        self.stop_distance, self.resume_distance = stop_distance, resume_distance
        self.clear_duration = clear_duration
        self.active, self.reason = False, ''
        self.direction = (1.0, 0.0, 0.0)
        self.clear_since = None

    def latch(self, reason, direction):
        first = not self.active
        if first:
            self.direction = direction if any(abs(v) > 1e-4 for v in direction) else (1.0, 0.0, 1.0)
            self.reason = reason
        self.active, self.clear_since = True, None
        return first

    def observe(self, valid, clearance, now):
        self.observe_clear(valid, clearance > self.resume_distance, now)

    def observe_clear(self, valid, clear, now):
        if not valid or not clear:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = now

    def reset(self, now, valid):
        if (not valid or self.clear_since is None
                or now - self.clear_since < self.clear_duration):
            return False
        self.active, self.reason, self.clear_since = False, '', None
        return True
