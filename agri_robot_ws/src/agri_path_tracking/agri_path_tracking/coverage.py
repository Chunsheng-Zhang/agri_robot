"""A single coverage execution: waypoint approach, then forward path tracking."""
import math
from time import monotonic

from agri_base.coordinates import finite_positive, wrap_angle
from agri_waypoint.controller import WaypointController
from .controller import Polyline, PurePursuit


class AlignmentTimeout(ValueError):
    pass


class CoverageController:
    def __init__(self, path, start, start_tolerance=0.8,
                 alignment_tolerance=math.radians(10), alignment_hold_time=0.5,
                 alignment_max_angular=0.3, alignment_timeout=30.0, **tracking):
        finite_positive(start_tolerance, 'start_tolerance')
        if not all(math.isfinite(v) for v in start):
            raise ValueError('Invalid approach start position')
        self.tracker = PurePursuit(path, **tracking)
        self.path = path
        self.target = path.points[0]
        self.start = tuple(start)
        self.start_distance = math.dist(self.start, self.target)
        self.distance_to_start = self.start_distance
        for name, value in (('alignment_tolerance', alignment_tolerance),
                            ('alignment_hold_time', alignment_hold_time),
                            ('alignment_max_angular', alignment_max_angular),
                            ('alignment_timeout', alignment_timeout)):
            finite_positive(value, name)
        if alignment_tolerance >= math.pi / 2:
            raise ValueError('alignment_tolerance must be less than pi/2')
        self.alignment_tolerance = alignment_tolerance
        self.alignment_hold_time = alignment_hold_time
        self.alignment_max_angular = min(alignment_max_angular, self.tracker.max_angular)
        self.alignment_timeout = alignment_timeout
        self.phase = 'APPROACHING' if self.start_distance > start_tolerance else 'ALIGNING'
        self.alignment_next_phase = 'TRACKING'
        self.alignment_reason = 'start'
        self.alignment_yaw = self.path.at(0)[2] if self.phase == 'ALIGNING' else None
        self.alignment_error = None
        self.alignment_started = self.alignment_since = None
        self.alignment_last_sample = None
        self.approach_path = Polyline([self.start, self.target]) if self.phase == 'APPROACHING' else None
        self.approach = WaypointController(
            max_linear=self.tracker.speed, min_linear=min(0.03, self.tracker.speed),
            max_angular=self.tracker.max_angular,
            arrival_tolerance=min(0.15, start_tolerance / 2))

    def align_for_resume(self):
        # Preserve the original leg and monotonic progress. Never restart at s=0.
        target_phase = self.alignment_next_phase if self.phase == 'ALIGNING' else self.phase
        self.begin_alignment(target_phase, 'resume')

    def begin_alignment(self, next_phase, reason):
        self.phase, self.alignment_next_phase = 'ALIGNING', next_phase
        self.alignment_reason = reason
        self.alignment_yaw = self.alignment_error = None
        self.alignment_started = self.alignment_since = None
        self.alignment_last_sample = None

    def active_leg(self):
        return self.alignment_next_phase if self.phase == 'ALIGNING' else self.phase

    @property
    def progress(self):
        return self.tracker.progress

    def error(self, x, y):
        """Deviation from the active leg, also needed while paused by safety."""
        if self.active_leg() == 'APPROACHING':
            distance, _, lateral, tangent = self.approach_path.project(x, y, 0, self.approach_path.total)
        else:
            distance, _, lateral, tangent = self.path.project(
                x, y, self.progress, self.tracker.window)
        return distance, lateral, tangent

    def command(self, x, y, yaw, now=None, sample_stamp=None):
        now = monotonic() if now is None else now
        self.distance_to_start = math.dist((x, y), self.target)
        if self.phase == 'ALIGNING':
            distance, lateral, tangent = self.error(x, y)
            if self.alignment_started is None:
                self.alignment_started = now
            if now - self.alignment_started >= self.alignment_timeout:
                raise AlignmentTimeout('Heading alignment timed out')
            if self.alignment_yaw is None:
                self.alignment_yaw = (math.atan2(self.target[1]-y, self.target[0]-x)
                                      if self.alignment_next_phase == 'APPROACHING' else tangent)
            self.alignment_error = wrap_angle(self.alignment_yaw - yaw)
            if abs(self.alignment_error) <= self.alignment_tolerance:
                # Reusing one odometry message cannot complete the dwell.
                if sample_stamp is not None and sample_stamp == self.alignment_last_sample:
                    return 0.0, 0.0, distance, lateral, self.alignment_error, False
                self.alignment_last_sample = sample_stamp
                # Stop rotating and require a continuous settling interval.
                if self.alignment_since is None:
                    self.alignment_since = now
                if now - self.alignment_since >= self.alignment_hold_time:
                    self.phase = self.alignment_next_phase
                return 0.0, 0.0, distance, lateral, self.alignment_error, False
            self.alignment_since = None
            self.alignment_last_sample = sample_stamp
            angular = max(-self.alignment_max_angular,
                          min(self.alignment_max_angular, self.alignment_error))
            return 0.0, angular, distance, lateral, self.alignment_error, False
        if self.phase == 'APPROACHING':
            v, w, _, heading, arrived = self.approach.command(x, y, yaw, self.target)
            distance, lateral, _ = self.error(x, y)
            if arrived:
                self.begin_alignment('TRACKING', 'approach_complete')
                self.alignment_yaw = self.path.at(0)[2]
                # No forward command crosses the approach/alignment boundary.
                distance, lateral, _ = self.error(x, y)
                return 0.0, 0.0, distance, lateral, heading, False
            return v, w, distance, lateral, heading, False
        return self.tracker.command(x, y, yaw)
