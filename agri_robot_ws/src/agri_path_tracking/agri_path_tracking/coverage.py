"""A single coverage execution: waypoint approach, then forward path tracking."""
import math

from agri_base.coordinates import finite_positive
from agri_waypoint.controller import WaypointController
from .controller import Polyline, PurePursuit


class CoverageController:
    def __init__(self, path, start, start_tolerance=0.8, **tracking):
        finite_positive(start_tolerance, 'start_tolerance')
        if not all(math.isfinite(v) for v in start):
            raise ValueError('Invalid approach start position')
        self.tracker = PurePursuit(path, **tracking)
        self.path = path
        self.target = path.points[0]
        self.start = tuple(start)
        self.start_distance = math.dist(self.start, self.target)
        self.distance_to_start = self.start_distance
        self.phase = 'APPROACHING' if self.start_distance > start_tolerance else 'TRACKING'
        self.approach_path = Polyline([self.start, self.target]) if self.phase == 'APPROACHING' else None
        self.approach = WaypointController(
            max_linear=self.tracker.speed, min_linear=min(0.03, self.tracker.speed),
            max_angular=self.tracker.max_angular,
            arrival_tolerance=min(0.15, start_tolerance / 2))

    @property
    def progress(self):
        return self.tracker.progress

    def error(self, x, y):
        """Deviation from the active leg, also needed while paused by safety."""
        if self.phase == 'APPROACHING':
            distance, _, lateral, tangent = self.approach_path.project(x, y, 0, self.approach_path.total)
        else:
            distance, _, lateral, tangent = self.path.project(
                x, y, self.progress, self.tracker.window)
        return distance, lateral, tangent

    def command(self, x, y, yaw):
        self.distance_to_start = math.dist((x, y), self.target)
        if self.phase == 'APPROACHING':
            v, w, _, heading, arrived = self.approach.command(x, y, yaw, self.target)
            distance, lateral, _ = self.error(x, y)
            if arrived:
                self.phase = 'TRACKING'
                # One zero-command cycle at the handover. The original path and
                # its bounded progress are retained; no mode/lease reset occurs.
                distance, lateral, _ = self.error(x, y)
                return 0.0, 0.0, distance, lateral, heading, False
            return v, w, distance, lateral, heading, False
        return self.tracker.command(x, y, yaw)
