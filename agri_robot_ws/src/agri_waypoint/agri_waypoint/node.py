import math
from pathlib import Path
from time import monotonic

import yaml

from agri_base.coordinates import finite_positive
from agri_base.runtime import TaskNode, run, yaw_of
from .controller import WaypointController


class WaypointNode(TaskNode):
    def __init__(self):
        super().__init__('waypoint', 'waypoint')
        defaults = {'waypoints_file': '', 'max_linear': 0.2, 'min_linear': 0.03,
                    'max_angular': 0.5, 'arrival_tolerance': 0.2, 'heading_kp': 1.0,
                    'distance_kp': 0.5, 'rotate_threshold': 1.0, 'align_threshold': 0.2,
                    'dwell_time': 2.0, 'control_rate': 20.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.controller = WaypointController(**{name: self.get_parameter(name).value for name in (
            'max_linear', 'min_linear', 'max_angular', 'arrival_tolerance', 'heading_kp',
            'distance_kp', 'rotate_threshold', 'align_threshold')})
        self.dwell_default = finite_positive(self.get_parameter('dwell_time').value, 'dwell_time', True)
        self.points = []
        self.index, self.dwell = 0, None
        self.last_tick = monotonic()
        rate = finite_positive(self.get_parameter('control_rate').value, 'control_rate')
        self.create_timer(1 / rate, self.tick)

    def prepare(self):
        path = Path(self.get_parameter('waypoints_file').value).expanduser()
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list) or not data['waypoints']:
            raise ValueError('YAML must contain a nonempty waypoints list')
        points = []
        for p in data['waypoints']:
            x, y = self.frame.xy(float(p['latitude']), float(p['longitude']))
            dwell = finite_positive(p.get('dwell_time', self.dwell_default), 'dwell_time', True)
            points.append((x, y, dwell))
        self.points, self.index, self.dwell = points, 0, None
        self.controller.rotating = False

    def tick(self):
        now = monotonic()
        dt, self.last_tick = min(0.2, now - self.last_tick), now
        distance, error = None, None
        if self.ready_to_step():
            if self.dwell is not None:
                self.send()
                self.dwell -= dt
                if self.dwell <= 0:
                    self.index += 1
                    self.dwell = None
                    if self.index >= len(self.points):
                        self.finish('COMPLETED')
            else:
                p = self.odom.pose.pose
                v, w, distance, error, arrived = self.controller.command(
                    p.position.x, p.position.y, yaw_of(p.orientation), self.points[self.index])
                self.send(v, w)
                if arrived:
                    self.dwell = self.points[self.index][2]
        self.publish_status(index=self.index, count=len(self.points), distance=distance,
                            heading_error=error, dwelling=self.dwell is not None)


def main(args=None):
    run(WaypointNode, args)
