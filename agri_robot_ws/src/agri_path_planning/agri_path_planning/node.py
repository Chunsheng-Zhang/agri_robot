"""Standalone action client for the existing OpenNav coverage server."""

import json
import math
from datetime import datetime
from pathlib import Path as FilePath
from time import monotonic

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates

from agri_base.coordinates import frame_from_node, finite_positive
from agri_base.runtime import LATCHED, run, yaw_of
from .field import load_field, validate_path


class CoverageAdapter(Node):
    def __init__(self):
        super().__init__('coverage_adapter')
        self.frame = frame_from_node(self)
        defaults = {'boundary_file': '', 'output_directory': '', 'robot_width': 0.35,
                    'headland_width': 2.0, 'swath_angle': 0.0,
                    'sampling_distance': 0.1, 'planning_timeout': 120.0,
                    'generate_decomp': False}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('robot_width', 'sampling_distance', 'planning_timeout'):
            finite_positive(self.get_parameter(name).value, name)
        finite_positive(self.get_parameter('headland_width').value, 'headland_width', True)
        if not math.isfinite(self.get_parameter('swath_angle').value):
            raise ValueError('Invalid swath_angle')
        self.client = ActionClient(self, ComputeCoveragePath, '/compute_coverage_path')
        self.path_pub = self.create_publisher(Path, '/reference_path', LATCHED)
        self.status = self.create_publisher(String, '/planning/status', LATCHED)
        self.create_subscription(String, '/localization/origin', self.on_origin, LATCHED)
        self.create_service(Trigger, '/planning/plan', self.plan)
        self.create_service(Trigger, '/planning/cancel', self.cancel)
        self.state, self.reason = 'IDLE', ''
        self.origin_matches = False
        self.token = 0
        self.goal_handle = None
        self.deadline = math.inf
        self.create_timer(0.2, self.tick)

    def on_origin(self, msg):
        self.origin_matches = json.loads(msg.data) == self.frame.identity

    def plan(self, request, response):
        response.success = False
        if self.state == 'PLANNING' or not self.origin_matches or not self.client.server_is_ready():
            response.message = 'Busy, coordinate origin unavailable, or coverage action unavailable'
            return response
        try:
            self.polygon = load_field(self.get_parameter('boundary_file').value, self.frame)
            goal = ComputeCoveragePath.Goal()
            goal.frame_id = 'map'
            goal.generate_decomp = self.get_parameter('generate_decomp').value
            goal.generate_headland = self.get_parameter('headland_width').value > 0
            goal.generate_route = True
            goal.generate_path = True
            goal.headland_mode.mode = 'CONSTANT'
            goal.headland_mode.width = float(self.get_parameter('headland_width').value)
            goal.swath_mode.mode, goal.swath_mode.objective = 'SET_ANGLE', 'LENGTH'
            goal.swath_mode.best_angle = float(self.get_parameter('swath_angle').value)
            goal.route_mode.mode = 'TSP' if goal.generate_decomp else 'BOUSTROPHEDON'
            goal.path_mode.mode, goal.path_mode.continuity_mode = 'DUBIN', 'CONTINUOUS'
            goal.path_mode.turn_point_distance = float(self.get_parameter('sampling_distance').value)
            rings = [self.polygon.exterior, *self.polygon.interiors]
            for ring in rings:
                coords = Coordinates()
                coords.coordinates = [Coordinate(axis1=float(x), axis2=float(y)) for x, y in ring.coords]
                goal.polygons.append(coords)
            self.token += 1
            token = self.token
            self.state, self.reason = 'PLANNING', ''
            self.deadline = monotonic() + self.get_parameter('planning_timeout').value
            # Invalidate a previous idle reference until this request succeeds.
            empty = Path()
            empty.header.frame_id = 'map'
            self.path_pub.publish(empty)
            future = self.client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.on_goal(f, token))
            response.success, response.message = True, 'PLANNING; inspect /planning/status'
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.state, self.reason = 'FAILED', str(exc)
            response.message = str(exc)
        return response

    def on_goal(self, future, token):
        try:
            handle = future.result()
            if token != self.token:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                raise ValueError('Coverage goal rejected')
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.on_result(f, token))
        except Exception as exc:
            if token == self.token:
                self.state, self.reason = 'FAILED', str(exc)

    def on_result(self, future, token):
        if token != self.token:
            return
        self.goal_handle = None
        try:
            wrapped = future.result()
            result = wrapped.result
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
                raise ValueError(f'Coverage action failed: {result.error_code}')
            path = result.nav_path
            if path.header.frame_id != 'map':
                raise ValueError('Planner returned a different coordinate frame')
            backward = getattr(result.coverage_path, 'is_backward', [])
            velocities = getattr(result.coverage_path, 'velocities', [])
            if any(backward) or any(v < 0 for v in velocities):
                raise ValueError('Reverse paths require a future reverse-capable controller')
            points = []
            for pose in path.poses:
                # Local OpenNav toMsg(PathState) leaves individual headers empty.
                # Inherit the validated Path header; reject explicit contradictions.
                if pose.header.frame_id not in ('', 'map'):
                    raise ValueError('Mixed-frame path')
                pose.header = path.header
                yaw_of(pose.pose.orientation)
                x, y = pose.pose.position.x, pose.pose.position.y
                if not math.isfinite(x + y) or math.hypot(x, y) > self.frame.max_radius:
                    raise ValueError('Path outside local operating radius')
                points.append((x, y))
            validate_path(points, self.polygon, self.get_parameter('robot_width').value)
            if not self.origin_matches:
                raise ValueError('Coordinate origin changed during planning')
            self.save(path, velocities, backward)
            self.path_pub.publish(path)
            self.state, self.reason = 'SUCCEEDED', f'{len(points)} path points'
        except Exception as exc:
            self.state, self.reason = 'FAILED', str(exc)

    def save(self, path, velocities, backward):
        folder = self.get_parameter('output_directory').value
        if not folder:
            return
        target = FilePath(folder).expanduser()
        target.mkdir(parents=True, exist_ok=True)
        filename = datetime.now().strftime('coverage_%Y%m%d_%H%M%S_%f.json')
        data = {'frame_id': 'map', 'projection': self.frame.identity,
                'velocities': list(velocities), 'is_backward': list(backward),
                'points': [{'x': p.pose.position.x, 'y': p.pose.position.y,
                            'yaw': yaw_of(p.pose.orientation)} for p in path.poses]}
        with (target / filename).open('x', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)

    def cancel(self, request, response):
        self.token += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.state, self.reason = 'CANCELED', ''
        response.success, response.message = True, self.state
        return response

    def tick(self):
        if self.state == 'PLANNING' and monotonic() > self.deadline:
            self.cancel(None, Trigger.Response())
            self.state, self.reason = 'FAILED', 'Planning timeout'
        self.status.publish(String(data=json.dumps({'state': self.state, 'reason': self.reason})))


def main(args=None):
    run(CoverageAdapter, args)
