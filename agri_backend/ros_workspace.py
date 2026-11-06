"""ROS service gateway and expiring Web teleoperation; never publishes /cmd_vel."""
import math
import secrets
from threading import Event, Lock
from time import monotonic, time

from fastapi import HTTPException

from .devices import device_cache


def wait_result(future, timeout=2):
    event = Event()
    future.add_done_callback(lambda _: event.set())
    if not event.wait(timeout):
        raise HTTPException(504, 'ROS 确认超时；请先查看实际状态，避免重复开始任务')
    try:
        return future.result()
    except Exception as exc:
        raise HTTPException(503, f'ROS 服务异常：{exc}') from exc


class RosWorkspace:
    def __init__(self, node):
        from geometry_msgs.msg import TwistStamped
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        from nav2_msgs.srv import ManageLifecycleNodes
        from agri_interfaces.srv import AcquireControl
        from std_srvs.srv import Trigger
        self.node, self.clients = node, {}
        from .experiment_recording import ExperimentRecorder
        self.recorder = ExperimentRecorder(node)
        self.types = {'trigger': Trigger, 'get': GetParameters,
                      'set': SetParametersAtomically, 'lifecycle': ManageLifecycleNodes, 'lease': AcquireControl}
        for mode in ('base', 'waypoint', 'coverage'):
            self.add(f'/system/mode/{mode}', 'trigger')
        for action in ('plan', 'cancel'):
            self.add(f'/planning/{action}', 'trigger')
        for name in ('waypoint', 'path_tracker', 'coverage_adapter', 'coverage_server'):
            self.add(f'/{name}/get_parameters', 'get')
            self.add(f'/{name}/set_parameters_atomically', 'set')
        self.add('/coverage_lifecycle_manager/manage_nodes', 'lifecycle')
        self.add('/safety/acquire_control', 'lease')
        self.publisher = node.create_publisher(TwistStamped, '/control/web/cmd_vel', 1)
        self.lock = Lock()
        self.token, self.seq, self.last_command, self.expiry = None, -1, -math.inf, -math.inf
        self.v = self.w = 0.0
        self.lease_revision, self.lease_generation = 0, 0
        node.create_timer(0.05, self.tick)

    def add(self, name, kind):
        self.clients[name] = self.node.create_client(self.types[kind], name)

    def call(self, name, request, timeout=2):
        client = self.clients[name]
        if not client.service_is_ready():
            raise HTTPException(503, f'ROS 服务未就绪：{name}；请确认核心已启动且在同一 ROS Domain')
        return wait_result(client.call_async(request), timeout)

    def trigger(self, name):
        result = self.call(name, self.types['trigger'].Request())
        if not result.success:
            raise HTTPException(409, result.message)
        return {'success': True, 'message': result.message}

    def parameters(self, node, values):
        from rclpy.parameter import Parameter
        request = self.types['set'].Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        result = self.call(f'/{node}/set_parameters_atomically', request).result
        if not result.successful:
            raise HTTPException(409, result.reason)

    def get_parameters(self, node, names):
        from rclpy.parameter import Parameter
        from rcl_interfaces.msg import Parameter as ParameterMessage
        result = self.call(f'/{node}/get_parameters', self.types['get'].Request(names=names))
        return {name: Parameter.from_parameter_msg(ParameterMessage(name=name, value=value)).value
                for name, value in zip(names, result.values)}

    def configure_server(self, values):
        # The bundled server updates widths dynamically, but reads radius/curvature at configure.
        # Use its lifecycle manager, so its bonds stay consistent; no hardware is restarted.
        current = self.get_parameters('coverage_server', list(values))
        if any(current.get(k) != values[k] for k in ('min_turning_radius', 'linear_curv_change') if k in values):
            lifecycle = self.types['lifecycle']
            result = self.call('/coverage_lifecycle_manager/manage_nodes', lifecycle.Request(command=3), 8)
            if not result.success:
                raise HTTPException(409, '规划器重配置失败，尚未开始规划')
            self.parameters('coverage_server', values)
            result = self.call('/coverage_lifecycle_manager/manage_nodes', lifecycle.Request(command=0), 8)
            if not result.success:
                raise HTTPException(503, '规划器未重新激活，请检查 ROS 日志')
        else:
            self.parameters('coverage_server', values)

    def publish(self, v=0.0, w=0.0):
        from geometry_msgs.msg import TwistStamped
        msg = TwistStamped()
        msg.header.frame_id, msg.header.stamp = 'base_link', self.node.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.publisher.publish(msg)

    def arm(self):
        safety = device_cache.snapshot()['safety']
        if not safety['fresh'] or safety.get('mode') != 'base' or safety.get('owner'):
            raise HTTPException(409, '请先切换到基础待命模式，并释放现有控制权')
        revision, generation = safety['mode_revision'], secrets.randbits(53)
        result = self.call('/safety/acquire_control', self.types['lease'].Request(
            source='web', enable=True, mode_revision=revision, task_generation=generation))
        if not result.success:
            raise HTTPException(409, result.message)
        with self.lock:
            self.token, self.seq = secrets.token_hex(16), -1
            self.lease_revision, self.lease_generation = revision, generation
            self.last_command = monotonic()
            self.expiry = -math.inf
            self.v = self.w = 0.0
            self.publish()
            return {'token': self.token, 'server_time': time(), 'limits': safety.get('limits', {}),
                    'heartbeat_protocol': 4}

    def command(self, token, seq, linear, angular, expires_at, tolerate_expired=False):
        safety = device_cache.snapshot()['safety']
        with self.lock:
            if token != self.token or self.token is None or seq <= self.seq:
                raise HTTPException(409, '遥控会话过期或指令乱序，请重新启用')
            if not safety['fresh'] or safety.get('owner') != 'web' or safety.get('mode') != 'base':
                raise HTTPException(409, 'Web 没有底盘控制权')
            if monotonic() - self.last_command > 0.5:
                raise HTTPException(409, '遥控心跳持续中断，请重新启用')
            limits = safety.get('limits', {})
            if abs(linear) > limits.get('linear', 0) or abs(angular) > limits.get('angular', 0):
                raise HTTPException(409, '指令超出 ROS 安全限速')
            remaining = expires_at - time()
            if not math.isfinite(remaining) or remaining > 0.4:
                raise HTTPException(409, '遥控时间戳异常，请重新启用以校准时钟')
            if remaining <= 0:
                overdue = round(min(1e6, -remaining * 1000), 1)
                if not tolerate_expired:
                    raise HTTPException(409, f'遥控指令已过期（晚到{overdue}毫秒），请重新启用')
                self.seq = seq
                # Drop only this frame: preserve the last accepted command and
                # its original expiry. A stale frame cannot renew either timer.
                return {'success': True, 'seq': seq, 'applied': False,
                        'reason': 'expired', 'overdue_ms': overdue}
            self.seq, self.v, self.w = seq, linear, angular
            self.last_command, self.expiry = monotonic(), monotonic() + min(0.2, remaining)
        return {'success': True, 'seq': seq, 'applied': True}

    def stop(self, token=None):
        with self.lock:
            if token is not None and token != self.token:
                return {'success': True}
            self.token = None
            self.v = self.w = 0.0
            self.publish()
            client = self.clients['/safety/acquire_control']
            if client.service_is_ready():
                client.call_async(self.types['lease'].Request(source='web', enable=False,
                    mode_revision=self.lease_revision, task_generation=self.lease_generation))
        return {'success': True}

    def tick(self):
        safety = device_cache.snapshot()['safety']
        with self.lock:
            if self.token is None:
                return
            valid = safety['fresh'] and safety.get('mode') == 'base' and not safety.get('emergency_stop')
            if not valid or monotonic() - self.last_command > 0.5:
                self.token = None
                self.publish()
                client = self.clients['/safety/acquire_control']
                if client.service_is_ready():
                    client.call_async(self.types['lease'].Request(source='web', enable=False,
                        mode_revision=self.lease_revision, task_generation=self.lease_generation))
                return
            moving = monotonic() < self.expiry and safety.get('owner') == 'web'
            if not moving and (self.v != 0 or self.w != 0):
                # Expiry still stops output. Only a newly generated, fresh
                # command from the active held input may resume this session.
                self.v = self.w = 0.0
                self.expiry = -math.inf
            self.publish(self.v if moving else 0, self.w if moving else 0)
