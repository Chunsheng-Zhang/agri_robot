"""Safety extension of the original X3 teleop; original vendor source is unmodified."""

import copy
import json
import math
from time import monotonic

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool

from .runtime import LATCHED, fresh_stamp, run


class TypedPublisher:
    """Adapt vendor PC branch's primitive publish calls to ROS 2 messages."""
    def __init__(self, publisher, message_type):
        self.publisher, self.message_type = publisher, message_type

    def publish(self, value):
        self.publisher.publish(value if isinstance(value, self.message_type) else self.message_type(data=value))


class VelocityPublisher:
    def __init__(self, node):
        self.node = node

    def publish(self, twist):
        msg = TwistStamped()
        # No timer repeats the last nonzero command. Only fresh raw Joy invokes this.
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist = twist
        self.node.safe_velocity.publish(msg)


def validate_buttons(profile, estop, reset):
    used = {7, 9, 11, 13, 14} if profile == 'jetson' else {5, 7, 9, 10}
    if profile not in ('jetson', 'pc') or min(estop, reset) < 0 or estop == reset or used & {estop, reset}:
        raise ValueError('Estop/reset must be distinct unused vendor buttons')


def make_adapter():
    from yahboomcar_ctrl.yahboom_joy_X3 import JoyTeleop

    class JoyAdapter(JoyTeleop):
        def __init__(self):
            super().__init__('joy_adapter')
            for name, default in {'joy_profile': 'jetson', 'joy_estop_button': 0,
                                  'joy_reset_button': 3, 'joy_timeout': 0.5,
                                  'reset_hold_seconds': 1.0}.items():
                self.declare_parameter(name, default)
            self.profile = self.get_parameter('joy_profile').value
            self.estop_index = self.get_parameter('joy_estop_button').value
            self.reset_index = self.get_parameter('joy_reset_button').value
            validate_buttons(self.profile, self.estop_index, self.reset_index)
            self.user_name = 'jetson' if self.profile == 'jetson' else 'pc'
            self.timeout = float(self.get_parameter('joy_timeout').value)
            self.reset_hold = float(self.get_parameter('reset_hold_seconds').value)
            if not math.isfinite(self.timeout + self.reset_hold) or min(self.timeout, self.reset_hold) <= 0:
                raise ValueError('Invalid joystick timeout/reset hold')
            # Destroy the original final velocity publisher before creating any input.
            self.destroy_publisher(self.pub_cmdVel)
            self.safe_velocity = self.create_publisher(TwistStamped, '/control/manual/cmd_vel', 1)
            self.pub_cmdVel = VelocityPublisher(self)
            self.pub_RGBLight = TypedPublisher(self.pub_RGBLight, Int32)
            self.pub_Buzzer = TypedPublisher(self.pub_Buzzer, Bool)
            self.manual_status = self.create_publisher(String, '/manual/status', 1)
            self.stop_pub = self.create_publisher(Bool, '/emergency_stop', 1)
            self.mode_client = self.create_client(SetBool, '/safety/manual/enable')
            self.reset_client = self.create_client(SetBool, '/safety/emergency_stop')
            self.create_subscription(String, '/safety/status', self.on_safety_status, LATCHED)
            self.previous_buttons, self.previous_cancel = [], False
            self.last_joy, self.owner = -math.inf, ''
            self.estop_active = True
            self.reset_since, self.reset_sent, self.pending = None, False, False
            self.fault_reason = 'Waiting for Joy'
            self.create_timer(0.05, self.heartbeat)

        def on_safety_status(self, msg):
            data = json.loads(msg.data)
            self.estop_active = data['emergency_stop']
            self.owner = data['owner']
            if not self.pending and (data['emergency_stop'] or self.owner != 'manual'):
                self.Joy_active = False

        def request_mode(self, active):
            if self.pending or not self.mode_client.service_is_ready():
                self.Joy_active = False
                return
            self.pending = True
            future = self.mode_client.call_async(SetBool.Request(data=active))
            def done(result):
                self.pending = False
                try:
                    self.Joy_active = bool(active and result.result().success)
                    if not result.result().success:
                        self.fault_reason = result.result().message
                except Exception as exc:
                    self.Joy_active, self.fault_reason = False, str(exc)
            future.add_done_callback(done)

        def cancel_nav(self):
            previous = self.Joy_active
            super().cancel_nav()
            if self.Joy_active != previous:
                self.request_mode(self.Joy_active)

        def buttonCallback(self, msg):
            if not hasattr(self, 'estop_index'):
                return
            # A pressed estop is forwarded even if axes/button dimensions are bad.
            if self.estop_index < len(msg.buttons) and msg.buttons[self.estop_index]:
                self.stop_pub.publish(Bool(data=True))
                self.Joy_active = False
            required_axes, required_buttons = (3, 15) if self.profile == 'jetson' else (6, 11)
            if (len(msg.axes) < required_axes or len(msg.buttons) < max(required_buttons, self.estop_index + 1, self.reset_index + 1)
                    or not fresh_stamp(self, msg.header.stamp, self.timeout)
                    or not all(math.isfinite(v) and abs(v) <= 1.01 for v in msg.axes)):
                self.fault_reason = 'Malformed or stale Joy'
                self.last_joy = -math.inf
                return
            self.last_joy, self.fault_reason = monotonic(), ''
            neutral = all(abs(v) < 0.2 for v in msg.axes[:3])
            if (self.estop_active and msg.buttons[self.reset_index]
                    and not msg.buttons[self.estop_index] and neutral):
                if self.reset_since is None:
                    self.reset_since = monotonic()
                if not self.reset_sent and monotonic() - self.reset_since >= self.reset_hold and self.reset_client.service_is_ready():
                    self.reset_client.call_async(SetBool.Request(data=False))
                    self.reset_sent = True
            else:
                self.reset_since, self.reset_sent = None, False
            data = copy.deepcopy(msg)
            # Preserve original button functions, but execute discrete toggles once per press.
            discrete = (7, 9, 11, 13, 14) if self.profile == 'jetson' else (5, 7, 9, 10)
            for i in discrete:
                if i < len(self.previous_buttons) and self.previous_buttons[i] and data.buttons[i]:
                    data.buttons[i] = 0
            cancel = self.profile == 'pc' and msg.axes[5] == -1
            if cancel and self.previous_cancel:
                data.axes[5] = 0.0
            self.previous_cancel = cancel
            self.previous_buttons = list(msg.buttons)
            # Original PC branch always emits velocity; the safety lease still gates it.
            super().buttonCallback(data)

        def heartbeat(self):
            connected = monotonic() - self.last_joy < self.timeout
            if not connected:
                self.Joy_active = False
                self.reset_since, self.reset_sent = None, False
            self.manual_status.publish(String(data=json.dumps({
                'stamp': self.get_clock().now().nanoseconds * 1e-9,
                'connected': connected, 'profile': self.profile, 'owner': self.owner,
                'enabled': self.Joy_active, 'reason': self.fault_reason,
                'estop_button': self.estop_index, 'reset_button': self.reset_index,
                'linear_gear': self.linear_Gear, 'angular_gear': self.angular_Gear})))

    return JoyAdapter()


def main(args=None):
    run(make_adapter, args)
