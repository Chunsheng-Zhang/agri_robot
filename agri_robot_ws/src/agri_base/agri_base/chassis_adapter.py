"""Watchdog and standard Twist boundary in front of an unmodified driver."""

import json
import math
from time import monotonic

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .runtime import LATCHED, run


class ChassisAdapter(Node):
    def __init__(self):
        super().__init__('chassis_adapter')
        self.last_cmd, self.last_safety = -math.inf, -math.inf
        self.enabled = False
        self.inhibited = True
        self.idle_quiet = False
        self.owner = ''
        self.command = Twist()
        self.declare_parameter('allow_lateral', False)
        self.pub = self.create_publisher(Twist, '/base/driver_cmd_vel', 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        self.create_subscription(String, '/safety/status', self.on_safety, LATCHED)
        self.create_timer(0.05, self.tick)

    def on_safety(self, msg):
        data = json.loads(msg.data)
        inhibited = (bool(data['emergency_stop'])
                     or bool(data.get('motion_inhibited', False))
                     or bool(data.get('obstacle_stop', False)))
        enabled = bool(data['owner']) and not inhibited
        if data['owner'] != self.owner or enabled != self.enabled:
            self.command, self.last_cmd = Twist(), -math.inf
        self.owner, self.enabled, self.inhibited = data['owner'], enabled, inhibited
        self.last_safety = monotonic()
        self.tick()

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        self.command = msg if all(math.isfinite(v) for v in values) else Twist()
        self.last_cmd = monotonic()

    def tick(self):
        now = monotonic()
        # A rogue publisher must not share either control boundary.
        unique = (self.count_publishers('/cmd_vel') == 1
                  and self.count_publishers('/base/driver_cmd_vel') == 1)
        safety_fresh = now - self.last_safety < 0.3
        idle = not self.owner and not self.inhibited and safety_fresh and unique
        if idle:
            # Stop once when relinquishing Jetson control, then leave the MCU
            # alone for direct SBUS input. This does not detect SBUS ownership.
            if self.idle_quiet:
                return
            self.command, self.last_cmd = Twist(), -math.inf
        self.idle_quiet = idle
        # Estop/inhibit, missing safety or competing publishers never enter the
        # quiet idle state; they retain continuous zero output, even ownerless.
        ok = self.enabled and unique and now - self.last_cmd < 0.25 and safety_fresh
        out = Twist()
        if ok:
            out.linear.x = self.command.linear.x
            if self.get_parameter('allow_lateral').value:
                out.linear.y = self.command.linear.y
            out.angular.z = self.command.angular.z
        self.pub.publish(out)

    def destroy_node(self):
        self.pub.publish(Twist())
        return super().destroy_node()


def main(args=None):
    run(ChassisAdapter, args)
