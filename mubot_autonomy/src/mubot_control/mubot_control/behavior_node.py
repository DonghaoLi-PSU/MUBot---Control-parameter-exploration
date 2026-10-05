"""behavior: state machine + arbiter + VFH+ + heading cascade → Primitive.

Skeleton: the decision logic lives in the tested modules; this node wires
topics to them. TODO: goal handling from mission actions, map-frame goals.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from mubot_interfaces.msg import ObstacleArray, Primitive, Target
from mubot_control import gait
from mubot_control.arbiter import Arbiter
from mubot_control.heading_cascade import HeadingCascade, HeadingGains
from mubot_control.state_machine import BACKWARD, ESCAPE, BehaviorStateMachine, StateMachineParams
from mubot_control.vfh import select_direction


class Behavior(Node):
    def __init__(self):
        super().__init__('behavior')
        g = HeadingGains(
            k_psi=self.declare_parameter('k_psi', 1.5).value,
            r_max=self.declare_parameter('r_max', 0.8).value,
            kp=self.declare_parameter('kp', 4.0).value,
            ki=self.declare_parameter('ki', 2.0).value,
            b_max=self.declare_parameter('b_max', 5.0).value)
        self.t_crit = self.declare_parameter('t_crit', 1.5).value
        self.d_min = self.declare_parameter('d_min', 0.08).value
        self.vfh_threshold = self.declare_parameter('vfh_threshold', 0.5).value
        self.vfh_min_gap = self.declare_parameter('vfh_min_gap_bins', 3).value
        self.dt = 1.0 / self.declare_parameter('rate', 20.0).value

        self.cascade = HeadingCascade(g)
        self.arbiter = Arbiter()
        self.fsm = BehaviorStateMachine(StateMachineParams())
        self.psi, self.r = 0.0, 0.0
        self.obstacles, self.target, self.teleop = None, None, None
        self.goal_bearing = 0.0  # body frame; TODO: from mission goal and map pose

        self.pub = self.create_publisher(Primitive, '/mubot/primitive', 1)
        self.create_subscription(Float64, '/mubot/perception/heading', self.on_heading, 10)
        self.create_subscription(Float64, '/mubot/perception/yaw_rate', self.on_rate, 10)
        self.create_subscription(ObstacleArray, '/mubot/perception/obstacles', self.on_obstacles, 10)
        self.create_subscription(Target, '/mubot/perception/target', self.on_target, 10)
        self.create_subscription(Primitive, '/mubot/primitive_override', self.on_teleop, 1)
        self.create_timer(self.dt, self.tick)

    def on_heading(self, m): self.psi = m.data
    def on_rate(self, m): self.r = m.data
    def on_obstacles(self, m): self.obstacles = m
    def on_target(self, m): self.target = m
    def on_teleop(self, m): self.teleop = m

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        o = self.obstacles
        collision = o is not None and (o.min_ttc < self.t_crit or o.front_range < self.d_min)
        target_psi = None
        if self.target is not None and self.target.visible:
            target_psi = self.psi + self.target.bearing
            self.goal_bearing = self.target.bearing
        avoid_psi = None
        if o is not None and o.histogram:
            d = select_direction(o.histogram, o.histogram_min_angle, o.histogram_bin_width,
                                 self.goal_bearing, self.vfh_threshold, self.vfh_min_gap)
            if d is not None and abs(d - self.goal_bearing) > 1e-3:
                avoid_psi = self.psi + d

        decision = self.arbiter.decide(self.psi, teleop=self.teleop,
                                       backward_source='range' if collision else None,
                                       avoid_psi=avoid_psi, target_psi=target_psi)
        self.teleop = None
        if decision.kind == 'override':
            self.pub.publish(decision.override)
            return
        err = 0.0 if decision.psi_ref is None else math.remainder(decision.psi_ref - self.psi, 2 * math.pi)
        state = self.fsm.update(t, err, collision, front_clear=not collision, psi=self.psi)
        msg = Primitive(amplitude_scale=1.0)
        if state == BACKWARD:
            msg.mode, msg.bias = gait.BACKWARD, 0.0
        elif state == ESCAPE:
            msg.mode, msg.bias = gait.TURN, self.fsm.escape_sign * self.cascade.g.b_max
        else:
            b = self.cascade.step(decision.psi_ref, self.psi, self.r, self.dt)
            msg.mode, msg.bias = (gait.TURN if state == 'TURN' else gait.FORWARD), b
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Behavior())
    rclpy.shutdown()
