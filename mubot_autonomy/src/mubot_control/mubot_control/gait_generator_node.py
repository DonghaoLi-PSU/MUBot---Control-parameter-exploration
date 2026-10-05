"""gait_generator: γ* + primitive → GaitCmd (50 Hz) and GaitState.

Mode switches are flagged `switch_at_phase` so the actuator system applies
them at the next gait-phase zero crossing. The bias is ramped over one stroke.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
import yaml

from mubot_interfaces.msg import GaitCmd, GaitState, Primitive
from mubot_control import gait


class GaitGenerator(Node):
    def __init__(self):
        super().__init__('gait_generator')
        policy_file = self.declare_parameter('policy_file', '').value
        self.stiffness = self.declare_parameter('stiffness', 3.75e-3).value
        self.tail_ratio = self.declare_parameter('tail_ratio', 5.0).value
        self.rate = self.declare_parameter('rate', 50.0).value
        with open(policy_file) as f:
            p = yaml.safe_load(f)
        self.policy = gait.GaitPolicy(p['amplitude'], p['phase'], p['frequency'])
        self.ramp = gait.BiasRamp(1.0 / self.policy.frequency)
        self.clock = gait.PhaseClock()
        self.request = Primitive(mode=gait.STOP, amplitude_scale=1.0)
        self.mode = gait.STOP

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         deadline=Duration(seconds=0.1))
        self.cmd_pub = self.create_publisher(GaitCmd, '/mubot/gait_cmd', qos)
        self.state_pub = self.create_publisher(GaitState, '/mubot/gait_state', 10)
        self.create_subscription(Primitive, '/mubot/primitive', self.on_primitive, qos)
        self.create_timer(1.0 / self.rate, self.tick)

    def on_primitive(self, msg):
        self.request = msg

    def tick(self):
        dt = 1.0 / self.rate
        r = self.request
        target = gait.apply_primitive(self.policy, r.mode, r.bias, r.amplitude_scale)
        bias = self.ramp.step(target.bias, dt, span=gait.max_bias(self.policy))
        switching = r.mode != self.mode
        self.mode = r.mode
        self.clock.step(target.frequency, dt)

        cmd = GaitCmd(amplitude=target.amplitude, phase=target.phase, frequency=target.frequency,
                      bias=bias, stiffness=self.stiffness, tail_ratio=self.tail_ratio,
                      switch_at_phase=switching)
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(cmd)
        state = GaitState(mode=self.mode, phase=self.clock.phase, frequency=target.frequency,
                          amplitude_scale=r.amplitude_scale, bias=bias)
        state.header = cmd.header
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GaitGenerator())
    rclpy.shutdown()
