"""gait_odometry: speed model + gait state → body twist for the EKF.

μBot has no wheels; forward speed comes from v = g(k, b, mode) fitted from
ground-truth runs (tools/fit_speed_model.py). The robot swims toward −x of
the body frame, so forward speed is published as −v along x.
"""
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node

from mubot_control.speed_model import SpeedModel
from mubot_interfaces.msg import GaitState, Primitive


class GaitOdometry(Node):
    def __init__(self):
        super().__init__('gait_odometry')
        self.model = SpeedModel.from_yaml(self.declare_parameter('speed_model', '').value)
        self.var = self.declare_parameter('speed_variance', 1e-4).value
        self.frame = self.declare_parameter('base_frame', 'segment_head').value
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/mubot/gait_odometry/twist', 10)
        self.create_subscription(GaitState, '/mubot/gait_state', self.on_state, 10)

    def on_state(self, m):
        if m.mode == Primitive.STOP or m.frequency <= 0:
            v = 0.0
        elif m.mode == Primitive.BACKWARD:
            v = -self.model.speed_at('backward', m.amplitude_scale, m.bias)
        else:
            v = self.model.speed_at('forward', m.amplitude_scale, m.bias)
        out = TwistWithCovarianceStamped()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.frame
        out.twist.twist.linear.x = -v
        out.twist.covariance[0] = self.var
        out.twist.covariance[7] = self.var  # lateral speed ~0 on average
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GaitOdometry())
    rclpy.shutdown()
