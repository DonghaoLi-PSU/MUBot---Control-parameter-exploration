"""cmd_vel (Nav2, teleop) → primitive (mode, k, b).

Forward speed v → amplitude scale k through the inverted speed model.
Turn rate ω → bias b through the inner yaw-rate loop of the heading cascade.
"""
from mubot_control import gait
from mubot_control.heading_cascade import HeadingCascade, HeadingGains


class TwistAdapter:
    def __init__(self, speed_model, gains: HeadingGains, min_speed: float = 0.005):
        self.model = speed_model
        self.cascade = HeadingCascade(gains)
        self.min_speed = min_speed

    def convert(self, v: float, omega: float, yaw_rate: float, dt: float, b_max: float):
        if abs(v) < self.min_speed and abs(omega) < 1e-3:
            self.cascade.reset()
            return gait.STOP, 0.0, 0.0
        mode = gait.BACKWARD if v < 0 else (gait.TURN if abs(omega) > 1e-3 else gait.FORWARD)
        table = 'backward' if v < 0 else 'forward'
        k = self.model.k_for_speed(table, abs(v))
        b = self.cascade.rate_step(omega, yaw_rate, dt, b_max)
        return mode, k, b


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from mubot_interfaces.msg import Primitive
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    from mubot_control.speed_model import SpeedModel

    class TwistToPrimitive(Node):
        def __init__(self):
            super().__init__('twist_to_primitive')
            model = SpeedModel.from_yaml(self.declare_parameter('speed_model', '').value)
            self.b_max = self.declare_parameter('b_max', 5.0).value
            self.adapter = TwistAdapter(model, HeadingGains(b_max=self.b_max))
            self.yaw_rate = 0.0
            self.cmd = (0.0, 0.0)
            self.pub = self.create_publisher(Primitive, '/mubot/primitive_override', 1)
            self.create_subscription(Twist, '/cmd_vel', self.on_twist, 1)
            self.create_subscription(Imu, '/mubot/imu', self.on_imu, 10)
            self.create_timer(0.05, self.tick)

        def on_twist(self, m):
            self.cmd = (m.linear.x, m.angular.z)

        def on_imu(self, m):
            self.yaw_rate = m.angular_velocity.z

        def tick(self):
            mode, k, b = self.adapter.convert(*self.cmd, self.yaw_rate, 0.05, self.b_max)
            msg = Primitive(mode=mode, bias=b, amplitude_scale=k)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)

    rclpy.init(args=args)
    rclpy.spin(TwistToPrimitive())
    rclpy.shutdown()
