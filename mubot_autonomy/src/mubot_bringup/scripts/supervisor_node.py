#!/usr/bin/env python3
"""supervisor: mode switch, e-stop, command-deadline monitoring.

Skeleton. TODO: drive lifecycle transitions of perception/behavior/slam through
lifecycle_manager, aggregate /diagnostics, switch the actuator command source
for train mode.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from mubot_interfaces.msg import ActuatorState
from mubot_interfaces.srv import SetMode


class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estop_pub = self.create_publisher(Bool, '/mubot/estop', latched)
        self.estop_pub.publish(Bool(data=False))
        self.mode = SetMode.Request.DEPLOY
        self.create_service(SetMode, '/mubot/set_mode', self.on_set_mode)
        self.create_subscription(ActuatorState, '/mubot/actuator_state', self.on_state, 10)

    def on_set_mode(self, req, res):
        self.mode = req.mode
        res.success, res.message = True, f'mode {req.mode}'
        return res

    def on_state(self, m):
        if m.watchdog_tripped:
            self.get_logger().warn('actuator watchdog tripped: no gait commands for 200 ms',
                                   throttle_duration_sec=2.0)


def main():
    rclpy.init()
    rclpy.spin(Supervisor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
