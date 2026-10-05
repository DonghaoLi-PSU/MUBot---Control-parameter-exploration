"""mission: SeekTarget, Explore, ReturnHome action servers.

Skeleton. Each action turns its goal into a heading bearing for behavior_node
(goals.bearing_to_point) and reports progress. TODO: publish the goal bearing
topic consumed by behavior_node, search pattern when the target is not
visible, frontier selection loop for Explore.
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from mubot_interfaces.action import Explore, ReturnHome, SeekTarget


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.home = None  # first SLAM pose
        ActionServer(self, SeekTarget, '/mubot/seek_target', self.seek_target)
        ActionServer(self, Explore, '/mubot/explore', self.explore)
        ActionServer(self, ReturnHome, '/mubot/return_home', self.return_home)

    def seek_target(self, goal_handle):
        goal_handle.abort()
        return SeekTarget.Result(success=False, message='not implemented yet')

    def explore(self, goal_handle):
        goal_handle.abort()
        return Explore.Result(success=False)

    def return_home(self, goal_handle):
        goal_handle.abort()
        return ReturnHome.Result(success=False)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Mission())
    rclpy.shutdown()
