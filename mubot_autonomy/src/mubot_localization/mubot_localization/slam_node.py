"""slam: landmark pose-graph SLAM + occupancy grid; publishes map → odom.

Skeleton wiring the tested FrontEnd, PoseGraph and OccupancyMapper.
TODO: lifecycle states, landmark MarkerArray, keyframe-pose buffering so the
grid is rebuilt after loop closures, look up sensor offsets from TF.
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from mubot_interfaces.msg import LandmarkArray
from mubot_localization.frontend import FrontEnd
from mubot_localization.occupancy_mapper import OccupancyMapper
from mubot_localization.pose_graph import PoseGraph


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Slam(Node):
    def __init__(self):
        super().__init__('slam')
        self.front = FrontEnd(self.declare_parameter('keyframe_translation', 0.05).value,
                              math.radians(self.declare_parameter('keyframe_rotation_deg', 10.0).value),
                              self.declare_parameter('max_landmark_range', 0.8).value)
        self.graph = PoseGraph()
        self.mapper = OccupancyMapper(resolution=self.declare_parameter('grid_resolution', 0.01).value)
        self.odom = None
        self.landmarks = []
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.tf = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(OccupancyGrid, '/mubot/map', latched)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/mubot/pose', 10)
        self.create_subscription(Odometry, '/mubot/odometry/filtered', self.on_odom, 10)
        self.create_subscription(LandmarkArray, '/mubot/perception/landmarks', self.on_lm, 10)
        self.create_subscription(LaserScan, '/mubot/range', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.5, self.publish_map)
        self.create_timer(0.05, self.publish_tf)

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.front.needs_keyframe(self.odom):
            prev = self.front.keyframes[-1].odom if self.front.keyframes else None
            kf = self.front.add(self.odom, self.landmarks)
            self.landmarks = []
            delta = FrontEnd.relative(prev, self.odom) if prev else None
            guess = self.compose(self.map_to_odom, self.odom)
            x, y, a = self.graph.add_keyframe(delta, guess, kf.observations)
            self.map_to_odom = self.between((x, y, a), self.odom)
            out = PoseWithCovarianceStamped()
            out.header.stamp, out.header.frame_id = m.header.stamp, 'map'
            out.pose.pose.position.x, out.pose.pose.position.y = x, y
            out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
            self.pub_pose.publish(out)

    def on_lm(self, m):
        self.landmarks.extend(m.landmarks)

    def on_scan(self, m):
        if self.odom is None:
            return
        pose = self.compose(self.map_to_odom, self.odom)
        bearings = [m.angle_min + i * m.angle_increment for i in range(len(m.ranges))]
        self.mapper.integrate(pose, (0.0, 0.0, math.pi), bearings, list(m.ranges))

    @staticmethod
    def compose(a, b):
        c, s = math.cos(a[2]), math.sin(a[2])
        return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
                math.remainder(a[2] + b[2], 2 * math.pi))

    @staticmethod
    def between(map_pose, odom_pose):
        """T_map_odom such that map_pose = T_map_odom ∘ odom_pose."""
        a = math.remainder(map_pose[2] - odom_pose[2], 2 * math.pi)
        c, s = math.cos(a), math.sin(a)
        return (map_pose[0] - (c * odom_pose[0] - s * odom_pose[1]),
                map_pose[1] - (s * odom_pose[0] + c * odom_pose[1]), a)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        x, y, a = self.map_to_odom
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(a / 2), math.cos(a / 2)
        self.tf.sendTransform(t)

    def publish_map(self):
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.info.resolution = self.mapper.res
        g.info.width, g.info.height = self.mapper.nx, self.mapper.ny
        g.info.origin.position.x, g.info.origin.position.y = self.mapper.origin
        g.info.origin.orientation.w = 1.0
        g.data = self.mapper.to_ros_data()
        self.pub_map.publish(g)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Slam())
    rclpy.shutdown()
