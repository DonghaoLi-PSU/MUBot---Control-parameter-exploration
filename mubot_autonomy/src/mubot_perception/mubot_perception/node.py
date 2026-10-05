"""perception: IMU, range and camera in; heading, obstacles, target, landmarks out.

Estimation only; decisions live in behavior_node. TODO: lifecycle states,
diagnostics, sensor-data QoS on subscriptions.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float64

from mubot_interfaces.msg import (GaitState, Landmark, LandmarkArray, Obstacle, ObstacleArray,
                                  Primitive, Target)
from mubot_perception.channels.aruco import detect_markers
from mubot_perception.channels.range import RangeChannel
from mubot_perception.channels.vision import VisionChannel, detect_red_blob
from mubot_perception.heading import HeadingEstimator


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Perception(Node):
    def __init__(self):
        super().__init__('perception')
        self.diameter = self.declare_parameter('target_diameter', 0.05).value
        self.marker_size = self.declare_parameter('marker_size', 0.04).value
        self.max_range = self.declare_parameter('max_range', 1.0).value
        self.heading = HeadingEstimator(self.declare_parameter('complementary_alpha', 0.98).value)
        self.range = None
        self.vision = None
        self.cam = None
        self.stroke_period = 0.0
        self.straight = False
        from cv_bridge import CvBridge
        self.bridge = CvBridge()

        self.pub_heading = self.create_publisher(Float64, '/mubot/perception/heading', 10)
        self.pub_rate = self.create_publisher(Float64, '/mubot/perception/yaw_rate', 10)
        self.pub_obs = self.create_publisher(ObstacleArray, '/mubot/perception/obstacles', 10)
        self.pub_target = self.create_publisher(Target, '/mubot/perception/target', 10)
        self.pub_lm = self.create_publisher(LandmarkArray, '/mubot/perception/landmarks', 10)
        q = qos_profile_sensor_data
        self.create_subscription(Imu, '/mubot/imu', self.on_imu, q)
        self.create_subscription(LaserScan, '/mubot/range', self.on_scan, q)
        self.create_subscription(Image, '/mubot/camera/image_raw', self.on_image, q)
        self.create_subscription(CameraInfo, '/mubot/camera/camera_info', self.on_info, q)
        self.create_subscription(GaitState, '/mubot/gait_state', self.on_gait, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gait(self, m):
        self.stroke_period = 1.0 / m.frequency if m.frequency > 0 else 0.0
        self.straight = m.mode == Primitive.FORWARD

    def on_imu(self, m):
        psi, r = self.heading.update(self.now(), m.angular_velocity.z,
                                     yaw_from_quaternion(m.orientation), self.straight,
                                     self.stroke_period)
        self.pub_heading.publish(Float64(data=psi))
        self.pub_rate.publish(Float64(data=r))

    def on_scan(self, m):
        if self.range is None:
            angles = [m.angle_min + i * m.angle_increment for i in range(len(m.ranges))]
            self.range = RangeChannel(angles, max_range=self.max_range)
        e = self.range.update(self.now(), list(m.ranges))
        out = ObstacleArray(front_range=e.front_range, min_ttc=e.min_ttc, histogram=e.histogram,
                            histogram_min_angle=e.histogram_min_angle,
                            histogram_bin_width=e.histogram_bin_width)
        out.header = m.header
        out.obstacles = [Obstacle(bearing=s.bearing, range=s.range, range_rate=s.range_rate,
                                  ttc=s.ttc, confidence=1.0) for s in e.sectors]
        self.pub_obs.publish(out)

    def on_info(self, m):
        self.cam = (m.k[0], m.k[2])  # fx, cx
        if self.vision is None:
            self.vision = VisionChannel(m.k[0], m.k[2], self.diameter)

    def on_image(self, m):
        if self.vision is None:
            return
        img = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        e = self.vision.update(self.now(), detect_red_blob(img))
        t = Target(visible=e.visible, bearing=e.bearing, distance=e.distance,
                   bearing_rate=e.bearing_rate, looming_ttc=e.looming_ttc)
        t.header = m.header
        self.pub_target.publish(t)

        import cv2
        fx, cx = self.cam
        lms = detect_markers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), fx, cx, self.marker_size)
        arr = LandmarkArray(landmarks=[Landmark(id=o.id, bearing=o.bearing, range=o.range,
                                                bearing_std=o.bearing_std, range_std=o.range_std)
                                       for o in lms])
        arr.header = m.header
        self.pub_lm.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Perception())
    rclpy.shutdown()
