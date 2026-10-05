"""Turn mission goals into a body-frame bearing for behavior_node."""
import math


def bearing_to_point(pose, point):
    """pose (x, y, yaw) in map; point (x, y). Returns (bearing, distance) in the body frame.

    The robot swims toward body −x, so the forward direction is yaw + π.
    """
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    heading_forward = pose[2] + math.pi
    return math.remainder(math.atan2(dy, dx) - heading_forward, 2 * math.pi), math.hypot(dx, dy)
