"""SLAM front end: keyframe selection and landmark association by marker id."""
from dataclasses import dataclass, field
import math
from typing import Dict, List, Tuple


@dataclass
class Keyframe:
    index: int
    odom: Tuple[float, float, float]     # (x, y, yaw) from the EKF
    observations: List[object] = field(default_factory=list)


class FrontEnd:
    def __init__(self, min_translation=0.05, min_rotation=math.radians(10), max_range=0.8):
        self.min_translation = min_translation
        self.min_rotation = min_rotation
        self.max_range = max_range
        self.keyframes: List[Keyframe] = []
        self.landmark_seen: Dict[int, int] = {}   # marker id → times observed

    def needs_keyframe(self, odom) -> bool:
        if not self.keyframes:
            return True
        x0, y0, a0 = self.keyframes[-1].odom
        x, y, a = odom
        return (math.hypot(x - x0, y - y0) >= self.min_translation or
                abs(math.remainder(a - a0, 2 * math.pi)) >= self.min_rotation)

    def add(self, odom, observations) -> Keyframe:
        obs = [o for o in observations if o.range <= self.max_range]
        kf = Keyframe(len(self.keyframes), tuple(odom), obs)
        for o in obs:
            self.landmark_seen[o.id] = self.landmark_seen.get(o.id, 0) + 1
        self.keyframes.append(kf)
        return kf

    @staticmethod
    def relative(a, b):
        """Pose of b in the frame of a: the odometry factor between keyframes."""
        dx, dy = b[0] - a[0], b[1] - a[1]
        c, s = math.cos(a[2]), math.sin(a[2])
        return (c * dx + s * dy, -s * dx + c * dy, math.remainder(b[2] - a[2], 2 * math.pi))
