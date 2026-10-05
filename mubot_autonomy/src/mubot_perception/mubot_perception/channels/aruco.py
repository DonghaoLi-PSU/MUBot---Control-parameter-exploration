"""ArUco landmarks: marker id, bearing and range for SLAM."""
from dataclasses import dataclass
import math
from typing import List

from mubot_perception.channels.vision import bearing_from_pixel


@dataclass
class LandmarkObservation:
    id: int
    bearing: float
    range: float
    bearing_std: float
    range_std: float


def observation_from_corners(marker_id: int, corners, fx: float, cx: float, size: float,
                             bearing_std: float = math.radians(1.0),
                             range_rel_std: float = 0.05) -> LandmarkObservation:
    """corners: 4 (u, v) image points. Range from the mean side length."""
    us = [c[0] for c in corners]
    u = sum(us) / 4.0
    sides = [math.dist(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    side = max(sum(sides) / 4.0, 1e-6)
    rng = fx * size / side
    return LandmarkObservation(marker_id, bearing_from_pixel(u, fx, cx), rng, bearing_std,
                               range_rel_std * rng)


def detect_markers(image_gray, fx, cx, size, dictionary='DICT_4X4_50') -> List[LandmarkObservation]:
    import cv2
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)))
    corners, ids, _ = det.detectMarkers(image_gray)
    if ids is None:
        return []
    return [observation_from_corners(int(i), c.reshape(4, 2).tolist(), fx, cx, size)
            for i, c in zip(ids.flatten(), corners)]
