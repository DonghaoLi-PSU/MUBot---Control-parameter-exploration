"""Vision channel: colour-blob target, bearing/distance, Kalman tracking, looming."""
from dataclasses import dataclass
import math
from typing import Optional

from mubot_perception.channels.base import Channel
from mubot_perception.kalman import CVKalman


@dataclass
class Blob:
    u: float      # centroid column [px]
    width: float  # apparent width [px]


@dataclass
class TargetEstimate:
    visible: bool
    bearing: float = 0.0
    distance: float = math.inf
    bearing_rate: float = 0.0
    looming_ttc: float = math.inf


def bearing_from_pixel(u: float, fx: float, cx: float) -> float:
    """Body-frame bearing (+ = left). Image u grows to the right."""
    return -math.atan2(u - cx, fx)


def distance_from_width(width_px: float, fx: float, diameter: float) -> float:
    return fx * diameter / max(width_px, 1e-6)


def looming_ttc(w_prev: float, w_now: float, dt: float) -> float:
    """TTC from image expansion, w / ẇ. Needs no knowledge of object size."""
    if dt <= 0:
        return math.inf
    w_dot = (w_now - w_prev) / dt
    return w_now / w_dot if w_dot > 1e-6 else math.inf


def detect_red_blob(image_bgr, min_area: float = 20.0) -> Optional[Blob]:
    """HSV threshold for red (two hue bands), largest contour."""
    import cv2
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | \
        cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    x, _, w, _ = cv2.boundingRect(c)
    return Blob(u=x + w / 2.0, width=float(w))


class VisionChannel(Channel):
    def __init__(self, fx: float, cx: float, diameter: float, lost_after: float = 0.5,
                 q_bearing=1.0, r_bearing=0.01 ** 2, q_dist=0.5, r_dist=0.01 ** 2):
        self.fx, self.cx, self.diameter = fx, cx, diameter
        self.lost_after = lost_after
        self.kf_bearing = CVKalman(q_bearing, r_bearing)
        self.kf_dist = CVKalman(q_dist, r_dist)
        self.t = None
        self.last_seen = None
        self.last_width = None
        self.ttc = math.inf

    def update(self, t: float, blob: Optional[Blob]) -> TargetEstimate:
        dt = 0.0 if self.t is None else t - self.t
        self.t = t
        if self.kf_bearing.initialized and dt > 0:
            self.kf_bearing.predict(dt)
            self.kf_dist.predict(dt)
        if blob is not None:
            if self.last_width is not None and self.last_seen is not None:
                self.ttc = looming_ttc(self.last_width, blob.width, t - self.last_seen)
            self.kf_bearing.update(bearing_from_pixel(blob.u, self.fx, self.cx), circular=True)
            self.kf_dist.update(distance_from_width(blob.width, self.fx, self.diameter))
            self.last_seen, self.last_width = t, blob.width
        visible = self.last_seen is not None and t - self.last_seen <= self.lost_after
        if not visible:
            return TargetEstimate(False)
        return TargetEstimate(True, self.kf_bearing.x, self.kf_dist.x, self.kf_bearing.v, self.ttc)
