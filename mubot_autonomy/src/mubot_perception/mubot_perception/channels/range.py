"""Range fan channel: median filter, per-sector Kalman (d, ḋ), TTC, histogram."""
from collections import deque
from dataclasses import dataclass
import math
from statistics import median
from typing import List

from mubot_perception.channels.base import Channel
from mubot_perception.kalman import CVKalman


@dataclass
class SectorEstimate:
    bearing: float
    range: float
    range_rate: float
    ttc: float


@dataclass
class RangeEstimate:
    sectors: List[SectorEstimate]
    front_range: float
    min_ttc: float
    histogram: List[float]
    histogram_min_angle: float
    histogram_bin_width: float


def time_to_collision(d: float, d_dot: float) -> float:
    return d / -d_dot if d_dot < -1e-6 else math.inf


class RangeChannel(Channel):
    def __init__(self, angles, max_range=1.0, front_half_angle=math.radians(15),
                 bin_width=math.radians(5), fov_half=math.radians(60), q=2.0, r=0.003 ** 2):
        self.angles = list(angles)
        self.max_range = max_range
        self.front_half_angle = front_half_angle
        self.bin_width = bin_width
        self.fov_half = fov_half
        self.history = [deque(maxlen=3) for _ in self.angles]
        self.filters = [CVKalman(q, r) for _ in self.angles]
        self.t = None

    def update(self, t: float, ranges):
        dt = 0.0 if self.t is None else t - self.t
        self.t = t
        sectors = []
        for i, (a, z) in enumerate(zip(self.angles, ranges)):
            z = self.max_range if (z is None or not math.isfinite(z)) else min(z, self.max_range)
            self.history[i].append(z)
            zf = median(self.history[i])
            kf = self.filters[i]
            if dt > 0:
                kf.predict(dt)
            kf.update(zf)
            sectors.append(SectorEstimate(a, kf.x, kf.v, time_to_collision(kf.x, kf.v)))
        front = [s.range for s in sectors if abs(s.bearing) <= self.front_half_angle]
        return RangeEstimate(
            sectors=sectors,
            front_range=min(front) if front else self.max_range,
            min_ttc=min((s.ttc for s in sectors), default=math.inf),
            histogram=self.histogram(sectors),
            histogram_min_angle=-self.fov_half,
            histogram_bin_width=self.bin_width)

    def histogram(self, sectors):
        """Obstacle density per bin: 1 at contact, 0 at max range. Unobserved bins = 0."""
        n = int(round(2 * self.fov_half / self.bin_width))
        h = [0.0] * n
        for s in sectors:
            i = int((s.bearing + self.fov_half) // self.bin_width)
            if 0 <= i < n:
                h[i] = max(h[i], 1.0 - min(s.range, self.max_range) / self.max_range)
        return h
