"""Gait speed model v = g(k, b, mode), fitted from ground-truth runs.

Used forward by gait odometry (speed estimate for the EKF) and inverted by
twist_to_primitive (desired speed → amplitude scale k).
"""
from bisect import bisect_left
from typing import Dict, List

import yaml


def _interp(xs: List[float], ys: List[float], x: float) -> float:
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect_left(xs, x)
    x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


class SpeedModel:
    """Table: speed[mode][bias_index][k_index] in m/s (forward positive)."""

    def __init__(self, k: List[float], bias: List[float], speed: Dict[str, List[List[float]]]):
        self.k = list(k)
        self.bias = list(bias)
        self.speed = speed

    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        return cls(d['amplitude_scale'], d['bias'], d['speed'])

    def speed_at(self, mode: str, k: float, b: float = 0.0) -> float:
        rows = self.speed[mode]
        per_bias = [_interp(self.k, row, k) for row in rows]
        return _interp(self.bias, per_bias, abs(b))

    def k_for_speed(self, mode: str, v: float, b: float = 0.0) -> float:
        """Smallest amplitude scale reaching speed v (speed must rise with k)."""
        vs = [self.speed_at(mode, k, b) for k in self.k]
        if abs(v) <= abs(vs[0]):
            return self.k[0]
        if abs(v) >= abs(vs[-1]):
            return self.k[-1]
        return _interp([abs(x) for x in vs], self.k, abs(v))
