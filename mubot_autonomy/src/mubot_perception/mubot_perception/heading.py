"""Heading estimation from the IMU.

Complementary filter: integrate the bias-corrected gyro, pull slowly toward
the absolute yaw from the orientation estimate. Gyro bias is learned while the
robot swims straight. A moving average over exactly one stroke period (1/f)
removes the residual head swing, which is periodic at f.
"""
from collections import deque
import math


class StrokeAverage:
    """Moving average over the last 1/f seconds (circular-safe for angles)."""

    def __init__(self, circular: bool = False):
        self.buf = deque()
        self.circular = circular

    def add(self, t: float, x: float, period: float) -> float:
        self.buf.append((t, x))
        while self.buf and self.buf[0][0] <= t - period:
            self.buf.popleft()
        if self.circular:
            s = sum(math.sin(v) for _, v in self.buf)
            c = sum(math.cos(v) for _, v in self.buf)
            return math.atan2(s, c)
        return sum(v for _, v in self.buf) / len(self.buf)


class HeadingEstimator:
    def __init__(self, alpha: float = 0.98, bias_rate: float = 0.01):
        self.alpha = alpha          # weight on the integrated gyro
        self.bias_rate = bias_rate  # learning rate of the gyro bias
        self.yaw = None
        self.bias = 0.0
        self.t = None
        self.avg_yaw = StrokeAverage(circular=True)
        self.avg_rate = StrokeAverage()
        self.avg_gyro = StrokeAverage()

    def update(self, t: float, gyro_z: float, abs_yaw: float, straight: bool,
               stroke_period: float):
        """Returns (ψ, r) after the stroke average."""
        if self.yaw is None:
            self.yaw, self.t = abs_yaw, t
        dt = max(t - self.t, 0.0)
        self.t = t
        period = stroke_period if stroke_period > 0 else 0.2
        rate = gyro_z - self.bias
        predicted = self.yaw + rate * dt
        err = math.remainder(abs_yaw - predicted, 2 * math.pi)
        self.yaw = math.remainder(predicted + (1 - self.alpha) * err, 2 * math.pi)
        gyro_mean = self.avg_gyro.add(t, gyro_z, period)
        if straight and dt > 0:
            # Swimming straight, the true yaw rate averages to ~0 over a stroke,
            # so the stroke-averaged gyro reading is the bias.
            self.bias += self.bias_rate * (gyro_mean - self.bias)
        return self.avg_yaw.add(t, self.yaw, period), self.avg_rate.add(t, rate, period)
