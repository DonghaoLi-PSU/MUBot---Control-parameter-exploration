"""Constant-velocity Kalman filter for one scalar (distance, bearing, ...)."""
import math


class CVKalman:
    """State [x, ẋ]. Process noise is white acceleration with std `q`."""

    def __init__(self, q: float, r: float, x0: float = 0.0):
        self.q, self.r = q, r
        self.x, self.v = x0, 0.0
        self.P = [[r, 0.0], [0.0, 1.0]]
        self.initialized = False

    def predict(self, dt: float):
        P, q = self.P, self.q
        self.x += self.v * dt
        p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + q * q * dt ** 4 / 4
        p01 = P[0][1] + dt * P[1][1] + q * q * dt ** 3 / 2
        p11 = P[1][1] + q * q * dt * dt
        self.P = [[p00, p01], [p01, p11]]

    def update(self, z: float, circular: bool = False):
        if not self.initialized:
            self.x, self.v, self.initialized = z, 0.0, True
            self.P = [[self.r, 0.0], [0.0, 1.0]]
            return
        y = z - self.x
        if circular:
            y = math.remainder(y, 2 * math.pi)
        s = self.P[0][0] + self.r
        k0, k1 = self.P[0][0] / s, self.P[1][0] / s
        self.x += k0 * y
        self.v += k1 * y
        P = self.P
        self.P = [[(1 - k0) * P[0][0], (1 - k0) * P[0][1]],
                  [P[1][0] - k1 * P[0][0], P[1][1] - k1 * P[0][1]]]
