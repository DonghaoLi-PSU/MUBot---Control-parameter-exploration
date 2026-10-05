"""Cascaded heading controller: the only writer of the bias voltage.

Outer loop: P on heading error → yaw-rate setpoint (limited to ±r_max).
Inner loop: PI on yaw-rate error → bias b (clipped to ±b_max, anti-windup).
"""
from dataclasses import dataclass
import math


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class HeadingGains:
    k_psi: float = 1.5      # [1/s]  heading error → yaw-rate setpoint
    r_max: float = 0.8      # [rad/s]
    kp: float = 4.0         # [V/(rad/s)]
    ki: float = 2.0         # [V/rad]
    b_max: float = 5.0      # [V]; set from 15 − max e_j at runtime


class HeadingCascade:
    def __init__(self, gains: HeadingGains):
        self.g = gains
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def rate_setpoint(self, psi_ref: float, psi: float) -> float:
        r = self.g.k_psi * wrap_angle(psi_ref - psi)
        return max(-self.g.r_max, min(self.g.r_max, r))

    def rate_step(self, r_ref: float, r: float, dt: float, b_max: float = None) -> float:
        """Inner loop only. Used directly by twist_to_primitive (ω commands)."""
        limit = self.g.b_max if b_max is None else b_max
        err = r_ref - r
        candidate = self.integral + err * dt
        b = self.g.kp * err + self.g.ki * candidate
        if abs(b) <= limit or (b > limit and err < 0) or (b < -limit and err > 0):
            self.integral = candidate  # integrate only when it doesn't push further into saturation
        b = self.g.kp * err + self.g.ki * self.integral
        return max(-limit, min(limit, b))

    def step(self, psi_ref: float, psi: float, r: float, dt: float, b_max: float = None) -> float:
        return self.rate_step(self.rate_setpoint(psi_ref, psi), r, dt, b_max)
