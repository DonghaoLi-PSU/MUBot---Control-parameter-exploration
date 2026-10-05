import math

import pytest

from mubot_control.heading_cascade import HeadingCascade, HeadingGains, wrap_angle


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_rate_setpoint_takes_short_way_and_saturates():
    c = HeadingCascade(HeadingGains(k_psi=2.0, r_max=0.5))
    assert c.rate_setpoint(math.radians(170), math.radians(-170)) < 0  # short way is negative
    assert c.rate_setpoint(1.0, 0.0) == pytest.approx(0.5)


def test_bias_is_clipped_and_integrator_does_not_wind_up():
    c = HeadingCascade(HeadingGains(kp=4.0, ki=2.0, b_max=3.0))
    for _ in range(1000):
        b = c.rate_step(1.0, 0.0, 0.05)
    assert b == pytest.approx(3.0)
    assert abs(c.g.ki * c.integral) <= 3.0 + c.g.kp * 1.0
    # When the error reverses, the bias leaves saturation within a few steps.
    for _ in range(5):
        b = c.rate_step(-1.0, 0.0, 0.05)
    assert b < 0


def test_closed_loop_on_first_order_turn_model():
    # Plant: yaw rate follows K_r·b with time constant tau (fit from a bias-step test).
    K_r, tau, dt = 0.15, 0.4, 0.05
    c = HeadingCascade(HeadingGains(k_psi=1.0, r_max=0.5, kp=4.0, ki=4.0, b_max=5.0))
    psi, r = 0.0, 0.0
    for _ in range(400):
        b = c.step(math.radians(45), psi, r, dt)
        r += dt / tau * (K_r * b - r)
        psi += r * dt
    assert math.degrees(psi) == pytest.approx(45, abs=2.0)
