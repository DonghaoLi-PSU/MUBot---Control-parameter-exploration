import math

import pytest

from mubot_perception.channels.aruco import observation_from_corners
from mubot_perception.channels.range import RangeChannel, time_to_collision
from mubot_perception.channels.vision import (Blob, VisionChannel, bearing_from_pixel,
                                              distance_from_width, looming_ttc)
from mubot_perception.heading import HeadingEstimator, StrokeAverage
from mubot_perception.kalman import CVKalman


def test_kalman_tracks_ramp():
    kf = CVKalman(q=1.0, r=1e-4)
    for i in range(200):
        if i:
            kf.predict(0.02)
        kf.update(1.0 - 0.1 * i * 0.02)
    assert kf.v == pytest.approx(-0.1, abs=0.01)


def test_stroke_average_removes_periodic_swing():
    avg = StrokeAverage()
    f, dt, out = 2.0, 0.005, None
    for i in range(400):
        t = i * dt
        out = avg.add(t, 0.3 + 0.2 * math.sin(2 * math.pi * f * t), 1 / f)
    assert out == pytest.approx(0.3, abs=0.01)


def test_heading_learns_gyro_bias_and_removes_swing():
    h = HeadingEstimator(alpha=0.98, bias_rate=0.02)
    f, dt = 2.0, 0.005
    for i in range(4000):
        t = i * dt
        swing = 0.1 * math.sin(2 * math.pi * f * t)
        rate = 0.1 * 2 * math.pi * f * math.cos(2 * math.pi * f * t)
        psi, r = h.update(t, rate + 0.05, 0.5 + swing, True, 1 / f)
    assert h.bias == pytest.approx(0.05, abs=0.01)
    assert psi == pytest.approx(0.5, abs=0.02)
    assert r == pytest.approx(0.0, abs=0.02)


def test_range_channel_ttc_and_histogram():
    angles = [math.radians(a) for a in (-30, -15, 0, 15, 30)]
    ch = RangeChannel(angles)
    e = None
    for i in range(50):
        d = 0.8 - 0.1 * i * 0.02          # closing at 0.1 m/s in front
        e = ch.update(i * 0.02, [1.0, 1.0, d, 1.0, 1.0])
    assert e.front_range == pytest.approx(0.7, abs=0.01)
    assert e.min_ttc == pytest.approx(7.0, rel=0.15)
    assert max(e.histogram) == pytest.approx(0.3, abs=0.02)
    assert time_to_collision(1.0, 0.1) == math.inf


def test_range_channel_rejects_spike():
    ch = RangeChannel([0.0])
    for i in range(5):
        ch.update(i * 0.02, [0.5])
    e = ch.update(0.12, [0.05])           # single spike
    assert e.front_range > 0.4


def test_vision_geometry():
    assert bearing_from_pixel(160, 260, 160) == 0.0
    assert bearing_from_pixel(100, 260, 160) > 0      # left of centre → positive
    assert distance_from_width(26, 260, 0.05) == pytest.approx(0.5)
    assert looming_ttc(20, 22, 0.1) == pytest.approx(1.1)
    assert looming_ttc(22, 20, 0.1) == math.inf


def test_vision_channel_tracks_and_loses_target():
    ch = VisionChannel(fx=260, cx=160, diameter=0.05, lost_after=0.5)
    for i in range(10):
        e = ch.update(i / 15, Blob(u=120, width=26))
    assert e.visible and e.distance == pytest.approx(0.5, abs=0.01)
    assert e.bearing == pytest.approx(math.atan2(40, 260), abs=0.01)
    assert not ch.update(10 / 15 + 0.6, None).visible


def test_aruco_observation():
    o = observation_from_corners(7, [(150, 110), (170, 110), (170, 130), (150, 130)], 260, 160, 0.04)
    assert o.id == 7 and o.bearing == pytest.approx(0.0)
    assert o.range == pytest.approx(0.52)
