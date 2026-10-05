import math

import pytest

from mubot_control import gait

GAMMA = [12.0, 0.25, 11.0, 0.5, 10.0, 0.75, 9.0, 3.0]  # [e1, Ψ2, e2, Ψ3, e3, Ψ4, e4, f]


def test_from_vector_matches_training_order():
    p = gait.GaitPolicy.from_vector(GAMMA)
    assert p.amplitude == [12.0, 11.0, 10.0, 9.0]
    assert p.phase == [0.0, 0.25, 0.5, 0.75]
    assert p.frequency == 3.0
    assert p.to_vector() == GAMMA


def test_phase_wraps_like_em_pgpe():
    p = gait.GaitPolicy.from_vector([12.0, 1.25, 11.0, 3.0])
    assert p.phase == [0.0, 0.25]


def test_backward_negates_phases():
    p = gait.GaitPolicy.from_vector(GAMMA)
    c = gait.apply_primitive(p, gait.BACKWARD)
    assert c.phase == pytest.approx([0.0, 0.75, 0.5, 0.25])
    assert c.amplitude == p.amplitude


def test_backward_is_time_reversal():
    p = gait.GaitPolicy.from_vector(GAMMA)
    fwd = gait.apply_primitive(p, gait.FORWARD)
    back = gait.apply_primitive(p, gait.BACKWARD)
    # E_back(t) = -E_fwd(-t) for every joint
    for t in (0.03, 0.11, 0.2):
        vf = gait.voltages(fwd, (-p.frequency * t) % 1.0)
        vb = gait.voltages(back, (p.frequency * t) % 1.0)
        assert vb == pytest.approx([-v for v in vf], abs=1e-9)


def test_turn_bias_is_clipped_to_headroom():
    p = gait.GaitPolicy.from_vector(GAMMA)
    assert gait.max_bias(p) == pytest.approx(3.0)
    c = gait.apply_primitive(p, gait.TURN, bias=10.0)
    assert c.bias == pytest.approx(3.0)
    c = gait.apply_primitive(p, gait.TURN, bias=-10.0, amplitude_scale=0.5)
    assert c.bias == pytest.approx(-9.0)
    assert max(abs(v) for v in gait.voltages(c, 0.3)) <= 15.0


def test_forward_ignores_bias_and_stop_is_silent():
    p = gait.GaitPolicy.from_vector(GAMMA)
    assert gait.apply_primitive(p, gait.FORWARD, bias=2.0).bias == 0.0
    s = gait.apply_primitive(p, gait.STOP)
    assert s.frequency == 0.0 and all(v == 0.0 for v in gait.voltages(s, 0.4))


def test_bias_ramp_reaches_target_in_one_stroke():
    ramp = gait.BiasRamp(ramp_time=0.5)
    for _ in range(25):
        ramp.step(3.0, 0.02, span=3.0)
    assert ramp.value == pytest.approx(3.0)
    ramp2 = gait.BiasRamp(ramp_time=0.5)
    ramp2.step(3.0, 0.02, span=3.0)
    assert ramp2.value == pytest.approx(0.12)


def test_phase_clock_reports_wrap():
    c = gait.PhaseClock()
    wraps = sum(c.step(3.0, 0.01) for _ in range(100))
    assert wraps == 3
    assert c.phase == pytest.approx(0.0, abs=1e-9) or c.phase == pytest.approx(1.0, abs=1e-9)
