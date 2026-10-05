import pytest

from mubot_control.speed_model import SpeedModel
from mubot_control.twist_to_primitive import TwistAdapter
from mubot_control.heading_cascade import HeadingGains
from mubot_control import gait

MODEL = SpeedModel(
    k=[0.0, 0.5, 1.0], bias=[0.0, 4.0],
    speed={'forward': [[0.0, 0.05, 0.10], [0.0, 0.04, 0.08]],
           'backward': [[0.0, 0.02, 0.04], [0.0, 0.015, 0.03]]})


def test_speed_interpolates_in_k_and_bias():
    assert MODEL.speed_at('forward', 0.75) == pytest.approx(0.075)
    assert MODEL.speed_at('forward', 1.0, 2.0) == pytest.approx(0.09)
    assert MODEL.speed_at('forward', 1.0, -2.0) == pytest.approx(0.09)


def test_inverse_speed():
    assert MODEL.k_for_speed('forward', 0.075) == pytest.approx(0.75)
    assert MODEL.k_for_speed('forward', 1.0) == 1.0


def test_twist_adapter_modes():
    a = TwistAdapter(MODEL, HeadingGains())
    assert a.convert(0.0, 0.0, 0.0, 0.05, 3.0)[0] == gait.STOP
    mode, k, b = a.convert(0.05, 0.0, 0.0, 0.05, 3.0)
    assert mode == gait.FORWARD and k == pytest.approx(0.5)
    mode, k, b = a.convert(-0.02, 0.3, 0.0, 0.05, 3.0)
    assert mode == gait.BACKWARD and k == pytest.approx(0.5) and b > 0
