import math

import pytest

from mubot_control.arbiter import Arbiter
from mubot_control.state_machine import BACKWARD, ESCAPE, FORWARD, TURN, BehaviorStateMachine
from mubot_control.vfh import select_direction


def test_arbiter_priority():
    a = Arbiter()
    assert a.decide(0.0, teleop='x', backward_source='range', avoid_psi=1, target_psi=2).kind == 'override'
    assert a.decide(0.0, backward_source='camera', avoid_psi=1, target_psi=2).source == 'camera'
    assert a.decide(0.0, avoid_psi=1.0, target_psi=2.0).psi_ref == 1.0
    assert a.decide(0.0, target_psi=2.0).psi_ref == 2.0
    held = a.decide(0.5)
    assert held.source == 'hold' and held.psi_ref == 2.0


def test_fsm_turn_hysteresis():
    f = BehaviorStateMachine()
    assert f.update(0, math.radians(3), False, True, 0) == FORWARD
    assert f.update(1, math.radians(6), False, True, 0) == TURN
    assert f.update(2, math.radians(3), False, True, 0) == TURN
    assert f.update(3, math.radians(1), False, True, 0) == FORWARD


def test_fsm_collision_backward_escape_forward():
    f = BehaviorStateMachine()
    assert f.update(0, 0, True, False, 0.0, obstacle_bearing=0.2) == BACKWARD
    assert f.escape_sign == -1.0           # obstacle on the left → escape right
    assert f.update(1, 0, False, False, 0.0) == BACKWARD
    assert f.update(3, 0, False, False, 0.0) == ESCAPE   # t_back exceeded
    assert f.update(4, 0, False, True, math.radians(30)) == ESCAPE
    assert f.update(5, 0, False, True, math.radians(-65)) == FORWARD


def test_vfh_goal_free_keeps_goal():
    h = [0.0] * 13
    d = select_direction(h, -math.radians(32.5), math.radians(5), 0.0, 0.5, 3)
    assert d == pytest.approx(0.0)


def test_vfh_steers_around_blocked_centre():
    h = [0.0] * 13
    for i in range(4, 9):
        h[i] = 1.0                       # blocked from -12.5° to +12.5°
    h[0] = 1.0                           # far right edge blocked too
    d = select_direction(h, -math.radians(32.5), math.radians(5), math.radians(2), 0.5, 3)
    assert d is not None and d > math.radians(12.5)   # the left gap is closer to the goal


def test_vfh_all_blocked():
    assert select_direction([1.0] * 13, -0.5, 0.08, 0.0, 0.5, 3) is None
