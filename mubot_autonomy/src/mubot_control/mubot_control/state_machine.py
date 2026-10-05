"""Behaviour state machine: FORWARD, TURN, BACKWARD, ESCAPE.

Transitions are requested here and applied by the gait generator at the next
gait-phase zero crossing.
"""
from dataclasses import dataclass
import math

FORWARD, TURN, BACKWARD, ESCAPE = 'FORWARD', 'TURN', 'BACKWARD', 'ESCAPE'


@dataclass
class StateMachineParams:
    turn_enter: float = math.radians(5.0)    # |ψ_ref − ψ| above this → TURN
    turn_exit: float = math.radians(2.0)     # below this → FORWARD (hysteresis)
    t_back: float = 2.0                      # max time in BACKWARD [s]
    escape_angle: float = math.radians(60.0)


class BehaviorStateMachine:
    def __init__(self, params: StateMachineParams = StateMachineParams()):
        self.p = params
        self.state = FORWARD
        self.entered_at = 0.0
        self.escape_start_heading = 0.0
        self.escape_sign = 1.0

    def _go(self, state, t):
        self.state = state
        self.entered_at = t

    def update(self, t: float, heading_error: float, collision: bool, front_clear: bool,
               psi: float, obstacle_bearing: float = 0.0) -> str:
        s = self.state
        if collision and s in (FORWARD, TURN):
            self._go(BACKWARD, t)
            # Escape away from the side the obstacle is on.
            self.escape_sign = -1.0 if obstacle_bearing > 0 else 1.0
        elif s == BACKWARD and (front_clear or t - self.entered_at > self.p.t_back):
            self.escape_start_heading = psi
            self._go(ESCAPE, t)
        elif s == ESCAPE:
            turned = abs(math.remainder(psi - self.escape_start_heading, 2 * math.pi))
            if turned >= self.p.escape_angle and front_clear:
                self._go(FORWARD, t)
        elif s == FORWARD and abs(heading_error) > self.p.turn_enter:
            self._go(TURN, t)
        elif s == TURN and abs(heading_error) < self.p.turn_exit:
            self._go(FORWARD, t)
        return self.state
