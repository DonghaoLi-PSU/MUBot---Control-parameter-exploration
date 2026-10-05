"""Swimming primitives built from one optimized forward gait.

Turn = forward + bias on every active actuator.
Backward = forward with every phase negated (time-reversed travelling wave).
No ROS imports: unit-tested directly.
"""
from dataclasses import dataclass, field
import math
from typing import List

VOLTAGE_LIMIT = 15.0

FORWARD, TURN, BACKWARD, STOP = 0, 1, 2, 3  # same values as Primitive.msg


@dataclass
class GaitPolicy:
    """Optimized forward gait γ*."""

    amplitude: List[float]  # e_j [V], one per active joint
    phase: List[float]      # Ψ_j [cycles], phase[0] = 0
    frequency: float        # f [Hz]

    @property
    def noa(self) -> int:
        return len(self.amplitude)

    @classmethod
    def from_vector(cls, gamma):
        """Build from the training vector [e1, Ψ2, e2, Ψ3, e3, …, Ψn, en, f].

        This is the ordering used by EM_pgpe.py and the paper's Eq. (14).
        """
        gamma = list(gamma)
        if len(gamma) < 2 or len(gamma) % 2:
            raise ValueError('gamma must have 2*NoA entries')
        n = len(gamma) // 2
        amp = [abs(gamma[0])]
        phase = [0.0]
        for j in range(1, n):
            phase.append(gamma[2 * j - 1] % 1.0)
            amp.append(abs(gamma[2 * j]))
        return cls(amp, phase, abs(gamma[-1]))

    def to_vector(self):
        out = [self.amplitude[0]]
        for j in range(1, self.noa):
            out += [self.phase[j], self.amplitude[j]]
        return out + [self.frequency]


@dataclass
class GaitCommand:
    amplitude: List[float] = field(default_factory=list)
    phase: List[float] = field(default_factory=list)
    frequency: float = 0.0
    bias: float = 0.0


def max_bias(policy: GaitPolicy, amplitude_scale: float = 1.0) -> float:
    """Largest |b| that never clips: 15 − max(k·e_j)."""
    return max(0.0, VOLTAGE_LIMIT - amplitude_scale * max(policy.amplitude))


def apply_primitive(policy: GaitPolicy, mode: int, bias: float = 0.0,
                    amplitude_scale: float = 1.0, bias_limit: float = None) -> GaitCommand:
    """Turn a primitive request into a full actuator command."""
    k = min(max(amplitude_scale, 0.0), 1.0)
    if mode == STOP:
        return GaitCommand([0.0] * policy.noa, [0.0] * policy.noa, 0.0, 0.0)
    amp = [k * e for e in policy.amplitude]
    if mode == BACKWARD:
        phase = [(-p) % 1.0 for p in policy.phase]
    else:
        phase = list(policy.phase)
    limit = max_bias(policy, k) if bias_limit is None else min(bias_limit, max_bias(policy, k))
    b = 0.0
    if mode in (TURN, BACKWARD):
        b = min(max(bias, -limit), limit)
    return GaitCommand(amp, phase, policy.frequency, b)


def voltages(cmd: GaitCommand, phase_now: float):
    """Joint voltages at gait phase `phase_now`, after clipping."""
    out = []
    for e, p in zip(cmd.amplitude, cmd.phase):
        v = e * math.sin(2.0 * math.pi * (phase_now + p)) + cmd.bias
        out.append(min(max(v, -VOLTAGE_LIMIT), VOLTAGE_LIMIT))
    return out


class BiasRamp:
    """Ramps the applied bias toward its target over about one stroke."""

    def __init__(self, ramp_time: float):
        self.ramp_time = max(ramp_time, 1e-6)
        self.value = 0.0

    def step(self, target: float, dt: float, span: float) -> float:
        """Move toward `target` at a rate that covers `span` volts in ramp_time."""
        rate = max(span, 1e-9) / self.ramp_time
        delta = target - self.value
        max_step = rate * dt
        self.value += max(-max_step, min(max_step, delta))
        return self.value


class PhaseClock:
    """Integrates the gait phase and reports zero crossings."""

    def __init__(self):
        self.phase = 0.0

    def step(self, frequency: float, dt: float) -> bool:
        previous = self.phase
        self.phase = (self.phase + frequency * dt) % 1.0
        return self.phase < previous  # wrapped this step
