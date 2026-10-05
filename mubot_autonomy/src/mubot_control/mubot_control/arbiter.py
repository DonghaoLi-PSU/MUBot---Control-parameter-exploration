"""Fixed-priority arbiter: teleop > BACKWARD > avoid > target > hold.

Inputs never set the bias directly. The winner is either an override
primitive, a BACKWARD request, or a heading reference for the cascade.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class Decision:
    kind: str                       # 'override' | 'backward' | 'heading'
    source: str                     # 'teleop' | 'range' | 'camera' | 'avoid' | 'target' | 'hold'
    psi_ref: Optional[float] = None
    override: object = None


class Arbiter:
    def __init__(self):
        self.last_psi_ref = None

    def decide(self, psi: float, teleop=None, backward_source: Optional[str] = None,
               avoid_psi: Optional[float] = None, target_psi: Optional[float] = None) -> Decision:
        if teleop is not None:
            return Decision('override', 'teleop', override=teleop)
        if backward_source is not None:
            return Decision('backward', backward_source)
        if avoid_psi is not None:
            self.last_psi_ref = avoid_psi
            return Decision('heading', 'avoid', psi_ref=avoid_psi)
        if target_psi is not None:
            self.last_psi_ref = target_psi
            return Decision('heading', 'target', psi_ref=target_psi)
        if self.last_psi_ref is None:
            self.last_psi_ref = psi
        return Decision('heading', 'hold', psi_ref=self.last_psi_ref)
