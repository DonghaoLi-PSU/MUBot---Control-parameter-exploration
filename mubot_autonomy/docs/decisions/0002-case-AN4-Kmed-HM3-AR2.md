# 0002. Reference case: NoA = 4, medium stiffness, HM-3, AR = 2

**Status:** accepted

**Decision.** The robot used throughout is the 4-actuator μBot (6 segments,
4 active joints + 1 passive caudal joint), K = 3.75e-3 N·m/rad (K̄ = 0.75,
caudal 5×), hydro model HM-3 (Ca = 1, Cp = 0.5), aspect ratio 2, using the
ADD (added-mass inertia included) model.

**Why.** Middle of the paper's design space; NoA = 4 gives most of the speed
gain of NoA = 6 with fewer actuators.

**Consequences.** Other cases are config files (`mubot_learning/config/cases/`),
not code changes. "ADD" is inferred from the URDF numbers, not documented; confirm
with the URDF authors.
