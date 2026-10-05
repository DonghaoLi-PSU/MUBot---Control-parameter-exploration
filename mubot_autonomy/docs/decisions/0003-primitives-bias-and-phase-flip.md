# 0003. Swimming primitives from one forward gait

**Status:** accepted

**Decision.** Train only the forward gait γ*. Turn = forward + bias ±b on
every active actuator. Backward = forward with every phase negated.

**Consequences.** No retraining for new manoeuvres. Bias reduces amplitude
headroom (|b| ≤ 15 − max e_j). Steady bend per volt is small, so measured turn
rate must drive tuning. Mode switches happen at gait-phase zero crossings and
bias is ramped over one stroke.
