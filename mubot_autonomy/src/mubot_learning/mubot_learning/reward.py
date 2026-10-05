"""Rollout reward: mean forward speed of the centre of gravity.

Legacy Hydro_plugin.cc records the mass-weighted CoG velocity from t = 4 s,
and the reward averages the next 8000 samples at 4 kHz, i.e. t = 4–6 s of a
6.1 s rollout. The robot swims toward −x, so forward speed is −v_x.
"""
import numpy as np


def cog_velocity(link_velocities_x, link_masses):
    """Mass-weighted CoG x-velocity per sample. Shape (samples, links) → (samples,)."""
    v = np.asarray(link_velocities_x, dtype=float)
    m = np.asarray(link_masses, dtype=float)
    return v @ m / m.sum()


def forward_speed_reward(t, v_cog_x, window=(4.0, 6.0)):
    t = np.asarray(t, dtype=float)
    v = np.asarray(v_cog_x, dtype=float)
    sel = (t > window[0]) & (t <= window[1])
    if not sel.any():
        return 0.0  # legacy crashed here with a NameError; an empty window scores zero
    return float(-v[sel].mean())
