"""EM-based Policy Hyper-parameter Exploration (EPHE), as in the paper.

Port of legacy scripts/EM_pgpe.py, generalized to any number of actuators.
Parameter vector: [e1, Ψ2, e2, Ψ3, e3, …, Ψn, en, f] (paper Eq. 14).
Even indices are amplitudes, odd indices are phases, the last entry is the
frequency. The update rule is unchanged:

  * keep the `select` best rollouts,
  * weights = reward + 0.1,
  * μ = weighted mean (phases wrapped to [0, 1)),
  * σ = weighted standard deviation around the new μ,
  * σ may shrink by at most 20 % per episode and never below a floor.
"""
import numpy as np


def is_phase_index(i: int, n_params: int) -> bool:
    return i % 2 == 1 and i != n_params - 1


class EMPGPE:
    def __init__(self, noa: int, n_rollout: int = 50, select: int = 25, seed=None):
        self.noa = noa
        self.n_params = 2 * noa
        self.n_rollout = n_rollout
        self.select = select
        self.rng = np.random.default_rng(seed)
        # Floors: 1e-2 for amplitudes, 1e-6 for phases and frequency (legacy sig_range).
        self.sigma_floor = np.array(
            [1e-6 if (i % 2 == 1) else 1e-2 for i in range(self.n_params)])

    def initial_distribution(self, amp_range=(10.0, 15.0), freq_range=(0.0, 7.0),
                             sigma_amp=5.0, sigma_phase=0.3, sigma_freq=2.5):
        """Random start, as in legacy start_em_pgpe_sin.py."""
        mu = np.empty(self.n_params)
        sigma = np.empty(self.n_params)
        for i in range(self.n_params):
            if i == self.n_params - 1:
                mu[i], sigma[i] = self.rng.uniform(*freq_range), sigma_freq
            elif i % 2 == 1:
                mu[i], sigma[i] = self.rng.uniform(0.0, 1.0), sigma_phase
            else:
                mu[i], sigma[i] = self.rng.uniform(*amp_range), sigma_amp
        return mu, sigma

    def sample(self, mu, sigma):
        return self.rng.normal(mu, sigma, size=(self.n_rollout, self.n_params))

    def update(self, thetas, rewards, mu_prev, sigma_prev):
        thetas = np.asarray(thetas, dtype=float)
        rewards = np.asarray(rewards, dtype=float)
        best = np.argsort(rewards)[::-1][:self.select]
        w = rewards[best] + 0.1
        s = thetas[best]
        mu = (w[:, None] * s).sum(axis=0) / w.sum()
        for i in range(self.n_params):
            if is_phase_index(i, self.n_params):
                mu[i] %= 1.0
        sigma = np.sqrt((w[:, None] * (s - mu[None, :]) ** 2).sum(axis=0) / w.sum())
        sigma = np.maximum(sigma, 0.8 * np.asarray(sigma_prev))
        sigma = np.maximum(sigma, self.sigma_floor)
        return mu, sigma
