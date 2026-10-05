import numpy as np
import pytest

from mubot_learning.em_pgpe import EMPGPE, is_phase_index


def test_phase_indices():
    assert [i for i in range(8) if is_phase_index(i, 8)] == [1, 3, 5]


def test_initial_distribution_ranges():
    em = EMPGPE(4, seed=0)
    mu, sigma = em.initial_distribution()
    assert mu.shape == (8,)
    assert all(10 <= mu[i] <= 15 for i in (0, 2, 4, 6))
    assert all(0 <= mu[i] <= 1 for i in (1, 3, 5))
    assert 0 <= mu[7] <= 7
    assert list(sigma) == [5, 0.3, 5, 0.3, 5, 0.3, 5, 2.5]


def test_update_matches_legacy_rule():
    em = EMPGPE(2, n_rollout=4, select=2, seed=1)
    thetas = np.array([[10, 0.9, 12, 3], [11, 0.95, 13, 4], [14, 0.1, 10, 2], [9, 0.5, 9, 1]], float)
    rewards = np.array([0.02, 0.04, 0.01, 0.0])
    mu_prev, sigma_prev = np.full(4, 1.0), np.full(4, 1e-9)
    mu, sigma = em.update(thetas, rewards, mu_prev, sigma_prev)
    w = np.array([0.14, 0.12])                       # top two, reward + 0.1
    s = thetas[[1, 0]]
    expect = (w[:, None] * s).sum(0) / w.sum()
    expect[1] %= 1.0
    assert mu == pytest.approx(expect)
    assert sigma[0] == pytest.approx(np.sqrt((w * (s[:, 0] - expect[0]) ** 2).sum() / w.sum()))
    # sigma never shrinks by more than 20 %
    _, sigma2 = em.update(thetas, rewards, mu_prev, np.full(4, 10.0))
    assert np.all(sigma2 >= 8.0)


def test_converges_on_known_optimum():
    from mubot_learning.worker import AnalyticWorker
    optimum = np.array([12.0, 0.3, 11.0, 0.6, 10.0, 0.8, 9.0, 3.0])
    w = AnalyticWorker(optimum)
    em = EMPGPE(4, n_rollout=50, select=25, seed=3)
    mu, sigma = em.initial_distribution()
    for _ in range(40):
        th = em.sample(mu, sigma)
        mu, sigma = em.update(th, np.array([w.evaluate(t, {}) for t in th]), mu, sigma)
    assert w.evaluate(mu, {}) > 0.9
