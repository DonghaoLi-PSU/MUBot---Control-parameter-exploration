import numpy as np
import pytest
import yaml

from mubot_learning import trainer
from mubot_learning.reward import cog_velocity, forward_speed_reward
from mubot_learning.worker import AnalyticWorker


def test_reward_uses_last_two_seconds_and_negative_x():
    t = np.arange(0, 6.1, 0.00025)
    v = np.where(t > 4.0, -0.05, 0.3)
    assert forward_speed_reward(t, v) == pytest.approx(0.05)
    assert forward_speed_reward([0.1], [1.0]) == 0.0


def test_cog_velocity_weights_by_mass():
    assert cog_velocity([[1.0, 3.0]], [1.0, 3.0]) == pytest.approx([2.5])


def test_reference_case_loads():
    c = trainer.load_case('AN4_Kmed_HM3_AR2')
    assert c['noa'] == 4 and c['stiffness'] == pytest.approx(3.75e-3)
    assert c['hydro'] == 'HM3' and c['n_rollout'] == 50 and c['select'] == 25


def test_train_writes_legacy_files(tmp_path):
    case = {**trainer.load_case('AN4_Kmed_HM3_AR2'), 'n_trial': 1, 'n_episodes': 3, 'n_rollout': 10,
            'select': 5}
    workers = [AnalyticWorker([12, 0.3, 11, 0.6, 10, 0.8, 9, 3])] * 2
    out = tmp_path / 'run'
    reward, mu, trial = trainer.train(case, workers, out, seed=0, log=lambda *_: None)
    tdir = out / 'trial_1'
    assert np.loadtxt(tdir / 'param.csv', delimiter=',').shape == (4, 16)
    assert np.loadtxt(tdir / 'reward.csv', delimiter=',').shape == (3, 10)
    assert np.loadtxt(tdir / 'rollout_param.csv', delimiter=',').shape == (30, 8)
    with pytest.raises(FileExistsError):
        trainer.train(case, workers, out, seed=0, log=lambda *_: None)
