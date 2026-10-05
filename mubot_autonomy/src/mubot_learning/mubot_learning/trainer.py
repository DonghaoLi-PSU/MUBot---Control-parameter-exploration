"""EM-PGPE training loop with parallel rollout workers.

Replaces legacy start_em_pgpe_sin.py: no CSV file bus, no hard-coded paths,
no deletion of previous results, rollouts evaluated in parallel.

Output per trial (same files as the legacy script):
  param.csv          one row per episode: μ then σ (first row = initial)
  reward.csv         one row per episode: the n_rollout rewards
  rollout_param.csv  every sampled parameter vector
and the best final μ as a policy YAML.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from mubot_learning.em_pgpe import EMPGPE

HERE = Path(__file__).resolve().parent.parent


def load_case(name_or_path, config_dir=HERE / 'config'):
    p = Path(name_or_path)
    if not p.exists():
        p = config_dir / 'cases' / f'{name_or_path}.yaml'
    with open(config_dir / 'em_pgpe.yaml') as f:
        algo = yaml.safe_load(f)
    with open(p) as f:
        case = yaml.safe_load(f)
    return {**algo, **case}


def train(case, workers, out_dir: Path, seed=None, log=print):
    out_dir.mkdir(parents=True, exist_ok=False)  # never overwrite earlier runs
    with open(out_dir / 'case.yaml', 'w') as f:
        yaml.safe_dump(case, f)
    best = (-np.inf, None, None)
    rng = np.random.default_rng(seed)
    with ThreadPoolExecutor(max_workers=len(workers)) as pool:
        for trial in range(1, case['n_trial'] + 1):
            em = EMPGPE(case['noa'], case['n_rollout'], case['select'], seed=rng.integers(1 << 31))
            mu, sigma = em.initial_distribution()
            tdir = out_dir / f'trial_{trial}'
            tdir.mkdir()
            with open(tdir / 'param.csv', 'w') as fp, open(tdir / 'reward.csv', 'w') as fr, \
                    open(tdir / 'rollout_param.csv', 'w') as fro:
                np.savetxt(fp, np.hstack((mu, sigma))[None], fmt='%.6e', delimiter=',')
                for episode in range(1, case['n_episodes'] + 1):
                    thetas = em.sample(mu, sigma)
                    jobs = [pool.submit(workers[i % len(workers)].evaluate, th, case)
                            for i, th in enumerate(thetas)]
                    rewards = np.array([j.result() for j in jobs])
                    mu, sigma = em.update(thetas, rewards, mu, sigma)
                    np.savetxt(fp, np.hstack((mu, sigma))[None], fmt='%.6e', delimiter=',')
                    np.savetxt(fr, rewards[None], fmt='%.4e', delimiter=',')
                    np.savetxt(fro, thetas, fmt='%.6e', delimiter=',')
                    log(f'trial {trial} episode {episode}: mean {rewards.mean():.4f} '
                        f'max {rewards.max():.4f}')
            final = workers[0].evaluate(mu, case)
            if final > best[0]:
                best = (final, mu.copy(), trial)
    return best


def write_policy(path: Path, mu, case, reward, trial):
    from mubot_control.gait import GaitPolicy  # same vector ordering
    p = GaitPolicy.from_vector(mu)
    doc = {'case': case['name'], 'trained': datetime.now().isoformat(timespec='seconds'),
           'trial': int(trial), 'reward_mps': float(reward), 'gamma': [float(x) for x in mu],
           'amplitude': p.amplitude, 'phase': p.phase, 'frequency': p.frequency,
           'stiffness': case['stiffness'], 'tail_ratio': case['tail_ratio']}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case', default='AN4_Kmed_HM3_AR2')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', default='data/train')
    ap.add_argument('--seed', type=int, default=None)
    a = ap.parse_args(argv)
    case = load_case(a.case)
    from mubot_learning.worker import GzWorker
    workers = [GzWorker(i, case['world'], case['model']) for i in range(a.workers)]
    try:
        out = Path(a.out) / case['name'] / datetime.now().strftime('%Y%m%d_%H%M%S')
        reward, mu, trial = train(case, workers, out, a.seed)
        write_policy(out / f"gamma_star_{case['name']}.yaml", mu, case, reward, trial)
    finally:
        for w in workers:
            w.close()
