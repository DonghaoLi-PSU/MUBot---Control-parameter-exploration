#!/usr/bin/env python3
"""Plot mean and best reward per episode for every trial of a training run.

Usage: plot_training.py data/train/<case>/<run> [-o training.png]
Reads trial_*/reward.csv written by mubot_learning.trainer (same format as the
legacy script, so legacy result folders work too).
"""
import argparse
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('-o', '--out', default='training.png')
    a = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 4))
    for trial in sorted(Path(a.run_dir).glob('trial_*')):
        r = np.atleast_2d(np.loadtxt(trial / 'reward.csv', delimiter=','))
        ep = np.arange(1, len(r) + 1)
        line, = ax.plot(ep, r.mean(axis=1), label=f'{trial.name} mean')
        ax.plot(ep, r.max(axis=1), '--', color=line.get_color(), alpha=0.6)
    ax.set_xlabel('episode')
    ax.set_ylabel('forward speed [m/s]')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
