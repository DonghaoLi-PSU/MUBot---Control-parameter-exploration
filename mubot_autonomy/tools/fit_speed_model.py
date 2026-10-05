#!/usr/bin/env python3
"""Fit the gait speed model v = g(k, b, mode) from ground-truth runs.

Input CSV: mode,k,bias,speed_mps (one row per run; mode = forward|backward).
Output: speed_model.yaml for mubot_localization/config, averaging repeated runs
on a grid of k and |bias| values.
"""
import argparse
import csv
from collections import defaultdict

import yaml


def fit(rows):
    ks = sorted({float(r['k']) for r in rows})
    bs = sorted({abs(float(r['bias'])) for r in rows})
    acc = defaultdict(list)
    for r in rows:
        acc[(r['mode'], abs(float(r['bias'])), float(r['k']))].append(float(r['speed_mps']))
    speed = {}
    for mode in ('forward', 'backward'):
        speed[mode] = [[round(sum(acc[(mode, b, k)]) / len(acc[(mode, b, k)]), 5)
                        if acc[(mode, b, k)] else None for k in ks] for b in bs]
    missing = [(m, b, k) for m in speed for bi, b in enumerate(bs) for ki, k in enumerate(ks)
               if speed[m][bi][ki] is None]
    if missing:
        raise SystemExit(f'missing grid points (mode, |b|, k): {missing}')
    return {'amplitude_scale': ks, 'bias': bs, 'speed': speed}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs_csv')
    ap.add_argument('-o', '--out', default='speed_model.yaml')
    a = ap.parse_args()
    with open(a.runs_csv) as f:
        model = fit(list(csv.DictReader(f)))
    with open(a.out, 'w') as f:
        f.write('# Fitted by tools/fit_speed_model.py\n')
        yaml.safe_dump(model, f, sort_keys=False)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
