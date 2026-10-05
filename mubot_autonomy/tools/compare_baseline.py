#!/usr/bin/env python3
"""Compare gait speeds from the Gazebo Harmonic port against the ROS 1 baseline.

Usage: compare_baseline.py legacy/baseline/baseline_speeds.csv new_speeds.csv [--tol 0.05]
Both files: case,noa,K,hm,ar,gamma,speed_mps. Exit code 1 if any case differs
by more than the relative tolerance.
"""
import argparse
import csv
import sys


def load(path):
    with open(path) as f:
        return {r['case']: float(r['speed_mps']) for r in csv.DictReader(f)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('baseline')
    ap.add_argument('candidate')
    ap.add_argument('--tol', type=float, default=0.05)
    a = ap.parse_args()
    base, new = load(a.baseline), load(a.candidate)
    if not base:
        sys.exit('baseline is empty: record ROS 1 speeds first (legacy/README.md)')
    worst = 0.0
    for case, v0 in sorted(base.items()):
        if case not in new:
            print(f'{case:30s} missing in candidate')
            worst = float('inf')
            continue
        rel = abs(new[case] - v0) / max(abs(v0), 1e-9)
        worst = max(worst, rel)
        print(f'{case:30s} baseline {v0:.4f}  port {new[case]:.4f}  diff {100 * rel:5.1f} %')
    sys.exit(1 if worst > a.tol else 0)


if __name__ == '__main__':
    main()
