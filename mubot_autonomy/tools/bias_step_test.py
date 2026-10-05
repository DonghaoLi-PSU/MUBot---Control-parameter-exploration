#!/usr/bin/env python3
"""Fit the bias → yaw-rate response from a step test and suggest PI gains.

Input CSV: t,bias,yaw_rate recorded while the bias steps from 0 to a constant.
Model: first order plus dead time, r(s)/b(s) = K e^{-θs} / (τs + 1).
Gains: SIMC rules for the inner yaw-rate PI of the heading cascade.
"""
import argparse
import csv


def fit_fopdt(t, b, r):
    i0 = next(i for i, v in enumerate(b) if abs(v) > 1e-9)
    t0, step = t[i0], b[i0]
    tail = [v for ti, v in zip(t, r) if ti > t[-1] - 0.2 * (t[-1] - t0)]
    r_ss = sum(tail) / len(tail)
    K = r_ss / step
    t10 = next(ti for ti, v in zip(t[i0:], r[i0:]) if abs(v) >= 0.1 * abs(r_ss))
    t63 = next(ti for ti, v in zip(t[i0:], r[i0:]) if abs(v) >= 0.632 * abs(r_ss))
    theta = max(t10 - t0 - 0.105 * (t63 - t10) / 0.527, 0.0)
    tau = max(t63 - t0 - theta, 1e-3)
    return K, tau, theta


def simc(K, tau, theta, tau_c=None):
    # Default closed-loop time constant = tau: moderate, robust to model error.
    # Smaller tau_c (down to theta) is faster but less tolerant of the fit.
    tau_c = tau if tau_c is None or tau_c <= 0 else max(tau_c, theta, 0.05)
    kp = tau / (K * (tau_c + theta))
    ti = min(tau, 4 * (tau_c + theta))
    return kp, kp / ti


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--tau-c', type=float, default=None, help='closed-loop time constant [s]')
    a = ap.parse_args()
    with open(a.csv) as f:
        rows = list(csv.DictReader(f))
    t = [float(x['t']) for x in rows]
    b = [float(x['bias']) for x in rows]
    r = [float(x['yaw_rate']) for x in rows]
    K, tau, theta = fit_fopdt(t, b, r)
    kp, ki = simc(K, tau, theta, a.tau_c)
    print(f'K = {K:.4f} (rad/s)/V, tau = {tau:.3f} s, dead time = {theta:.3f} s')
    print(f'suggested inner PI: kp = {kp:.3f} V/(rad/s), ki = {ki:.3f} V/rad')


if __name__ == '__main__':
    main()
