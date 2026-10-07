"""Hall FOC ripple by electrical order across capture sets.

    python tools/hall_ripple.py <label>=<dir> ... [--speeds 200,400,800]
"""
import argparse
import csv
import os

import numpy as np


def stats(path):
    rows = list(csv.DictReader(open(path)))
    a = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    t, st = a["t"], a["state"]
    run = np.flatnonzero(st == 1)
    te = t[run[-1]]
    m = (t > te - 1.1) & (t < te - 0.1) & (st == 1)
    th = np.unwrap(a["theta_e"][m])
    iq = a["i_q"][m] - a["i_q"][m].mean()
    amp = lambda n: abs(2 * np.mean(iq * np.exp(-1j * n * th)))  # noqa: E731
    return dict(iq_std=iq.std(), h2=amp(2), h6=amp(6), w_std=a["omega_hall"][m].std(), w=a["omega_hall"][m].mean())


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("sets", nargs="+")
    ap.add_argument("--speeds", default="200,400,800")
    ap.add_argument("--prefix", default="ref_w")
    a = ap.parse_args()
    sets = [s.split("=", 1) for s in a.sets]
    print(f"{'speed':>6} {'set':14s} {'ω':>7} {'i_q σ mA':>9} {'2nd mA':>7} {'6th mA':>7} {'speed σ':>8}")
    for w in a.speeds.split(","):
        for lab, d in sets:
            f = os.path.join(d, f"{a.prefix}{w}.csv")
            if os.path.exists(f):
                s = stats(f)
                print(f"{w:>6} {lab:14s} {s['w']:7.1f} {1e3 * s['iq_std']:9.1f} {1e3 * s['h2']:7.1f} {1e3 * s['h6']:7.1f} {s['w_std']:8.1f}")
