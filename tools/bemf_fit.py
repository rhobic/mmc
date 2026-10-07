"""Back-EMF shape from a coasting phase terminal (tools/bemf_scope.py).

With every switch open the star point floats at a bias V_b set by the board,
so each terminal is e_k − mean(e) + V_b; when that would put a terminal below
−V_f its low-side diode clamps it there and lifts the others with it (no
current flows: one diode alone has no return path). The rotor decelerates
while it coasts, so the angle is a cubic in time. Fitted: the angle
trajectory, ψ, the 5th and 7th flux harmonics (amplitude relative to the
fundamental, phase), V_b and V_f. Triplen harmonics cancel out of every
terminal and cannot be seen this way.

    python tools/bemf_fit.py testresults/motor3-bemf/coast_800.csv [--t0 0 --t1 0.035] [--json out.json]
"""
import argparse
import json
import math

import numpy as np
from scipy.optimize import least_squares

G = 2 * math.pi / 3


def flux_shape(x, h):
    """ψ_k / ψ and its derivative for electrical angle x: fundamental plus
    harmonics h = [(n, rel, phase)]."""
    f = np.cos(x)
    df = -np.sin(x)
    for n, rel, ph in h:
        f = f + rel * np.cos(n * x + ph)
        df = df - rel * n * np.sin(n * x + ph)
    return f, df


def terminal(t, p, orders):
    th0, w0, acc, jerk, psi, vb, vf = p[:7]
    h = [(n, p[7 + 2 * i], p[8 + 2 * i]) for i, n in enumerate(orders)]
    th = th0 + w0 * t + 0.5 * acc * t ** 2 + jerk * t ** 3 / 6
    w = w0 + acc * t + 0.5 * jerk * t ** 2
    e = np.array([w * psi * flux_shape(th - k * G, h)[1] for k in range(3)])
    u = e - e.mean(0) + vb
    low = u.min(0)
    u = u + np.where(low < -vf, -vf - low, 0.0)
    return u[0]


def main(path, t0, t1, out, orders):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t, v = d[:, 0], d[:, 1]
    m = (t >= t0) & (t <= t1)
    t, v = t[m] - t0, v[m]
    # Starting speed from the hump spacing: rising crossings of a smoothed
    # trace through its midpoint, with hysteresis so noise cannot add any.
    k = max(1, int(2e-4 / np.median(np.diff(t))))
    vs = np.convolve(v, np.ones(k) / k, "same")
    mid, band = 0.5 * (vs.max() + vs.min()), 0.15 * (vs.max() - vs.min())
    rises, high = [], vs[0] > mid
    for i, x in enumerate(vs):
        if not high and x > mid + band:
            rises.append(t[i])
            high = True
        elif high and x < mid - band:
            high = False
    period = np.diff(rises)[:2].mean() if len(rises) > 2 else 0.008
    w0 = 2 * math.pi / period
    lo = [-10, 50, -1e5, -1e8, 1e-3, -1.0, 0.1] + [-0.3, -10] * len(orders)
    hi = [10, 3000, 1e3, 1e8, 2e-2, 6.0, 1.5] + [0.3, 10] * len(orders)
    best = None
    for scale in (0.8, 1.0, 1.25):
        for th0 in np.linspace(-math.pi, math.pi, 8, endpoint=False):
            p0 = [th0, w0 * scale, -w0 * scale / max(t[-1], 1e-3) * 0.5, 0.0, 6.6e-3, 2.4, 0.6] + [0.01, 0.0] * len(orders)
            r = least_squares(lambda p: terminal(t, p, orders) - v, p0, bounds=(lo, hi), x_scale="jac", max_nfev=3000)
            if best is None or r.cost < best.cost:
                best = r
    p = best.x
    rms = math.sqrt(np.mean(best.fun ** 2))
    print(f"fit over {t[0] + t0:.4f}–{t[-1] + t0:.4f} s, {len(t)} points: rms residual {1e3 * rms:.1f} mV")
    print(f"  ω start {p[1]:.1f} rad/s el, decel {-p[2]:.0f} rad/s²; ψ {1e3 * p[4]:.3f} mWb; V_b {p[5]:.2f} V; V_f {p[6]:.2f} V")
    harm = []
    for i, n in enumerate(orders):
        rel, ph = p[7 + 2 * i], p[8 + 2 * i]
        if rel < 0:
            rel, ph = -rel, ph + math.pi
        ph = (ph + math.pi) % (2 * math.pi) - math.pi
        harm.append({"order": n, "rel": rel, "phase": ph})
        print(f"  harmonic {n}: {100 * rel:.2f} % of the fundamental flux, phase {math.degrees(ph):+.0f}° "
              f"(back-EMF {100 * n * rel:.1f} %)")
    if out:
        json.dump({"psi": p[4], "v_bias": p[5], "v_diode": p[6], "harmonics": harm, "rms_mV": 1e3 * rms,
                   "source": path, "window_s": [t0, t1]}, open(out, "w"), indent=1)
        print(f"wrote {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=0.035)
    ap.add_argument("--orders", default="5,7")
    ap.add_argument("--json")
    a = ap.parse_args()
    main(a.csv, a.t0, a.t1, a.json, [int(x) for x in a.orders.split(",")])
