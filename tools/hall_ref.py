"""Observer-vs-hall table from `tools/hall_ref_sweep.sh` captures.

Per steady speed: hall speed, observer speed, and the circular mean (and
spread) of `theta_err` = observer − hall angle. The fit splits that error into
an even part (constant offset), an odd constant (sign(ω)·h: a direction-
dependent hall switching lag) and an odd slope (a relative time delay).

    python tools/hall_ref.py <dir>
"""
import argparse
import csv
import glob
import json
import os

import numpy as np


def main(d, hyst_now=None, out=None):
    rows = []
    for f in glob.glob(os.path.join(d, "ref_w*.csv")):
        r = list(csv.DictReader(open(f)))
        a = lambda n: np.array([float(x[n]) for x in r])  # noqa: E731
        t, st = a("t"), a("state")
        m = (st == 1) & (t > t[-1] - 1.2)
        err = a("theta_err")[m]
        c = float(np.angle(np.mean(np.exp(1j * err))))
        s = float(np.std(np.angle(np.exp(1j * (err - c)))))
        rows.append((a("omega_hall")[m].mean(), a("omega_est")[m].mean(), c, s, a("i_q")[m].mean()))
    rows.sort()
    print("  omega_hall  omega_obs   speed err   obs-hall angle [rad]      i_q")
    for wh, we, c, s, iq in rows:
        print(f"  {wh:9.1f}  {we:9.1f}   {100 * (we - wh) / wh:+6.1f}%    {c:+.3f} ± {s:.3f}        {iq:+.3f}")
    use = [r for r in rows if abs(r[0]) >= 150]  # the observer's usable range
    w = np.array([r[0] for r in use])
    c = np.array([r[2] for r in use])
    A = np.c_[np.ones_like(w), np.sign(w), w]
    (k0, ks, kw), *_ = np.linalg.lstsq(A, c, rcond=None)
    res = c - A @ np.array([k0, ks, kw])
    print(f"fit (|w| >= 150): offset {k0:+.3f} rad, sign(w) step {ks:+.3f} rad, "
          f"delay {kw * 1e6:+.0f} us; rms resid {np.sqrt(np.mean(res ** 2)):.3f} rad")
    # The sign(w) step is the hall estimate lagging in the direction of
    # travel by the switching hysteresis it does not yet correct for.
    if hyst_now is not None:
        hyst = max(0.0, hyst_now + ks)
        print(f"hall_hyst: {hyst_now:.3f} applied during the sweep -> {hyst:.3f}")
        if out:
            json.dump({"hall_hyst": hyst}, open(out, "w"), indent=2)
            print(f"wrote {out} (mmc-host apply --profile {out})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--hyst-now", type=float, help="hall_hyst the device ran the sweep with")
    ap.add_argument("--json", help="write the corrected hall_hyst for `apply`")
    a = ap.parse_args()
    main(a.dir, a.hyst_now, a.json)
