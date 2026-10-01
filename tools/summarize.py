"""Quick text summary of a capture CSV: the state sequence, then per-window
means of the channels that matter for judging a drive run.

    python tools/summarize.py capture.csv [--windows 0.5:1.5,3.0:3.6] [--cols a,b]
"""

import argparse
import csv

import numpy as np

DEFAULT_COLS = ["state", "omega_hall", "omega_est", "omega_m", "iq_ref", "i_q", "i_d",
                "v_q", "theta_err", "duty_a", "vbus"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--windows", help="t0:t1 pairs relative to drive start, comma separated")
    ap.add_argument("--cols", help="comma-separated channel names")
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.csv)))
    col = lambda n: np.array([float(r[n]) for r in rows])  # noqa: E731
    t, st = col("t"), col("state")
    seq = [int(x) for i, x in enumerate(st) if i == 0 or x != st[i - 1]]
    run = np.flatnonzero(st != 0)
    t0 = t[run[0]] if len(run) else t[0]
    print(f"states: {seq}   (drive starts at t={t0:.3f}s, capture {t[-1]:.2f}s)")
    cols = a.cols.split(",") if a.cols else DEFAULT_COLS
    if a.windows:
        wins = [tuple(float(x) for x in w.split(":")) for w in a.windows.split(",")]
    else:
        span = t[-1] - t0
        wins = [(span * f, span * f + 0.1 * span) for f in (0.2, 0.45, 0.85)]
    print("window(s)      " + " ".join(f"{c:>11s}" for c in cols))
    for w0, w1 in wins:
        m = (t >= t0 + w0) & (t < t0 + w1)
        if not m.any():
            continue
        vals = []
        for c in cols:
            x = col(c)[m]
            vals.append(f"{x.mean():8.3f}±{x.std():<5.2g}"[:11].rjust(11))
        print(f"{w0:5.2f}-{w1:5.2f}    " + " ".join(vals))


if __name__ == "__main__":
    main()
