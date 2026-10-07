"""Hall sector widths from steady-speed captures.

At constant speed each hall state lasts in proportion to its electrical
width, so the time share of each state over many revolutions measures the
sensor placement. Captures in both directions cancel the hysteresis (it
shifts every edge the same way). Writes the widths in SEQUENCE order as the
hall_w0..hall_w5 params (radians) for `mmc-host apply --profile`.

    python tools/hall_widths.py <capture.csv>... [--json hall_widths.json] [--min-speed 150]

Use captures with the `hall`, `state` and `t` channels at a steady speed
(hall FOC, e.g. testresults/motor3-18v/hall_ref/ref_w{±200,±400}.csv).
"""
import argparse
import csv
import json
import math

import numpy as np

SEQUENCE = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101]


def shares(path, min_speed):
    rows = list(csv.DictReader(open(path)))
    a = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    t, st, h = a["t"], a["state"], a["hall"].astype(int)
    run = np.flatnonzero(st == 1)
    if not len(run):
        return None
    te = t[run[-1]]
    m = (t > te - 3.0) & (t < te - 0.1) & (st == 1)
    if "omega_hall" in a and np.abs(a["omega_hall"][m]).mean() < min_speed:
        return None
    t, h = t[m], h[m]
    edges = np.flatnonzero(np.diff(h) != 0) + 1
    dur = {s: [] for s in SEQUENCE}
    for e0, e1 in zip(edges[:-1], edges[1:]):
        if h[e0] in dur:
            dur[h[e0]].append(t[e1] - t[e0])
    if any(len(v) < 3 for v in dur.values()):
        return None
    mean = np.array([np.mean(dur[s]) for s in SEQUENCE])
    return mean / mean.sum()


def main(files, out, min_speed):
    got = [(f, shares(f, min_speed)) for f in files]
    used = [(f, s) for f, s in got if s is not None]
    if not used:
        raise SystemExit("no capture had a steady stretch above --min-speed")
    for f, s in used:
        print(f"{f}: " + "  ".join(f"{SEQUENCE[k]:03b} {360 * s[k]:5.1f}°" for k in range(6)))
    avg = np.mean([s for _, s in used], axis=0)
    widths = 2 * math.pi * avg / avg.sum()
    print("widths: " + "  ".join(f"{SEQUENCE[k]:03b} {math.degrees(w):5.1f}°" for k, w in enumerate(widths)))
    if out:
        json.dump({f"hall_w{k}": float(w) for k, w in enumerate(widths)}, open(out, "w"), indent=1)
        print(f"wrote {out} (mmc-host apply --profile {out})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--json")
    ap.add_argument("--min-speed", type=float, default=150.0)
    a = ap.parse_args()
    main(a.captures, a.json, a.min_speed)
