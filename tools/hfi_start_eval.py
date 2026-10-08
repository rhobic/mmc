"""Score sensorless HFI starts against the halls (truth only; the drive does
not read them). The hall state's sector centre (calibrated offset, direction
and widths) is the rotor angle to ±half a sector — plenty to judge polarity
(right: the estimate within ±90° of it).

    python tools/hfi_start_eval.py testresults/hfi/start_p*.csv [--offset -61.7] [--widths hall_widths.json]
"""
import argparse
import csv
import json
import math

import numpy as np

SEQUENCE = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101]


def centres(offset_deg, widths):
    c, acc = {}, 0.0
    for k, s in enumerate(SEQUENCE):
        if k:
            acc += 0.5 * (widths[k - 1] + widths[k])
        c[s] = math.radians(offset_deg) + acc
    return c


def main(a):
    widths = list(json.load(open(a.widths)).values()) if a.widths else [math.pi / 3] * 6
    cen = centres(a.offset, widths)
    rows = []
    for p in a.captures:
        r = list(csv.DictReader(open(p)))
        d = {k: np.array([float(x[k]) for x in r]) for k in r[0]}
        t, st = d["t"], d["state"]
        on = np.flatnonzero(st != 0)
        t0 = t[on[0]]
        hall_th = np.array([cen.get(int(h), np.nan) for h in d["hall"]])
        k = np.flatnonzero((t > t0 + a.pol_t - 0.02) & (t < t0 + a.pol_t + 0.01))
        e = np.degrees(np.angle(np.exp(1j * (d["theta_est"][k] - hall_th[k]))))
        e_pol = float(np.nanmedian(e)) if len(k) else float("nan")
        w = d["omega_hall"]
        early = (t > t0 + 0.45) & (t < t0 + 1.5)
        end = (t > t[on[-1]] - 0.6) & (t < t[on[-1]] - 0.1)
        rows.append((p, e_pol, float(w[early].min()), float(w[end].mean())))
        print(f"{p}: estimate − hall sector after polarity {e_pol:+7.1f}°  -> polarity {'RIGHT' if abs(e_pol) < 90 else 'WRONG'};"
              f"  slowest early {w[early].min():+6.0f}, end {w[end].mean():+6.0f} rad/s el")
    right = sum(abs(x[1]) < 90 for x in rows)
    reached = sum(abs(x[3]) > 0 and x[3] > 0.8 * max(abs(r[3]) for r in rows) for x in rows)
    print(f"polarity right {right}/{len(rows)}; reached speed forward {reached}/{len(rows)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--offset", type=float, default=-61.7, help="hall_offset [deg el]")
    ap.add_argument("--widths", default="testresults/motor3-hallcal/hall_widths.json")
    ap.add_argument("--pol-t", type=float, default=0.45, help="time after the start the polarity test has ended [s]")
    main(ap.parse_args())
