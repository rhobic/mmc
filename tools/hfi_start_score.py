"""Score batches of HFI sensorless starts (tools/hfi_start_batch.sh): per
batch, overcurrent trips, polarity right (estimate within ±60° of the hall
sector centre, read at rest after the pulses, as tools/hfi_start_eval.py),
rotor motion during the pulses (hall edges) and the peak |i_d| there.

    python tools/hfi_start_score.py testresults/motor3-hfi-start p25x3 p6x8 p4x12
"""
import argparse
import csv
import glob
import json
import math
import os

import numpy as np

from hfi_start_eval import centres

LOCK_S = 0.3


def main(a):
    widths = list(json.load(open(a.widths)).values())
    cen = centres(a.offset, widths)
    print(f"{'batch':8} {'starts':>6} {'OC':>3} {'pol ok':>7} {'reached':>8} {'edges med/max':>14} {'|id| med/max':>13}")
    for prof in a.profiles:
        cfg = json.load(open(os.path.join(a.dir, prof + ".json")))
        pulses = 2 * cfg.get("hfi_pol_n", 8) * cfg.get("hfi_pol_s", 0.006)
        files = sorted(glob.glob(os.path.join(a.dir, f"{prof}_s*.csv")))
        oc = pol = reached = 0
        edges, idpk = [], []
        for f in files:
            r = list(csv.DictReader(open(f)))
            d = {k: np.array([float(x[k]) for x in r]) for k in ("t", "state", "hall", "theta_est", "i_d", "omega_hall")}
            t, st = d["t"], d["state"].astype(int)
            t0 = t[np.flatnonzero(st != 0)[0]]
            win = (t > t0 + LOCK_S) & (t < t0 + LOCK_S + pulses)
            edges.append(int(np.count_nonzero(np.diff(d["hall"][win]))))
            idpk.append(float(np.abs(d["i_d"][win]).max()))
            if 2 in st:
                oc += 1
                continue
            k = (t > t0 + LOCK_S + pulses + 0.02) & (t < t0 + LOCK_S + pulses + 0.08)
            hall_th = np.array([cen.get(int(h), np.nan) for h in d["hall"][k]])
            e = np.degrees(np.angle(np.exp(1j * (d["theta_est"][k] - hall_th))))
            pol += bool(len(e)) and abs(float(np.nanmedian(e))) < 60
            end = (t > t0 + 1.5) & (st == 6)
            reached += bool(end.any()) and float(d["omega_hall"][end].mean()) * math.copysign(1, a.target) > 0.5 * abs(a.target)
        print(f"{prof:8} {len(files):6} {oc:3} {pol:4}/{len(files) - oc:<2} {reached:5}/{len(files) - oc:<2}"
              f" {int(np.median(edges)):6}/{max(edges):<6} {np.median(idpk):6.2f}/{max(idpk):.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("profiles", nargs="+")
    ap.add_argument("--offset", type=float, default=-61.7, help="hall_offset [deg el]")
    ap.add_argument("--widths", default="testresults/motor3-hallcal/hall_widths.json")
    ap.add_argument("--target", type=float, default=20.0, help="start target [rad/s el]")
    main(ap.parse_args())
