"""Score low-speed sensorless captures against the halls (truth only).

Window: from `--settle` s after the drive reached its final target (the 60 %
retarget when the capture had one, else the drive start) to 0.2 s before the
drive stopped. True mean speed comes from counting hall steps (exact at any
speed, unlike the edge-timed `omega_hall`); its spread from `omega_hall`
over the window; the longest gap between hall edges flags a stall.

    python tools/lowspeed_eval.py testresults/motor3-hfi-vs-obs/*.csv
"""
import argparse
import csv
import json
import math
import pathlib

import numpy as np

SEQUENCE = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101]
POS = {s: k for k, s in enumerate(SEQUENCE)}


def load(p):
    r = list(csv.DictReader(open(p)))
    return {k: np.array([float(x[k]) for x in r]) for k in r[0]}


def score(p, settle):
    d = load(p)
    meta = json.load(open(pathlib.Path(p).with_suffix(".meta.json")))
    prm = meta["params"]
    t, st = d["t"], d["state"]
    on = np.flatnonzero(st != 0)
    dur = prm["duration_s"]
    stepped = prm.get("drive_step") is not None
    t_target = 0.6 * dur if stepped else t[on[0]]
    w = (t > t_target + settle) & (t < t[on[-1]] - 0.2)
    tw = t[w]
    # Target [rad/s el] from the title's last number is fragile; take it
    # from the drive strings instead.
    src = prm["drive_step"] if stepped else prm["drive"]
    target = float(src.split("omega_e:")[1].strip(" })"))
    # Signed hall steps.
    h = [POS.get(int(x)) for x in d["hall"][w]]
    steps, edges = 0, [tw[0]]
    for k in range(1, len(h)):
        if h[k] is None or h[k - 1] is None or h[k] == h[k - 1]:
            continue
        s = (h[k] - h[k - 1] + 3) % 6 - 3
        steps += s
        edges.append(tw[k])
    edges.append(tw[-1])
    span = tw[-1] - tw[0]
    w_true = steps * (math.pi / 3) / span
    gap = float(np.max(np.diff(edges)))
    wh = d["omega_hall"][w]
    err = np.degrees(d["theta_err"][w])
    iq = d["i_q"][w]
    return dict(
        file=pathlib.Path(p).name,
        target=target,
        w_true=w_true,
        w_hall_sd=float(np.std(wh)),
        w_est=float(np.mean(d["omega_est"][w])),
        err_mean=float(np.mean(err)),
        err_sd=float(np.std(err)),
        iq_rms=float(np.sqrt(np.mean(iq**2))),
        max_gap=gap,
        states=sorted({int(x) for x in st[w]}),
    )


def main(a):
    rows = [score(p, a.settle) for p in a.captures]
    print(f"{'file':28} {'target':>7} {'true':>7} {'sd_hall':>7} {'est':>7} {'th_err':>7} {'sd_th':>6} {'iq_rms':>7} {'gap s':>6}  states")
    for r in rows:
        print(f"{r['file']:28} {r['target']:7.0f} {r['w_true']:7.1f} {r['w_hall_sd']:7.1f} {r['w_est']:7.1f}"
              f" {r['err_mean']:+7.1f} {r['err_sd']:6.1f} {r['iq_rms']:7.3f} {r['max_gap']:6.2f}  {r['states']}")
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--settle", type=float, default=1.5, help="seconds after reaching the target before scoring")
    ap.add_argument("--json", help="also write the scores here")
    main(ap.parse_args())
