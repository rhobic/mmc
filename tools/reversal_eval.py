"""Score live retargets of a sensorless drive (`a_to_b` captures from
tools/lowspeed_sweep.sh), with the halls as truth: the state sequence after
the retarget (6 = on HFI, 1 = on the flux observer, 2 overcurrent, 8 stall),
how long the rotor dwelt near zero, how long until it was within 10 % of the
new target, the peak |i_q|, and the angle error against the halls while
crossing (|ω| < 2·`--slow`).

    python tools/reversal_eval.py testresults/motor3-fullspeed/*_to_*.csv
"""
import argparse
import csv
import json
import math
import pathlib

import numpy as np

SEQUENCE = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101]
POS = {s: k for k, s in enumerate(SEQUENCE)}


def hall_speed(t, hall, win=0.05):
    """True speed [rad/s el] from counted hall steps over a sliding window."""
    pos = np.zeros(len(hall))
    for k in range(1, len(hall)):
        a, b = POS.get(int(hall[k - 1])), POS.get(int(hall[k]))
        pos[k] = pos[k - 1] + (0 if a is None or b is None else (b - a + 3) % 6 - 3)
    pos *= math.pi / 3
    return (np.interp(t + win / 2, t, pos) - np.interp(t - win / 2, t, pos)) / win


def main(a):
    print(f"{'file':28} {'a':>6} {'b':>6} {'states after retarget':24} {'end':>7} {'near0 s':>7} {'settle s':>8} {'|iq|max':>7} {'err@cross':>11}")
    for p in a.captures:
        r = list(csv.DictReader(open(p)))
        d = {k: np.array([float(x[k]) for x in r]) for k in ("t", "state", "hall", "i_q", "theta_err")}
        meta = json.load(open(pathlib.Path(p).with_suffix(".meta.json")))["params"]
        wa = float(meta["drive"].split("omega_e:")[1].strip(" })"))
        wb = float(meta["drive_step"].split("omega_e:")[1].strip(" })"))
        t, st = d["t"], d["state"].astype(int)
        t_step = 0.6 * meta["duration_s"]
        after = t >= t_step
        on = np.flatnonzero(st != 0)
        t_end = t[on[-1]] if len(on) else t[-1]
        seq = []
        for s in st[after & (t <= t_end)]:
            if not seq or seq[-1] != s:
                seq.append(int(s))
        w = hall_speed(t, d["hall"])
        live = after & (t < t_end - 0.1)
        near0 = float(np.count_nonzero(live & (np.abs(w) < a.slow)) * np.median(np.diff(t)))
        ok = np.flatnonzero(live & (np.abs(w - wb) < 0.1 * abs(wb)))
        settle = float(t[ok[0]] - t_step) if len(ok) else float("nan")
        end = float(np.mean(w[(t > t_end - 0.6) & (t < t_end - 0.1)]))
        cross = live & (np.abs(w) < 2 * a.slow)
        err = np.degrees(d["theta_err"][cross])
        e = f"{np.mean(err):+5.0f}+-{np.std(err):3.0f}" if len(err) else "-"
        print(f"{pathlib.Path(p).name:28} {wa:6.0f} {wb:6.0f} {str(seq):24} {end:7.0f} {near0:7.2f} {settle:8.2f} "
              f"{np.abs(d['i_q'][after]).max():7.2f} {e:>11}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--slow", type=float, default=20.0, help="'near zero' [rad/s el]")
    main(ap.parse_args())
