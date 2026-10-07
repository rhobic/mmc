"""Flux linkage from a coast, out of the drive's own telemetry (no scope).

Spin up, cut the stage (`mmc-host capture ... --step-kind off`) and record
the terminal-sense channels with the halls. With every switch open the
terminals carry the back-EMF on the board's float bias, so wherever no
terminal is clamped (all above the divider's floor), the spread of the
three about their mean is the EMF amplitude whatever the angle:

    Σ_k (v_k − v̄)² = 1.5·E²,   E = ψ·ω_e

That needs all three unclamped, which at speed only happens near the end
of the coast. The main estimate uses any two unclamped terminals instead:
a clamp lifts every terminal alike, so their difference is the line EMF,

    v_j − v_k = ψ·ω_e·[cos(θ − j·2π/3 + δ) − cos(θ − k·2π/3 + δ)]

linear in (ψ·cos δ, ψ·sin δ), δ being the hall angle's unknown offset.
The angle and speed come from the hall edges: their angles (calibrated
sector widths) against time, fitted with a smooth polynomial over the
coast, so the edge-timed speed's lag while decelerating does not enter.

    python tools/coast_flux.py testresults/motor3-bemf/coast_1000_telem.csv \
        [--widths testresults/motor3-hallcal/hall_widths.json] [--floor 0.15]

Cross-check on motor 3: the scope fit of a coasting terminal
(tools/bemf_fit.py) gives 6.618 mWb.
"""
import argparse
import csv
import json
import math

import numpy as np

SEQUENCE = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101]


def load(path):
    rows = list(csv.DictReader(open(path)))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def fit(path, widths=None, floor=0.15, window=0.045, out=print):
    """ψ from one coast capture; `widths` in SEQUENCE order [rad el]."""
    widths = widths or [math.pi / 3] * 6
    d = load(path)
    t, st, h = d["t"], d["state"], d["hall"].astype(int)
    run = np.flatnonzero(st == 1)
    t_off = t[run[-1]] + 1e-3
    m = t > t_off
    t, h = t[m], h[m]
    V = np.c_[d["vb_u"][m], d["vb_v"][m], d["vb_w"][m]]
    # Hall edges -> electrical angle at each edge (edge into SEQUENCE[k]
    # sits at the cumulative width up to k).
    seq = {s: k for k, s in enumerate(SEQUENCE)}
    bound = np.concatenate([[0.0], np.cumsum(widths)])
    edges = np.flatnonzero(np.diff(h) != 0) + 1
    te, th, turn, last = [], [], 0, None
    for e in edges:
        s = int(h[e])
        if s not in seq:
            continue
        k = seq[s]
        if last is not None and (k - last) % 6 == 1 and k == 0:
            turn += 1
        if last is not None and (k - last) % 6 != 1:
            break  # reversal or missed state: stop at the first one
        # Edge time: halfway between the last frame before and this one.
        te.append(0.5 * (t[e - 1] + t[e]))
        th.append(2 * math.pi * turn + bound[k])
        last = k
    te, th = np.array(te), np.array(th)
    if len(te) < 12:
        raise SystemExit(f"only {len(te)} hall edges in the coast")
    span = te < te[0] + window
    c = np.polyfit(te[span] - te[0], th[span], 3)
    dc = np.polyder(c)
    t_end = te[span][-1]
    resid = th[span] - np.polyval(c, te[span] - te[0])
    inside = (t >= te[0]) & (t <= t_end)
    tt, VV = t[inside] - te[0], V[inside]
    w_all = np.polyval(dc, tt)
    th_all = np.polyval(c, tt)
    G = 2 * math.pi / 3
    rows, ys = [], []
    for j, k in ((0, 1), (1, 2), (2, 0)):
        ok = (VV[:, j] > floor) & (VV[:, k] > floor)
        cj, ck = np.cos(th_all[ok] - j * G), np.cos(th_all[ok] - k * G)
        sj, sk = np.sin(th_all[ok] - j * G), np.sin(th_all[ok] - k * G)
        rows.append(np.c_[w_all[ok] * (cj - ck), -w_all[ok] * (sj - sk)])
        ys.append(VV[ok, j] - VV[ok, k])
    A, y = np.vstack(rows), np.concatenate(ys)
    (pc, ps), *_ = np.linalg.lstsq(A, y, rcond=None)
    psi = float(math.hypot(pc, ps))
    rms = float(np.sqrt(np.mean((y - A @ [pc, ps]) ** 2)))
    out(f"coast {te[0]:.3f}–{t_end:.3f} s: {span.sum()} hall edges, angle fit rms {math.degrees(resid.std()):.1f}° el, "
          f"ω {w_all.max():.0f} → {w_all.min():.0f} rad/s el")
    out(f"line EMF, {len(y)} terminal pairs: ψ = {1e3 * psi:.3f} mWb (hall offset {math.degrees(math.atan2(ps, pc)):+.1f}° el, "
          f"residual {1e3 * rms:.0f} mV)")
    use = inside & (V.min(1) > floor)
    if use.sum() >= 3:
        w = np.polyval(dc, t[use] - te[0])
        dv = V[use] - V[use].mean(1, keepdims=True)
        E = np.sqrt((dv ** 2).sum(1) / 1.5)
        out(f"all three unclamped, {use.sum()} samples: ψ = {1e3 * float(np.dot(w, E) / np.dot(w, w)):.3f} mWb; "
              f"float bias (terminal mean) {np.median(V[use].mean(1)):.2f} V")
    return {"psi": psi, "pairs": int(len(y)), "residual_V": rms,
            "omega_range": [float(w_all.min()), float(w_all.max())], "source": path}


def main(a):
    widths = list(json.load(open(a.widths)).values()) if a.widths else None
    r = fit(a.capture, widths, a.floor, a.window)
    if a.json:
        json.dump(r, open(a.json, "w"), indent=1)
        print(f"wrote {a.json}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--widths")
    ap.add_argument("--floor", type=float, default=0.15, help="lowest terminal reading taken as unclamped [V]")
    ap.add_argument("--window", type=float, default=0.045, help="coast stretch to use [s] (motor 3 falls 1000 → 400 rad/s el in ~45 ms)")
    ap.add_argument("--json")
    main(ap.parse_args())
