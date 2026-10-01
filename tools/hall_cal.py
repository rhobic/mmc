"""Hall-sensor calibration from two slow open-loop captures (fwd + rev).

In open-loop voltage mode at low speed the current sits on the d axis, so the
rotor's flux follows the forced angle `theta_e` with a small lag that flips
sign with direction. Each hall edge is logged at the forced angle where it
happened; averaging the forward and reverse angle of the *same physical edge*
cancels the lag (and the telemetry latency), leaving where the edge sits in
electrical angle. Half their difference is the lag itself.

Reports, per edge: its electrical angle, the spacing to the next edge (ideal
60°), and the lag; then the `mmc_core::hall::HallMap` (offset of SEQUENCE
step 0's center, direction) that a sensored drive needs.

    python tools/hall_cal.py fwd.csv rev.csv [--json out.json]
"""

import argparse
import csv
import json

import numpy as np

SEQUENCE = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101]


def load(path):
    rows = list(csv.DictReader(open(path)))
    col = lambda n: np.array([float(r[n]) for r in rows])  # noqa: E731
    st = col("state")
    run = st == 1
    t = col("t")
    # Skip the start-up transient (rotor pulling into lock).
    t0 = t[run][0] + 1.0
    m = run & (t > t0)
    return col("theta_e")[m], col("hall")[m].astype(int), col("omega_m")[m]


def edges(theta, hall):
    """{(from, to): [forced angle at the edge, ...]}"""
    out = {}
    k = np.flatnonzero(np.diff(hall) != 0) + 1
    for i in k:
        a, b = hall[i - 1], hall[i]
        if a in SEQUENCE and b in SEQUENCE:
            out.setdefault((a, b), []).append(theta[i])
    return out


def circ_mean(v):
    return float(np.angle(np.mean(np.exp(1j * np.asarray(v)))))


def circ_std(v, c):
    return float(np.std(np.angle(np.exp(1j * (np.asarray(v) - c)))))


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fwd")
    ap.add_argument("rev")
    ap.add_argument("--json")
    a = ap.parse_args()

    ef = edges(*load(a.fwd)[:2])
    er = edges(*load(a.rev)[:2])

    # Which way does SEQUENCE order run when theta_e increases?
    nf = sum(len(v) for (x, y), v in ef.items()
             if SEQUENCE.index(y) == (SEQUENCE.index(x) + 1) % 6)
    nb = sum(len(v) for v in ef.values()) - nf
    seq_dir = 1.0 if nf >= nb else -1.0
    print(f"forward capture: {nf} SEQUENCE-order edges, {nb} reverse -> "
          f"SEQUENCE order is {'+' if seq_dir > 0 else '-'} electrical rotation")

    # Physical edge k = the boundary between SEQUENCE[k] and SEQUENCE[k+1];
    # forward crosses it one way, reverse the other.
    rows = []
    for k in range(6):
        s0, s1 = SEQUENCE[k], SEQUENCE[(k + 1) % 6]
        f = ef.get((s0, s1) if seq_dir > 0 else (s1, s0), [])
        r = er.get((s1, s0) if seq_dir > 0 else (s0, s1), [])
        if not f or not r:
            print(f"edge {s0:03b}|{s1:03b}: missing (fwd {len(f)}, rev {len(r)})")
            rows.append(None)
            continue
        cf, cr = circ_mean(f), circ_mean(r)
        mid = wrap(cr + wrap(cf - cr) / 2)
        lag = wrap(cf - cr) / 2
        rows.append(dict(k=k, a=s0, b=s1, angle=mid, lag=lag, nf=len(f), nr=len(r),
                         sf=circ_std(f, cf), sr=circ_std(r, cr)))

    print("\nedge (SEQUENCE k|k+1)   angle    spacing->next   lag    jitter fwd/rev  (deg)")
    for i, e in enumerate(rows):
        if e is None:
            continue
        nxt = rows[(i + 1) % 6]
        sp = np.degrees(wrap(seq_dir * (nxt["angle"] - e["angle"]))) if nxt else float("nan")
        print(f"  {e['a']:03b}|{e['b']:03b} (k={e['k']})   {np.degrees(e['angle']):7.1f}   "
              f"{sp:7.1f}        {np.degrees(e['lag']):5.1f}   "
              f"{np.degrees(e['sf']):4.1f}/{np.degrees(e['sr']):4.1f}   n={e['nf']}/{e['nr']}")

    good = [e for e in rows if e]
    if len(good) == 6:
        # Sector k spans edge (k-1) -> edge k; its center is their midpoint.
        # HallMap.offset = center of SEQUENCE step 0, by the least-squares
        # fit of all six edges to an ideal 60° comb.
        ideal = np.array([e["angle"] - seq_dir * (e["k"] + 0.5) * np.pi / 3 for e in good])
        offset = circ_mean(ideal)
        resid = np.degrees(wrap(ideal - offset))
        spacing = [np.degrees(wrap(seq_dir * (good[(i + 1) % 6]["angle"] - good[i]["angle"])))
                   for i in range(6)]
        print(f"\nHallMap: offset = {np.degrees(offset):.1f} deg ({offset:.4f} rad), dir = {seq_dir:+.0f}")
        print(f"edge placement vs ideal 60 deg comb: max {np.max(np.abs(resid)):.1f} deg, "
              f"rms {np.sqrt(np.mean(resid ** 2)):.1f} deg; spacings {np.round(spacing, 1)}")
        mean_lag = np.degrees(np.mean([e["lag"] for e in good]))
        print(f"mean open-loop lag {mean_lag:.1f} deg (rotor behind the forced angle)")
        if a.json:
            # `hall_offset`/`hall_dir` are device param names, so
            # `mmc-host apply --profile` writes this file straight back.
            json.dump(dict(hall_offset=offset, hall_dir=seq_dir, edges=[
                dict(a=e["a"], b=e["b"], angle=e["angle"], lag=e["lag"]) for e in good],
                spacing_deg=spacing, resid_deg=list(resid)), open(a.json, "w"), indent=2)
            print(f"wrote {a.json}")


if __name__ == "__main__":
    main()
