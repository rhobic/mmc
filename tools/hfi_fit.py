"""Rotor angle at standstill from an HFI sweep (`mmc-host hfi`).

The device demodulated the current answering a ±V_h carrier along 24 test
angles θx (`mmc_core::hfi`). Along the test axis the response is ∝ 1/L(θx);
with saliency it carries a 2θ term,

    a_d(θx) = A0 + A2·cos 2(θx − θ2)          (+ A1·cos(θx − θ1) if saturated)

so the rotor's axis is θ2 (mod π) — or θ2 + π/2 if the d axis is the more
inductive one. Across the test axis the response is the null channel
a_q(θx) ∝ sin 2(θx − θ2): nothing on a round rotor. The 1θ term, if any, is
saturation along the magnet: its phase picks the magnet's north out of the
two candidates the 2θ term leaves.

Per sweep this prints the saliency ratio A2/A0 (≈ (Lq−Ld)/(Lq+Ld)), the
fitted axis, the 1θ term, and the error against the align angle and the
hall angle the device recorded. Over sweeps at many align angles the error
should be a constant offset (the convention) with a small spread.

    python tools/hfi_fit.py testresults/hfi/sweep.csv [--ref align|hall] [--json out.json]
"""
import argparse
import csv
import json
import math

import numpy as np


def wrap(x, period=2 * math.pi):
    return (x + period / 2) % period - period / 2


def fit(theta, y):
    A = np.c_[np.ones_like(theta), np.cos(theta), np.sin(theta), np.cos(2 * theta), np.sin(2 * theta)]
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ c
    return c, float(np.sqrt(np.mean(resid ** 2)))


REF = "align"


def main(path, out):
    rows = list(csv.DictReader(open(path)))
    sweeps = {}
    for r in rows:
        sweeps.setdefault(int(r["sweep"]), []).append(r)
    res = []
    for s, rs in sorted(sweeps.items()):
        th = np.radians([float(r["theta_deg"]) for r in rs])
        ad = np.array([float(r["ad"]) for r in rs])
        aq = np.array([float(r["aq"]) for r in rs])
        sgn = np.sign(ad.mean()) or 1.0  # carrier/latency sign
        ad, aq = ad * sgn, aq * sgn
        c, rms = fit(th, ad)
        cq, rmsq = fit(th, aq)
        a0 = c[0]
        a1, th1 = math.hypot(c[1], c[2]), math.atan2(c[2], c[1])
        a2, th2 = math.hypot(c[3], c[4]), 0.5 * math.atan2(c[4], c[3])
        q2 = math.hypot(cq[3], cq[4])
        align = float(rs[0]["align_deg"])
        hall = float(rs[0]["hall_deg"])
        ref = hall if (REF == "hall" and not math.isnan(hall)) or math.isnan(align) else align
        e2 = math.degrees(wrap(th2 - math.radians(ref), math.pi)) if not math.isnan(ref) else float("nan")
        e1 = math.degrees(wrap(th1 - math.radians(ref))) if not math.isnan(ref) else float("nan")
        res.append({"sweep": s, "align_deg": align, "hall_deg": hall, "a0": a0, "xi": a2 / a0,
                    "axis_deg": math.degrees(th2) % 180, "err2_deg": e2, "a1_rel": a1 / a0,
                    "pol_deg": math.degrees(th1) % 360, "err1_deg": e1, "q2_rel": q2 / a0,
                    "noise_rel": rms / a0})
        print(f"sweep {s}: align {align:6.1f}°  hall {hall:6.1f}°  |  saliency A2/A0 {a2 / a0:+.4f} "
              f"(cross-axis {q2 / a0:.4f}, residual {rms / a0:.4f})  axis {math.degrees(th2) % 180:6.1f}° "
              f"(err {e2:+6.1f}°)  |  1θ {a1 / a0:.4f} at {math.degrees(th1) % 360:6.1f}° (err {e1:+6.1f}°)")
    e = np.array([r["err2_deg"] for r in res if not math.isnan(r["err2_deg"])])
    if len(e) > 1:
        # Circular mean/spread of the axis error (period 180°).
        z = np.exp(2j * np.radians(e))
        m = math.degrees(np.angle(z.mean())) / 2
        spread = math.degrees(np.sqrt(-2 * np.log(abs(z.mean())))) / 2
        print(f"axis error over {len(e)} sweeps: mean {m:+.1f}°, circular spread {spread:.1f}° (el)")
    if out:
        json.dump(res, open(out, "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--json")
    ap.add_argument("--ref", choices=["align", "hall"], default="align",
                    help="truth to score against: the align angle, or the hall angle the device read")
    a = ap.parse_args()
    REF = a.ref
    main(a.csv, a.json)
