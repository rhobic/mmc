"""Fit a motor profile from `mmc-host profile` captures (MS6).

Inputs (in the profile directory):
  rl_step.csv     locked-rotor d-axis voltage step, (i_d, v_d) at 20 kHz
                  -> R from the level change (differential: dead-time and
                     offsets cancel), L from the exponential time constant.
  sweep_*.csv     rotating I-f operating points -> flux psi via the
                  hang-angle-aware joint fit (imported from fit_params.py;
                  its L output is ill-conditioned and ignored here).
  accel.csv       sensorless run with a speed retarget -> friction from the
                  steady i_q levels, inertia J from the extra i_q during the
                  500 rad/s^2 (electrical) reference slew.

Output: profile.json in the same directory — the exact parameter set
`mmc-host apply` writes back to the device — plus a printed report.

Usage: python tools/profile.py [testresults/ms6-profile]
"""

import csv
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fit_params as fp  # noqa: E402  (hang-angle sweep fit lives there)

POLE_PAIRS = 7.0
OMEGA_SLEW = 500.0  # firmware reference slew [rad/s^2 electrical]
CUR_BW = 1000.0  # current-loop design bandwidth to write back [rad/s]
SPEED_BW = 40.0  # speed-loop design bandwidth [rad/s electrical]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def fit_rl(path):
    """Square-wave (i_d, v_d) record: τ = L/R is near the sample period, so
    fold every edge — the plateau-normalized settling fraction z_j averaged
    over ~60 edges resolves a sub-sample τ (the fit is on the slope of
    ln(z_j), so the fractional PWM latency only shifts the intercept)."""
    c = load(path)
    i, v = c["i_d"], c["v_d"]
    dt = float(c["t"][1] - c["t"][0])

    edges = np.flatnonzero(np.abs(np.diff(v)) > 1e-3)  # v changes at e+1
    if len(edges) < 8:
        raise SystemExit(f"rl_step: only {len(edges)} edges — not a square-wave record?")
    half = int(np.median(np.diff(edges)))

    z_acc, r_list, di_list = [], [], []
    for e in edges[1:-1]:
        if e - 4 < 0 or e + half >= len(i):
            continue
        pre_i = np.mean(i[e - 3 : e + 1])  # settled end of previous level
        post_i = np.mean(i[e + half - 4 : e + half])  # settled end of this one
        dv = v[e + 1] - v[e]
        di = post_i - pre_i
        if abs(di) < 1e-3:
            continue
        r_list.append(dv / di)
        di_list.append(abs(di))
        z_acc.append((post_i - i[e + 1 : e + half - 4]) / (post_i - pre_i))

    r = float(np.mean(r_list))
    zbar = np.mean(np.array(z_acc), axis=0)
    k = np.flatnonzero((zbar > 0.02) & (zbar < 0.9))
    if len(k) < 2:
        raise SystemExit("rl_step: settling fraction unusable even after folding")
    slope, _ = np.polyfit((k + 1) * dt, np.log(zbar[k]), 1)
    tau = -1.0 / slope
    l = r * tau
    return {
        "r": r,
        "l": l,
        "tau_us": tau * 1e6,
        "edges": len(r_list),
        "delta_i": float(np.mean(di_list)),
        "r_sigma": float(np.std(r_list)),
        "exp_points": len(k),
    }


def fit_accel(path, kt):
    c = load(path)
    st, w, iq, t = c["state"], c["omega_est"], c["i_q"], c["t"]
    run = st == 1.0
    w_lo, w_hi = 300.0, 900.0
    lo_steady = run & (np.abs(w - w_lo) < 15.0)
    hi_steady = run & (np.abs(w - w_hi) < 15.0)
    if not lo_steady.any() or not hi_steady.any():
        raise SystemExit(
            f"accel: no steady plateau at {w_lo:.0f} and/or {w_hi:.0f} rad/s — "
            "the sensorless run didn't reach its targets (startup failure?)."
        )
    # Last second of each steady plateau.
    lo_idx = np.flatnonzero(lo_steady)
    lo_idx = lo_idx[t[lo_idx] > t[lo_idx[-1]] - 1.0]
    hi_idx = np.flatnonzero(hi_steady)
    hi_idx = hi_idx[t[hi_idx] > t[hi_idx[-1]] - 1.0]
    iq_lo, iq_hi = float(np.mean(iq[lo_idx])), float(np.mean(iq[hi_idx]))

    # Slew segment: between the plateaus, after the retarget.
    seg = np.flatnonzero(run & (w > w_lo + 50) & (w < w_hi - 50) & (t > t[lo_idx[-1]]))
    if len(seg) < 20:
        raise SystemExit("accel: no usable slew segment found")
    iq_acc = float(np.mean(iq[seg]))
    iq_fric_mid = (iq_lo + iq_hi) / 2.0
    alpha_m = OMEGA_SLEW / POLE_PAIRS
    j = kt * (iq_acc - iq_fric_mid) / alpha_m
    return {
        "j": j,
        "iq_fric_lo": iq_lo,
        "iq_fric_hi": iq_hi,
        "iq_accel": iq_acc,
        "slew_samples": len(seg),
    }


def main(dir_):
    rl = fit_rl(os.path.join(dir_, "rl_step.csv"))
    print(f"R/L probe: R = {rl['r']:.3f} +- {rl['r_sigma']:.3f} ohm, "
          f"L = {rl['l'] * 1e3:.3f} mH (tau {rl['tau_us']:.0f} us, "
          f"{rl['edges']} edges folded, {rl['exp_points']} exp samples, "
          f"dI {rl['delta_i']:.3f} A)")

    sweep = sorted(glob.glob(os.path.join(dir_, "sweep_*.csv")))
    pts = fp.collect(sweep)
    good = [p for p in pts if p["wraps"] == 0 and p["terr_std"] < 0.3]
    if len(good) < 4:
        raise SystemExit(
            f"flux sweep: only {len(good)}/{len(pts)} points held sync — the motor "
            "slipped (check v_d ~ -w*psi for spin vs ~0 for stall). Rerun "
            "`mmc-host profile`; if it persists, raise the sweep currents."
        )
    theta, sig, _ = fp.fit(good)
    psi = float(theta[0])
    kt = 1.5 * POLE_PAIRS * psi
    print(f"flux sweep: psi = {psi * 1e3:.4f} +- {sig[0] * 1e3:.4f} mWb "
          f"-> kt = {kt * 1e3:.3f} mN*m/A ({len(good)}/{len(pts)} points)")

    acc = fit_accel(os.path.join(dir_, "accel.csv"), kt)
    t_fric = kt * (acc["iq_fric_lo"] + acc["iq_fric_hi"]) / 2.0
    print(f"accel: J = {acc['j'] * 1e6:.3f} uN*m*s^2, friction i_q "
          f"{acc['iq_fric_lo'] * 1e3:.0f}/{acc['iq_fric_hi'] * 1e3:.0f} mA "
          f"(T_fric ~ {t_fric * 1e6:.0f} uN*m), accel i_q {acc['iq_accel'] * 1e3:.0f} mA")

    speed_kp = acc["j"] * SPEED_BW / (kt * POLE_PAIRS)
    speed_ki = speed_kp * SPEED_BW / 4.0
    profile = {
        "r": round(rl["r"], 4),
        "l": rl["l"],
        "flux": psi,
        "cur_bw": CUR_BW,
        "speed_kp": speed_kp,
        "speed_ki": speed_ki,
        "fit": {
            "tau_us": rl["tau_us"],
            "psi_sigma": float(sig[0]),
            "kt": kt,
            "j": acc["j"],
            "t_fric": t_fric,
            "pole_pairs_assumed": POLE_PAIRS,
            "speed_bw": SPEED_BW,
        },
    }
    out = os.path.join(dir_, "profile.json")
    with open(out, "w") as f:
        json.dump(profile, f, indent=2)
    print()
    print(f"speed PI @ bw={SPEED_BW:.0f}: kp = {speed_kp:.6f}, ki = {speed_ki:.6f}")
    print(f"wrote {out}")
    print(f"apply: mmc-host apply --serial auto --baud 1000000 --profile {out}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "testresults/ms6-profile")
