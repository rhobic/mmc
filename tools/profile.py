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

POLE_PAIRS = 7.0  # fallback; the device snapshot overrides (pole_pairs_for)
OMEGA_SLEW = 500.0  # firmware reference slew [rad/s^2 electrical]
CUR_BW = 1000.0  # current-loop design bandwidth to write back [rad/s]
SPEED_BW = 40.0  # speed-loop design bandwidth [rad/s electrical]


def pole_pairs_for(dir_):
    """`mmc-host profile` snapshots the device parameter table into
    profile_state.json; pole_pairs there tracks the connected motor. Falls
    back to the module default for captures predating the snapshot."""
    try:
        with open(os.path.join(dir_, "profile_state.json")) as f:
            pp = float(json.load(f)["device"]["params"]["pole_pairs"])
            if pp >= 1:
                return pp
    except (OSError, KeyError, ValueError):
        pass
    print(f"note: no pole_pairs in profile_state.json — assuming {POLE_PAIRS:.0f}")
    return POLE_PAIRS


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


def fit_accel(path, kt, pole_pairs, targets=(300.0, 900.0)):
    c = load(path)
    st, w, iq, t = c["state"], c["omega_est"], c["i_q"], c["t"]
    run = st == 1.0
    w_lo, w_hi = targets
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
    alpha_m = OMEGA_SLEW / pole_pairs
    j = kt * (iq_acc - iq_fric_mid) / alpha_m
    return {
        "j": j,
        "iq_fric_lo": iq_lo,
        "iq_fric_hi": iq_hi,
        "iq_accel": iq_acc,
        "slew_samples": len(seg),
    }


def main(dir_):
    """Fit whatever stages have been captured (`mmc-host profile` tracks them
    in profile_state.json); missing stages are skipped with a pointer instead
    of aborting the whole fit. `apply` skips absent keys, so a partial
    profile.json is safe to push."""
    pole_pairs = pole_pairs_for(dir_)
    fp.POLE_PAIRS = pole_pairs  # collect() reconstructs omega_e from omega_m
    profile = {}
    fit_info = {"pole_pairs": pole_pairs}

    rl_path = os.path.join(dir_, "rl_step.csv")
    if os.path.exists(rl_path):
        rl = fit_rl(rl_path)
        print(f"R/L probe: R = {rl['r']:.3f} +- {rl['r_sigma']:.3f} ohm, "
              f"L = {rl['l'] * 1e3:.3f} mH (tau {rl['tau_us']:.0f} us, "
              f"{rl['edges']} edges folded, {rl['exp_points']} exp samples, "
              f"dI {rl['delta_i']:.3f} A)")
        profile["r"] = round(rl["r"], 4)
        profile["l"] = rl["l"]
        profile["cur_bw"] = CUR_BW
        fit_info["tau_us"] = rl["tau_us"]
    else:
        print("R/L probe: rl_step.csv missing — skipped "
              "(capture: mmc-host profile --only rl)")

    psi = kt = None
    sweep = sorted(glob.glob(os.path.join(dir_, "sweep_*.csv")))
    if sweep:
        pts = fp.collect(sweep)
        good = [p for p in pts if p["wraps"] == 0 and p["terr_std"] < 0.3]
        if len(good) < 3:
            raise SystemExit(
                f"flux sweep: only {len(good)}/{len(pts)} points held sync — the motor "
                "slipped (check v_d ~ -w*psi for spin vs ~0 for stall). Rerun "
                "`mmc-host profile --only sweep --redo`; if it persists, raise the "
                "sweep currents (or lower the speeds — from-rest I-f only reaches "
                "what the ramp torque allows)."
            )
        if "r" in profile and "l" in profile:
            # R and L are measured (the probe conditions them far better than
            # the sweep can), so extract psi directly per point from the
            # back-EMF vector e = (v_d + w*L*i_q, v_q - R*i_q), |e| = w*psi.
            # The joint hang-angle fit below exists for when L is unknown,
            # and is ill-conditioned when the motor hangs near pi/2 (heavy
            # friction — exactly where big motors sit).
            psis = []
            for p in good:
                e_d = p["vd"] + p["w"] * profile["l"] * p["iq"]
                e_q = p["vq"] - profile["r"] * p["iq"]
                psis.append(math.hypot(e_d, e_q) / p["w"])
            psi = float(np.mean(psis))
            sig0 = float(np.std(psis))
            method = f"direct back-EMF, {len(good)}/{len(pts)} points"
        else:
            theta, sig, _ = fp.fit(good)
            psi = float(theta[0])
            sig0 = float(sig[0])
            method = f"hang-angle joint fit, {len(good)}/{len(pts)} points"
        kt = 1.5 * pole_pairs * psi
        print(f"flux sweep: psi = {psi * 1e3:.4f} +- {sig0 * 1e3:.4f} mWb "
              f"-> kt = {kt * 1e3:.3f} mN*m/A ({method})")
        profile["flux"] = psi
        fit_info["psi_sigma"] = sig0
        fit_info["kt"] = kt
    else:
        print("flux sweep: no sweep_*.csv — skipped "
              "(capture: mmc-host profile --only sweep)")

    accel_path = os.path.join(dir_, "accel.csv")
    if os.path.exists(accel_path) and kt is not None:
        targets = (300.0, 900.0)
        try:
            with open(os.path.join(dir_, "profile_state.json")) as f:
                targets = tuple(json.load(f)["stages"]["accel"]["targets"])
        except (OSError, KeyError, ValueError):
            pass
        acc = fit_accel(accel_path, kt, pole_pairs, targets)
        t_fric = kt * (acc["iq_fric_lo"] + acc["iq_fric_hi"]) / 2.0
        print(f"accel: J = {acc['j'] * 1e6:.3f} uN*m*s^2, friction i_q "
              f"{acc['iq_fric_lo'] * 1e3:.0f}/{acc['iq_fric_hi'] * 1e3:.0f} mA "
              f"(T_fric ~ {t_fric * 1e6:.0f} uN*m), accel i_q {acc['iq_accel'] * 1e3:.0f} mA")
        speed_kp = acc["j"] * SPEED_BW / (kt * pole_pairs)
        speed_ki = speed_kp * SPEED_BW / 4.0
        profile["speed_kp"] = speed_kp
        profile["speed_ki"] = speed_ki
        fit_info["j"] = acc["j"]
        fit_info["t_fric"] = t_fric
        fit_info["speed_bw"] = SPEED_BW
        print(f"speed PI @ bw={SPEED_BW:.0f}: kp = {speed_kp:.6f}, ki = {speed_ki:.6f}")
    elif os.path.exists(accel_path):
        print("accel: accel.csv present but kt unknown (needs the flux sweep) — skipped")
    else:
        print("accel: accel.csv missing — skipped "
              "(capture: mmc-host profile --only accel)")

    if not profile:
        raise SystemExit("nothing to fit — run `mmc-host profile` first (see --list)")
    profile["fit"] = fit_info
    out = os.path.join(dir_, "profile.json")
    with open(out, "w") as f:
        json.dump(profile, f, indent=2)
    print()
    print(f"wrote {out}")
    print(f"apply: mmc-host apply --serial auto --baud 1000000 --profile {out}")
    if os.path.exists(os.path.join(dir_, "saliency.csv")):
        print(f"saliency verdict: python tools/saliency.py {dir_}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "testresults/ms6-profile")
