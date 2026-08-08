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


def drive_path_r(dir_):
    """Series resistance of the rig's drive path (shunt + driver FETs), which
    the probes measure on top of the winding. The host records it in the
    device snapshot (0 for the sim); for captures predating that, fall back
    to the G474+IHM16M1 figure unless the device was the sim. Keep the
    fallback in sync with mmc-host profile.rs R_DRIVE_PATH."""
    try:
        with open(os.path.join(dir_, "profile_state.json")) as f:
            dev = json.load(f)["device"]
        if "r_path" in dev:
            return float(dev["r_path"])
        if "sim" in str(dev.get("name", "")).lower():
            return 0.0
    except (OSError, KeyError, ValueError):
        pass
    return 0.85


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


def vdead_shape(i_d, i_th):
    """Per-amp shape of the bridge's error, seen on the d axis.

    A DC vector at theta = 0 puts i_a = i_d and i_b = i_c = -i_d/2 (the
    amplitude-invariant inverse Clarke), so each leg sits at a different point
    on its own sign curve. Clarke back to alpha:

        e_d = (2/3) * (f(i_d) + f(i_d/2)),   f(x) = clip(x/i_th, -1, 1)

    which bends twice — phase A saturates at i_d = i_th, phases B and C only
    at 2*i_th. Those two knees are what separate v_dead from i_th; a single
    saturating curve would leave them degenerate.
    """
    f = lambda x: np.clip(x / i_th, -1.0, 1.0)  # noqa: E731
    return (2.0 / 3.0) * (f(i_d) + f(i_d / 2.0))


def fit_vdead(points):
    """Fit v_d = R*i_d + v_dead*shape(i_d) over the DC ladder.

    Linear in (R, v_dead) once i_th is fixed, so scan i_th and solve the
    2-parameter least squares at each — no initial guess to get wrong, and
    the residual curve doubles as the confidence statement. (scipy is not a
    dependency of this repo; numpy is.)
    """
    i = np.array([p["i"] for p in points])
    v = np.array([p["v"] for p in points])
    if len(i) < 6:
        raise SystemExit(f"vdead: only {len(i)} points — need the full ladder")

    # i_th below the smallest measured current is unidentifiable (every point
    # saturated); above the largest, the shape is a straight line and merges
    # with R. Scan strictly inside.
    lo, hi = max(1e-3, 0.3 * i.min()), 2.0 * i.max()
    grid = np.geomspace(lo, hi, 400)
    best = None
    for i_th in grid:
        a = np.column_stack([i, vdead_shape(i, i_th)])
        sol, *_ = np.linalg.lstsq(a, v, rcond=None)
        resid = float(np.sum((a @ sol - v) ** 2))
        if best is None or resid < best[0]:
            best = (resid, i_th, float(sol[0]), float(sol[1]))
    resid, i_th, r, v_dead = best

    # Model-free cross-check: on any straight segment the INTERCEPT reads
    # v_dead without the shape function, so a disagreement means the shape is
    # wrong rather than just the numbers.
    #
    # Which segment is reachable is set by the hardware, not by preference.
    # Phase A saturates at i_d = i_th but phases B/C, carrying half the
    # current, only at 2*i_th — and the 1.5 A trip caps this ladder at about
    # 2*i_th on the bench motor, so the fully-saturated regime is normally out
    # of reach. Between the knees the curve is still a line, just a steeper
    # one:  v = [R + v_dead/(3*i_th)]*i + (2/3)*v_dead.
    line = None
    for name, sel, k_int, k_slope in (
        ("saturated", i > 2.0 * i_th, 4.0 / 3.0, 0.0),
        ("between knees", (i > 1.4 * i_th) & (i <= 2.0 * i_th), 2.0 / 3.0, 1.0),
    ):
        if sel.sum() >= 3:
            slope, intercept = np.polyfit(i[sel], v[sel], 1)
            line = {
                "regime": name,
                "v_dead": float(intercept) / k_int,
                "r_implied": float(slope) - k_slope * v_dead / (3.0 * i_th),
                "points": int(sel.sum()),
            }
            break

    # Thermal check: the ladder runs up then back down, so if the winding
    # warmed, the descending branch needs more volts for the same current.
    drift = None
    ups = [p for p in points if p["leg"] == "up"]
    downs = [p for p in points if p["leg"] == "down"]
    if ups and downs:
        pred = lambda pts: np.array(  # noqa: E731
            [r * p["i"] + v_dead * vdead_shape(p["i"], i_th) for p in pts]
        )
        du = float(np.mean(np.array([p["v"] for p in ups]) - pred(ups)))
        dd = float(np.mean(np.array([p["v"] for p in downs]) - pred(downs)))
        drift = {"up_resid": du, "down_resid": dd, "delta": dd - du}

    # With no dead time there is no knee, so i_th is unidentifiable and the
    # scan returns whatever fits the noise. Say so rather than reporting a
    # confident number for a parameter the data cannot constrain. The floor
    # is deliberately well under any plausible bridge (250 ns on a 40 kHz
    # 12 V leg is 120 mV) and above the fit's own residual.
    rms = float(np.sqrt(resid / len(i)))
    measurable = v_dead > max(3.0 * rms, 5e-3)

    return {
        "v_dead": v_dead,
        "i_thresh": float(i_th) if measurable else None,
        "measurable": bool(measurable),
        "r_from_ladder": r,
        "points": len(i),
        "rms_resid": rms,
        "i_range": [float(i.min()), float(i.max())],
        "line_check": line if measurable else None,
        "thermal": drift,
    }


def load_vdead(dir_):
    """One steady-state (i_d, v_d) point per ladder capture.

    The firmware slews open-loop voltage at 5 V/s and the electrical time
    constant is ~34 us, so everything after the slew is settled; take the
    last 40% of each record and let the remainder cover both.
    """
    points = []
    for path in sorted(glob.glob(os.path.join(dir_, "vdead_*.csv"))):
        c = load(path)
        t = c["t"]
        keep = t >= t[0] + 0.6 * (t[-1] - t[0])
        if keep.sum() < 20:
            continue
        leg = "down" if "_down_" in os.path.basename(path) else "up"
        points.append(
            {
                "leg": leg,
                "v": float(np.mean(c["v_d"][keep])),
                "i": float(np.mean(c["i_d"][keep])),
                "i_sd": float(np.std(c["i_d"][keep])),
                "file": os.path.basename(path),
            }
        )
    return points


def fit_accel(path, kt, pole_pairs, targets=(300.0, 900.0), omega_slew=OMEGA_SLEW):
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
    alpha_m = omega_slew / pole_pairs
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

    vd_points = load_vdead(dir_)
    if vd_points:
        vd = fit_vdead(vd_points)
        span = (f"{vd['points']} DC points over {vd['i_range'][0]:.2f}-"
                f"{vd['i_range'][1]:.2f} A, rms resid {vd['rms_resid'] * 1e3:.1f} mV")
        if vd["measurable"]:
            print(f"dead time: v_dead = {vd['v_dead'] * 1e3:.1f} mV, "
                  f"i_thresh = {vd['i_thresh']:.3f} A ({span})")
        else:
            print(f"dead time: none measurable (v_dead = {vd['v_dead'] * 1e3:.1f} mV, "
                  f"below the noise floor; i_thresh unidentifiable) ({span})")
        # The ladder's own R is an independent read of the same quantity the
        # R/L probe fits. The probe differences across folded edges where the
        # current sign never changes, so it CANNOT see v_dead; the ladder can.
        # Agreement on R is therefore the check that the extra term is real
        # and not R being mis-assigned.
        print(f"           ladder R = {vd['r_from_ladder']:.3f} ohm", end="")
        if "r" in profile:
            d = 100.0 * (vd["r_from_ladder"] - profile["r"]) / profile["r"]
            print(f" vs probe R {profile['r']:.3f} ({d:+.1f}%)")
        else:
            print(" (no R/L probe to compare against)")
        if vd["line_check"]:
            lc = vd["line_check"]
            print(f"           model-free cross-check, {lc['regime']} "
                  f"({lc['points']} pts): v_dead = {lc['v_dead'] * 1e3:.1f} mV, "
                  f"R = {lc['r_implied']:.3f} ohm")
        if vd["thermal"]:
            th = vd["thermal"]
            warn = "  <-- winding warmed; rerun cooler" if abs(th["delta"]) > 0.01 else ""
            print(f"           thermal: down-branch residual "
                  f"{th['delta'] * 1e3:+.1f} mV vs up{warn}")
        # Device params 13/14. Only written when the fit found something:
        # `apply` skips absent keys, so an unmeasurable bridge leaves the
        # device's compensation off rather than pushing a zero over a value
        # someone measured properly earlier.
        if vd["measurable"]:
            profile["v_dead"] = vd["v_dead"]
            profile["i_thresh"] = vd["i_thresh"]
        fit_info["vdead"] = vd
    else:
        print("dead time: no vdead_*.csv — skipped "
              "(capture: mmc-host profile --only vdead)")

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
        omega_slew = OMEGA_SLEW
        try:
            with open(os.path.join(dir_, "profile_state.json")) as f:
                state = json.load(f)
            targets = tuple(state["stages"]["accel"]["targets"])
            # The retarget ran at the device's omega_accel param, not the old
            # firmware constant — J scales directly with it.
            omega_slew = float(state["device"]["params"].get("omega_accel", OMEGA_SLEW))
        except (OSError, KeyError, ValueError):
            pass
        acc = fit_accel(accel_path, kt, pole_pairs, targets, omega_slew)
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

    # Summary of what `apply` will push to the device (units for reading).
    disp = {
        "r": ("R", "ohm", 1.0),
        "l": ("L", "mH", 1e3),
        "flux": ("flux", "mWb", 1e3),
        "cur_bw": ("current BW", "rad/s", 1.0),
        "speed_kp": ("speed Kp", "", 1.0),
        "speed_ki": ("speed Ki", "", 1.0),
        "v_dead": ("dead-time V", "mV", 1e3),
        "i_thresh": ("zero-I band", "A", 1.0),
    }
    print()
    print(f"Profile (pole pairs = {pole_pairs:.0f}) -- applied by `apply`:")
    for key, (label, unit, scale) in disp.items():
        if key in profile:
            print(f"  {label:11}= {profile[key] * scale:.4g} {unit}".rstrip())
    if "r" in profile:
        print("  (R is the drive-path value the control loop sees -- winding plus")
        print("   shunt and driver FETs -- so it reads several times a datasheet's")
        print("   winding-only figure. That is correct for control.)")

    # At-the-motor equivalents: what a multimeter/LCR at the terminals or a
    # datasheet quotes (wye: line-line = 2x per-phase). Display only — the
    # control params above keep the drive-path values.
    r_path = drive_path_r(dir_)
    if "r" in profile or "l" in profile:
        print()
        print("At the motor (est., wye winding) -- for meter/datasheet comparison:")
        if "r" in profile:
            r_m = profile["r"] - r_path
            if r_m > 0.005:
                print(f"  R ~ {2 * r_m:.3f} ohm line-line ({r_m:.3f} ohm/phase)"
                      f"   [drive-path {r_path:.2f} ohm subtracted]")
            else:
                print(f"  R ~ n/a (fit {profile['r']:.3f} <= drive-path {r_path:.2f} ohm)")
        if "l" in profile:
            print(f"  L ~ {2 * profile['l'] * 1e3:.3f} mH line-line "
                  f"({profile['l'] * 1e3:.3f} mH/phase)")
    print()
    print(f"wrote {out}")
    print(f"apply: mmc-host apply --serial auto --baud 1000000 --profile {out}")
    if os.path.exists(os.path.join(dir_, "saliency.csv")):
        print(f"saliency: python tools/saliency.py {dir_}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "testresults/ms6-profile")
