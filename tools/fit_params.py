"""Motor parameter ID from rotating I-f captures (MS5 Stage F0).

THE TRAP this script exists to document: in I-f the dq frame is *forced*,
not rotor-aligned. The rotor rides ahead of the forced frame by a hang
angle delta = pi/2 - gamma (gamma = rotor-flux -> current-vector angle,
set by load torque), so the rotor flux projects onto BOTH forced axes:

    v_d = c_d      - w * (L*i_q + psi*cos(gamma))
    v_q = R_q*i_q  + w *  psi*sin(gamma),   psi*sin(gamma) = T_f / (1.5*p*i_q)

A naive rotor-aligned fit reads psi*cos(gamma) as "w*L*i_q" and reports a
wildly wrong L (it gave 2.9 mH for this ~0.1 mH motor). Two data sets that
were each perfectly consistent with the naive model were split by:
  * a 0.2 A sync test (the low-psi interpretation required sin(gamma)>1
    -> pull-out; the motor held sync), and
  * the v_d slope at 0.45 A (matches the high-psi model within 5%).

So this fits the full model jointly across current levels: parameters
(psi, L, t_hat=T_f/(1.5p), R_q apparent, c_d offset) by Gauss-Newton on
per-(current,speed) steady-state means. c_d and R_q absorb dead-time
distortion (constant with speed at fixed current); slopes carry the physics.

CONDITIONING WARNING: with gamma near pi/2 (light load), L*i_q is a small
share of the v_d slope, so L comes out with wide error bars here — psi is
the trustworthy output. A locked-rotor or HF-injection probe (MS6) is the
right instrument for L.

Usage: python tools/fit_params.py  (globs testresults/ms5-g474-bringup/f_paramid_*.csv)
"""

import csv
import glob
import math
import sys

import numpy as np

POLE_PAIRS = 7.0
OMEGA_SLEW = 500.0  # firmware forced-frequency slew [rad/s^2 electrical]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def steady_window(c):
    """Drive on, frequency ramp finished, transients dead."""
    run = c["state"] == 1.0
    w_target = np.max(np.abs(c["omega_m"])) * POLE_PAIRS
    settled = run & (np.abs(c["omega_m"] * POLE_PAIRS) >= w_target - 1e-3)
    idx = np.flatnonzero(settled)
    keep = idx[len(idx) // 2 :]
    return keep, w_target * np.sign(np.mean(c["omega_m"][idx]))


def collect(paths):
    pts = []
    for p in sorted(paths):
        c = load(p)
        keep, w = steady_window(c)
        pts.append(
            {
                "path": p,
                "w": w,
                "vd": float(np.mean(c["v_d"][keep])),
                "vq": float(np.mean(c["v_q"][keep])),
                "iq": float(np.mean(c["i_q"][keep])),
                "terr": float(np.mean(c["theta_err"][keep])),
                "terr_std": float(np.std(c["theta_err"][keep])),
                "wraps": int(np.sum(np.abs(np.diff(c["theta_err"][keep])) > 3.0)),
                "cols": c,
            }
        )
        print(
            f"{p}: w={w:6.1f} iq={pts[-1]['iq']:.3f} vd={pts[-1]['vd']:+.4f} "
            f"vq={pts[-1]['vq']:+.4f} theta_err={pts[-1]['terr']:+.3f}"
            f"(std {pts[-1]['terr_std']:.3f}) wraps={pts[-1]['wraps']}"
        )
    return pts


def model(theta, w, iq):
    psi, ll, t_hat, r_q, c_d = theta
    s = t_hat / iq  # psi*sin(gamma)
    cosg = np.sqrt(np.maximum(psi**2 - s**2, 1e-12))
    vd = c_d - w * (ll * iq + cosg)
    vq = r_q * iq + w * s
    return vd, vq


def fit(pts):
    w = np.array([p["w"] for p in pts])
    iq = np.array([p["iq"] for p in pts])
    vd = np.array([p["vd"] for p in pts])
    vq = np.array([p["vq"] for p in pts])

    # The model is linear in (L, c_d, R_q) once (psi, t_hat) are fixed, so
    # nest linear least squares inside a refined 2-D grid — Gauss-Newton
    # diverges on the near-collinear psi/L directions, this can't.
    def solve_linear(psi, t_hat):
        s = t_hat / iq
        if np.any(s >= psi):
            return np.inf, (0.0, 0.0, 0.0)
        cosg = np.sqrt(psi**2 - s**2)
        a_d = np.column_stack([-w * iq, np.ones_like(w)])
        b_d = vd + w * cosg
        (ll, c_d), *_ = np.linalg.lstsq(a_d, b_d, rcond=None)
        r_d = b_d - a_d @ (ll, c_d)
        b_q = vq - w * s
        r_q = float(np.dot(iq, b_q) / np.dot(iq, iq))
        res_q = b_q - r_q * iq
        return float(r_d @ r_d + res_q @ res_q), (float(ll), float(c_d), r_q)

    lo = np.array([0.3e-3, 0.02e-3])
    hi = np.array([1.5e-3, 0.10e-3])
    best = None
    for _ in range(4):
        psis = np.linspace(lo[0], hi[0], 61)
        thats = np.linspace(lo[1], hi[1], 61)
        for psi in psis:
            for t_hat in thats:
                sse, lin = solve_linear(psi, t_hat)
                if best is None or sse < best[0]:
                    best = (sse, psi, t_hat, lin)
        span = (hi - lo) / 10.0
        center = np.array([best[1], best[2]])
        lo, hi = center - span, center + span
    sse, psi, t_hat, (ll, c_d, r_q) = best
    theta = np.array([psi, ll, t_hat, r_q, c_d])

    def residuals(th):
        mvd, mvq = model(th, w, iq)
        return np.concatenate([vd - mvd, vq - mvq])

    r = residuals(theta)
    # 1-sigma from the numeric Jacobian (pinv: psi/L near-collinear).
    scale = np.abs(theta) + 1e-6
    jac = np.empty((len(r), len(theta)))
    for j in range(len(theta)):
        d = np.zeros_like(theta)
        d[j] = 1e-6 * scale[j]
        jac[:, j] = (residuals(theta + d) - r) / d[j]
    dof = max(len(r) - len(theta), 1)
    cov = np.linalg.pinv(jac.T @ jac) * (r @ r) / dof
    sigma = np.sqrt(np.abs(np.diag(cov)))
    return theta, sigma, r


def main(paths):
    pts = collect(paths)
    good = [p for p in pts if p["wraps"] == 0 and p["terr_std"] < 0.3]
    skipped = [p for p in pts if p not in good]
    for p in skipped:
        print(f"  (excluded from fit: {p['path']} — oscillating/slipping)")

    theta, sig, r = fit(good)
    psi, ll, t_hat, r_q, c_d = theta
    kt = 1.5 * POLE_PAIRS * psi
    t_fric = 1.5 * POLE_PAIRS * t_hat

    print()
    print(f"flux psi   = {psi * 1e3:.4f} +- {sig[0] * 1e3:.4f} mWb")
    print(f"L          = {ll * 1e3:.3f} +- {sig[1] * 1e3:.3f} mH   (ill-conditioned here; see header)")
    print(f"T_friction = {t_fric * 1e3:.3f} +- {1.5 * POLE_PAIRS * sig[2] * 1e3:.3f} mN*m")
    print(f"R apparent = {r_q:.3f} +- {sig[3]:.3f} ohm (true R + deadtime)")
    print(f"kt         = {kt * 1e3:.3f} mN*m/A;  bemf = {psi * POLE_PAIRS * 1e3:.3f} mV/(rad/s mech)")
    print(f"fit residual max {np.max(np.abs(r)) * 1e3:.1f} mV over {len(r)} equations")

    for p in good:
        gamma = math.asin(min(t_hat / (psi * p["iq"]), 1.0))
        print(
            f"  w={p['w']:6.1f} iq={p['iq']:.2f}: gamma={gamma:+.3f} rad, "
            f"hang angle delta={math.pi / 2 - gamma:+.3f} rad "
            f"(pull-out margin {psi * p['iq'] / t_hat:.2f}x)"
        )

    # Inertia from the constant-accel ramp of the fastest clean 0.3 A run:
    # torque above friction during the ramp is J*alpha. gamma during the
    # ramp comes from the shadow observer (theta_err ~ rotor hang angle).
    fast = max((p for p in good if abs(p["iq"] - 0.3) < 0.05), key=lambda p: abs(p["w"]))
    c = fast["cols"]
    w_inst = c["omega_m"] * POLE_PAIRS
    ramping = (c["state"] == 1.0) & (w_inst > 0.35 * fast["w"]) & (w_inst < 0.9 * fast["w"])
    idx = np.flatnonzero(ramping)
    if len(idx) > 50:
        gamma_ramp = math.pi / 2 - c["theta_err"][idx]
        t_total = kt * c["i_q"][idx] * np.sin(gamma_ramp)
        alpha_m = OMEGA_SLEW / POLE_PAIRS
        j_est = np.mean(t_total - t_fric) / alpha_m
        print()
        print(f"J (from ramp) = {j_est * 1e6:.3f} uN*m*s^2 ({len(idx)} ramp samples)")
        bw = 40.0
        kp = j_est * bw / (kt * POLE_PAIRS)
        print(f"speed PI @ bw={bw:.0f} rad/s el: kp={kp:.6f} A/(rad/s), ki={kp * bw / 4:.6f}")


if __name__ == "__main__":
    args = sys.argv[1:] or glob.glob("testresults/ms5-g474-bringup/f_paramid_*.csv")
    main(args)
