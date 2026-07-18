"""Fit the saliency (Ld/Lq) sweep captured by `mmc-host profile --only saliency`.

Physics: with the machine at standstill, square-wave d-axis voltage along an
electrical angle theta_x, rotor at theta_r, delta = theta_x - theta_r. At DC
the current settles to V/R along the applied axis at EVERY angle (R is
isotropic), so the cross-axis current i_q is a null channel: its post-edge
transient exists only if Ld != Lq. In 1/L the saliency is a clean second
harmonic: 1/L(delta) = G0 + G1*cos(2*delta).

With A = exp(-t/tau_d), B = exp(-t/tau_q), P = (A+B)/2, Q = (A-B)/2, the
folded post-edge samples obey (normalized by the plateau step dI):

    d-axis deficit:  X_b = P + Q*cos(2*delta_b)
    cross-axis:      Y_b =     Q*sin(2*delta_b)

Estimator (exact for pure exponentials, and the unknown effective sample
latency t cancels — the same systematic that limits absolute L to +-30%
does not touch it):

    u  = atanh(Q/P) = -t*R*G1
    xi = -u / (ln cosh(u) - ln P) = G1/G0
    Lq/Ld = (1 + xi)/(1 - xi)

The schedule (angles, levels, timing) is reconstructed from the header the
firmware wrote into the burst (saved in saliency.meta.json), mirroring
`mmc-core/src/probe.rs` — the one place the two must agree.

Output: the measured saliency ratio Lq/Ld (and xi = (Lq-Ld)/(Lq+Ld)) with its
uncertainty, plus the R/L/pole-pairs used for the run. It is a measurement,
not a recommendation. A re-run with the rotor re-clamped ~45 deg el away is the
check that the signal tracks the rotor (real saliency) rather than the stator
(a gain artifact); the two runs' xi should agree.

Usage: python tools/saliency.py [testresults/ms6-profile]
"""

import csv
import json
import math
import os
import sys

import numpy as np

SIGNIF_SIGMA = 3.0  # xi is called significant above this many sigma


def used_params(dir_):
    """The device parameter table snapshotted into profile_state.json when the
    stages ran — the R/L/pole-pairs the firmware and fits actually used. Empty
    dict for captures predating the snapshot."""
    try:
        with open(os.path.join(dir_, "profile_state.json")) as f:
            return json.load(f)["device"]["params"]
    except (OSError, KeyError, ValueError):
        return {}


def j_offsets(half_ticks):
    """Post-edge sample offsets for the fit, scaled with the half-period
    (the firmware picks half ≈ 8·τ, so these land mid-decay where P ∈
    ~(0.2, 0.7) — early samples have no transient developed yet, late ones
    none left, and both amplify noise through the small ln P or atanh
    denominators)."""
    js = {max(1, round(half_ticks / 16)), max(1, round(half_ticks / 8)),
          max(2, round(3 * half_ticks / 16))}
    return tuple(sorted(j for j in js if j < half_ticks - 4))


def load(dir_):
    with open(os.path.join(dir_, "saliency.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    with open(os.path.join(dir_, "saliency.meta.json")) as f:
        hdr = json.load(f)["header"]
    return cols, hdr


def fold_blocks(c, hdr, js):
    """Per block: signed angle delta, plateau step dI, and folded post-edge
    responses X(j) (d-deficit) / Y(j) (cross-axis) normalized by dI.
    Rising-minus-falling folding kills DC offsets and any i_q pedestal."""
    bt = int(hdr["block_ticks"])
    ht = int(hdr["half_ticks"])
    i_d, i_q, delta = c["i_d"], c["i_q"], c["delta_e"]
    n_blocks = len(i_d) // bt
    out = []
    for b in range(n_blocks):
        t0 = b * bt
        edges = []  # (edge tick, +1 rising / -1 falling)
        for h in range(1, bt // ht):
            edges.append((t0 + h * ht, +1 if h % 2 == 1 else -1))
        di, xs, ys = [], {j: [] for j in js}, {j: [] for j in js}
        for e, sign in edges:
            pre = np.mean(i_d[e - 3 : e])
            post = np.mean(i_d[e + ht - 3 : e + ht])
            step = post - pre
            if abs(step) < 1e-4:
                continue
            di.append(abs(step))
            for j in js:
                xs[j].append((post - i_d[e + j]) / step)
                ys[j].append(sign * i_q[e + j])
        if not di:
            continue
        d_i = float(np.mean(di))
        out.append(
            {
                "delta": float(delta[t0]),
                "cycle": b // int(hdr["slots"]),
                "di": d_i,
                # X: deficit is already normalized per-edge; Y: fold then normalize.
                "X": {j: float(np.mean(xs[j])) for j in js},
                "Y": {j: float(np.mean(ys[j])) / d_i for j in js},
            }
        )
    return out


def fit_harmonic(blocks, j):
    """Joint linear LSQ over both channels: X = P + Qc*cos2d + Qs*sin2d,
    Y = Qc*sin2d - Qs*cos2d. Returns P, Q (signed), theta_r, rms residual."""
    rows, rhs = [], []
    for blk in blocks:
        c2, s2 = math.cos(2 * blk["delta"]), math.sin(2 * blk["delta"])
        rows.append([1.0, c2, s2])
        rhs.append(blk["X"][j])
        rows.append([0.0, s2, -c2])
        rhs.append(blk["Y"][j])
    a = np.array(rows)
    y = np.array(rhs)
    sol, *_ = np.linalg.lstsq(a, y, rcond=None)
    p, qc, qs = (float(v) for v in sol)
    theta_r = 0.5 * math.atan2(qs, qc)
    q = qc * math.cos(2 * theta_r) + qs * math.sin(2 * theta_r)
    # Fold theta_r into (-45, 45] deg el; outside it the fitted axis is the
    # other one and the sign of Q flips with it.
    if theta_r > math.pi / 4:
        theta_r -= math.pi / 2
        q = -q
    elif theta_r <= -math.pi / 4:
        theta_r += math.pi / 2
        q = -q
    resid = float(np.sqrt(np.mean((a @ sol - y) ** 2)))
    return p, q, theta_r, resid


def xi_from(p, q):
    """Exact latency-cancelling estimator; None when out of domain."""
    if not (0.0 < p < 1.0) or abs(q) >= p:
        return None
    u = math.atanh(q / p)
    denom = math.log(math.cosh(u)) - math.log(p)
    return -u / denom if denom > 0 else None


def main(dir_):
    c, hdr = load(dir_)
    js = j_offsets(int(hdr["half_ticks"]))
    blocks = fold_blocks(c, hdr, js)
    slots = int(hdr["slots"])
    cycles = int(hdr["cycles"])
    if len(blocks) < slots:
        raise SystemExit(
            f"saliency: only {len(blocks)} usable blocks — aborted capture? "
            "Rerun `mmc-host profile --only saliency --redo`."
        )
    dv = hdr["v_high"] - hdr["v_low"]
    invalid = []  # hard reasons the measurement (not the motor) is bad

    # R comes free: plateau step over the known voltage step.
    di = np.array([b["di"] for b in blocks])
    r = dv / float(np.mean(di))
    di_spread = float(np.std(di) / np.mean(di))
    print(
        f"saliency: {len(blocks)} blocks, half={int(hdr['half_ticks'])} ticks, "
        f"dI = {np.mean(di) * 1e3:.0f} mA (spread {di_spread * 100:.1f}%) "
        f"-> R = {r:.3f} ohm"
    )
    if di_spread > 0.05:
        invalid.append(
            f"plateau step varies {di_spread * 100:.0f}% with angle — the square "
            "wave is not settling (stale R/L gave the firmware a too-short "
            "half-period?) or the rotor moved"
        )
    elif di_spread > 0.02:
        print(
            "WARNING: plateau step varies with angle — R is isotropic, so this "
            "means rotor movement or a data problem; treat the result with care."
        )

    # Fit per post-edge offset j (latency cancels; agreement is a model check)
    # and per even/odd cycle (disagreement means the rotor moved/drifted).
    xis, thetas = [], []
    for j in js:
        p, q, theta_r, resid = fit_harmonic(blocks, j)
        xi = xi_from(p, q)
        if xi is None:
            print(f"  j={j}: P={p:+.3f} Q={q:+.4f} — out of model domain, skipped")
            continue
        xis.append(xi)
        thetas.append(theta_r)
        print(
            f"  j={j}: P={p:.3f} Q={q:+.4f} theta_r={math.degrees(theta_r):+.1f} deg el "
            f"xi={xi:+.4f} (resid {resid:.4f})"
        )
    if not xis:
        invalid.append(
            "no post-edge sample fits the settling model (P must be in (0,1) "
            "with |Q| < P)"
        )
    if cycles >= 2:
        for parity, name in ((0, "even"), (1, "odd")):
            sub = [b for b in blocks if b["cycle"] % 2 == parity]
            p, q, theta_r, _ = fit_harmonic(sub, js[len(js) // 2])
            xi = xi_from(p, q)
            print(
                f"  {name} cycles: theta_r={math.degrees(theta_r):+.1f} deg el "
                f"xi={'n/a' if xi is None else f'{xi:+.4f}'}"
            )
            if xi is not None:
                xis.append(xi)
                thetas.append(theta_r)
    else:
        print("  (single cycle at this half-period — even/odd drift check skipped)")

    if invalid:
        print()
        for reason in invalid:
            print(f"  bad: {reason}")
        raise SystemExit(
            "\nSaliency not measured: the capture is unusable (see above), which\n"
            "reflects the measurement, not the motor. Profile `rl` first, Apply so\n"
            "the firmware's R/L (and its half-period pick) match this motor, keep\n"
            "the shaft still, then `--only saliency --redo`."
        )
    xi = float(np.mean(xis[: len(js)]))
    sigma = float(np.std(xis)) if len(xis) > 1 else abs(xi)
    theta_spread = math.degrees(max(thetas) - min(thetas)) if len(thetas) > 1 else 0.0
    ratio = (1 + abs(xi)) / (1 - abs(xi))
    snr = abs(xi) / sigma if sigma > 0 else float("inf")

    # --- Saliency measurement (a number, not a recommendation) --------------
    print()
    print("Saliency (measured):")
    print(f"  Lq/Ld     = {ratio:.3f}   ({(ratio - 1) * 100:+.1f}% saliency)")
    print(f"  xi        = {abs(xi):.4f} +- {sigma:.4f}   "
          f"({snr:.0f} sigma, "
          f"{'significant' if abs(xi) > SIGNIF_SIGMA * sigma else 'within noise'})")
    print(f"  rotor axis theta_r spread across splits: {theta_spread:.1f} deg el")
    print("  note: |xi| only; the axis is ambiguous by 90 deg el for a clamped")
    print("        rotor at an unknown angle, so the sign is not meaningful.")
    if theta_spread > 15.0:
        print("  note: theta_r drifted >15 deg between splits; the rotor likely")
        print("        moved -- clamp the shaft and re-run for a cleaner number.")

    # --- Key values used in this profiling run ------------------------------
    dp = used_params(dir_)
    print()
    print("Values used this run:")
    print(f"  R          = {r:.3f} ohm   (from this saliency plateau)")
    if "r" in dp:
        print(f"  R (device) = {float(dp['r']):.3f} ohm   (param the firmware used)")
    if "l" in dp:
        print(f"  L          = {float(dp['l']) * 1e3:.3f} mH   "
              "(RL probe; sets the sweep's half-period)")
    if "flux" in dp:
        print(f"  flux       = {float(dp['flux']) * 1e3:.3f} mWb")
    if "pole_pairs" in dp:
        print(f"  pole pairs = {float(dp['pole_pairs']):.0f}")
    print(f"  half-period= {int(hdr['half_ticks'])} ticks   "
          f"({int(hdr['slots'])} angles, {int(hdr['cycles'])} cycle(s), "
          f"{hdr['v_low']:.2f}->{hdr['v_high']:.2f} V)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "testresults/ms6-profile")
