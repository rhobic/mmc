"""Load torque against mechanical position, from hall FOC runs at steady speed.

Each hall state is a known slice of the turn (its calibrated width over the
pole pairs), so its duration gives the mean speed over that slice, and the
change in kinetic energy between neighbouring slices gives the net torque
there. Subtracting the motor's own torque (kt·i_q) leaves the external one:

    τ_ext(θ) = J·(ω²_{k+1} − ω²_k) / (2·Δθ) − kt·i_q

averaged over every revolution in the capture. A conservative torque
(cogging, magnetic detent, a spring) has the same τ_ext(θ) in both
directions; friction opposes the motion, so it flips sign with direction.
Runs at ±ω therefore split into

    conservative c(θ) = (τ⁺ + τ⁻)/2,    friction F(θ) = (τ⁻ − τ⁺)/2.

The halls give no index, so every run's turn is aligned to the first by
the pole pair (one of 7 shifts) that best matches its sector-duration
profile (or τ_ext for the reverse runs).

    python tools/drag_profile.py testresults/motor3-hunting/drag_w{100,-100,200,-200,400,-400}.csv \
        [--widths testresults/motor3-hallcal/hall_widths.json] [--json out.json] [--harmonics 12]

Needs the `t`, `state`, `hall` and `i_q` channels at a frame rate well
above the hall edge rate.
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


def sectors(path, poles, widths, settle):
    """Per-slice (position index, duration, mean i_q) over the steady part
    of a run. Position index counts hall states around the turn, 0..6·poles."""
    d = load(path)
    t, st, h, iq = d["t"], d["state"], d["hall"].astype(int), d["i_q"]
    run = np.flatnonzero(st == 1)
    t0, te = t[run[0]], t[run[-1]]
    m = (t > t0 + settle) & (t < te - 0.1) & (st == 1)
    t, h, iq = t[m], h[m], iq[m]
    edges = np.flatnonzero(np.diff(h) != 0) + 1
    seq = {s: k for k, s in enumerate(SEQUENCE)}
    out, pos = [], None
    for e0, e1 in zip(edges[:-1], edges[1:]):
        s = int(h[e0])
        if s not in seq:
            continue
        k = seq[s]
        if pos is None:
            pos = k
        else:
            step = (k - pos) % 6
            if step == 1:
                pos += 1
            elif step == 5:
                pos -= 1
            else:  # missed or bounced state: lose track of this slice
                pos += {0: 0, 2: 2, 3: 3, 4: -2}[step]
        out.append((pos % (6 * poles), t[e1] - t[e0], iq[e0:e1].mean()))
    return np.array(out)


def profile(sec, poles, widths, J, kt):
    """τ_ext at each slice boundary (index b = boundary between slice b-1
    and b), mean speed per slice and the number of revolutions used."""
    n = 6 * poles
    wm = np.array([widths[int(p) % 6] for p in sec[:, 0]]) / poles  # mech rad
    omega = wm / sec[:, 1]
    dirn = 1.0 if np.mean(np.diff(sec[:, 0]) % n == 1) > 0.5 else -1.0
    # Signed mechanical speed: positive when the index climbs.
    w = omega * (1 if dirn > 0 else -1)
    tau = np.full(n, np.nan)
    acc = [[] for _ in range(n)]
    for k in range(len(sec) - 1):
        a, b = int(sec[k, 0]), int(sec[k + 1, 0])
        if (b - a) % n not in (1, n - 1):
            continue
        dth = 0.5 * (wm[k] + wm[k + 1]) * (1 if (b - a) % n == 1 else -1)
        net = J * (w[k + 1] ** 2 - w[k] ** 2) / (2 * dth)
        motor = kt * 0.5 * (sec[k, 2] + sec[k + 1, 2])
        boundary = b if (b - a) % n == 1 else a
        acc[boundary].append(net - motor)
    tau = np.array([np.mean(x) if x else np.nan for x in acc])
    speed = np.array([np.mean(omega[sec[:, 0] == p]) if np.any(sec[:, 0] == p) else np.nan for p in range(n)])
    dur = np.array([np.mean(sec[sec[:, 0] == p, 1]) if np.any(sec[:, 0] == p) else np.nan for p in range(n)])
    return tau, speed, dur, len(sec) / n, dirn


def align(ref, x):
    """Pole-pair shift (multiple of 6 slices) that best matches x to ref."""
    best = None
    for s in range(0, len(ref), 6):
        y = np.roll(x, s)
        ok = np.isfinite(ref) & np.isfinite(y)
        c = np.corrcoef(ref[ok], y[ok])[0, 1]
        if best is None or c > best[1]:
            best = (s, c)
    return best


def harmonics(tau, poles, widths, nmax):
    """Fourier series of τ(θ_m) on the slice boundaries (uneven spacing)."""
    edges = np.concatenate([[0], np.cumsum([widths[k % 6] for k in range(6 * poles)])])[:-1] / poles
    ok = np.isfinite(tau)
    th, y = edges[ok], tau[ok]
    cols = [np.ones_like(th)]
    for n in range(1, nmax + 1):
        cols += [np.cos(n * th), np.sin(n * th)]
    A = np.array(cols).T
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    fit = A @ c
    # τ = amp·cos(nθ + phase) = amp·sin(nθ + phase + π/2); `phase_sin` is
    # the drive's convention (params cog_p0…3, mmc_core::cogging).
    # Each slice's speed is a mean over the slice and τ a difference
    # between neighbours: two boxcars one slice wide, which shrink order n
    # by sinc²(πn/N). Undone here (checked against a simulated rotor with a
    # known torque).
    N = 6 * poles
    out = []
    for n in range(1, nmax + 1):
        ph = math.atan2(-c[2 * n], c[2 * n - 1])
        x = math.pi * n / N
        att = (math.sin(x) / x) ** 2
        out.append({"order": n, "amp": float(math.hypot(c[2 * n - 1], c[2 * n]) / att), "seen": float(math.hypot(c[2 * n - 1], c[2 * n])), "phase": float(ph),
                    "phase_sin": float((ph + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi)})
    return float(c[0]), out, float(np.sqrt(np.mean((y - fit) ** 2)))


def main(a):
    widths = [math.pi / 3] * 6
    if a.widths:
        widths = list(json.load(open(a.widths)).values())
    runs = []
    for f in a.captures:
        sec = sectors(f, a.poles, widths, a.settle)
        tau, speed, dur, revs, dirn = profile(sec, a.poles, widths, a.inertia, a.kt)
        runs.append({"file": f, "tau": tau, "speed": speed, "dur": dur, "revs": revs, "dir": dirn})
    # Align: same-direction runs by τ against the first of their
    # direction, then the reverse group to the forward one.
    ref = {}
    for r in runs:
        key = r["dir"]
        if key not in ref:
            ref[key] = r
            r["shift"], r["match"] = 0, 1.0
            continue
        s, c = align(ref[key]["tau"], r["tau"])
        r["shift"], r["match"] = s, c
    fwd = [r for r in runs if r["dir"] > 0]
    rev = [r for r in runs if r["dir"] < 0]
    if fwd and rev:
        tf = np.nanmean([np.roll(r["tau"], r["shift"]) for r in fwd], axis=0)
        tr = np.nanmean([np.roll(r["tau"], r["shift"]) for r in rev], axis=0)
        s, c = align(tf, tr)
        for r in rev:
            r["shift"] = (r["shift"] + s) % len(tf)
        print(f"reverse runs aligned to forward by {s // 6} pole pairs (τ correlation {c:.2f})")
    n = 6 * a.poles
    for r in runs:
        r["tau_al"] = np.roll(r["tau"], r["shift"])
        print(f"{r['file']}: {r['revs']:.0f} revs, dir {r['dir']:+.0f}, shift {r['shift'] // 6} pp (match {r['match']:.2f}); "
              f"τ_ext mean {1e3 * np.nanmean(r['tau']):+.2f} mN·m, sd {1e3 * np.nanstd(r['tau']):.2f}")
    result = {"poles": a.poles, "inertia": a.inertia, "kt": a.kt, "runs": []}
    for r in runs:
        result["runs"].append({"file": r["file"], "dir": r["dir"], "revs": r["revs"],
                               "tau": [None if not np.isfinite(x) else float(x) for x in r["tau_al"]]})
    if fwd and rev:
        tf = np.nanmean([r["tau_al"] for r in fwd], axis=0)
        tr = np.nanmean([r["tau_al"] for r in rev], axis=0)
        c = 0.5 * (tf + tr)
        F = 0.5 * (tr - tf)
        c0, ch, cres = harmonics(c, a.poles, widths, a.harmonics)
        F0, Fh, Fres = harmonics(F, a.poles, widths, a.harmonics)
        print(f"friction: mean {1e3 * F0:.2f} mN·m, ripple sd {1e3 * np.nanstd(F):.2f}; "
              f"conservative: mean {1e3 * c0:+.2f} (should be ~0), sd {1e3 * np.nanstd(c):.2f} mN·m")
        print("  order  conservative [mN·m]   friction [mN·m]")
        for x, y in zip(ch, Fh):
            print(f"  {x['order']:5d}  {1e3 * x['amp']:6.2f} @ {math.degrees(x['phase']):+5.0f}°     "
                  f"{1e3 * y['amp']:6.2f} @ {math.degrees(y['phase']):+5.0f}°")
        print(f"  residual beyond order {a.harmonics}: conservative {1e3 * cres:.2f}, friction {1e3 * Fres:.2f} mN·m")
        # Same-speed pairs, to see whether the split depends on speed.
        for r in fwd:
            for q in rev:
                if abs(abs(np.nanmean(r["speed"])) - abs(np.nanmean(q["speed"]))) < 0.2 * abs(np.nanmean(r["speed"])):
                    cc = 0.5 * (r["tau_al"] + q["tau_al"])
                    ff = 0.5 * (q["tau_al"] - r["tau_al"])
                    print(f"  pair {np.nanmean(r['speed']) * a.poles:5.0f} rad/s el: friction {1e3 * np.nanmean(ff):.2f} mN·m, "
                          f"conservative sd {1e3 * np.nanstd(cc):.2f}, friction sd {1e3 * np.nanstd(ff):.2f}")
        result.update({"conservative": {"mean": c0, "harmonics": ch, "residual": cres,
                                        "tau": [None if not np.isfinite(x) else float(x) for x in c]},
                       "friction": {"mean": F0, "harmonics": Fh, "residual": Fres,
                                    "tau": [None if not np.isfinite(x) else float(x) for x in F]}})
    if a.json:
        json.dump(result, open(a.json, "w"), indent=1)
        print(f"wrote {a.json}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--widths")
    ap.add_argument("--poles", type=int, default=7)
    ap.add_argument("--inertia", type=float, default=4.4e-6, help="rotor inertia [kg·m²]")
    ap.add_argument("--kt", type=float, default=1.5 * 7 * 6.645e-3, help="torque constant [N·m/A]")
    ap.add_argument("--settle", type=float, default=1.0, help="skip this long after the drive starts [s]")
    ap.add_argument("--harmonics", type=int, default=12)
    ap.add_argument("--json")
    main(ap.parse_args())
