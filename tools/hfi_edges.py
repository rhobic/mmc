"""Score the HFI tracker against the halls at hall edges (where the hall
angle is exact): `theta_err` holds the tracker's axis error (mod π) from the
last edge. Also its rms against the hall angle every frame (coarse between
edges at low speed).

    python tools/hfi_edges.py testresults/hfi/shadow_w*.csv
"""
import csv
import math
import sys

import numpy as np


def load(p):
    r = list(csv.DictReader(open(p)))
    return {k: np.array([float(x[k]) for x in r]) for k in r[0]}


def circ(e):
    z = np.exp(2j * e).mean()
    return math.degrees(np.angle(z) / 2), math.degrees(math.sqrt(max(0.0, -2 * math.log(max(abs(z), 1e-9)))) / 2)


for p in sys.argv[1:]:
    d = load(p)
    t, st, h = d["t"], d["state"], d["hall"].astype(int)
    run = np.flatnonzero(st == 1)
    if not len(run):
        print(f"{p}: never ran")
        continue
    m = (t > t[run[0]] + 1.5) & (st == 1)
    idx = np.flatnonzero(m)
    edges = [i for i in idx[1:] if h[i] != h[i - 1]]
    e = np.array([d["theta_err"][i] for i in edges])
    # Every frame against the hall angle, folded mod π.
    every = np.angle(np.exp(2j * (d["theta_est"][m] - d["theta_e"][m]))) / 2
    em, es = circ(e) if len(e) > 2 else (float("nan"), float("nan"))
    am, as_ = circ(every)
    print(f"{p}: {len(edges)} edges, edge error mean {em:+.1f}° spread {es:.1f}° (el, mod 180); "
          f"vs hall angle every frame mean {am:+.1f}° spread {as_:.1f}°; HFI speed {np.mean(d['omega_est'][m]):.0f} "
          f"(hall {np.mean(d['omega_hall'][m]):.0f}) rad/s el")
