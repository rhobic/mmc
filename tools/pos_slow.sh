#!/usr/bin/env bash
# Slow constant-speed moves in hall position mode: cruise at pos_vmax toward a
# far target; report the true average speed (hall edges are exact positions)
# and how evenly the edges arrive.
#   tools/pos_slow.sh <out-dir> <seconds> <vmax rad/s el> [vmax...]
set -e
out=$1; dur=$2; shift 2; mkdir -p "$out"
MASK=$(python -c "print(sum(1<<i for i in [0,2,14,22,24,25]))")
for v in "$@"; do
  echo "{\"pos_vmax\": $v}" > "$out/vmax_$v.json"
  timeout 60 ./target/release/mmc-host apply --serial COM9 --profile "$out/vmax_$v.json" > /dev/null
  f="$out/slow_v$v.csv"
  timeout $((dur + 30)) ./target/release/mmc-host capture --serial COM9 --baud 1000000 --duration "$dur" --divider 2 \
    --mask $MASK --drive hall-pos --amp 1.0 --hz=3600 --out "$f" --title "Hall position slow move at $v rad/s el" > /dev/null
  python - "$f" "$v" <<'PY'
import csv, sys
import numpy as np
r = list(csv.DictReader(open(sys.argv[1])))
a = lambda n: np.array([float(x[n]) for x in r])  # noqa: E731
v = float(sys.argv[2])
t, st, h = a("t"), a("state"), a("hall").astype(int)
run = np.flatnonzero(st != 0)
t0 = t[run[0]]
# Skip the acceleration and the first edge or two.
m = (st != 0) & (t > t0 + max(1.0, 2 * v / 300.0))
tm, hm = t[m], h[m]
e = np.flatnonzero(np.diff(hm) != 0) + 1
seq = [1, 3, 2, 6, 4, 5]
steps = [((seq.index(hm[k]) - seq.index(hm[k - 1])) % 6) for k in e]
fwd = sum(1 for s in steps if s == 1); back = sum(1 for s in steps if s == 5)
dt_e = np.diff(tm[e]) if len(e) > 2 else np.array([np.nan])
net = fwd - back
span = tm[e[-1]] - tm[e[0]] if len(e) > 1 else np.nan
speed = (net - 1) * (np.pi / 3) / span if len(e) > 1 and span > 0 else 0.0
ref_err = np.degrees(a("pos_ref")[m] - a("pos_m")[m])
print(f"vmax {v:5.2f} rad/s el ({v / 7 * 60 / (2 * np.pi):6.3f} rpm): edges fwd {fwd:3d} back {back:3d}  "
      f"true avg {speed:6.3f} rad/s el ({100 * (speed / v - 1):+5.1f}%)  edge interval {np.nanmean(dt_e) * 1e3:7.1f} ms "
      f"± {100 * np.nanstd(dt_e) / np.nanmean(dt_e):4.0f}%  lag ref-pos {ref_err.mean():+5.2f} ± {ref_err.std():4.2f}° mech  "
      f"i_q {a('i_q')[m].mean():+.3f} ± {a('i_q')[m].std():.3f}")
PY
done
