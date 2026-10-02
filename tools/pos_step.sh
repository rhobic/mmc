#!/usr/bin/env bash
# Hall position step(s) and a summary: settle, overshoot, hold.
#   tools/pos_step.sh <out-dir> <deg> [deg...]
set -e
out=$1; shift; mkdir -p "$out"
MASK=$(python -c "print(sum(1<<i for i in [0,2,9,14,22,23,24,25]))")
for deg in "$@"; do
  f="$out/step_$deg.csv"
  timeout 60 ./target/release/mmc-host capture --serial COM9 --baud 1000000 --duration 5 --divider 4 \
    --mask $MASK --drive hall-pos --amp 1.0 --hz=$deg --out "$f" --title "Hall position step to $deg deg mech" > /dev/null
  python - "$f" "$deg" <<'PY'
import csv, sys
import numpy as np
r = list(csv.DictReader(open(sys.argv[1])))
a = lambda n: np.array([float(x[n]) for x in r])  # noqa: E731
tgt = float(sys.argv[2])
t, st = a("t"), a("state")
run = st != 0
t0 = t[run][0]
pos = np.degrees(a("pos_m"))
s = np.sign(tgt) if tgt else 1.0
m1 = run & (t < t0 + 1.5)
over = max(0.0, (s * (pos[m1] - tgt)).max())
inband = np.abs(pos - tgt) < 4.3
settle = next((t[k] - t0 for k in range(len(t)) if run[k] and t[k] > t0 and inband[k:][run[k:]].all()), float("nan"))
m = run & (t > t0 + 2.5)
print(f"{tgt:7.1f} deg: states {sorted(set(st.astype(int)))}  settle(±4.3°) {settle:5.2f} s  overshoot {over:5.2f}°  "
      f"hold {pos[m].mean():8.2f} ± {pos[m].std():5.2f}°  i_q {a('i_q')[m].mean():+.3f} ± {a('i_q')[m].std():.3f}  "
      f"hall states {sorted(set(a('hall')[m].astype(int)))}")
PY
done
