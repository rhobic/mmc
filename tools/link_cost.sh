#!/usr/bin/env bash
# Per-frame cost of the host link at one telemetry rate, drive off: CPU
# shares and cycles per frame for snapshot+encode and for the whole link
# (Shared::encode_cycles / link_cycles), read twice 3 s apart over OpenOCD
# without halting.
#   tools/link_cost.sh <openocd target cfg> <&isr_sum_cycles> <divider> [mask]
set -euo pipefail
cfg=$1; base=$2; div=$3; mask=${4:-}
OO=${OPENOCD:-openocd}
(target/release/mmc-host.exe capture --serial COM9 --baud 1000000 --divider "$div" ${mask:+--mask $mask} \
    --duration 10 --out target/link_cost.csv > /dev/null 2>&1 &)
python -c "import time; time.sleep(4)"
"$OO" ${OPENOCD_SCRIPTS:+-s "$OPENOCD_SCRIPTS"} -f interface/stlink.cfg -c "transport select swd" -f "$cfg" \
    -c init -c "mdw 0xE0001004" -c "mdw $base 4" -c "sleep 3000" -c "mdw 0xE0001004" -c "mdw $base 4" -c shutdown 2>&1 \
  | grep "^0x" > target/link_cost.raw
python -c "import time; time.sleep(4)"
python - <<'PY'
import csv
w = [int(x, 16) for l in open("target/link_cost.raw") for x in l.split(":")[1].split()]
c0, i0, l0, d0, e0, c1, i1, l1, d1, e1 = w
m = lambda a, b: (b - a) & 0xFFFFFFFF
el = m(c0, c1)
t = [float(r["t"]) for r in csv.DictReader(open("target/link_cost.csv"))]
fps = (len(t) - 1) / (t[-1] - t[0])
frames = fps * el / 72e6
print(f"frames/s {fps:.0f}  link {100*m(l0,l1)/el:.1f} %  encode {100*m(e0,e1)/el:.1f} %  "
      f"cycles/frame: encode {m(e0,e1)/frames:.0f}, link {m(l0,l1)/frames:.0f}")
PY
