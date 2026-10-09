#!/usr/bin/env bash
# Where the CPU goes, per scenario, on a running board: the control tick,
# the host link (protocol + telemetry), sleep, and the rest (executor, USART/
# DMA/timer interrupts). Starts a capture with the scenario's arguments (or
# none for `-`: no host traffic at all), then reads the cycle counter and
# the firmware's accounting sums (`Shared::isr_sum_cycles`, `link_cycles`,
# `idle_cycles`, contiguous after `isr_max_cycles`) twice, 3 s apart, in one
# OpenOCD session that never halts the core. Prints shares of the elapsed
# cycles and the telemetry frame rate the capture saw.
#   tools/cpu_profile.sh <openocd target cfg> <&isr_sum_cycles> label "capture args"|- [...]
set -euo pipefail
target_cfg=$1; base=$2; shift 2
H=target/release/mmc-host.exe
OO=${OPENOCD:-openocd}
wait_s() { python -c "import time; time.sleep($1)"; }
CALC='
import csv, os, sys
label, out, args, raw = sys.argv[1:5]
words = [int(w, 16) for line in raw.splitlines() for w in line.split(":")[1].split()]
c0, i0, l0, d0, c1, i1, l1, d1 = words
m = lambda a, b: (b - a) & 0xFFFFFFFF
el = m(c0, c1)
isr, link, idle = m(i0, i1), m(l0, l1), m(d0, d1)
other = el - isr - link - idle
fps, states = "", ""
if args != "-" and os.path.exists(out):
    rows = list(csv.DictReader(open(out)))
    t = [float(r["t"]) for r in rows]
    fps = f"{(len(t) - 1) / (t[-1] - t[0]):.0f}" if len(t) > 1 else "0"
    seq = []
    for r in rows:
        s = int(float(r.get("state", -1)))
        if not seq or seq[-1] != s:
            seq.append(s)
    states = " ".join(map(str, seq))
pct = lambda x: f"{100 * x / el:.1f}"
print(f"{label},{pct(isr)},{pct(link)},{pct(idle)},{pct(other)},{fps},{states}")
'
echo "scenario,isr_pct,link_pct,idle_pct,other_pct,frames_per_s,states"
while (($# >= 2)); do
    label=$1; args=$2; shift 2
    out="target/cpu_$label.csv"
    if [[ $args != "-" ]]; then
        # shellcheck disable=SC2086
        ($H capture --serial COM9 --baud 1000000 --duration 12 $args --out "$out" > /dev/null 2>&1 &)
        wait_s 6
    fi
    raw=$("$OO" ${OPENOCD_SCRIPTS:+-s "$OPENOCD_SCRIPTS"} -f interface/stlink.cfg -c "transport select swd" \
        -f "$target_cfg" -c init -c "mdw 0xE0001004" -c "mdw $base 3" -c "sleep 3000" \
        -c "mdw 0xE0001004" -c "mdw $base 3" -c shutdown 2>&1 | grep -E "^0x")
    [[ $args != "-" ]] && wait_s 4.5
    python -c "$CALC" "$label" "$out" "$args" "$raw"
done
