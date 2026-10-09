#!/usr/bin/env bash
# Steady-state control-ISR cost per drive mode on a running board: start a
# capture with the mode, clear `isr_max_cycles` once it has settled, read it
# back before the capture ends, and report whether the run stayed healthy.
# Prints `label,cycles,states` per mode.
#
# The counter is read and cleared through OpenOCD's memory access port,
# which does NOT halt the core. (probe-rs read/write halts it for a moment:
# with the bridge switching, a frozen PWM tripped the overcurrent limit at
# 600 rad/s el, session 43.)
#   tools/isr_profile.sh <openocd target cfg> <isr_max address> label "capture args" [...]
# The address is `&SHARED.isr_max_cycles` (gdb, or the Shared layout).
set -euo pipefail
target_cfg=$1; addr=$2; shift 2
H=target/release/mmc-host.exe
OO=${OPENOCD:-openocd}
wait_s() { python -c "import time; time.sleep($1)"; }
oocd() { "$OO" ${OPENOCD_SCRIPTS:+-s "$OPENOCD_SCRIPTS"} -f interface/stlink.cfg -c "transport select swd" \
    -f "$target_cfg" -c init "$@" -c shutdown 2>&1; }
echo "mode,isr_max_cycles,states"
while (($# >= 2)); do
    label=$1; args=$2; shift 2
    out="target/isr_$label.csv"
    # shellcheck disable=SC2086
    ($H capture --serial COM9 --baud 1000000 --divider 20 --duration 14 $args --out "$out" > /dev/null 2>&1 &)
    wait_s 8
    oocd -c "mww $addr 0" > /dev/null
    wait_s 3
    v=$(oocd -c "mdw $addr" | sed -n "s/^$addr: \([0-9a-f]*\).*/\1/p")
    wait_s 4.5
    states=$(python -c "
import csv
s=[int(float(r['state'])) for r in csv.DictReader(open('$out'))]
seq=[]
for x in s:
    if not seq or seq[-1]!=x: seq.append(x)
print(' '.join(map(str,seq)))")
    echo "$label,$((16#${v:-0})),$states"
done
