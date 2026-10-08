#!/usr/bin/env bash
# Low-speed floor: closed-loop observer FOC (I-f spin-up to sl_handoff, then
# stepped down at 60 %) vs HFI (sensorless from rest straight to the target).
# Motor 3 on bench 2. Score with tools/lowspeed_eval.py.
#   tools/lowspeed_sweep.sh <out dir> obs|hfi <w1> <w2> ...   (rad/s el)
set -euo pipefail
out=$1; kind=$2; shift 2
H=target/release/mmc-host.exe
mkdir -p "$out"
$H apply --serial COM9 --profile "$out/$kind.json" | tail -1
for w in "$@"; do
    hz=$(python -c "import math; print($w / (2 * math.pi))")
    if [[ $kind == obs ]]; then
        $H capture --serial COM9 --baud 1000000 --divider 5 --duration 10 --drive sl --amp 0.5 \
            --hz 55.704 --step-hz "$hz" --out "$out/obs_w$w.csv" \
            --title "Observer FOC 350 -> $w rad/s el" | tail -1
    else
        $H capture --serial COM9 --baud 1000000 --divider 5 --duration 8 --drive sl --amp 0.5 \
            --hz "$hz" --out "$out/hfi_w$w.csv" \
            --title "HFI sensorless from rest to $w rad/s el" | tail -1
    fi
    python -c "import time; time.sleep(2)"
done
