#!/usr/bin/env bash
# Hall FOC at steady speeds in both directions; the shadow observer's angle
# and speed are compared against the calibrated halls.
#   tools/hall_ref_sweep.sh <out-dir> [serial] [hall_ref.py args...]
set -e
out=$1; port=${2:-COM9}; mkdir -p "$out"
for w in 100 200 400 600 800 -100 -200 -400 -600 -800; do
  hz=$(python -c "print(round($w/6.2832,2))")
  timeout 60 ./target/release/mmc-host capture --serial "$port" --baud 1000000 --duration 4 --divider 10 \
    --drive hall-foc --amp 1.0 --hz=$hz --out "$out/ref_w$w.csv" \
    --title "Hall FOC reference @ $w rad/s el: observer vs halls" > /dev/null
done
python tools/hall_ref.py "$out" "${@:3}"
