#!/usr/bin/env bash
# Repeated HFI sensorless starts (start reliability: overcurrent trips,
# polarity), each from wherever the last left the rotor. Score with
# tools/hfi_start_eval.py --pol-t <lock + 2·n·pulse>.
#   tools/hfi_start_batch.sh <out dir> <profile name> <starts> [target rad/s el]
set -euo pipefail
out=$1; prof=$2; n=$3; w=${4:-20}
H=target/release/mmc-host.exe
$H apply --serial COM9 --profile "$out/$prof.json" | tail -1
hz=$(python -c "import math; print($w / (2 * math.pi))")
for k in $(seq 1 "$n"); do
    $H capture --serial COM9 --baud 1000000 --divider 5 --duration 3 --drive sl --amp 0.5 \
        --hz="$hz" --out "$out/${prof}_s$k.csv" --title "HFI start $k ($prof) to $w rad/s el" > /dev/null
    python -c "import time; time.sleep(1.5)"
done
