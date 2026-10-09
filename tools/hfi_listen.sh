#!/usr/bin/env bash
# Listening comparison of HFI carrier settings on the bench: each setting
# holds the motor at a slow speed for a few seconds, labelled on the
# terminal, with a pause between. Pass profile JSONs (hfi_v, hfi_spread, …);
# the drive is left off with HFI disabled at the end.
#   tools/hfi_listen.sh [speed rad/s el] profile.json ...
set -euo pipefail
w=20
[[ ${1:-} =~ ^-?[0-9.]+$ ]] && { w=$1; shift; }
H=target/release/mmc-host.exe
hz=$(python -c "import math; print($w / (2 * math.pi))")
k=0
for prof in "$@"; do
    k=$((k + 1))
    $H apply --serial COM9 --profile "$prof" > /dev/null
    echo ">>> $k/$#: $(basename "$prof" .json)  ($(python -c "import json; d=json.load(open('$prof')); print(f\"hfi_v {d.get('hfi_v')} V, spread {int(d.get('hfi_spread', 0))}\")"))"
    $H capture --serial COM9 --baud 1000000 --divider 50 --duration 6.5 --drive sl --amp 0.5 \
        --hz="$hz" --out "$(dirname "$prof")/listen_$(basename "$prof" .json).csv" > /dev/null
    sleep 2
done
printf '{"hfi_v":0.0,"id_inject":0.0}' > /tmp/hfi_off.json 2>/dev/null || true
$H apply --serial COM9 --profile /tmp/hfi_off.json > /dev/null 2>&1 || true
echo "done (HFI off)"
