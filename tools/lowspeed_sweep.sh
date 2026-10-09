#!/usr/bin/env bash
# Low-speed floor: closed-loop observer FOC (I-f spin-up to ±sl_handoff, then
# retargeted at 60 %) vs HFI (sensorless from rest to the target). Motor 3
# on bench 2. Score with tools/lowspeed_eval.py.
#   tools/lowspeed_sweep.sh <out dir> obs|hfi[:<profile>] <run> ...
# <run> is a target `w` [rad/s el], or `a:b` to start at a and retarget live
# to b at 60 % (a zero crossing when the signs differ). <profile> names
# <out dir>/<profile>.json (default: the kind) and prefixes the file names.
set -euo pipefail
out=$1; spec=$2; shift 2
kind=${spec%%:*}; prof=${spec#*:}; [[ $prof == "$spec" ]] && prof=$kind
H=target/release/mmc-host.exe
hz() { python -c "import math; print($1 / (2 * math.pi))"; }
mkdir -p "$out"
$H apply --serial COM9 --profile "$out/$prof.json" | tail -1
for run in "$@"; do
    if [[ $run == *:* ]]; then a=${run%%:*}; b=${run#*:}
    elif [[ $kind == obs ]]; then a=$(python -c "print(350 if $run > 0 else -350)"); b=$run
    else a=$run; b=; fi
    name="${prof}_w${run/:/_to_}"
    args=(--drive sl --amp 0.5 --hz="$(hz "$a")" --duration "${DUR:-10}")
    title="$kind ($prof) $a rad/s el"
    if [[ -n $b ]]; then args+=(--step-hz="$(hz "$b")"); title+=" -> $b"; fi
    $H capture --serial COM9 --baud 1000000 --divider 5 "${args[@]}" \
        --out "$out/$name.csv" --title "$title" | tail -1
    python -c "import time; time.sleep(2)"
done
