#!/usr/bin/env bash
# The integer HFI step must stay integer on a Cortex-M0+: build the thumbv6m
# bench (crates/mmc-fixq-bench) and fail if `FixHfi::step` calls any float
# (__aeabi_f*/d*), 64-bit multiply/divide (__aeabi_l*) or libm routine.
# Integer divides (__aeabi_idiv/uidiv) are expected: two per tick.
set -euo pipefail
cd "$(dirname "$0")/../crates/mmc-fixq-bench"
cargo build --release
elf=target/thumbv6m-none-eabi/release/mmc-fixq-bench
bin="$(rustc --print sysroot)/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/bin"
dis=$("$bin/llvm-objdump" -d -C --no-show-raw-insn "$elf")
step=$(awk '/^[0-9a-f]+ <.*FixHfi>::step>:$/{on=1;next} /^[0-9a-f]+ <.*>:$/{on=0} on' <<<"$dis")
[[ -n $step ]] || { echo "error: FixHfi::step not found in $elf" >&2; exit 2; }
calls=$(grep -oE 'bl\s+0x[0-9a-f]+ <.* @' <<<"$step" | sed -E 's/^bl\s+0x[0-9a-f]+ <//; s/> @$//; s/\+0x[0-9a-f]+$//' | sort -u || true)
echo "FixHfi::step: $(grep -cE '^\s+[0-9a-f]+:' <<<"$step") instructions; calls:"
sed 's/^/  /' <<<"$calls"
if grep -E '__aeabi_(f|d|l)|sinf|cosf|sqrtf|__(add|sub|mul|div)(s|d)f3' <<<"$calls"; then
    echo "error: FixHfi::step calls a float or 64-bit routine" >&2
    exit 1
fi
