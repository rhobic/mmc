#!/usr/bin/env bash
# Flash and RAM use of a firmware image, failing when flash headroom drops
# below a margin.
#
#   tools/fw_size.sh <firmware crate dir> <min flash headroom, bytes>
#
# Run after `cargo build --release` in that directory. Flash used is the
# programmed image (vector table, code, constants and the .data initial
# values), measured as the size of the raw binary the probe would write.
# The flash and RAM lengths come from the linker's memory.x: the crate's own
# (F302, which keeps the last page for parameters) or the one embassy-stm32
# generates for the chip. Needs the toolchain's `llvm-tools` component.
set -euo pipefail

dir=${1:?firmware crate dir}
margin=${2:?minimum flash headroom in bytes}
cd "$dir"

name=$(sed -n 's/^name = "\(.*\)"/\1/p' Cargo.toml | head -n1)
triple=$(sed -n 's/^target = "\(.*\)"/\1/p' .cargo/config.toml | head -n1)
elf="target/$triple/release/$name"
[[ -f $elf ]] || { echo "error: $elf not found; run cargo build --release first" >&2; exit 2; }

bin="$(rustc --print sysroot)/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/bin"
[[ -x $bin/llvm-size ]] || { echo "error: llvm-size missing; rustup component add llvm-tools" >&2; exit 2; }

if [[ -f memory.x ]]; then
    memx=memory.x
else
    memx=$(ls -t target/"$triple"/release/build/embassy-stm32-*/out/memory.x 2>/dev/null | head -n1)
fi
[[ -n ${memx:-} && -f $memx ]] || { echo "error: no memory.x for $name" >&2; exit 2; }

# LENGTH of a MEMORY region, in bytes ("62K", "16K", "512K", "0x8000", ...).
region_len() {
    local v
    v=$(grep -E "^\s*$1\s*:" "$memx" | sed -E 's/.*LENGTH\s*=\s*([0-9A-Fa-fxX]+)\s*([KkMm]?).*/\1 \2/')
    read -r n unit <<<"$v"
    n=$((n))
    case $unit in K|k) n=$((n * 1024)) ;; M|m) n=$((n * 1024 * 1024)) ;; esac
    echo "$n"
}
flash_len=$(region_len FLASH)
ram_len=$(region_len RAM)

img=$(mktemp)
trap 'rm -f "$img"' EXIT
"$bin/llvm-objcopy" -O binary "$elf" "$img"
flash_used=$(stat -c %s "$img")

declare -A sec
while read -r s size _; do
    sec[$s]=$size
done < <("$bin/llvm-size" -A "$elf" | grep -E '^\.')
ram_used=$(( ${sec[.data]:-0} + ${sec[.bss]:-0} + ${sec[.uninit]:-0} ))

headroom=$((flash_len - flash_used))
printf '%s (%s, %s)\n' "$name" "$triple" "$(rustc -V)"
printf '  sections : .vector_table %s  .text %s  .rodata %s  .data %s  .bss %s\n' \
    "${sec[.vector_table]:-0}" "${sec[.text]:-0}" "${sec[.rodata]:-0}" "${sec[.data]:-0}" "${sec[.bss]:-0}"
printf '  flash    : %6d / %6d bytes, %d free (margin %d)\n' "$flash_used" "$flash_len" "$headroom" "$margin"
printf '  RAM      : %6d / %6d bytes static, %d left for the stack\n' "$ram_used" "$ram_len" $((ram_len - ram_used))

if ((headroom < margin)); then
    echo "::error::$name flash headroom is $headroom bytes, below the $margin-byte margin" \
        "($flash_used of $flash_len used). Shrink the image before adding more;" \
        "docs/PROGRESS.md session 37 lists where the bytes go and what was tried." >&2
    exit 1
fi
