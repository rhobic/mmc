#!/usr/bin/env bash
# Flash/RAM of a firmware crate for each mmc-drive feature set: none, each
# feature alone, the board's default, all, and all + fixq. Prints a CSV row
# per build (sections in bytes); feature cost = row − the "none" row.
#   tools/feature_sizes.sh crates/mmc-fw-f302 > sizes_f302.csv
set -euo pipefail
dir=$1
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$dir"
name=$(sed -n 's/^name = "\(.*\)"/\1/p' Cargo.toml | head -n1)
triple=$(sed -n 's/^target = "\(.*\)"/\1/p' .cargo/config.toml | head -n1)
bin="$(rustc --print sysroot)/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/bin"
echo "set,text,rodata,data,bss,flash,ram"
run() {
    local label=$1; shift
    local td="$root/target/featsize-$name"
    cargo build -q --release --target-dir "$td" "$@" >&2
    local elf="$td/$triple/release/$name"
    read -r text rodata data bss vt < <("$bin/llvm-size" -A "$elf" | awk '
        $1==".text"{t=$2} $1==".rodata"{r=$2} $1==".data"{d=$2} $1==".bss"{b=$2} $1==".vector_table"{v=$2}
        END{print t, r, d, b, v}')
    "$bin/llvm-objcopy" -O binary "$elf" "$td/img.bin"
    echo "$label,$((text + vt)),$rodata,$data,$bss,$(stat -c %s "$td/img.bin"),$((data + bss))"
}
run none --no-default-features
for f in sixstep estim cogging hall-pos; do run "$f" --no-default-features --features "$f"; done
run default
run all --no-default-features --features "sixstep estim cogging hall-pos"
run all+fixq --no-default-features --features "sixstep estim cogging hall-pos fixq"
