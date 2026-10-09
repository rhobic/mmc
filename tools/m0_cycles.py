"""Cortex-M0+ cycle count of the integer HFI drive step (mmc_core::fixq),
by instruction-level emulation of the thumbv6m build
(crates/mmc-fixq-bench) in a closed loop with a salient R-L motor model.

Each executed instruction is charged its Cortex-M0+ cycles (ARM DDI 0484C,
table 3-1, single-cycle multiplier as on the STM32G0): data processing 1,
loads/stores 2, LDM/STM/PUSH 1+N, POP 1+N (3+N with PC), taken branch 2,
BL 3, BX/BLX 2. That is zero-wait-state memory — code run from RAM, or
flash at ≤ 24 MHz. `--flash-ws` adds an estimate for code in flash with
wait states (G0 at 64 MHz: 2): one wait state per 32-bit fetch (two
16-bit instructions) and per literal load, and a refetch after each taken
branch.

Needs unicorn, capstone and pyelftools.

    (cd crates/mmc-fixq-bench && cargo build --release)
    python tools/m0_cycles.py crates/mmc-fixq-bench/target/thumbv6m-none-eabi/release/mmc-fixq-bench
"""
import argparse
import math

import capstone
import unicorn as uc
from elftools.elf.elffile import ELFFile
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP

FLASH, RAM = 0x0800_0000, 0x2000_0000
RET = 0x0000_1000  # return sentinel


def cost(insn, taken):
    m = insn.mnemonic.split(".")[0]
    ops = insn.op_str
    if m in ("ldr", "ldrb", "ldrh", "ldrsb", "ldrsh", "str", "strb", "strh"):
        return 2
    if m in ("ldm", "ldmia", "stm", "stmia"):
        return 1 + ops.count(",") if "{" in ops else 2
    if m == "push":
        return 1 + len(ops.strip("{}").split(","))
    if m == "pop":
        n = len(ops.strip("{}").split(","))
        return (3 + n) if "pc" in ops else (1 + n)
    if m == "bl":
        return 3
    if m in ("bx", "blx"):
        return 2
    if m.startswith("b") and m not in ("bic", "bics"):
        return 2 if taken else 1
    return 1


def main(a):
    elf = ELFFile(open(a.elf, "rb"))
    syms = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
    mu = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
    mu.mem_map(FLASH, 0x80000)
    mu.mem_map(RAM, 0x24000)
    mu.mem_map(0, 0x2000)
    for seg in elf.iter_segments():
        if seg["p_type"] == "PT_LOAD" and seg["p_filesz"]:
            mu.mem_write(seg["p_vaddr"], seg.data())
    md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
    decoded = {}
    state = {"cyc": 0, "fetch": 0, "lit": 0, "taken": 0, "prev": None}

    def hook(mu_, addr, size, _):
        p = state["prev"]
        if p is not None:
            pa, pins = p
            taken = addr != pa + pins.size
            state["cyc"] += cost(pins, taken)
            state["taken"] += taken and pins.mnemonic.startswith("b")
            if "[pc" in pins.op_str:
                state["lit"] += 1
        if addr not in decoded:
            code = mu_.mem_read(addr, size)
            decoded[addr] = next(md.disasm(bytes(code), addr))
        ins = decoded[addr]
        state["fetch"] += ins.size
        state["prev"] = (addr, ins)

    mu.hook_add(uc.UC_HOOK_CODE, hook)

    def call(fn, args=()):
        regs = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3]
        for r, v in zip(regs, args):
            mu.reg_write(r, v & 0xFFFF_FFFF)
        mu.reg_write(UC_ARM_REG_SP, RAM + 0x24000)
        mu.reg_write(UC_ARM_REG_LR, RET | 1)
        for k in ("cyc", "fetch", "lit", "taken"):
            state[k] = 0
        state["prev"] = None
        mu.emu_start(syms[fn] | 1, RET)
        p = state["prev"]
        if p is not None:  # the final return
            state["cyc"] += cost(p[1], True)
        ws = a.flash_ws * (state["fetch"] / 4 + state["lit"] + state["taken"])
        return mu.reg_read(UC_ARM_REG_R0), state["cyc"], state["cyc"] + ws

    call("fixq_bench_init")

    # Salient motor at standstill (rotor at `theta_r`), motor 3's R and L.
    dt, R, Ld, Lq, vbus = 1e-4, 1.42, 0.33e-3, 0.39e-3, 18.0
    th = a.theta_r
    i_d = i_q = 0.0
    duties_at = syms["FIXQ_DUTIES"]
    q_i, q_v = 32768 / 3.0, 32768 / 32.0
    rows = []
    for k in range(a.ticks):
        ia = i_d * math.cos(th) - i_q * math.sin(th)
        ib = i_d * math.sin(th) + i_q * math.cos(th)
        pa = ia
        pb = -0.5 * ia + math.sqrt(3) / 2 * ib
        pc = -0.5 * ia - math.sqrt(3) / 2 * ib
        iq_cmd, cyc, cyc_ws = call("fixq_bench_tick", [int(pa * q_i), int(pb * q_i), int(pc * q_i), int(vbus * q_v)])
        rows.append((k, cyc, cyc_ws))
        d = [int.from_bytes(mu.mem_read(duties_at + 4 * j, 4), "little", signed=True) / 32768 for j in range(3)]
        m = sum(d) / 3
        va, vb_, vc = [(x - m) * vbus for x in d]
        al = (2 * va - vb_ - vc) / 3
        be = (vb_ - vc) / math.sqrt(3)
        vd = al * math.cos(th) + be * math.sin(th)
        vq = -al * math.sin(th) + be * math.cos(th)
        i_d += (vd - R * i_d) / Ld * dt
        i_q += (vq - R * i_q) / Lq * dt

    def summary(lo, hi, label):
        sel = [r for r in rows if lo <= r[0] < hi]
        if not sel:
            return
        c = [r[1] for r in sel]
        w = [r[2] for r in sel]
        print(f"{label:22} ticks {len(sel):5}  zero-wait: mean {sum(c)/len(c):6.0f} max {max(c):5}   "
              f"flash {a.flash_ws} WS: mean {sum(w)/len(w):6.0f} max {max(w):6.0f}  cycles")

    lock = int(0.3 / dt)
    pol = int(2 * 8 * 0.006 / dt)
    summary(0, lock, "lock")
    summary(lock, lock + pol, "polarity pulses")
    summary(lock + pol, a.ticks, "run (speed loop)")
    summary(0, a.ticks, "all")
    worst = max(r[2] for r in rows)
    for mhz in (64, 48, 32):
        budget = mhz * 1e6 * dt
        print(f"  at {mhz} MHz, 10 kHz: budget {budget:5.0f} cycles; worst step {worst:5.0f} = {100*worst/budget:4.1f} %")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--ticks", type=int, default=6000)
    ap.add_argument("--theta-r", type=float, default=0.6, help="rotor angle [rad el]")
    ap.add_argument("--flash-ws", type=int, default=2, help="flash wait states for the estimate")
    main(ap.parse_args())
