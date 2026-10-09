# Integer HFI drive: can it run on a Cortex-M0+?

*2026-10-08, session 40. Bench 2 (F302 + IHM07M1, motor 3, 18 V);
captures `testresults/motor3-fixq/`.*

**Yes, with room to spare.** The low-speed sensorless drive this project
runs on HFI (current loop, tracker, start, speed loop) was rewritten in
integer arithmetic, run on motor 3 in place of the float version with the
same results, and cycle-counted as Cortex-M0+ code: about 1.9 k cycles a
tick at worst with code in RAM, under 3.7 k with code in 2-wait-state
flash — 29–57 % of a 10 kHz period on a 64 MHz STM32G0.

## What was built

`mmc_core::fixq` — the same algorithms as `foc`, `hfi` and the drive's HFI
start, integer only:

| Piece | Integer form |
|---|---|
| Units | current Q15 of `i_base`, voltage Q15 of `v_base`, angle u32 (2³² = 1 rev, wraps for free), speed in angle units per tick, duty Q15 |
| sin/cos | 257-entry Q15 table, linear interpolation (≤ 4 LSB) |
| Gains | `x·m >> s`, `m` in [2¹⁴, 2¹⁵), inputs clamped to ±2¹⁵: every product is i32 × i32 under 2³⁰ — no 64-bit multiply (the M0+ has none) |
| PI | Q30 integrator (15 bits below the output LSB), clamping anti-windup |
| Current loop | as `Foc`: decoupling feed-forward, voltage circle by integer square root, output advanced by `advance_periods` |
| HFI tracker | `++−−` carrier, difference demodulation, ξ-scaled PLL; one i32 division (the normalisation) |
| Start / run | lock, `hfi_pol_n` × `hfi_pol_s` polarity pulses (compared without dividing), speed loop with `hfi_kp`/`hfi_ki`, `hfi_xsat`, lock-up flag |
| Modulation | min-max (SVPWM-equivalent), one u32 division a tick for 1/v_bus |

Setup (`FixHfi::new`) converts the f32 parameters once; only `step` runs
per tick. In the drive it is the cargo feature `fixq` (mmc-drive,
mmc-fw-f302): HFI sensorless runs on the integer path, converted to and from
f32 only at the board boundary (an FPU-less board would hand over ADC counts
and take timer compares).

## Verification

**Unit tests** (`mmc_core::fixq`, 7): sin/cos, isqrt, gains across nine
decades, sub-LSB integration, Clarke/Park and modulation against the float
versions, and the integer tracker converging on the same salient rotor as
the float one from the same currents.

**Simulator**: all 22 `sim_board` tests pass with `fixq`, including the HFI
start, which reaches 200 rad/s el exactly as the float path does.

**Motor 3 on the bench**: same params as the float runs (HFI gains 8×,
`hfi_xsat` 0.44, pulses 6 ms × 8). True speed / stick-slip (`pos_pp`, ° el):

| target rad/s el | float (fw 23–25) | integer |
|---|---|---|
| +5 | 5.2 / 318 | 5.0 / 281 |
| −5 | −5.0 / 222 | −5.0 / 206 |
| +10 | 10.0 / 283 | 9.9 / 273 |
| −10 | −10.1 / 213 | −10.0 / 206 |
| +20 | 20.0 / 287 | 20.0 / 270 |
| −20 | −20.1 / 220 | −20.1 / 203 |
| +50 | 50.0 / 184 | 50.0 / 183 |
| −50 | −50.1 / 167 | −49.8 / 206 |
| 50 → −50 | −49.2 / 171 | −50.1 / 126 |
| 100 → −100 | −99.2 / 179, θ +25 ± 18° | −100.1 / 150, θ +24 ± 17° |

Starts: 17 of 17 (15 batched + 2), no overcurrent, polarity right, peak
i_d 0.81 A. (Three batched starts show 10–12 hall edges during the pulses:
the rotor had come to rest on the 3|2 hall boundary and the alternating
pulses toggle it there — two states only, no motion; `hfi_start_score.py`
counts them as edges.) The integer and float drives are indistinguishable on
this motor.

**Cycles**

| | worst | mean |
|---|---|---|
| F302 (Cortex-M4, 72 MHz), measured, steady 20 rad/s el | 1 508 | ~1 400 |
| Cortex-M0+, emulated, zero wait states (code in RAM) | 1 881 | 1 597 |
| Cortex-M0+, emulated, 2 flash wait states (estimate) | 3 617 | 3 057 |

The M0+ numbers come from `tools/m0_cycles.py`: the thumbv6m build
(`crates/mmc-fixq-bench`) run in an instruction emulator (unicorn) in closed
loop with a salient R-L motor model, each instruction charged its Cortex-M0+
cycles (single-cycle multiplier, as on the G0). The wait-state column is a
deliberately pessimistic model (a wait per 32-bit fetch, literal load and
taken branch; the G0's prefetch hides part of it). At 64 MHz and 10 kHz the
budget is 6 400 cycles: **29 % from RAM, ≤ 57 % from flash**. At 32 MHz
the flash case would not fit; from RAM it would (59 %).

`tools/fixq_m0_check.sh` (CI) fails if the M0+ `step` ever calls a float or
64-bit routine; today it calls only `__aeabi_idiv`/`__aeabi_uidiv`. Code:
`step` 2.5 KB, the whole bench image (with the float setup) 6.3 KB + the
514-byte table.

On the F302 the whole ISR with the integer step measured 5 643 of 7 200
cycles (it still runs the float telemetry, flux observer and hall code
around it). The `fixq` firmware fits the 62 KB flash only with the firmware
crate at opt-level "s" (CI builds it that way).

## What an M0+ drive still needs

- **Above HFI's range**: the flux observer and the handover to it, in
  integer (the integer path has no handover; it stays on HFI).
- **A G0 board**: TIM1 with dead time and break, one ADC sequencing the
  phases (fine for HFI — the skew is constant and the demodulation
  differences consecutive samples), external shunt amplifiers.
- **Silicon numbers**: the G0B1 dev board is not on the bench now; the
  bench crate is ready to run there under SysTick timing.
- **Dead-time compensation**: found while porting — the float HFI path
  rebuilds the duties from the uncompensated voltage when it adds the
  carrier, so `v_dead` compensation is off whenever HFI runs. Every HFI
  result so far, float and integer, was taken that way; the integer path
  matches it.

## Files

`crates/mmc-core/src/fixq.rs`; engine glue `Engine::fixq_tick`
(feature `fixq`); `crates/mmc-fixq-bench` (thumbv6m, outside the
workspace); `tools/m0_cycles.py` (needs unicorn, capstone, pyelftools);
`tools/fixq_m0_check.sh`.
