# Six-step conduction angle: 120° vs 180°

*2026-10-06, bench 2 (F302 + L6230, motor 3 = maxon EC-i 40, 18 V), fw 14.
Captures: `testresults/ms8-conduction/`; charted report: [sixstep-conduction.html](sixstep-conduction.html).*

**Verdict: keep 120°.** At matched speed and torque, 180° drew 1.5–3.7× the RMS
phase current and 1.4–1.7× the input power, and it tripped the 1.5 A
overcurrent limit at 640–710 rad/s el. 120° ran up to 1351 rad/s el at the
0.85 duty ceiling.

## The two patterns

| | 120° | 180° |
|---|---|---|
| each leg per rev | high 120°, open 60°, low 120°, open 60° | high 180°, low 180° |
| states | six phase-pair currents (`TABLE`) | six active voltage vectors (`TABLE_180`) |
| ideal fundamental | √3/π·V_dc = 0.551 V_dc | 2/π·V_dc = 0.637 V_dc (+15.5%) |
| voltage harmonics | 6k±1, each 1/n | same |
| harmonic current path | blocked by the open phase | R + jnωL only |
| position from | idle-phase back-EMF (sensorless) | halls or an observer |
| commutates | 30° after the idle phase's zero crossing | at each leg's own zero crossing |

A 180° state is the union of two consecutive 120° sectors, so its boundaries
are the 120° sector centres. **The instants 120° detects are the instants 180°
commutates** (test `state_180_legs_switch_at_their_own_zero_crossings`). That is
what a sensorless "150° hybrid" would exploit, but it does not remove the
harmonic current below.

## Why 180° costs current on a sinusoidal motor

The back-EMF has no 5th or 7th harmonic to oppose the square wave's, so

```
I_n = (V1/n) / |R + j·n·ω·L|,   V1 ≈ ψ·ω   →   ψ/(n²·L) once nωL ≫ R
```

That limit does not depend on speed or load, and it scales with the short-circuit current ψ/L:

| motor | ψ/L | 5th: ψ/25L | 7th: ψ/49L |
|---|---|---|---|
| 1 (30 µH) | 31 A | 1.25 A | 0.64 A |
| 2 (4-pole) | 50 A | 1.98 A | 1.01 A |
| 3 (EC-i 40) | 18.7 A | 0.75 A | 0.38 A |

Measured on motor 3 at 600 rad/s el (180°): 5th 0.41 A against 0.45 A
predicted, and 7th 0.29 A against 0.28 A. 120° carries about 0.02 A at each
harmonic. Input power minus 3·I²R agrees between the modes to within 0.1 W, so
the extra input power is copper loss. The 180° duty saving was only 5–7%, short
of the ideal 13%, because the 120° open terminal sits at the back-EMF rather
than at zero.

180° makes sense where ψ/L is small against the rated current, or at the top
of field weakening in traction drives. None of our motors qualifies. The extra
15% of voltage is better taken through FOC overmodulation, which keeps current
control.

## Implementation

- `mmc_core::sixstep::{TABLE_180, state_180, duties_180}`, with three unit tests:
  union of adjacent sectors, back-EMF alignment, and legs switching at their own
  zero crossings.
- Param `ss_conduction` (id 24, 120 or 180, default 120) selects the pattern
  for `six-hall`. nvparam is now v6, so the profile must be re-applied after
  flashing across the change.
- Sim regression: `hall_six_step_at_180_degrees_spins_both_ways`.
