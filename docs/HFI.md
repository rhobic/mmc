# Zero-speed position from saliency: high-frequency injection

*2026-10-08, bench 2 (F302 + X-NUCLEO-IHM07M1, motor 3 = maxon EC-i 40,
7 pole pairs, 18 V). Captures: `testresults/hfi/`.*

The flux observer needs back-EMF, which vanishes at standstill. A motor
whose inductance depends on the rotor's angle (saliency: from the rotor's
geometry, or from the magnet saturating the iron along its axis) shows its
angle in how the current answers a small high-frequency voltage, at any
speed down to zero. This needs no back-EMF sensing and no extra hardware:
the carrier rides on the normal PWM and the normal shunt samples.

## How it works

**Carrier.** Every control period the drive adds `±v_h` along its estimated
d axis in a `++−−` pattern (2.5 kHz at the 10 kHz control rate). At that
frequency the winding is mostly inductive (X/R ≈ 8 on motor 3), so the
current ripples by ≈ `v_h·T/L` along each axis.

**Why `++−−` and not `+−`.** The F302 samples the currents once per control
period and loads the new duty half a period later. A carrier that flips every
period puts half of `+v_h` and half of `−v_h` into every sample interval:
the response cancels, and what is left is dead-time distortion. Held for two
periods, each sign is seen whole by one interval; the demodulation weight
`(c[n−1] + c[n−2])/2` (`+1, 0, −1, 0`) is right for a mid-period duty load
and still right for a full-period one. The d response went from ~41 mA to
~310 mA at 1 V when the carrier changed.

**Demodulation.** The change in current between consecutive samples,
weighted by that pattern, cancels the drive's own slow current. Along the
estimate's d axis it measures ∝ 1/L; across it (q̂) it is zero on a round
rotor and `∝ −ξ·sin 2(θ̂ − θr)` on a salient one (ξ = (Lq − Ld)/(Lq + Ld)).

**Tracking** (`mmc_core::hfi::Tracker`). A PLL drives the normalised cross
response to zero; its gains are scaled by ξ so the bandwidth is what it is
set to. The axis comes out modulo π: saliency cannot tell north from south.

**Polarity.** A d current along the magnet's north saturates the iron and
lowers L; along the south it does not. So a +d bias makes the carrier
response larger than a −d bias, only if the estimate points north. The start
sequence compares them in three alternating pulse pairs and turns the
estimate by π if −d answered stronger.

**Dead time.** With the fundamental current near zero, the phase currents
cross zero on every carrier swing and the dead-time voltage flips with them,
swamping the cross response. A small d bias (`id_inject`, +0.5 A on motor 3)
keeps them off zero; the distortion is then a near-constant offset the
differencing cancels.

## What is in the drive

| Piece | Where |
|---|---|
| Standstill sweep: 24 test angles, response demodulated and accumulated on the device | `RunTest HFI_SWEEP` (`mmc_core::hfi`), `mmc-host hfi`, `tools/hfi_fit.py` |
| Real-time tracker | `mmc_core::hfi::Tracker`; params `hfi_v` (carrier [V], 0 = off), `hfi_bw` [rad/s], `hfi_xi` |
| Shadow under hall FOC, scored at hall edges | `hfi_v` > 0 in hall FOC: tracker angle on `theta_est`, its error at the last hall edge on `theta_err`, d response on `hfi_d` (channel 28) |
| Sensorless start from standstill | `hfi_v` > 0 in sensorless mode: lock (0.3 s), polarity (`hfi_pol_n` × ±0.6 A pulse pairs of `hfi_pol_s` each, default 8 × 6 ms), run on the tracker, hand over to the flux observer at `sl_handoff` |
| HFI speed gains | `hfi_kp`/`hfi_ki` while on the tracker (0 = `speed_kp`/`speed_ki`), back to the shared gains at the handover (motor 3: 8× the shared, persisted on bench 2) |
| Cross-saturation correction | `hfi_xsat` [rad el/A]: the drive's angle is the tracker's + `hfi_xsat`·i_q (motor 3: 0.44, persisted on bench 2) |
| Lock-up trip | HFI speed loop at ≥ 90 % of `iq_limit` with the tracker under half the reference, net 0.5 s → stall fault |
| Friction feed-forward | `sl_fric` [A], HFI speed loop only, faded in over ±5 rad/s el (measured no benefit on motor 3; off) |

Firmware: F302 fw 20, G474 fw 25, **nvparam v11** (`hfi_v`, `hfi_bw`,
`hfi_xi`, ids 48–50). Correction, trip and `sl_fric` (ids 51–52): F302
fw 24, G474 fw 29, **nvparam v12**. HFI gains and the pulse params (ids
53–56): F302 fw 25, G474 fw 30, **nvparam v13**.

## Results on motor 3

**Saliency.** Standstill sweeps at 24 rotor positions: ξ = 0.035–0.072
(mean 0.055, Lq/Ld ≈ 1.12), with the cross term matching it — real
saliency, not an artefact. The fitted axis agrees with the hall angle within
its ±30° sector uncertainty at rest (mean +6°, spread 28°).

**Polarity contrast.** Holding still, the d response under +0.7 A of d
current exceeded that under −0.7 A at all six positions tried, by 1.5–22 %.

**Tracking in shadow** (hall FOC, halls as truth). Standstill: locked and
steady (0.8° spread). 200 rad/s el: error at hall edges −2.7° mean, 10.3°
spread, speed 199 vs 200. 20–100 rad/s el: lost — the band where motor 3
hunts hardest without its cogging feed-forward (speed swinging 0–250).

**Sensorless start from standstill** (no hall input to the drive; halls as
truth only): **18 of 18 starts reached their target** — 12 to +300 rad/s el
from 12 rest positions, 3 to +600 (through the handover to the flux
observer), 3 to −300. **Polarity came out right in all 18**: read at rest
after the pulses, the estimate sits within the hall sector of the rotor, and
every start broke away in the commanded direction with no backwards run.
(An earlier count of 13/18 sampled the estimate during the last pulse, before
the flip is applied, and took the pulses' hall chatter for backwards runs;
`tools/hfi_start_eval.py` now reads after the sequence.)
Stronger polarity pulses (±0.75 A) trip the 1.5 A overcurrent limit.

**Interrupt time** (F302, 7200 cycles per period): HFI start 6173–6265;
hall FOC with the HFI shadow 7167 — a diagnostic, not to be combined with
the cogging feed-forward (6640 on its own).

## Low-speed floor: HFI vs the flux observer

*`testresults/motor3-hfi-vs-obs/`, `tools/lowspeed_sweep.sh`,
`tools/lowspeed_eval.py`. Forward only, one run per point, cogging
feed-forward off (it runs only in the hall modes).*

Observer: I-f to `sl_handoff` (350), closed on the flux observer, then
stepped down to the target. HFI: sensorless from rest straight to the
target, on the tracker throughout. True speed from counted hall steps;
angle error against the hall angle (HFI: at hall edges, mod π).

| target rad/s el | observer: speed | observer: θ err | HFI: speed | HFI: θ err |
|---|---|---|---|---|
| 200 | 200.1 | −2° ± 9° | 200.0 | −5° ± 5° |
| 150 | 149.9 | −5° ± 15° | 149.9 | −3° ± 6° |
| 100 | 100.8 | −10° ± 23° | 99.8 | −3° ± 9° |
| 75 | 57.9 | −8° ± 64° | 76.6 | −5° ± 9° |
| 50 | 41.5 | −22° ± 63° | 48.0 | +4° ± 9° |
| 40 | stall trip | | | |
| 30 | stall trip | | 29.3 | −4° ± 11° |
| 20 | stall trip | | 17.0 | +1° ± 9° |
| 10 | | | 8.8, in bursts | −5° ± 7° |
| 5 | | | never broke away | |

The observer converges down to **100 rad/s el**; at 75–50 it has lost the
rotor (±60°, speed short, i_q doubled), at 40 and below the stall detector
trips it. HFI holds the angle to ±5–11° at every speed it turns at, and the
mean speed to a few percent down to 20–30 rad/s el. Below ~75 both
stick-slip (hall speed σ 50–95): that is the speed loop against cogging, as
in hall FOC (session 30c), not the estimate. The shadow result (lost at
20–100) did not repeat with HFI in control.

**Reverse.** Observer: −100 holds (±20°); −75 and −50 average close to
target but on a lost angle (±56–69°). HFI holds −200 … −20 (−20.1, ±8°);
−10 averages −6.2.

**Zero crossing** (live retarget at 60 %, `a_to_b` files). The observer
stall-trips as the speed passes zero (350 → −100, 350 → −200). HFI
crosses: 200 → −200 clean; 100 → −100 clean but sticks ~1 s at zero before
breaking away; ±50 → ∓50 crosses and runs at 38–40 (stick-slip).

**Stick-slip is the speed loop's stiffness against stiction.** Binned true
speed shows the cycle: stuck while the integrator winds i_q from ~0.15 to
~0.33 A (breakaway), lurch to ~150, stick again, period 0.75–1 s. A PI speed
loop is a spring on position error with stiffness k_i, so the slip is about
i_break / k_i = 0.3 / 8.1e-3 ≈ 37 rad el ≈ 2100° el — measured ~1900°
(peak-to-peak hall position about its mean line, `pos_pp`). Scaling both
gains at ±20 and ±50 rad/s el:

| gains | +20 | +50 | −20 | −50 | mean pos_pp |
|---|---|---|---|---|---|
| ×1 | 17.0 / 2251° | 48.0 / 2269° | −20.1 / 1413° | −50.7 / 1543° | 1870° |
| ×2 | 20.7 / 1167° | 49.8 / 912° | −20.1 / 721° | −50.4 / 795° | 900° |
| ×4 | 20.5 / 652° | 49.7 / 531° | **lock-up** | −50.1 / 332° | 505° |
| ×4, `hfi_bw` 300 | 19.7 / 582° | OC at start | OC at start | OC at start | |

(true speed / pos_pp). The 1/k_i prediction holds (935°, 470°).

**More current moves the HFI angle (cross-saturation).** The ×4 lock-up at
−20: the rotor stuck, the integrator wound i_q to the 1.2 A limit, and the
tracker angle walked from inside the hall sector to +85° off the rotor —
no torque from q current, so more current, so more error: a self-locking
runaway, held at full current with no stall trip. Over every stuck
interval in these runs (rotor still, hall state unchanged > 0.15 s; 49 k
samples), the estimate moves ≈ −25° el per A of q current at small
currents (r = −0.88 overall) and runs away past ~0.6 A. So "more current
beats cogging" is capped by HFI itself: q current shifts the saliency axis
the tracker reads.

### With the correction (fw 22–24, `testresults/motor3-hfi-fix/`)

`hfi_xsat` = 0.44 rad/A (the −25° el/A drift). True speed / stick-slip
(`pos_pp`, ° el):

| | +5 | −5 | +10 | −10 | +20 | −20 | +50 | −50 |
|---|---|---|---|---|---|---|---|---|
| ×1, no correction (fw 20) | stuck | | 8.8 / 1388 | −6.2 / 1606 | 17.0 / 2251 | −20.1 / 1413 | 48.0 / 2269 | −50.7 / 1543 |
| ×4, no correction | 4.6 / 415 | −4.9 / 392 | 10.0 / 452 | **lock-up** | 19.9 / 499 | **lock-up** | 50.4 / 558 | −49.8 / 357 |
| ×4 + `hfi_xsat` | 5.0 / 444 | −4.9 / 314 | 10.2 / 495 | −10.2 / 374 | 19.4 / 598 | −20.0 / 321 | 49.8 / 418 | −50.0 / 247 |
| ×8 + `hfi_xsat` | 5.0 / 287 | −5.0 / 216 | 9.6 / 388 | −10.0 / 215 | 20.0 / 287 | −20.1 / 220 | 50.0 / 184 | −50.1 / 167 |

(×8 +20 from the fw 24 check; its fw 23 run tripped at start.) With the
correction every target from ±5 to ±50 rad/s el runs, both directions,
and q current drops (0.22 → 0.17 A rms at ×4, 0.16 at ×8: the torque is
aligned). Zero crossings at ×8: 20 → −20 (178°), 50 → −50 (130–136°),
100 → −100 (179°, angle +25 ± 18° — the one point where ×8 looked worse).
The floor is now **5 rad/s el (0.7 rad/s mechanical, ~7 rpm)** and not yet
found. ×8 gains were tried on HFI only: `speed_kp`/`speed_ki` are shared
with the observer and hall modes, so they are not persisted.

The lock-up trip fires on the uncorrected ×4 runs at −10 and −20 (stall at
5.6 s instead of holding 1.2 A); the first version missed −10, where
tracker speed noise reset its counter, and is now leaky.

**Friction feed-forward did not help.** `sl_fric` 0.12 A at ×4 + correction:
mean `pos_pp` 408° el, the same as without; 0.20 A no better. With the
stiff loop the integrator already supplies breakaway in milliseconds. On
the flux observer it made things worse (100 rad/s el `pos_pp` 341 → 1984,
75 and 50 stall-tripped), so it is HFI-only and off.

**Start overcurrent.** The polarity pulses trip the 1.5 A limit in about 1
start in 10 (2/40 on fw 22, 5/42 on fw 23, 4/19 in a dedicated batch). It
is not an edge overshoot — a 300 A/s slew on the pulses (fw 23) did not
help and was taken out again in fw 24. Through the whole pulse window the
measured i_d is ~2× the ±0.6 A command and the rotor turns 1–3 electrical
revolutions (10–21 hall edges in 0.2 s, up to ±90 rad/s el): the pulses
throw the rotor, the tracker frame slips under the current loop, and the
phase peaks reach the trip. Session 36's 18/18 was a small sample.

### Start fixed: short polarity pulses (fw 25, `testresults/motor3-hfi-start/`)

Why the pulses threw the rotor: the start's d bias (`id_inject`) aligns the
magnet's north with the tracker's +d during the lock, so the −d pulse
pushes against the magnet — an unstable equilibrium. Offsets from it grow
with τ = 1/√(1.5·p²·ψ·i/J) ≈ 4 ms at 0.6 A on motor 3; a 25 ms pulse is six
of those, enough to turn the rotor half a revolution, and the next +d pulse
turns it back. Pulses of a few τ cannot. The pulse length and count are now
params, and the polarity is read from the unfiltered response (`d_amp`'s
~2 ms filter would carry the previous pulse into a short one). 15 starts
each to +20 rad/s el, from wherever the last one left the rotor:

| pulses | overcurrent | polarity right | reached | hall edges during pulses (median / max) | peak \|i_d\| (median / max) |
|---|---|---|---|---|---|
| 25 ms × 3 (old) | 0 | 15/15 | 15/15 | 4 / 19 | 0.83 / 1.43 A |
| 6 ms × 8 | 0 | 15/15 | 15/15 | 0 / 0 | 0.79 / 0.80 A |
| 4 ms × 12 | 0 | 15/15 | 15/15 | 0 / 1 | 0.73 / 0.80 A |

6 ms × 8 (96 ms, was 150) is the default. Then 15 more starts to −20
(clean, rotor still) and 3 to 600 rad/s el through the handover (smooth,
i_q ≤ 0.35 A): **63 of 63 starts with the short pulses clean**. Peak i_d
is the pulse plus the carrier ripple; the old pulses reached 1.43 A, a
sample-rate's worth under the trip, which is where the ~10 % came from.
(The old shape happened not to trip in these 15.)

With the HFI gains on their own params (8× the shared, persisted) and
`hfi_xsat` = 0.44, on fw 25: ±5 → 5.2 / −5.0 rad/s el (stick-slip 318° /
222°), ±10 → 10.0 / −10.1 (283° / 213°), 50 → −50 crossing clean; the
observer at ±200 unaffected (shared gains).

## Is back-EMF sensing needed?

The terminal voltage dividers (back-EMF sensing) are used by: sensorless
six-step (zero crossings), flux from a coast (`coast_flux.py`, the profiler's
`coast` stage), and the coasting-terminal scope fit. They are **not** used
by FOC — the flux observer integrates the commanded voltage — nor by
anything here: HFI works from the current shunts alone. What they buy:
six-step without a flux-integrating substitute, catching a coasting rotor
from Off without halls (the zero-current way to see angle, speed and
direction), and an independent ψ measurement. Low-speed FOC control does not
need them; it needs saliency, which motor 3 has.

## Limits and next steps

- **Polarity margin** is thin: the contrast is 1.5–22 %, comparable to the
  saliency term, and the pulses jerk the rotor. Options for a motor with less d-axis saturation: read the
  contrast from the first current rise rather than the averaged ripple, or a
  short q-current nudge read on the tracker.
- **Polarity is read on a rotor the lock bias has already aligned**, so
  the pulses mostly confirm what the bias set up. A motor with less static
  friction or a heavier load may not align during the lock; the polarity
  read is then doing real work, and 63/63 here says little about it.
- **F302 flash**: 2.97 KB left (CI floor 2 KB).
- **Cross-saturation is linear only to ~0.6 A**: a table or quadratic
  `hfi_xsat` for loaded operation (motor 3 never needs more than ~0.3 A
  unloaded).
- **20–100 rad/s el** with heavy hunting (in shadow; under HFI control the
  tracker holds, see the low-speed floor above): the tracker needs the known torque
  (an acceleration feed-forward into its PLL) or the cogging feed-forward
  running alongside (which the interrupt budget only allows without the
  shadow).
- **The F302 ADC interrupt discards every second current sample.** Keeping
  it (the sample where the new duty loads) would sample the carrier
  response where it is largest.
- **Ground truth** at standstill is a hall sector (±30°). The motor's
  1024-line encoder (J4, not wired to this shield) would score the sweep and
  the tracker to a fraction of a degree.
