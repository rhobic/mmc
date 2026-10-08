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
| Sensorless start from standstill | `hfi_v` > 0 in sensorless mode: lock (0.3 s), polarity (3 × ±0.6 A pairs), run on the tracker, hand over to the flux observer at `sl_handoff` |

Firmware: F302 fw 20, G474 fw 25, **nvparam v11** (`hfi_v`, `hfi_bw`,
`hfi_xi`, ids 48–50).

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
observer), 3 to −300. Polarity came out right in **13 of 18**; the starts
where it did not ran briefly backwards (to −300 rad/s el) and recovered.
Stronger polarity pulses (±0.75 A) trip the 1.5 A overcurrent limit.

**Interrupt time** (F302, 7200 cycles per period): HFI start 6173–6265;
hall FOC with the HFI shadow 7167 — a diagnostic, not to be combined with
the cogging feed-forward (6640 on its own).

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

- **Polarity** is right 72 % of the time. The contrast (2–20 %) competes with
  the saliency term (5.5 % per cos 2Δ) whenever the rotor moves during the
  pulses. Options: pulses timed against the carrier's own response
  (current rise, not the averaged ripple), a short q-current nudge read on
  the tracker, or a higher current limit.
- **20–100 rad/s el** with heavy hunting: the tracker needs the known torque
  (an acceleration feed-forward into its PLL) or the cogging feed-forward
  running alongside (which the interrupt budget only allows without the
  shadow).
- **The F302 ADC interrupt discards every second current sample.** Keeping
  it (the sample where the new duty loads) would sample the carrier
  response where it is largest.
- **Ground truth** at standstill is a hall sector (±30°). The motor's
  1024-line encoder (J4, not wired to this shield) would score the sweep and
  the tracker to a fraction of a degree.
