# Calibrating a hall motor, and what the drive does with it

*2026-10-06/07, bench 2 (F302 + X-NUCLEO-IHM07M1, motor 3 = maxon EC-i 40,
7 pole pairs, 18 V). Captures under `testresults/motor3-*`.*

This is the measured detail behind five drive features: hall sector widths,
the online R/ψ estimator and its i_d dither, the R/L probe's current, flux
from a coast, and cancelling the torque the rotor meets around a turn.

| Feature | Where | Firmware |
|---|---|---|
| Hall sector widths | params `hall_w0…5` (26–31), `tools/hall_widths.py` | fw 16, nvparam v8 |
| Online R/ψ | `mmc_core::estim`; channels `r_hat`, `psi_hat` (26, 27); `mmc-host estimate` | fw 17 |
| i_d injection and dither | params `id_inject` (25), `id_dither`, `id_dither_period` (32, 33) | fw 15 / 18, nvparam v9 |
| Position-torque feed-forward | `mmc_core::cogging`; params `cog_ff`, `cog_shift`, `cog_n/a/p0…3` (34–47) | fw 19, nvparam v10 |
| Flux from a coast | `tools/coast_flux.py`, `mmc-host profile --only coast` | — |
| Torque around a turn | `tools/drag_profile.py` | — |

`mmc-host params [names…]` reads the parameter table back.

## Hall sector widths

At steady speed every hall state lasts in proportion to its electrical
width, so the time share of each state over many turns measures the sensor
placement. On motor 3 the six sectors run 56–63° (one sensor sits 3.4° el
off), while the drive assumed 60°. That left a 2nd-order i_q ripple in hall
FOC. With the measured widths as parameters (hall angle and hall speed both
use them):

| hall FOC, 2nd-order i_q | 60° assumed | measured widths |
|---|---|---|
| 400 rad/s el | 47.6 mA | 6.3 mA |
| 800 rad/s el | 156 mA | 47 mA |
| −400 rad/s el | 53 mA | 10 mA |

Speed σ at 400 rad/s el fell 24.9 → 19.0. Hall speed also had a related
flaw, fixed with it: its "cannot be faster than one sector in the time since
the edge" bound used the previous sector's width, so a wide sector after a
narrow one dipped the speed ~10 %.

## Online R and ψ

R is only visible with injected i_d (at speed ψ·ω dwarfs R·i_q). Three
things had to be right before the estimate meant anything:

1. **R from the d row only, ψ from the q row with R̂·i_q subtracted.**
   Letting the q row pull on R hands it every q-axis transient.
2. **A d-axis offset state, and R from steps in i_d.** Dead-time
   compensation residue and voltage-angle error put an offset on v_d that a
   constant i_d cannot tell from R.
3. **Block means, not samples.** Sample by sample, the current loop makes
   i_d and v_d move together with a slope that is not R: fed raw samples,
   the filter read as low as −0.27 Ω on the bench. 50 ms means fix it.

Bench, hall FOC, i_d stepped −0.2 ↔ −0.6 A: R̂ 1.31–1.37 Ω from 200 to 800
rad/s el (profiler, at standstill: 1.418 Ω; static slope Δv_d/Δi_d at 800:
1.40 Ω), ψ̂ 6.29–6.34 mWb. Like the profiler, the estimator sees R and ψ
referred to the *commanded* voltage, which is what the controller needs; ψ
reads ~5 % under the flux a coast measures.

The drive runs the estimator itself (fw 17) in hall FOC and sensorless FOC:
within 1 % of the host on the same frames. Since fw 18 it also steps i_d
itself (`id_dither` [A], `id_dither_period` [s]; between `id_inject` and
`id_inject + id_dither`, only above the estimator's 100 rad/s el floor), so
R̂ keeps updating with no host in the loop. Bench, −0.2/−0.6 A every
0.25 s: drive R̂ 1.286 / 1.316 Ω at 400 / 800 rad/s el, host on the same
frames 1.277 / 1.316 (`testresults/motor3-dither/`). Off by default.

## The R/L probe needs current

A winding is first order: after a voltage step its current never passes its
final value. On motor 3 it did, and the overshoot fell as the align current
rose: 62 % at 0.35 A (the default `--rl-volts 0.5,1.0`, which now fails the
fit), 4.7 % at 0.56 A, 1.1 % at 0.70 A, under 0.8 % from 0.83 A. The fitted L
went 0.23 → 0.31 → 0.348–0.351 mH, onto the datasheet's 0.34. At low
current the park is soft against the rotor's detent torque, every step moves
the rotor, and its back-EMF bends the exponential; at low current the PWM
ripple also crosses zero, where the dead-time voltage flips. Dead-time
compensation is not involved (`v_dead` = 0 repeats every reading).
`tools/profile.py` now warns above 1 % overshoot; motor 3 wants
`--rl-volts 1.2,2.0` (`testresults/motor3-rlprobe/`).

## Flux from a coast

A scope on a coasting terminal (`tools/bemf_scope.py`, `tools/bemf_fit.py`)
gives ψ = 6.618 mWb and flux harmonics of 0.10 % (5th) and 0.02 % (7th):
the back-EMF is sinusoidal. With every switch open the terminal mean sits at
1.15 V on this board, not 0 V.

The drive's own terminal sense does it without the scope: two terminals both
above the divider's floor differ by exactly the line EMF (a diode clamp
lifts all three alike), and the hall edges, fitted over the coast, give the
angle and speed. ψ is then a linear fit (`tools/coast_flux.py`): 6.592 and
6.559 mWb on two coasts from 1000 rad/s el. It is the profiler's opt-in
`coast` stage (`mmc-host profile --only coast`, needs halls), which
`tools/profile.py` prefers over the I-f sweep: 6.580 mWb. The coast also
checks the mechanics: 1000 → 421 rad/s el in 44 ms, J·α = 8.6 mN·m at 755
rad/s el against friction 7.4 + viscous 0.9.

## The torque around a turn

In hall FOC at 100 rad/s el motor 3 hunts: hall speed σ 44–53 rad/s.
Timing the 42 hall states of a turn over many turns, 89 % of that repeats
at the same rotor position: a load that depends on position, not a loop
cycle. `tools/drag_profile.py` turns steady runs into torque against
position. A state's duration gives the mean speed over its slice, the
kinetic-energy change between slices the net torque, and subtracting
k_t·i_q leaves the external torque. Runs in both directions split it:
friction flips sign with direction, a conservative torque does not.

- Friction: 7.40 mN·m, flat around the turn, the same at ±100 and ±200.
- Conservative: dominated by 12 per turn (one per slot), with smaller 15th,
  14th and 18th-or-24th orders (42 samples a turn cannot tell 18 from 24).
- The tool reads through two boxcars one slice wide (slice-mean speed, then
  a difference between slices), which shrink order n by sinc²(πn/42); the
  tool undoes it (checked against a simulated rotor with a known torque).

### Cancelling it

Hall FOC and hall position mode add `−cog_ff·τ(θ_m)/k_t` to their i_q
reference, `τ(θ_m) = Σ amp·sin(order·θ_m + phase)` over four terms.

**Which pole pair.** The halls carry no index, and the series counts from
one hall edge on one of 7 pole pairs. The drive counts hall states from
power-up and, while hall FOC holds a setpoint of 50–600 rad/s el, measures
the torque at every state boundary the way the tool does, then correlates
it with the series rotated by each pole pair. The best match goes into
`cog_shift` (−1 at boot; a host write of −1 asks for a new match, 0…6 forces
one); about 1.5 s at 100 rad/s el. The bench found a pole pair every boot,
and forcing all seven confirmed the found one is the one that cancels (speed
σ 48.8 against 65.6–149.5 for the others). Collection is gated on the
setpoint, not the measured speed: a hunting rotor is slowest exactly where
the torque is largest, and gating on speed starved that boundary.

**Where the rotor is.** Interpolating hall edges at constant speed is about
13° el off while the rotor hunts, which at 12 and 24 per turn is 22–44° of
phase; the feed-forward then cancels little. It is placed by the hall
tracker instead (the position mode's), which predicts between edges from the
measured current plus the known position torque and snaps at each edge.

**The series.** Fed forward as first measured, the series did not cancel
(order 12 alone made the hunting worse). Tuned on the bench at +100 rad/s el,
then at ±100 together: order 12 at 6.5 mN·m (the measured phase was right;
±0.35 rad is clearly worse), 15 and 14 at 1.5× their measured amplitude, no
24, the set rotated −0.005 rad mech
(`testresults/motor3-cogff/cogging_ff_bench.json`, persisted on bench 2).

**Bench, fw 19, hall FOC speed σ [rad/s el], feed-forward off → on**
(`testresults/motor3-cogff/sweep_*`):

| setpoint | 50 | 100 | −100 | 150 | −150 | 200 | −200 | 300 | −300 | 400 | 600 | 800 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| off | 81.5 | 44.9 | 54.5 | 21.4 | 21.8 | 16.9 | 17.3 | 16.0 | 16.5 | 18.9 | 28.8 | 42.5 |
| on | 61.8 | 19.6 | 40.4 | 11.9 | 16.0 | 15.2 | 13.0 | 15.5 | 13.8 | 17.6 | 29.3 | 43.8 |

It is off above 500 rad/s el, where it measured neutral. Reverse gains less
than forward (−26 % against −56 % at 100 rad/s el): the residual depends on
direction, likely through the hall edges' placement and hysteresis.

**Interrupt budget.** A first build peaked at 9414 of the F302's 7200 cycles
per control period (fw 18: 4473): the identification's decision summed 42
boundaries in one tick, a drive start rebuilt the counter struct, and the
series was evaluated twice a tick. Now the decision's sums are gathered
while scoring, a start only refreshes the geometry, and the series is
evaluated once a tick, alternating the tracker's angle and the
feed-forward's. Worst case after: 6499 cycles, across identification and
100/300/800 rad/s el with the feed-forward on or off. Measure it with
`probe-rs read b32 <&SHARED.isr_max_cycles> 1` (zero it first with
`probe-rs write`).

## Open

- Better cancellation would need the angle between hall edges to well under
  a degree (the encoder), or a learned table in place of four tuned terms.
- The reverse direction cancels less than forward.
