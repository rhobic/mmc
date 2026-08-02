# Six-step commutation and back-EMF sensing

The second control methodology in this project, alongside FOC. This is the
theory: what six-step does, why the idle phase can measure the rotor, where the
30° rule comes from, and which of those claims the bench has already tested.

Code: [`mmc-core/src/sixstep.rs`](../crates/mmc-core/src/sixstep.rs) (control),
[`mmc-sim/src/phase_motor.rs`](../crates/mmc-sim/src/phase_motor.rs) (machine
model), [`mmc-sim/src/sixstep_rig.rs`](../crates/mmc-sim/src/sixstep_rig.rs)
(the two joined, and the regression tests).
Milestone context: [PLAN.md](PLAN.md) MS8. Bench results: [PROGRESS.md](PROGRESS.md).

## 1. What it is, against what FOC is

FOC treats the machine as a continuously steerable current vector. It measures
all three phase currents, rotates them into the rotor frame, regulates `i_d`
and `i_q` with PI loops, and rotates the result back out to three duty cycles.
It needs a rotor angle every control tick, which is why most of this project so
far has been about producing one — a flux observer, an I-f startup, a profiler
to feed them parameters.

Six-step gives up the continuous vector. At any instant exactly two phases
conduct and the third is open. There are only six such states, and driving the
machine means stepping through them in order. The current vector therefore
jumps in 60° increments instead of rotating smoothly, which is the source of
both its main drawback and its main advantage:

|                     | FOC                              | Six-step                          |
| ------------------- | -------------------------------- | --------------------------------- |
| Rotor angle needed  | every tick, continuous           | six times per electrical rev      |
| Where angle is from | observer or encoder              | the idle phase measures it        |
| Torque ripple       | low                              | inherent, ~14% on a trapezoidal machine |
| Per-tick cost       | two Park/Clarke pairs, two PIs, SVPWM | a table lookup                    |
| Phase currents      | all three, simultaneously        | one, and only its magnitude       |
| Fails by            | losing the angle estimate        | mistiming a commutation           |

The last row is the honest summary. Both schemes need to know where the rotor
is; they differ in whether that knowledge comes from a model you maintain or a
measurement you take.

## 2. Why the idle phase can be read

This is the whole basis of sensorless six-step, and it is a three-line
derivation.

Write each phase as a resistance, an inductance and a back-EMF source, all
referred to the star point `v_n`:

```
v_x = v_n + R·i_x + L·di_x/dt + e_x        for x in {u, v, w}
```

Let `hi` and `lo` be the conducting phases and `f` the open one. The conducting
pair carries equal and opposite current (`i_hi = i, i_lo = −i`), so summing
their two equations cancels every resistive and inductive term:

```
v_hi + v_lo = 2·v_n + e_hi + e_lo
```

The open phase carries no current at all, so its terminal is just the star point
plus its own back-EMF: `v_f = v_n + e_f`. Substituting:

```
v_f = (v_hi + v_lo)/2 − (e_hi + e_lo)/2 + e_f          (★)
```

That is exact, for any back-EMF shape. **The idle terminal is an affine image of
the machine's own back-EMF, and the offset is set entirely by what the bridge is
doing.** No model, no parameters, no integration — this is why six-step can be
sensorless with arithmetic a 1980s microcontroller could do.

`(★)` is implemented directly as
[`PhaseMotor::idle_terminal`](../crates/mmc-sim/src/phase_motor.rs), and the
test `idle_terminal_follows_the_sensing_identity` checks it holds in all six
sectors.

### The two special cases, and why the flat top exists

The `−(e_hi + e_lo)/2` term is where machine design enters.

**Sinusoidal machine.** The three back-EMFs sum to zero, so
`e_hi + e_lo = −e_f`, and `(★)` collapses to:

```
v_f = (v_hi + v_lo)/2 + 1.5·e_f
```

**Trapezoidal machine.** An ideal 120°-flat back-EMF does *not* sum to zero — it
carries a third harmonic, and the sum is a triangle wave at three times the
electrical frequency (test: `trapezoidal_bemf_sum_is_pure_third_harmonic`). But
it doesn't matter, because of where the sectors sit: throughout each 60° window,
the two conducting phases are parked on *opposite flat tops*, so
`e_hi + e_lo = +1 − 1 = 0` exactly (test:
`trapezoidal_conducting_pair_cancels_across_each_window`), and:

```
v_f = (v_hi + v_lo)/2 + e_f
```

Lower gain than the sinusoidal case — 1.0 instead of 1.5 — but the third
harmonic is cancelled by construction rather than assumed away. That is what the
flat top is *for*. Any scheme that assumes the back-EMFs sum to zero (a
resistor-network "virtual neutral", for instance) inherits that harmonic as a
timing error on machines that have one.

Either way, the crossing of `(v_hi + v_lo)/2` happens exactly when `e_f` crosses
zero. That crossing is the measurement.

### Which is why the sample point decides everything

`(v_hi + v_lo)/2` is not a constant — it depends on where in the PWM period you
look:

- **High side conducting** (`v_hi = V_bus`, `v_lo = 0`): the idle terminal
  swings about **`V_bus/2`**, a reference the firmware already measures. The
  whole waveform stays between the rails.
- **Freewheel**, both driven legs low (`v_hi = v_lo = 0`): the idle terminal
  swings about **0**, so half of every revolution sits *below ground*.

That second case is where a low-side shunt must sample phase current, and it is
where this project's ADC trigger already was. On hardware, half of the waveform
was clipped away by the sense network's clamp diodes and the surviving half
carried no speed information at all — measured over a 15× speed range in MS8
step 1 (see [PROGRESS.md](PROGRESS.md)).

Both behaviours are reproduced in simulation, from `(★)` and a diode clamp
alone, by a matched pair of tests:

- `freewheel_sampling_with_a_clamped_network_cannot_lock` — asserts the drive
  **fails**. If it ever starts passing, the model has stopped describing the
  hardware.
- `on_time_sampling_rescues_the_same_clamped_network` — same machine, same
  clamp, sample moved into the on-time: tracks >95% of the time.

## 3. Why commutation happens 30° after the crossing

Each sector spans 60° electrical, centred on the rotor angle where its energised
pair produces maximum torque. For the sinusoidal shape the pair `U+ V−` gives
torque proportional to `s_u − s_v = −√3·sin(θ + 30°)`, maximised at
`θ = −120°`; that is [`SECTOR0_CENTRE`](../crates/mmc-core/src/sixstep.rs), and
the sector runs ±30° about it (test: `table_picks_max_torque_pair`).

Now ask where the idle phase's back-EMF is zero. In sector 0 the idle phase is
W, whose shape is zero at `θ = −120°` — the centre of the window (test:
`idle_phase_crosses_zero_at_window_centre`). So:

```
crossing at window centre  →  30° before the next commutation
crossings recur every 60°  →  30° is half a crossing interval
```

**The commutation delay is half the time between the last two crossings.** No
angle estimate, no speed loop, no trigonometry — a stopwatch. That is the entire
timing law, and it is why six-step needs so little computation.

Speed comes free from the same measurement: one interval is exactly 60°, so
`ω_e = (π/3)/T`. It is a direct measurement, not a model output; its noise is
the timing jitter of the crossings and nothing else.

## 4. The two things that actually go wrong

### Flyback, and why blanking is the critical constant

When a phase opens, its current does not stop. It keeps flowing through a
freewheel diode until the trapped energy is gone, which pins the terminal to a
rail for roughly `L·|i| / V_bus`. During that time the terminal says nothing
about the rotor, and it says it *loudly* — a rail-to-rail edge that a naive
detector reads as a crossing, commutating early, which opens a phase with more
current in it, which lengthens the next flyback. That runaway is the classic
six-step failure.

The fix is to ignore the terminal for a fixed time after each commutation
([`ZcCfg::blank`](../crates/mmc-core/src/sixstep.rs)). It is the most
consequential number in the whole scheme, and it is squeezed from both sides:

- too short, and flyback is mistaken for a crossing;
- too long, and at high speed the blanking swallows the crossing itself, because
  the window shrinks as `1/ω` while the flyback does not.

That trade-off is what sets the top speed of a simple six-step drive. The model
reproduces it: `PhaseMotor::commutate` starts a real freewheel, and
`blanking_rejects_post_commutation_transient` drives rail-to-rail noise into the
detector inside the blanking window and requires that nothing is detected.

### Startup, where there is nothing to measure

Back-EMF is proportional to speed, so at standstill there is no signal and the
scheme has no input. The drive has to guess: impose a commutation sequence and
accelerate it slowly enough to drag the rotor along
([`Ramp`](../crates/mmc-core/src/sixstep.rs)). This is the same bargain the I-f
startup in `sensorless.rs` makes for FOC, and it fails the same way — under load
the rotor slips behind, and the imposed sequence pulls out.

The handoff to sensing is the interesting part. The implementation runs the
detector *during* the ramp, watching without obeying, so that by the time the
ramp reaches handoff speed the crossings have usually already been consistent
for several sectors and the transfer is uneventful. If they have not, the timer
is seeded from the forced frequency and control is handed over anyway — a real
drive has no better option than to try.

## 5. Torque ripple is not a defect

Six-step's current vector is fixed within a sector while the rotor turns through
60°, so the angle between field and rotor sweeps ±30°. On a trapezoidal machine
torque per amp is flat across the window and the ripple is modest; on a
sinusoidal machine it varies as `cos` across the sector, giving about 14%
peak-to-peak.

This is inherent to the topology, not a tuning failure. Both bench motors here
are sinusoidal machines, so six-step on this hardware is expected to be rougher
and slightly less efficient than FOC — the interesting comparison is what it
costs *per tick* to get that, which is the point of the whole methodology.
`sinusoidal_machine_also_runs` covers this case explicitly.

## 6. What is proven where

| Claim | Where it is checked |
| --- | --- |
| Table energises the maximum-torque pair in every sector | `table_picks_max_torque_pair` |
| One full revolution commutates exactly six times, in order | `sectors_advance_monotonically_with_rotation` |
| Idle phase's back-EMF is zero at its window centre | `idle_phase_crosses_zero_at_window_centre` |
| Identity `(★)` holds in all six sectors | `idle_terminal_follows_the_sensing_identity` |
| Trapezoid's conducting pair cancels across each window | `trapezoidal_conducting_pair_cancels_across_each_window` |
| Commutation lands 30° after the crossing; speed recovered | `detector_times_commutation_thirty_degrees_after_crossing` |
| Flyback inside the blanking window is rejected | `blanking_rejects_post_commutation_transient` |
| Start, lock, hold speed against sim truth | `starts_locks_and_holds_speed`, `measured_speed_matches_truth` |
| Commutation stays within one sector of truth, incl. load step | `commutation_tracks_the_rotor`, `holds_sync_through_a_load_step` |
| Freewheel sampling + clamp **fails**; on-time sampling fixes it | `freewheel_sampling_with_a_clamped_network_cannot_lock`, `on_time_sampling_rescues_the_same_clamped_network` |
| Per-phase Hi-Z is real on hardware | bench, MS8 step 1: idle phase 2 mA vs ±485 mA driven |
| Freewheel sampling carries no speed information on hardware | bench, MS8 step 1: flat over a 15× speed range |

Everything above the line is simulation; the last two rows are the bench. The
one claim that is **modelled but not yet measured** is that moving the sample
into the PWM on-time fixes it on real hardware. That is the next bench step, and
the simulation exists so that it is the only thing left to find out.

## 7. Keeping the two schemes apart

`mmc-core` gates each methodology behind a cargo feature (`foc`, `sixstep`, both
on by default). They share only the genuinely common parts — `math`, `pi`,
`transforms`, `tuning`, `angle` — so either builds alone:

```sh
cargo build -p mmc-core --no-default-features --features sixstep
cargo build -p mmc-core --no-default-features --features foc
```

That split is the low-resource scaling story from the project overview made
concrete: a target too small for FOC can carry six-step alone, and the shared
commutation table means firmware and simulator cannot drift apart on
commutation order or sector alignment.
