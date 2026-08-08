# mmc — progress log

Newest first. Every session that changes the project appends here: what landed,
what was decided, what's next. The stable plan lives in [PLAN.md](PLAN.md).

## 2026-08-08 — session 28: the dead-time measurement, built and validated against the sim

**Bench is set up with the rotor locked (same motor 1), so this session built
the one measurement that needs exactly that condition — and validated the
whole pipeline against a simulator with a known answer before it goes near
hardware.**

### New profiler stage: `vdead`

A DC voltage ladder at ω_e = 0, ascending then descending, fitting
`v = R·i + v_dead·sign(i)`. No new firmware — it drives the existing
`OpenLoopVoltage` mode at zero frequency, which is what makes it available
today.

The shape has more structure than "a line with an intercept", and that is
what makes both parameters identifiable. A DC vector at θ = 0 puts
`i_a = i_d` but `i_b = i_c = −i_d/2`, so the legs sit at different points on
their own sign curves, and Clarke gives

```
e_d = (2/3)·[f(i_d) + f(i_d/2)],   f(x) = clip(x/i_thresh, −1, 1)
```

which bends **twice** — phase A saturates at `i_thresh`, phases B/C only at
`2·i_thresh`. Fitting is a scan over `i_thresh` with a 2-parameter linear
solve at each (numpy only; scipy is not a dependency), so there is no initial
guess to get wrong.

**Validated against `mmc-host serve --deadtime`, three controls:**

| truth | fit | model-free cross-check |
|---|---|---|
| `0.12, 0.5` | 118.6 mV, 0.497 A | **120.0 mV, R 0.904 Ω** |
| `0.30, 0.35` | 300.0 mV, 0.350 A | **300.0 mV, R 0.904 Ω** |
| ideal | **"none measurable"** | — |

The cross-check is model-free — a straight-line fit whose intercept reads
`v_dead` without the shape function — and it lands exactly. R comes back at
0.904 Ω against the sim's 0.904.

**A hardware constraint the sim surfaced before the bench could.** The 1.5 A
trip caps the ladder at about `2·i_thresh` on this motor, so the *fully
saturated* regime is out of reach and the cross-check has to run between the
knees, where the line is `[R + v_dead/(3·i_thresh)]·i + (2/3)·v_dead`. The
fit handles both regimes and reports which one it used.

**Two safety behaviours, because a locked winding has no rotation to carry
heat away.** The ladder sizes itself from the device's own `R` (0.7·I_trip),
and *predicts* each rung before commanding it — verified by lying to the sim
(`r = 2.7` against a true 0.904): it stops before the rung that would have
drawn 1.37 A, rather than after. The descending branch is the thermal
control: copper gains 0.39%/°C and a warming winding fits a resistance that
was never true at any single point, so if the two branches disagree the fit
says so.

### Two more sim/hardware divergences closed

**`apply` could not target the sim at all** — serial only, while `profile` and
`capture` both take `--addr`. That breaks the project's own "same host tooling
against sim and hardware" invariant. It now takes `--addr`.

**The sim rebuilt its parameter table on every connection.** So `apply` wrote,
verified, printed success — and the table reverted the moment the tool
disconnected. A `profile → fit → apply → capture` loop against the sim quietly
ran on defaults. The table now lives with the *device* (the `serve` loop), as
it does in firmware RAM; motor and controller state still restart per session,
which is correct. This is the third divergence of the same family in two
sessions, after the speed-gain regression and the 10-vs-13 param table.

**Operational note worth recording:** the sim server paces to wall-clock time,
so a loaded host degrades its captures — one busy run turned a 0.1 mV fit
residual into 3.0 mV and pulled `v_dead` to 96 mV. Idle, the same ladder
repeats bit-identically across fresh servers. Check the load before the code.

**Verification:** 76 workspace tests, clippy clean, fmt clean, `cargo check`
inside `crates/mmc-fw-g474/`.

**Next, on the locked bench:** run `--only rl,vdead` (in that order — the
ladder wants a fresh R), compare the ladder's R against the probe's as the
check that the extra term is real, then enable `Foc::deadtime` and re-measure.
Also available while the rotor is clamped and still open from session 9: the
**saliency re-clamp-45° confirmation**, which needs a rotor held at a *chosen*
angle — real saliency rotates with the rotor, a stator-locked gain artefact
does not.

## 2026-08-08 — session 27: back to FOC — two live bugs, and the sim grows the low-speed physics

**Six-step paused while the bench hardware is reviewed. This session is all
sim and core: two silent defects found by reading the FOC path end to end,
then the model work that has to land before any low-speed control work can be
believed.**

### Two live bugs, both silent

**1. The FOC speed loop has been running on six-step's gains since session
20.** `0882ae7` added the six-step duty→speed loop and, in the same diff,
changed the *sensorless FOC* block from `SPEED_KP`/`SPEED_KI` (param ids 4–5)
to `SS_KP`/`SS_KI` (ids 11–12). That is precisely what session 20's own note
said must never happen — and what `mmc-proto`'s doc comment on those ids still
says. Consequences:

- `tools/profile.py` fits `speed_kp`/`speed_ki` from J and kt, `mmc-host
  apply` writes them, the firmware verifies the read-back — and then never
  reads them. **MS6's speed-gain autotune has been a no-op for six sessions.**
- The loop instead ran the six-step defaults: **6× low on kp and 23× low on ki
  for motor 1** (fitted 0.00115/0.0115 at the profiler's 40 rad/s design
  bandwidth, against 2.0e-4/5.0e-4), and ~45× low on motor 2.
- Tuning either scheme silently retuned the other.

Fixed. Worth re-reading session 23's sensorless numbers with this in mind —
they were taken on a badly under-gained speed loop.

**2. The sim's parameter table had drifted three entries behind the
firmware's.** `sim_params` carried 10 values against `param::COUNT == 13`, so
its own `debug_assert_eq!` fires in a debug build, and `ontime_ccr5`/`ss_kp`/
`ss_ki` were NAK'd against the sim while the device accepted them. Restored to
13 with the firmware's defaults and ranges.

**Why no test caught either: the sim server never used the param table for its
speed loop.** It computed gains from the model directly, so `apply` →
behaviour was untested by construction. The sim now reads
`SPEED_KP`/`SPEED_KI` from the table like the firmware does, seeded from the
model so existing scenarios are unchanged.

### The sim's rotor was 50× too light at low speed

The profiler measures a **Coulomb friction torque** (`T_fric = kt·i_q_fric`,
0.78 mN·m on motor 1) and `PmsmParams` had nowhere to put it — only `viscous`.
So the virtual rotor coasted on viscous drag alone and drew **1.5 mA** holding
100 rad/s el where the real one draws **79 mA**. Any low-speed study on that
model was studying the wrong machine.

`PmsmParams::coulomb` added, applied as a velocity decrement that cannot
reverse the rotor (standstill is a fixed point, and stiction comes free).
`bench_g474` carries the measured 0.78 mN·m; `small_bldc` stays frictionless,
so every MS2–MS4 regression and every committed scenario is untouched.
**Cross-check: the sim now settles at i_q = 0.084 A against the bench's
measured 0.078 A.**

### Dead-time voltage error: modelled, and it is not what it looks like

New `mmc-core::inverter::DeadtimeModel` — one model, two users, the same split
the commutation table got: the simulator's inverter **subtracts** it, `Foc`
**adds it back** before modulating. `FocOutput.v_ab` deliberately stays the
*uncompensated* vector, because that is what lands on the winding once the
bridge takes its cut, and that is what the observer must integrate.

The interesting part is what the model says about *this* rig. `rs` already
absorbs every resistive drop in the drive path (0.885 Ω against a ~0.1 Ω
winding), so what is left is the `sign(i)` term — and the R/L probe cancels it
exactly, by differencing across folded edges where the current sign never
changes. It has never appeared in a fit. But on 30 µH at 40 kHz the ripple is
~1.2 A pk-pk, so the drive lives *inside* the zero-current band at its
0.08–0.5 A operating currents, and there the error is linear in current:

> **On this rig dead time does not look like zero-crossing distortion. It
> looks like 0.24 Ω of apparent resistance the observer does not know about —
> ~25% on top of a fitted 0.885 Ω.**

Which changes the prediction, and the sim confirms it quantitatively:

| ω_e [rad/s el] | flux/ψ ideal | flux/ψ, dead time | predicted bias `ΔR·|i|/ω` |
|---|---|---|---|
| 40 | 0.994 | **1.554** | +0.564 |
| 60 | 0.998 | 1.379 | +0.376 |
| 100 | 0.999 | 1.226 | +0.226 |
| 300 | 1.000 | 1.078 | +0.075 |
| 600 | 1.000 | 1.062 | +0.038 |

A current-aligned voltage error integrates to a flux perturbation along **+d**
— *parallel* to the rotor flux. So it inflates the **magnitude** and barely
touches the **angle** (rms 0.0095 vs 0.0073 rad at 40 rad/s el). Both halves
matter:

- **The angle surviving is why sensorless FOC works as well as it does on an
  uncompensated bridge.** "Dead time is why low speed is hard" is retired as
  the primary story.
- **The stall detector is the casualty.** It trips below 0.35·ψ; at 40 rad/s el
  the flux reads 1.55·ψ, so a real stall would have to drag the estimate
  through a 1.2·ψ offset before the detector noticed. **It goes blind exactly
  where stalls happen.**

Compensation returns the ideal-bridge result to within 0.01·ψ at every speed,
and a half-calibrated `v_dead` removes about half the bias and nothing worse.
`Foc::deadtime` stays `None` until `v_dead` is measured — over-compensating is
the dangerous direction.

### The observer compensated its angle for the leak, but not its magnitude

Falling out of the above: the leaky integrator's `jω/(jω+leak)` costs
`|ω|/√(ω²+leak²)` of magnitude — **11% at 40 rad/s el** on the default 20 rad/s
leak — and `flux_mag()` reported the raw integral. The angle has been
lead-compensated analytically since MS4; the magnitude never was. So the one
number used as a health indicator read *low* in proportion to how slowly the
drive was running, which is backwards for a low-speed stall detector.

`flux_mag()` now divides the attenuation back out (clamped at cos(π/4), the
same place the angle compensation clamps); `flux_mag_raw()` keeps the value
the PLL normalizes by. Measured across 40–600 rad/s el: **0.889→0.994 at the
bottom of the range, and within 2% of ψ everywhere.** The stall threshold now
means the same thing at every speed; a stall reads ≈0.01·ψ here, so the margin
to 0.35 is untouched.

**Verification:** 76 workspace tests (7 new), clippy clean, `cargo check` +
clippy inside `crates/mmc-fw-g474/`, and `mmc-core` builds on thumbv7em and on
thumbv6m under `foc`-only and `sixstep`-only. Nothing here has been on
hardware — it is all model and core work, staged for the bench.

**Next:** measure `v_dead`, which needs no new firmware — locked rotor,
`OpenLoopVoltage` at ω_e = 0, sweep v_d and fit `v = R·i + v_dead`: the
*intercept* is `v_dead` and the knee width is `i_thresh`. Then the rest of the
MS9 list in [PLAN.md](PLAN.md).

## 2026-08-03 — session 26: the six-step review — and first lock on the small motor

**A hostile review of every six-step conclusion, prompted by the fact that
hobby-ESC practice runs six-step on exactly this class of motor. The full
review with all evidence is [SIXSTEP-REVIEW.md](SIXSTEP-REVIEW.md); the
outcome: closed-loop sensorless six-step locked at ~865 rad/s el on the small
motor — the motor session 25 declared incapable of it.**

The chain, compressed — each item overturned a prior conclusion:

1. **Sector rate is not rotor speed on a forced drive.** A coast-tail check
   showed the rotor at 771 rad/s against a 1200 rad/s commutation clock. All
   high-speed forced-six-step claims measured the clock; the high-speed
   sensing data was taken on a slipping rotor and meant nothing.
2. **The ramp trips were pull-out, not commutation transients** — the
   feedforward's voltage-budget error starves current at speed, torque
   collapses, and the desync surge (0.9→1.47 A across ~15 windows) trips OC.
   No hunting line in the envelope spectrum.
3. **The low-speed floor is real; the high-speed floor was fiction.** On a
   *synchronized* rotor (carried by FOC), the idle-phase ramps at 898 rad/s
   are textbook — ±1.5 V about the measured mid, crossing mid-window.
4. **Live FOC → six-step handover shipped (fw v15)** — sector seeded from the
   observer's angle (conventions verified by a deliberate-π contradiction
   test), detector seeded with ω̂, duty preloaded; plus a guard that stops
   any unsupported live mode switch from dereferencing uninitialized state
   (previously a silent panic-halt). Host: `capture --step-kind/--step-amp`.
5. **First failure tracked 5 sectors then lost the one whose crossing arrived
   inside the 250 µs blank** (21% of a window). Demag here is 2.5 µs. At
   **30 µs blanking: lock in 6 ms, held 8 s, zero faults, reproduced.**
   Settled current 0.03–0.14 A — torque balance, same economy as FOC.

ISR worst case at 40 kHz PWM: idle 580 · FOC 4118 · sensorless 4247 ·
six-step ≤ 4247, of 8500. The f_sw question the review set out to answer is a
split verdict (good for ripple and measurement fidelity, actively bad for the
low-speed sensing floor); the table is in the review doc. Captures in
`testresults/ms8-handover/`; the forced sweep legs at 597/898 in
`ms8-speed-sweep` now carry unverified-speed caveats in their metadata.

**Next:** adaptive blanking in core with sim coverage, then the lock-band map
and the honest closed-loop-vs-closed-loop comparison.

## 2026-08-03 — session 25: PWM decoupled from control; six-step still blocked, but for a different reason

**PWM now switches at 40 kHz while the control loop still ticks at 20 kHz.**
`PWM_ARR` halved and the ADC interrupt, which arrives every PWM period, drops
every second conversion (`PWM_DIV`). Every gain, slew rate and tick counter
downstream keeps the timebase it was tuned for. Measured on the bench:
**control loop 20005 Hz**, telemetry unchanged. ADC1 stays on CC4 at the
counter peak — dividing instead via the repetition counter and TRGO was
rejected because in centre-aligned mode the update event lands on the
*underflow*, i.e. the on-time, which is exactly the position that caused the
current-sense regression two sessions ago.

Two follow-on corrections came with it: the `ontime_ccr5` upper bound now
tracks `PWM_ARR/2` rather than being a fixed 2000 counts, and the on-window is
half as long in absolute time, so six-step back-EMF sensing needs **duty ≳
0.10** at 40 kHz where 0.07 sufficed at 20 kHz.

**A measurement bias surfaced.** At identical commanded voltage the open-loop
current draw rose 1.47× (0.474 → 0.697 A per volt) purely from doubling the
switching frequency. True average current cannot depend on switching frequency
— same resistance, same back-EMF — so the 20 kHz reading was biased low.
Halving the ripple halved the bias, which is consistent with the sample sitting
off the centre of the ripple triangle. The R/L probe is unaffected (R 0.887 vs
0.885 Ω, L 0.029 vs 0.030 mH, and *tighter* error bars) because it takes a
differential across folded edges, where a ripple-position bias cancels. This
also explains the standing puzzle of the driver faulting while telemetry showed
modest current: the drive was under-reading, and the hardware protection was
seeing the real thing.

**Six-step got much closer and is still blocked.** With the feedforward ramp at
40 kHz the ramp holds 0.35–0.5 A and reaches **1641 rad/s el**, against ~1200
before, with applied duty tracking the feedforward prediction exactly (0.194
against 0.194). But it still faults short of handoff, and the reason is not
average current: the trips are **spikes of 1.2–1.36 A that do not scale with
the target** — dropping the ramp current from 0.45 A to 0.30 A left them
unchanged. That is commutation transient, not ripple. At each commutation
`di/dt = V_bus/2L` = 200 A/ms on this 60 µH series pair, and a faster PWM does
nothing to that: the transient is set by bus voltage and inductance alone.

**Conclusion: this motor cannot run six-step on this hardware.** 30 µH at 12 V
puts commutation transients into the 1.5 A protection regardless of switching
frequency or commanded current. The 4-pole motor is 377 µH — 12.5× the
inductance, so 12.5× smaller transients — and 20× the flux, so its back-EMF
clears the sense noise floor as well. It is the right machine for six-step, and
that is why the MS8 results happened on it. The PWM work stands on its own
merits: halved ripple, a corrected current measurement, and headroom for any
low-inductance motor.

## 2026-08-03 — session 24: what actually blocks six-step on this motor

**It is not the zero-cross detector. It is PWM ripple current, and the sample
point has been hiding it the whole time.**

**Two firmware changes landed.** The six-step ramp applied `amp` as a flat duty
from standstill, where there is no back-EMF to oppose it and the whole bus
voltage lands on the winding — so the duty needed to reach a high handoff is
the same duty that trips overcurrent on the way there. The ramp now feeds
forward the duty its commanded speed implies, `(ψ·ω + i·2R)/V_bus`, clamped to
`amp` as a ceiling. Measured on the bench: duty climbs 0.123 → 0.213 across the
ramp and holds current near 0.75 A where a flat duty sat at 1.3 A from rest.

The second is a bug this exposed. `f32::clamp` **panics when min > max**, and
the ramp branch runs from the first tick of the mode — one tick where the
amplitude has not landed yet puts the ceiling below the 0.01 floor, and a
panicking `no_std` firmware simply halts. It did: the device stopped answering
the serial port until reset. Both this and the pre-existing occurrence in the
speed-loop branch are now non-panicking `max`/`min`.

**The real blocker.** This motor is 30 µH. Six-step conducts two phases in
series, so at 20 kHz the ripple current is `V_bus·d·(1−d)/(2L·f_sw)`:

| duty | ripple pk-pk | sampled | true peak |
|---|---|---|---|
| 0.10 | 0.90 A | 0.45 A | 1.35 A |
| 0.16 | 1.35 A | 0.62 A | 1.97 A |
| 0.20 | 1.60 A | 1.18 A | 2.78 A |
| 0.35 | 2.28 A | 0.45 A | 2.73 A |

**The telemetry sample sits at the counter peak — the middle of the freewheel,
which is the ripple minimum.** The true peak is a full ΔI above it and has
never been visible in any capture. That is why the gate driver faulted at
1.12 A "measured": the actual peak was about 2.6 A, well past the 1.5 A limit.
Every earlier reading of "current" in a six-step run is a ripple trough, not an
average and certainly not a peak.

This also retires the earlier suspicion of the detector's 200 µs blanking as
the high-speed cause. The detector genuinely cannot work at low speed on this
motor — back-EMF there is 0.1–0.3 V against a 1–3 V artefact floor, and the
idle-phase signal *shrank* as speed tripled, which back-EMF cannot do. But the
high-speed path never got far enough to test the detector at all, because the
bridge faults first.

**What it would take.** Ripple scales as `1/f_sw`, so the fix is a faster PWM:
about 32 kHz to bring the true peak under the limit at 0.5 A, 40–60 kHz for
margin. Control cannot simply move with it — the FOC ISR is 4061 cycles and a
40 kHz tick allows 4250, which is no margin at all — so PWM and control have to
decouple, triggering the ADC every other PWM period. That is a change to the
same timer and ADC trigger configuration that produced the current-sense
regression two sessions ago, so it is being raised rather than assumed.

## 2026-08-03 — session 23: motor swapped back, refit, and a real speed sweep

**The bench motor was changed back to the small one because OCP kept blocking
the work. Re-identifying it first turned out to matter more than expected: it
invalidates a chunk of what MS8 recorded.**

**What was in flash was neither motor's truth.** The stored blob decoded to
R 1.056 Ω, L 377 µH, flux 18.7 mWb, **2 pole pairs** — the 4-pole motor,
written by a 10-parameter firmware. The current 13-parameter build CRC-fails
that blob and silently falls back to compiled-in defaults, so the device had
been running defaults, not the stored fit, for some time.

**Refit of the small motor** (`mmc-host profile`, then `tools/profile.py`):

| | fitted now | previously on record |
|---|---|---|
| R (drive path) | 0.8847 Ω | 0.97 Ω |
| L | 30.15 µH | 28 µH |
| flux | 0.937 mWb ±0.025 (5/5 points) | 0.894 mWb |
| J | 1.980 µN·m·s² | 1.75 µN·m·s² |
| kt | 9.839 mN·m/A | — |

Applied and persisted; the flash blob is now valid for the 13-parameter build.

**Correction to MS8: the six-step characterisation was done on the 4-pole
motor, not this one.** Back-EMF is `ψ·ω`, so the "40 rad/s el sensing floor"
recorded for six-step corresponds to about **0.75 V** — which needs the 4-pole
motor's 18.7 mWb. This motor has **20× less flux**, so the same 0.75 V would
need roughly 800 rad/s el. Every speed-referenced six-step number in the MS8
notes is specific to that motor and does not transfer.

**Six-step closed loop does not lock on this motor at all.** Forced
commutation is fine: it ramps to **898 rad/s el and holds it indefinitely**,
with current falling 0.94 → 0.72 A as back-EMF builds. Closed loop, same duty,
same speed, climbs the same ramp to 646 rad/s and then **collapses to ~40 rad/s
within half a second** of handoff, free-running on the timeout fallback and
never reporting lock (`ST_SL_RAMP` → `ST_SS_UNLOCKED`). It collapses to the
same ~40 rad/s for **every** commanded handoff from 31 to 898 rad/s, so the
outcome is independent of the command. Not a back-EMF magnitude problem — at
646 rad/s el this motor produces ~0.6 V. Diagnosing further needs 20 kHz
visibility the 1 kHz telemetry cannot give; the detector's fixed 200 µs
blanking against a 34 µs electrical time constant is the first suspect.

### Three schemes across 100–900 rad/s electrical

Six-step duty trimmed per speed so the current vector matches the FOC legs at
~0.53 A, and I-f commanded at 0.500 A — so the two forced legs differ only in
modulation:

| ω_e [rad/s el] | six-step \|i\| ripple | I-f FOC \|i\| ripple | ratio | six-step jitter |
|---|---|---|---|---|
| 101 | 5.9 % | 0.7 % | 8.4× | 2.1 % |
| 302 | 15.4 % | 1.2 % | 12.8× | 3.9 % |
| 597 | 29.4 % | 1.9 % | 15.5× | 13.8 % |
| 898 | 42.6 % | 3.0 % | 14.2× | 20.3 % |

**Sinusoidal modulation is an order of magnitude smoother, and the gap widens
with speed** — six-step ripple grows 7× across the range while FOC's grows 4×
from a far lower base, and six-step's commutation jitter grows 10×.

**The closed-loop result is the more interesting one.** Sensorless FOC holds
the same speeds on about **0.08 A against the forced legs' 0.50 A** — roughly
6× less current for the same mechanical job — because a closed speed loop
supplies only the torque friction actually demands, while a forced leg pushes
whatever it was told to. That is a much stronger argument for closed-loop
sensorless control than ripple is. Its low-speed limit is the 150 rad/s
handoff: at 101 rad/s el the observer never converges and the run stalls.

**Two measurement limits, stated rather than buried.** The 6×-electrical
ripple is well resolved only at the low end — 25 samples per ripple period at
101 rad/s, 8.3 at 302, but **4.2 at 597 and 2.8 at 898**, where the figures are
lower bounds. Both schemes are sampled identically at each speed, so the ratio
column survives even where the absolute values do not. And the sensorless leg's
ripple percentage is **not** comparable to the 0.5 A legs: at 0.078 A the
0.020 A noise floor is 26 % of the signal, so that number is noise, not torque.

**Next:** the six-step lock failure is now the blocking item for a like-for-like
closed-loop comparison, and it needs on-device instrumentation rather than more
captures — log the detector's crossing decisions in the ISR and stream those.

## 2026-08-03 — session 22: the FOC blocker was ours, not the bench

**Last session's blocker was a regression we shipped, and the diagnosis in that
entry was wrong.** No scope was needed, the motor and the driver were fine, and
it did not predate the six-step work — it arrived with it.

**Method: A/B the firmware, not the theory.** The last pre-six-step build
(`92fb226`) was checked out into a detached worktree, flashed, and driven with
the *identical* host command as the current build — same bench, same bus, same
session, so the binary is the only variable. Protocol drift between the two is
purely additive (one telemetry channel, three params, two drive codes), and the
firmware clamps `mask & channel::ALL` and bounds-checks param ids, so a current
host talks to the old firmware unmodified.

Open-loop voltage, 1.5 V commanded at 8 Hz electrical, duties identical at
0.500 ± 0.090 in every run:

| firmware | phase current RMS |
|---|---|
| `92fb226`, pre-six-step (fw 7) | **0.7125 A** |
| `89fc095`, current (fw 14) | **0.0035 A** |
| current + this fix | **0.7108 A** |

**Cause: `984ba81` moved ADC1's trigger along with ADC2's.** That commit's own
message and its inline comment both state that ADC1 keeps CC4 at the counter
peak so current sensing is untouched — but the diff set `jextsel(8)`
(TIM1_TRGO2, inside the PWM on-time) on *both* ADCs. ADC1 carries the phase
shunts. Low-side shunts only conduct at the counter peak, so sampling them
inside the high-side on-time reads zero no matter what the bridge is doing.

**The current was always flowing; the measurement was blind.** That is why I-f
faulted: the current loop saw zero, wound up, and drove real current until the
gate driver's own overcurrent protection tripped. Nothing in our firmware was
limiting it — worth remembering when a current reading looks impossibly clean.

**Fix:** ADC1 back to `jextsel(1)` (tim1_cc4); ADC2 stays on TRGO2, which is
what the on-time work actually needed. One line, plus a comment saying why the
two ADCs must not share a trigger.

**Verified on hardware:** open-loop voltage back to 0.7108 A, matching the old
firmware to 0.15%; I-f at 0.3 A regulates `i_q` to **0.3004 A** and holds five
seconds with no fault, where it previously tripped in ~50 ms; six-step closed
loop still commutates (sectors cycling 0–5, BEMF nodes swinging the full bus),
so the fix costs nothing that `984ba81` was for.

**This invalidated committed results.** `ms8-compare/if.csv` was a fault trace,
not a comparison — 0.0037 A at state 3 (`fault_drv`) throughout — and
`six.csv` was sampled at the wrong point in the PWM period. Both legs are
re-taken below. The ISR cycle counts from session 21 stand: that arithmetic ran
regardless of the values it ran on.

### MS8 step 5 completed: the torque-quality half

**Torque is matched by construction, so current is a result, not a knob.** Both
schemes hold the same steady speed against the same unloaded rotor friction, so
their average torque is equal by definition. That makes the honest comparison
"same speed, same job — what does each scheme spend, and how clean is it?"

The operating point is set by the rig, not by preference: six-step cannot go
faster (duty 0.18+ trips the 1.5 A limit), and **closed-loop sensorless FOC
stalls here** — at 52 rad/s electrical the back-EMF is only about **47 mV**, so
the flux observer never converges and the run ends in `ST_STALL`. The FOC leg
therefore has to be forced-angle I-f, which is what session 21 had planned.

Matched at 52.4 rad/s electrical, currents matched within 4 % for equal copper
loss (`--drive if --amp 0.23` against `--drive six-cl --amp 0.17`):

| metric | six-step closed loop | I-f current FOC |
|---|---|---|
| settled speed | 52.42 rad/s el (8.34 Hz) | 52.34 rad/s el (forced) |
| phase current RMS | 0.1688 A | 0.1754 A |
| current-vector \|i\| mean | 0.2207 A | 0.2308 A |
| **\|i\| ripple, sd/mean** | **27.5 %** | **8.4 %** |
| \|i\| ripple, peak-to-peak | 136.3 % | 36.8 % |
| commutation jitter | 19.5 % sd | n/a (angle is forced) |
| ISR cost (session 21) | 2653 cyc | 4061 cyc |

**FOC's current vector is 3.3× smoother for the same speed and the same RMS
current.** That is the trade the milestone was built to measure: six-step buys
its 35 % cheaper ISR with 3.3× the current ripple. The six-step figure is
structural, not noise — |i| collapses and rebuilds through every commutation,
which is what the 136 % peak-to-peak swing is. Cross-checked against 2.4 kHz
reduced-mask captures (27.5 % and 7.9 %), so it is not a sampling artefact; the
idle measurement noise floor is about 0.02 A peak, ~2 % of these means.

**Six-step lock is not reliable at this handoff.** Five starts on the identical
command: four capture lock at 52.34–52.42 rad/s (jitter 19.1–19.5 %, current
0.169 A — repeatable to three digits), and **one collapses to 17.3 rad/s** with
41 % jitter and 0.798 A, the documented below-floor signature of mistimed but
periodic commutation. The handoff at 50.3 rad/s is only 1.26× the measured
40 rad/s sensing floor. The committed capture is a locked run, and the first
archival attempt was a collapsed one — worth knowing that a single capture of
this drive proves nothing about the drive.

**Caveat carried forward:** with no rotor-angle sensor, I-f sync cannot be
confirmed from telemetry — the current loop regulates its magnitude whether or
not the rotor is following, and at this speed the back-EMF is too small to
settle it. That is MS7's job.

## 2026-08-02 — session 21: MS8 step 5 — ISR cost measured; FOC path blocked on hardware

**One axis of the comparison completed, one blocked by a hardware fault that
needs a scope.**

**Found first: the flux observer was running inside six-step**, fed a zero
voltage vector, producing a meaningless estimate. It is now gated off for both
six-step modes (fw v14) — one phase current is unmeasured by construction and
the applied vector is not a rotating one, so there is nothing for it to
estimate. Worth 842 cycles a tick, and it is most of why six-step costs less
than FOC.

**ISR worst case by scheme** (DWT counter at `ISR_MAX_CYCLES`, 170 MHz, 20 kHz
loop, 8500-cycle budget):

| scheme | cycles | µs | % of tick | marginal over idle |
|---|---|---|---|---|
| idle, no drive | 586 | 3.45 | 6.9% | — |
| FOC current loop (I-f) | 4061 | 23.89 | 47.8% | 3487 |
| six-step closed loop | 2653 | 15.61 | 31.2% | 2079 |

**Six-step costs 35% less per tick, and 41% less in control-specific work.**
Worth stating plainly: that is a good deal less than the "two Park/Clarke pairs,
two PIs and SVPWM versus a table lookup" framing implies. Fixed overhead — ADC
reads, protection checks, the 24-channel telemetry snapshot — is 586 cycles
before any control runs, and this six-step implementation is unoptimised float
arithmetic with divides in the crossing timer.

**BLOCKER: every FOC-path drive mode produces essentially no phase current at
12 V, while six-step runs normally.** Measured, not inferred:

- open-loop voltage (SVPWM), 1.5 V commanded: duties correct at 0.500 ± 0.108
  across all three phases, a **2.48 V** differential, `v_d` exactly 1.500 —
  and **0.004 A**.
- I-f: same, and then faults `fault_drv` after ~50 ms, which is the current loop
  winding up against current that never arrives.
- six-step, same session, same rig: **0.221 A**, runs clean.
- Bus steady at 12.04 V throughout, no sag. MOE set, all three enables high,
  `BDTR = 0x8000`.

Ruled out along the way: it is not an aftereffect of a six-step run (reproduced
from a clean reset), not supply sag, not the modulator, and **not the new CC5
trigger** — TIM1 `CCR5 = 0x64`, `CCMR3 = 0x68` (OC5 in PWM mode 1 with preload)
read back correctly once read at the right addresses. *Note for next time: this
PAC puts CCR5 at offset 0x48 and CCMR3 at 0x50, not the 0x58/0x54 the reference
manual numbering suggests — reading the wrong addresses cost a detour.*

FOC has not been run since the supply was 25 V, so this may predate today
entirely. Next step is physical: scope the three phase outputs under SVPWM to
see whether the bridge is switching at all, and check the motor connection and
the driver's own state.

**Next:** unblock the FOC path, then finish step 5 with the torque-quality half
of the comparison (current and speed ripple at matched electrical speed).

## 2026-08-02 — session 20: six-step regulates speed on device; the back-EMF sensing floor measured

**Closed the loop twice over: the simulation now covers the reference fix, and
the device regulates speed rather than duty.**

**Simulation gap closed.** `PhaseMotor` gained a `Bridge` — high- and low-side
conducting resistances (the latter including the shunt) and the sense network's
full scale. Their effect on *current* is deliberately not added to the circuit
equation, because `PmsmParams::rs` is already the whole drive-path resistance
the profiler measures; what they change is where the driven terminals sit, and
therefore what the correct reference is. Session 19's hardware finding is now a
paired regression test:

| reference | tracking | speed error | final |
|---|---|---|---|
| `V_bus/2` | 49.7% | 507% | 51 rad/s el |
| measured mid-point | 100% | 1.42% | 594 rad/s el |

Modelling the drops also **reproduced, unprompted, the constraint that forced
the supply down to 12 V**: with a fixed divider ratio a 24 V bus pushes the
driven terminals past the sense full scale, so the mid-point cannot be measured
and the reference has nowhere to fall back to. The simulator found that on its
own, from the drops and a clip.

**Speed loop on device (fw v13).** Duty→speed PI closed on the crossing
interval, with back-calculation anti-windup because a six-step bridge has no
braking quadrant. Gains are **dedicated params `ss_kp`/`ss_ki` (ids 11–12)**,
deliberately *not* the FOC loop's `speed_kp`/`speed_ki` — the two schemes must
not share a tuning knob. New state code 9 distinguishes commutating-but-unlocked
from confident sensing.

**Result: it regulates, above a floor.**

| target | held | error | duty | current | locked |
|---|---|---|---|---|---|
| 44.0 rad/s el | 47.0 | +6.8% | 0.155 | 0.219 A | 100% |
| 50.3 rad/s el | 52.3 | +4.1% | 0.169 | 0.221 A | 100% |
| 31.4 rad/s el | 16.3 | −48% | **0.170 (pinned)** | **0.931 A** | 100% |
| 18.8 rad/s el | 16.5 | −12% | **0.170 (pinned)** | **0.928 A** | 100% |

**There is a back-EMF sensing floor at roughly 40 rad/s electrical on this rig,
and it is the expected physics, not a defect.** Above it the loop keeps
authority (duty unsaturated) and holds target within ±7% at 0.22 A. Below it the
drive parks at ~16.5 rad/s whatever the command, with duty pinned at the ceiling
and **four times the current** — the signature of mistimed commutation, because
`e ∝ ω` and the ramp across a window shrinks with speed until it no longer
outweighs the residual reference error. The detector reports `locked` throughout,
which is itself worth knowing: lock is not the same as correct.

The forced ramp exists precisely to cross this region, so the handoff speed must
be set above the floor.

**Next:** MS8 step 5, the FOC-vs-six-step comparison on one motor and one
profile — torque ripple, acoustics, ISR cost.

Captures: `testresults/ms8-closedloop/`, `testresults/ms8-sixstep-sim/`.

## 2026-08-02 — session 19: sensorless six-step runs closed-loop on hardware

**Commutation is now timed by the motor's own back-EMF.** Bus dropped to 12 V,
which bought both a wider PWM on-window and the torque headroom the 1.5 A limit
had been denying.

**The blocker was the reference, not the sense network.** A sample-point sweep
settled it: `ontime_ccr5` became a runtime parameter (id 10) so the ADC trigger
could be moved over the wire, and with the rotor held still the idle terminal
reads **6.73 V at every sample point from 0.18 to 1.18 µs before the valley** —
flat to ±0.02 V. The network settles fine; the RC-settling hypothesis is dead.

What the sweep exposed instead was a **constant +0.70 V offset from V_bus/2**.
With no rotation there is no back-EMF, so that is pure artefact:

| | |
|---|---|
| v_hi (driven high) | 12.26 V |
| v_lo (driven low) | **1.60 V**, not 0 — switch + shunt drop at ~1 A |
| measured mid | 6.93 V |
| V_bus/2 | 6.03 V |
| idle vs measured mid | **−0.20 V** (correct) |
| idle vs V_bus/2 | **+0.70 V** (artefact) |

The back-EMF ramp across a window is ~1.5 V at 10 Hz el, so a 0.70 V reference
error is half the signal — enough to keep one sector parity permanently on one
side of the threshold. **That is the parity asymmetry from session 18,
explained and fixed:** compare against the measured `(v_hi + v_lo)/2` instead,
where the drops cancel identically. Theory updated in [SIXSTEP.md](SIXSTEP.md).

**Closed loop then worked (fw v11).** Commanding the same 15 Hz handoff at four
different duties, the drive settles at four different speeds — which a forced
drive could not do:

| duty | el rev/s | ω electrical | current |
|---|---|---|---|
| 0.08 | 2.95 | 18.6 rad/s | 0.214 A |
| 0.11 | 4.80 | 30.1 rad/s | 0.209 A |
| 0.14 | 6.58 | 41.4 rad/s | 0.215 A |
| 0.17 | 8.37 | 52.6 rad/s | 0.221 A |

Speed is linear in duty (11.3 ± 0.15 rad/s el per 0.03 duty) at roughly constant
current, which is the signature of a voltage-fed machine self-commutating
against a friction load. Ten-second runs, no faults.

**Next:** a duty→speed loop on the device, then MS8 step 5, the FOC-vs-six-step
comparison. The sim should also grow the switch/shunt drop so this failure mode
is covered by a regression test rather than only by this log.

Captures: `testresults/ms8-closedloop/`.

## 2026-08-02 — session 18: on-time sampling on hardware — back-EMF is there, closed loop is not yet

**The MS8 step 1 finding is superseded: the idle phase now carries back-EMF.**
Firmware v9 moved the idle-phase sample into the PWM on-time; v10 added
closed-loop commutation. Bench, motor 2, V_bus 25.16 V, duty 0.07.

**The trigger (fw v9).** TIM1 CH5 is not routed to a pin, so it is free to
place a second ADC trigger: OC5 in PWM mode 1 with `CCR5 = 100` counts puts
OC5REF's rising edge just before the counter valley, i.e. inside the high-side
on-time. `CR2.MMS2 = OC5REF` publishes it as TRGO2 and ADC2's `JEXTSEL` moves
from 1 (TIM1_CC4) to 8 (TIM1_TRGO2). **ADC1 keeps CC4 at the counter peak**,
where the low-side shunts carry phase current, so current sensing is untouched.

**A timing constraint had to be solved first.** ADC2's four conversions at 47.5
cycles take 5.65 µs and *cannot* fit the on-window, which is only 3.5 µs at duty
0.07. Dropping to 6.5 cycles gives 1.79 µs, which fits with margin — and the
divider's ~1.8 kΩ Thevenin source still settles in about eleven time constants.

**It works, and the identity is confirmed on hardware.** The idle phase now
parks at **12.85 V measured against the predicted V_bus/2 = 12.58** — where in
step 1 it sat near ground. Back-EMF is visible with the correct alternating
slope: at 10 Hz el the odd (rising) sectors ramp **+1.53 V against a predicted
1.5·e = 1.77 V (86%)**, and the even sectors ramp negative as the detector
expects.

**Two honest limits, both measured:**
- **Only about half the sectors carry a clean ramp.** Odd sectors track the
  prediction; even sectors are weak (−0.45 V vs −1.77 at 10 Hz) and at 2–5 Hz
  every even window hits a rail. Not yet understood.
- **Closed loop does not track.** Mode 6 ramps, hands off, then commutates at
  ~1.9 el rev/s (~12 rad/s el) on the timeout fallback rather than on real
  crossings, drawing 0.81 A with the rotor barely turning. The front end is not
  yet clean enough to time commutation.

**Rig headroom bit again:** the first closed-loop attempt at duty 0.10–0.12
tripped overcurrent within 23 ms (1.5 A limit). Everything here runs at 0.07.

**An aggregate metric misled me and is worth recording.** Peak-to-peak swing per
window *fell* with speed (13.8 V at 2 Hz → 0.32 V at 30 Hz), which looks like
"no back-EMF" — but it was dominated by rail excursions from flyback and ADC
clipping at low speed, not signal. Measuring the *monotonic ramp* within each
window, split by sector parity, shows the opposite and correct picture. The
shape matters; the summary statistic lied.

**Note on ADC range:** at the on-time sample point the driven high phase sits at
V_bus → 4.54 V at the ADC input and saturates. That is harmless and expected —
only the idle phase is meaningful, and the crossing sits at mid-scale, so
clipping the extremes cannot affect zero-cross timing. The idle phase's own
positive peak clips above ~203 rad/s el (32 Hz) for the same reason.

**Next:** find why one sector parity is weak — the first suspects are the ADC
sequence position within the on-window (each conversion samples 0.45 µs later
than the last) and charge injected by the clamp diodes during the long off-time
being held on the divider. Then retune blanking and retry closed loop.

Captures: `testresults/ms8-ontime/` (speed sweep) and
`testresults/ms8-closedloop/`.

## 2026-08-02 — session 17: MS8 steps 2–3 — six-step in simulation, and the theory written down

**The control concept is now built, tested and documented; the one thing left
to learn is on the bench.** [SIXSTEP.md](SIXSTEP.md) carries the derivations.

**`mmc-core::sixstep`** — commutation table, sector mapping, back-EMF
zero-cross detector with blanking and the 30° timer, and a forced-commutation
startup ramp. Speed falls out of the crossing interval (`ω = (π/3)/T`) as a
measurement, not a model output. 8 unit tests, including one that proves the
table energises the maximum-torque pair in every sector and one that proves the
idle phase's back-EMF is zero at its window centre — the assumption the whole
30° rule rests on.

**`mmc-sim::phase_motor`** — a phase-domain machine model, because the dq model
assumes all three phases are driven, which is exactly what six-step breaks.
Carries one line current, exposes the idle terminal voltage, and supports both
sinusoidal and trapezoidal back-EMF.

**The sensing identity, derived and implemented exactly:**
`v_f = (v_hi + v_lo)/2 − (e_hi + e_lo)/2 + e_f`. Two things fell out that were
not obvious going in:
- **An ideal 120° trapezoid does not sum to zero** — it carries a third
  harmonic, so the tidy `1.5·e_f` form is the *sinusoidal* special case, not a
  general law. Anything assuming the sum vanishes (a resistor virtual neutral)
  inherits that harmonic as timing error.
- **The flat top cancels it anyway**, because through each 60° window the
  conducting pair sits on opposite flat tops, so `e_hi + e_lo = 0` exactly and
  `v_f = mid + e_f`. That is what the flat top is *for* — a nicer result than
  the one being replaced.

**The A/B that justifies the next hardware change**, same machine, same clamp,
only the sample point differing:

| sample point | lock | tracking | speed error | final speed |
|---|---|---|---|---|
| freewheel (today's trigger) | never | 50.6% | 324% | 94 rad/s el |
| PWM on-time | 0.305 s | 100% | 1.52% | 592 rad/s el |

Two paired tests enforce it: the freewheel case asserts the drive **fails** (if
it ever passes, the model has stopped describing the hardware) and the on-time
case asserts it works.

**Control-loop defects found and fixed by the sim**, both real rather than
cosmetic: the duty loop's integral corner sat *above* its proportional
crossover (guaranteed hunting), and the external duty clamp wound the
integrator up on every deceleration. Retuned to a 9:1 corner ratio with
back-calculation anti-windup; both machines now settle within 1.5% of target.

**Firmware corrected by the shared code.** The core's alignment test showed the
firmware's own sector formula was offset by 30° electrical — it used `θ + π`
where the max-torque alignment is `θ + 5π/6`. Firmware now calls
`sixstep::sector_of` and indexes `sixstep::TABLE`, so firmware and simulator
cannot drift apart on commutation order or alignment again.

**Clean separation of methodologies.** `mmc-core` gates `foc` and `sixstep`
behind cargo features (both default). Either builds alone, and a new CI job
proves it — including six-step-only on thumbv6m, which is the low-resource
scaling story made concrete.

**Captures:** `testresults/ms8-sixstep-sim/` — trapezoidal and sinusoidal
baselines, a load step, and the sample-point A/B pair. 60 runs on the dashboard.

**Next:** the only unmeasured claim is that moving the sample into the PWM
on-time fixes it on real hardware. That needs the second ADC trigger (TIM1
CC5/CC6 so the current sense keeps its counter-peak trigger) and is the next
bench session.

## 2026-08-02 — session 16: MS8 step 1 — the idle phase is NOT readable at the current sample point

**Answered the question that gates six-step, on hardware, with a measurement
rather than an argument.** Firmware v8 adds forced six-step commutation (drive
mode 5, `SetDrive` wire code 5 — 4 is the R/L probe) and `stage_phases(mask)`,
so two phases conduct and the third is Hi-Z.

**What works: per-phase Hi-Z is real.** Static commutation, sector 3 (V+ / U-,
W idle): driven pair carries ±0.485 A, the idle phase carries **2 mA**. The
enables do exactly what six-step needs.

**What does not: the idle phase carries a signal, but it is not back-EMF.**
Swept 2 → 30 Hz electrical at duty 0.07 (12.6 → 188.5 rad/s el, 15×) and the
idle-phase reading is **flat at 1.8–2.1 V, then falls to 1.26** — 0.68× over a
range where back-EMF must rise 15×. Anchored at the slowest point, the
prediction at 30 Hz is 28 V against 1.26 measured. Whatever the channel reads is
set by current, not speed: the resistive drop of the conducting pair plus the
divider bias.

**Why, and it is the predicted reason.** ADC2 triggers at TIM1_CC4, the
counter peak, because that is where every low side conducts and the shunts carry
the phase currents. At that instant both driven terminals are at ground, so the
idle terminal is referenced to ~0 and its negative half is clipped by the
divider's clamp diodes. The window shapes show it directly: at 5 Hz, sectors
1/3/5 carry a hump peaking mid-window (1.63–1.86 V) while sectors 0/2/4 sit
flat at 0.00 — exactly half the windows rectified away.

**Two secondary findings:**
- **No torque headroom on this rig.** Duty 0.10 at 15 Hz tripped overcurrent
  (peak 1.44 A against the 1.5 A limit), so the sync-authority control could not
  be run. Forced commutation loses the rotor somewhere above 15 Hz el, which
  contaminates the top of the sweep independently of the sensing question.
- **1 kHz telemetry cannot resolve a commutation window.** 82 samples/window at
  2 Hz, 6 at 30 Hz. Detection has to live in the 20 kHz ISR; telemetry is only
  for showing the work afterwards.

**Next:** move the idle-phase sample into the PWM on-time, where the star point
sits near V_bus/2, the idle terminal swings about a reference the firmware
already measures, and the clamps never conduct. TIM1 has spare compare channels
(CC5/CC6), so the current sense keeps its counter-peak trigger. That is the one
change that converts this negative result into a working zero-cross front end.

Captures: `testresults/ms8-sixstep/` (7 speeds + the static Hi-Z proof).

## 2026-08-02 — session 15: session 14 committed; a build break it had been hiding; MS8 scheduled

Short session: pick up two weeks cold, land the pending work, pick a direction.

**Session 14 had never been committed** — 10 files, +353/−23, hardware-verified
on 2026-07-20, sitting in the working tree ever since. Committed now.

**It did not compile.** A duplicate `use embassy_stm32::flash::Blocking;` at
file scope (already imported at line 39) — E0252, left behind by end-of-session
tidying *after* the hardware verification. One line deleted; `cargo check` and
clippy clean on `thumbv7em`.

**Why nothing caught it, which is the actual lesson:** session 14 reported "all
41 workspace tests pass" and that was true and irrelevant — **`mmc-fw-g474` is
its own workspace**, not in the root `members`, so `cargo test --workspace`
never compiles the firmware. CI *does* build it (the `firmware-g474` job), but
CI only runs on push, and the work was never pushed. So the one gate that would
have caught it was bypassed by the same omission that left the work uncommitted.
**Rule going forward: firmware changes are not "verified" until
`cargo check` runs inside `crates/mmc-fw-g474/` (or the commit is pushed).**
Green workspace tests say nothing about either firmware crate.

**Direction chosen: MS8 (six-step) before MS7 (encoder).** MS7 is blocked on
hardware — there's no rotor-angle sensor on the bench, and neither an encoder
nor the motor 2's hall sensors are wired to the shield's connector yet. Six-step
needs nothing new: the BEMF front-end was characterized in session 13 and
per-phase Hi-Z is a `stage_phases(mask)` split of the existing EN handling.
Both milestones are now written up in [PLAN.md](PLAN.md), including the
first bench experiment (is the floating phase readable at the *current* ADC2
sample point?) and the `CMD_MODE` 4 numbering trap.

## 2026-07-20 — session 14: two bench papercuts killed — VCP retry + flash persistence

**Cleared the two recurring hardware-session frictions.** Both had cost real
time across the last several sessions (each ~3–4 retry cycles / ~4 by-hand
param re-entries).

**1. VCP connect retry (`mmc-host/src/link.rs`).** The host used to open the
debug-probe VCP immediately after `probe-rs reset`, before the device finished
booting/re-enumerating — the "no response to 0x01 / TimedOut" that forced a
manual retry every reset. `Link::serial()` now retries open + ping-until-Pong
over a 6 s deadline (200 ms cadence), printing "waiting for device…" once.
Verified: reset + immediate connect now succeeds in ~0.7 s where it used to
time out.

**2. Flash parameter persistence (fw v7 + host/panel/proto/sim).** Runtime
params lived only in RAM and reset to firmware defaults on every reflash/
power-cycle. Now they persist:
- **Firmware** `nvparam` module: CRC32'd blob (magic "MMCP", version 1) in the
  **last page of flash bank 2** (0x0807F800). Boot does a plain memory-mapped
  read, CRC-checks, and **range-validates each value against `param_range`**
  before accepting it — a corrupt/stale/absent blob is ignored and defaults
  load, so a bad save can never brick startup. Save/erase use embassy blocking
  `Flash` owned by `rx_task`; the page erase (~22 ms) runs on bank 2 while the
  control ISR keeps executing from bank 1 (**read-while-write**). Save/erase
  are **gated on a quiet stage** (drive off, not calibrating, no burst) and NAK
  otherwise.
- **Protocol**: `SaveParams` (0x0B) / `EraseParams` (0x0C) messages, ack/nak.
- **Host**: `apply --persist` saves after applying; panel parameter card gains
  **Save to flash** / **Erase flash** buttons.
- **Sim**: acks both as no-ops (RAM sim has nothing to persist).

**Verified on hardware (both directions):** set motor 2's table → Save →
`probe-rs reset` → reconnect → params read back as motor 2's values (not
defaults) = **PERSISTED CORRECTLY**. Then Erase → reset → reverted to firmware
defaults (r=1.0, pp=7, handoff/accel/iq=150/500/0.8). Bench left with motor 2's
params saved in flash. All 41 workspace tests pass, clippy clean.

**Deferred/next:** MS7 encoder remains the main path; the saliency
gate/two-position differential (motor 1 re-clamp-45° confirmation) is still
open. Six-step trapezoidal is the natural BEMF follow-on from session 13.

## 2026-07-20 — session 13: BEMF terminal-voltage sensing wired + characterized

**Wired the inverter shield's populated-but-unused BEMF divider network and
learned what it can (and can't) do.** fw v6: GPIOC + PC9 (divider enable, low),
ADC2 injected on the same TIM1_CC4 trigger as ADC1 (parallel, zero cost to the
control-loop timing budget — no ISR), 4 conversions PC0–PC3, scaled by the
12.2/2.2 divider ratio into three new telemetry channels `vb_u/vb_v/vb_w`
(ids 18–20) with a matching panel chart. Sim reports the model's ideal EMF
(clean bipolar sine, verified = 2·ψ·ω).

**Empirical findings (the point of the session):**
- **Pin mapping resolved on hardware** (the schematic was ambiguous):
  BEMF1=U→PC0, BEMF3=W→PC1, **BEMF2=V→PC3** (not PC2 — PC2 is the SPEED pot,
  which read a railed 18.3 V and gave it away). Confirmed in firmware.
- **All three phases track BEMF in coast, ∝ ω_e**: spin sensorless, cut to
  Hi-Z, pk-pk = 4.3–5.3 V at 150 rad/s el and 9.5–10.0 V at 300 (2× speed →
  2× amplitude, clean).
- **The decisive finding — this network is a *zero-cross / coast-down*
  instrument, not a live terminal-voltage sense.** The shield's Schottky clamp
  diodes rectify the signal (reads 0..peak, not bipolar), and under
  PWM the channels are dominated by the switched rail (~1.7 V mean, useless).
  **So the original goal — feed the observer *measured* phase voltage during
  drive to lower the sensorless floor / kill the 7.8% inverter-drop error —
  is NOT achievable with this shield.** That needs filtered or in-line
  terminal-voltage sensing the inverter shield doesn't provide. What this hardware
  *does* give: commutation/spin visualization, a coarse coast-down flux
  cross-check, and the front-end for six-step BEMF zero-cross (its actual
  design purpose).
- **Coast ψ cross-check**: pk-pk/(2·ω) at 300 rad/s el = 16.7 mWb vs the
  profiled 18.7 — ~11% low, consistent with the ~0.3 V Schottky drop plus
  divider loss. A sanity check, not a precision source (ψ is already measured
  better two ways).

**Deferred/next:** six-step trapezoidal drive is now the natural follow-on —
the BEMF zero-cross front-end it needs is wired and characterized. The
"BEMF as observer input" idea is closed as hardware-limited on this shield.

## 2026-07-19 — session 12: sensorless generalized — motor 2 closes the MS6 loop

**Closed-loop sensorless runs on motor 2, first attempt, and its profile is
complete.** The blockers were three compile-time constants sized for motor 1;
they are now runtime params (ids 7–9): `sl_handoff`, `omega_accel` (drives
the forced-mode slew, the sequencer ramp, and retargets — and the accel-J fit
now reads the value actually used from the capture snapshot), and `iq_limit`
(replaces both I_AMP_MAX and SL_IQ_LIMIT). Firmware bumped to v5. motor 2
settings: 100 / 250 / 1.0 A.

**Closing proof, MS6-style, on the second motor:** profile → fit → apply →
sensorless on its own numbers: starts cleanly, holds 150 rad/s el, live
retarget to 250, and a 150→300 speed step lands with **2.0% overshoot,
settling at 299.6**. Accel stage (150→350 el at 0.9 A via the panel) fit
**J = 25.1 µN·m·s²** — the datasheet's 23 (-116 variant) plus ~9% coupling —
and friction 14.8 mN·m; fitted speed gains kp=0.0090/ki=0.090 applied.
(First fit read J=12.6: the fit assumed the old fixed 500 rad/s² slew while
the device ran the new param's 250 — exactly why the snapshot records it.)

**Robustness landed with it:**
- **Stall fault (state 8)**: in closed-loop sensorless, 100 ms of observer
  flux below 0.35·ψ — the stalled rotor's confident L·i fake-lock, the
  MS4/MS5 lesson — trips the stage off; STOP re-arms. Verified in sim: a
  `--locked` rotor faults ~100 ms after "handoff" (state history
  6→7→1×50ticks→8), a healthy run never trips. Panel shows the fault chip.
- **Adaptive blend-kick taper**: the ramp's hang angle measures the load
  (load fraction = **cos γ** — the sign convention that also corrected motor
  2's drag estimate from ~30 down to ~9–12 mN·m standstill), and the blend
  tapers i_start toward measured-load + 30% (floor 0.4): light loads kill
  the handoff kick, heavy loads keep full current. A fixed 0.5 taper failed
  the heavy-load regression corner (worst error 0.72 rad); the adaptive one
  passes the whole sweep — the corner where load ≈ 95% of i_start tapers
  not at all.
- Speed-loop preloads follow the actual taper (`Sequencer::taper_end()`),
  keeping the takeover bumpless in fw, sim server, and rig.
- `StageTuning.accel_amps` (CLI `--accel-amps`, panel `"accel_amps"`) — the
  accel stage's startup current was the last hard-coded 0.5 A.

41 workspace tests green (the taper change initially broke the
speed×load sweep and the fix made the taper *smarter*, not weaker).

## 2026-07-19 — session 11: at-the-motor R/L normalization + meter entry

The profiler's R is the drive path (winding + shunt + FETs ≈ +0.85 Ω on this
rig) — right for control, wrong for humans holding a multimeter. Now both
directions convert:

- **`R_DRIVE_PATH = 0.85 Ω`** lives in `mmc-host/src/profile.rs` (0 for the
  sim, whose model has no inverter resistance) and is recorded into each
  capture's device snapshot, so the fits normalize old, sim, and hardware
  captures correctly from one source of truth.
- `profile.py`/`saliency.py` print **"At the motor"** estimates (per-phase and
  line-line, wye assumed) beside the drive-path values; the panel's parameter
  card shows the same live line.
- The panel gains a **"from a meter"** entry: type line-line R/L measured at
  the motor terminals, it sets r/l with the conversion (r = R_ll/2 + path,
  l = L_ll/2). Validation both ways on the motor 2: displayed estimate
  0.41 Ω line-line vs datasheet 0.35–0.45; entering the datasheet's
  0.35 Ω/1.0 mH produces r within 3% of the profiled value.
- Bench note: the device power-cycled again and lost its RAM params (restored
  by hand) — the flash-persistence backlog item keeps earning its place.

## 2026-07-18 — session 10: profiler output neutralized; motor 2 matched to its datasheet

**Motor 2 was matched to a catalogue part** (datasheet listed in
[../hw/README.md](../hw/README.md); likely the longer-body variant by the L
comparison — check the 94 vs 116 mm body to confirm). Full
profiler-vs-datasheet comparison in
`testresults/motor2-4pole/datasheet-comparison.md`. Highlights:

- **Pole pairs = 2 confirmed twice**: the datasheet says 4 poles, and the kt
  cross-check agrees — 1.5·pp·ψ = 56.2 mN·m/A vs the datasheet's 63 (block
  convention, −11%); pp=4 would read +78%. ψ also predicts ≈3700 RPM at 36 V
  vs the 4000 RPM rating.
- **The R "discrepancy" closes exactly**: profiler R = 1.047–1.057 Ω vs
  datasheet winding 0.175–0.225 Ω. The gap is the drive path — gate driver
  conducting switch ≈0.5 Ω (R_DSon HS+LS = 1 Ω typ, verified from
  see hw/README.md) + 0.33 Ω low-side shunt (duty-weighted ≈0.32) — summing
  to 0.99–1.05 Ω predicted. The profiler's R is what the control loop sees;
  correct for control, ~5–6× a datasheet winding figure by construction.
  `profile.py` now says so in its summary.
- L: 0.377 mH measured vs 0.50 (-116) / 0.70 (-94) per phase — -116 within
  typical tolerance. J not measured (accel stage blocked by the fixed
  sensorless handoff + ~30 mN·m bench drag); rotor-only speed-gain seeds from
  the datasheet J live in `testresults/motor2-4pole/datasheet-derived.json`
  (apply-able). Hand-tuned gains found live on the device (kp=0.018 ≈ 2.2×
  the rotor-only seed — consistent with coupled load inertia) were left as-is;
  the measured R/L/flux were (re)applied around them.
- Rig context: the shield's 1.5 A limit drives this 5–6.7 A motor at ≤25%
  rated current, hence the near-90° hang angles under ~10%-of-rated drag.

**Also landed:** profiler fit output is now *neutral* — `saliency.py` reports
the measured Lq/Ld, ξ ± σ (with significance) and the R/L/pole-pairs/schedule
actually used, instead of USABLE/NOT-USABLE application verdicts; the
unusable-capture path says "measurement invalid", not a motor claim.
`profile.py` prints the applied-parameter summary (with units) after fitting.
`testresults/panel-profile/` (the panel's default scratch output) is
gitignored; the datasheet PDF was renamed to the `hw/` convention.

## 2026-07-17 — session 9: panel-driven profiler + second motor (4-pole) profiled

**A second, very different motor (4 poles ⇒ 2 pole pairs, rotor free) was
profiled end-to-end from the control panel** — and it stress-tested every
assumption the profiler inherited from the first bench motor. Final numbers:
**R = 1.047 Ω, L = 0.377 mH (τ = 360 µs = 13× motor 1), ψ = 18.74 ± 0.37 mWb
(≈21× motor 1), kt = 56.2 mN·m/A, saliency |ξ| = 0.036 ± 0.004 → Lq/Ld ≈ 1.07
(AMBIGUOUS — real but below the 0.05 INFORM line, unlike motor 1's 1.16)**.
Closing validation on applied params: observer tracks I-f at **0.1% error**,
mech-speed telemetry correct for p=2, R̂ tile reads +3% ≈ +7 °C.

**Landed:**
- **`pole_pairs` is runtime param id 6** (fw v4): scales the omega_m telemetry
  and the host kt/J fits; `profile` snapshots the device param table into
  `profile_state.json` so `profile.py` tracks the connected motor. Panel param
  card row added; R̂ uses it. (Sim `sim_params` array grown — the predicted
  SetParam-panic trap.)
- **Profiler runs from the panel**: a Profiler card (stage checkboxes → Run →
  live log → Apply) backed by `Cmd::RunProfile`/`ApplyProfile`; the shared
  `profile::run_stages` engine serves CLI and panel; Python fits shell out
  automatically and stream into the card. Telemetry freezes during a run and
  stale queued commands are dropped after.
- **τ-adaptive saliency schedule**: half-period picked from live L/R
  (8/16/32/64 ticks, cycle count keeps the burst exactly full), post-edge fit
  samples picked from the recorded header; `saliency.py` now declares
  **MEASUREMENT INVALID** (with remedy) instead of a false "not usable" when
  plateaus don't settle. Sequencing rule: rl → apply → saliency.
- **Per-motor excitation (`StageTuning`)**: `--rl-volts`, `--sweep-points
  a@w,…`, `--accel-targets lo,hi` (also via panel API); accel targets recorded
  in state for the fit. Closes the "lift hard-coded excitation" backlog item.
- **Direct back-EMF ψ estimator** in `profile.py` when R/L are known
  (e = (v_d+ωL·i_q, v_q−R·i_q), |e| = ωψ): the hang-angle joint fit is
  ill-conditioned when the motor hangs near π/2 (this one: ~78° — heavy
  friction), where the direct method gave σ = 0.4 mWb vs the joint fit's 12.
  `fit_params.POLE_PAIRS` threaded from the device snapshot.
- **R̂ hang-angle fix**: EMF power projected with cos θ_err — in forced-frame
  I-f a heavily-hanging rotor absorbs only the aligned EMF component; the
  uncorrected tile read −0.4 Ω on this motor, corrected +1.075 (+3%).

**Motor-2 lessons (all now documented/handled):** from-rest I-f sync is
ramp-torque-limited (0.3 A stalled; 0.6 A holds to ~60 rad/s el, 0.9 A to
~90); the ψ ≈ 19 mWb ceiling ω_max ≈ 0.7·(VBUS/√3)/ψ ≈ 240 rad/s el makes
the old default sweep speeds physically unreachable; first saliency attempt
ran with stale R/L and the validity detectors caught the unsettled plateaus
exactly as designed.

**Deferred:** the `accel` stage (and sensorless generally) on motor 2 — the
sensorless startup's handoff speed (150 rad/s el), ramp slew, and startup
current are firmware constants sized for motor 1; motor 2 needs them as
runtime params to reach a reliable handoff. Backlogged in PLAN.md.

## 2026-07-17 — session 8: staged profiler + saliency probe — the bench motor IS salient

**Headline: the zero-speed-sensorless gate came back OPEN.** The new saliency
probe, run on the real motor with the rotor clamped, measures
**|ξ| = 0.0737 ± 0.0004 → Lq/Ld ≈ 1.16 — verdict USABLE** — against the
session-7 analysis's prediction of ≈1.0 (surface BLDC). Likely
saturation-induced saliency. Every validity check is clean: j=1/j=2 fits agree
within 1% while the transient amplitude changes 3.7× (a gain artifact cannot
track the exponential like that), even/odd cycle splits agree to 0.3%, θ_r
stable at −3.6° el, plateau-step spread 1.2%. *Confirmation still recommended:*
re-clamp ~45° el away and rerun — real saliency rotates with the rotor.
A same-day corroboration: the R/L probe read L = 36 µH vs MS6's 28 µH —
expected, since a clamped rotor defeats its self-alignment so it measured an
arbitrary d/q mixture.

**Landed:**
- **Staged, stateful profiler** (`mmc-host profile`): named stages
  (`sweep`/`accel`/`rl`/`saliency`) each declaring what it measures and what
  the bench must provide (free-spinning vs parks-rotor — no stage needs a
  mechanical clamp); `--list`, `--only`, `--redo`, `--yes`, `--addr` (sim);
  completed stages tracked in `profile_state.json`; enforced ordering
  (spinning stages before parking probes — the separatrix lesson); NAK codes
  translated to human-readable reasons. Runbook: [PROFILER.md](PROFILER.md).
- **Saliency probe** end to end: `test::L_THETA` (fw v4), shared schedule in
  `mmc-core/src/probe.rs` (16 ±paired angles × 8 interleaved cycles × 32-tick
  blocks in ONE 205 ms burst — ± pairing cancels net torque so a free rotor
  only dithers ~1° el; interleaving turns thermal R drift into common mode),
  (i_d, i_q) recorded in the excitation frame behind a self-describing header
  (+8 f32 on the burst buffer). The i_q transient is a null channel — it
  exists only if Ld ≠ Lq. Firmware clamps the sweep voltage to
  0.75·I_trip·R̂ using the live R param.
- **`tools/saliency.py`**: rising−falling folding (kills offsets/pedestals),
  joint linear LSQ over both channels for (P, Q, θ_r), and the exact
  latency-cancelling estimator ξ = −u/(ln cosh u − ln P), u = atanh(Q/P) —
  the ±30% absolute-L systematic cancels in ξ identically. Free validity
  checks: ΔI-vs-angle spread (R is isotropic), even/odd-cycle θ_r drift,
  j-consistency. Verdict thresholds 0.05/0.02 with a 3σ noise floor.
- **Sim server runs both probes** (was: NAK) with firmware-identical
  schedules; `--motor bench` (28 µH τ<Ts regime), `--saliency <ratio>`,
  `--locked`. **Controls pass: 1.5 → fit 1.484 (free rotor, ~1° dither);
  1.0 → NOT-USABLE.** RL_STEP is sim-testable for the first time: recovers
  R = 0.904 / L = 28 µH exactly.
- **`tools/profile.py` fits partial captures** — missing stages skip with a
  pointer instead of aborting; partial profile.json is safe (`apply` skips
  absent keys).
- **Panel: R̂ apparent tile** — (v·i − ω_e·ψ·i_q)/|i|² over ~1 s of the
  telemetry already streamed, with ΔT from copper's 0.39%/°C. Winding
  thermometry with zero firmware cost; exact at standstill, ψ-sensitive at
  speed; includes the inverter drop (deliberately: it's the R the control
  loop actually sees). Amber/red past +12%/+25%.

**Verified:** 41 workspace tests green (6 new schedule tests); sim positive +
negative controls; hardware run on the clamped bench motor (fw v4 flashed;
first attempt timed out because `probe-rs download` left the core halted —
`probe-rs reset` fixed it, now in PROFILER.md troubleshooting).

**Consequences:** zero-speed sensorless torque is *physically available* on
this motor (16% saliency), pending the re-clamp confirmation. Productizing it
still needs di/dt sampling during active vectors (in-line shunts or clever
windowing) and INFORM-style estimation — X/R ≈ 2 at Nyquist rules out
rotating-carrier injection. Encoder (MS7) remains the main path; this probe
is the INFORM primitive if zero-speed sensing is ever pursued.

## 2026-07-13 — session 7: live parameter editing in the control panel

**The panel (`mmc-host panel`) can now read, edit, and apply the profiler's
6 runtime parameters live** — the same `GetParam`/`SetParam` path `apply` uses,
but interactive, with read-back verification and firmware range enforcement
surfaced in the UI. No firmware change: the wire protocol already had it.

**Landed:**
- `panel.rs`: HTTP thread → pump now carries a `Cmd` enum (`Send` fire-and-forget
  drive/i_q, `RefreshParams`, `SetParam`). The pump owns the single `Link`, so
  parameter ops run inline as synchronous request/response (dropping a telemetry
  sample or two — negligible). Params + names + a status line ride in `/data`;
  new `/cmd` verbs `getparams` / `setparam`. Startup reads the table *before*
  streaming (quiet line), and **bails on the first NAK** so a device without a
  param table (the sim pre-change, the G0B1) starts instantly instead of eating
  6×500 ms timeouts.
- `panel.html`: a "Motor parameters" card — 6 rows (R, L, flux, current BW,
  speed Kp/Ki) with friendlier display units (mH, mWb), per-field range hints,
  Set / Refresh / "Apply all edited", dirty-field highlighting, and a live
  status line. Edited/focused fields aren't clobbered by the 150 ms poll.
- `server.rs`: **the sim is now param-aware** — stores + range-validates the
  same 6 params (seeded from the sim motor), mirroring the firmware's
  `param_range`. Makes the whole feature testable/regressable without hardware
  (the sim stores but doesn't yet re-tune its loop from a write — documented).
- `server.rs`: **the sim now implements `SetDrive` (all three drive modes)** —
  previously it only handled `SetIqRef`, so the panel's "Apply drive" button
  silently did nothing against the sim (NAK, ignored). New `SimControl` mirrors
  the firmware ISR's mode dispatch against the one virtual rig: mode 0 = legacy
  truth-angle torque (`SetIqRef`, keeps `capture --iq` working), 1 = open-loop
  voltage, 2 = I-f current, 3 = sensorless (Sequencer → observer handoff →
  speed loop). STATE channel now reflects the mode (idle/run/if-ramp/blend), and
  θ̂/ω̂/θ_err estimates stream too. The panel now drives the sim exactly like
  hardware — the "cannot tell the difference" invariant restored for drive.

**Verified** (scratch build, sim over TCP, panel driving it): live params —
startup read (r=0.5 Ω, L=0.6 mH, ψ=8 mWb), valid write + read-back (r→1.25 Ω,
ψ→0.9 mWb), out-of-range reject (r=50 → "rejected by device", unchanged).
Drive modes — I-f 0.3 A @ 188 rad/s el → rotor follows at 26.9 rad/s mech
(=188/7); sensorless startup → closed loop at the 628 rad/s-el target with
ω̂ tracking; open-loop voltage rotor follows commanded frequency; STOP coasts
down. Telemetry uninterrupted; workspace `cargo test` green.

**Decided / deferred:** parameter **flash persistence** (device keeps params
across power-cycle) is now an explicit [PLAN.md](PLAN.md) backlog item — user
chose to hold. Today durability = keep `profile.json` and re-`apply` (or re-Set
in the panel) each boot.

**Next candidates:** flash persistence (§backlog); one-click profile-from-panel
(run `profile::run` inline + fit + review + apply).

## 2026-07-12/13 — session 6: MS6 profiler/auto-tune, end to end on hardware

**The MS6 loop is closed: `mmc-host profile` measures the motor,
`tools/profile.py` fits it, `mmc-host apply` writes the parameters back over
the protocol (no reflash), and the sensorless controller demonstrably
improved on its own measured numbers** — handoff transient peak
1316 → 527 rad/s, speed-step overshoot 9.7 % → 4.8 %
(`ms6-profile/validate_sensorless`).

**Landed:**
- `mmc-proto`: `RunTest{kind,a,b}` / `ReadBurst{offset}` / `BurstData`
  (chunked read-back of an on-device sample buffer, 20 f32s/frame) and
  `SetParam` / `GetParam` / `ParamValue` — 6 runtime params (r, l, flux,
  cur_bw, speed_kp, speed_ki; `mmc_proto::param`), range-checked, RAM-only
  (flash persistence is future work).
- `mmc-fw-g474` (fw v3): runtime `PARAMS` table replaces the hardcoded motor
  constants at clean drive-start; drive mode 4 = locked-rotor R/L probe:
  align the rotor at `a` volts d-axis for 300 ms, then **square-wave**
  between `a`/`b` (32-tick half-period), recording (i_d, v_d) every 50 µs
  tick into a 4096-pair burst buffer. Square wave, not a single step,
  because τ = L/R ≈ 31 µs is *under one sample period* — the host folds
  ~118 edges and fits the averaged settling fraction (the slope of ln z is
  immune to the fractional PWM latency). `burst_abort()` on every fault/
  deadman/off path so a killed probe hands back a partial buffer.
- `mmc-host profile` — sweeps → accel run → probe, all one command;
  `mmc-host apply` — JSON in, SetParam + read-back verification out.
- `tools/profile.py` — R/L fold fit, flux via the hang-angle-aware sweep fit
  (imports fit_params.py), J/friction from the accel run, speed-gain calc,
  writes `profile.json`.

**Fitted profile (this bench):** R = 0.904 ± 0.004 Ω (copper, dead-time
cancelled), **L = 28 µH** (τ = 31 µs — at the resolution floor, ±~30 %),
ψ = 0.888 ± 0.048 mWb (5/5 sweep points, matches Stage F0's 0.894),
friction ≈ 0.75 mN·m (matches 0.78), **J = 1.75 µN·m·s²** (5.6× the crude
Stage-F ramp estimate; trusted more — dedicated slew segment — and the
validation run's much-improved handoff backs it).

**The debugging story (two false leads, both instructive):**
1. At the mid-session checkpoint the flux sweeps were all stalling and the
   suspect was the observer's L default. Wrong on its face: in I-f mode the
   observer is pure shadow — it cannot stall a rotor. The raw voltages
   (v_q = R·i_q exactly, v_d ≈ −ω·L·i_q) said the rotor physically never
   spun.
2. Power balance then said the load torque was 5.4 mN·m (7× session-5
   friction) — **artifact**. It used the probe's R (0.896, dead-time
   cancelled); the *apparent* R at an operating point includes ~0.07 Ω of
   inverter drop (F0's v_q intercept: 0.966). The phantom torque scaled
   with i² across two current levels — the signature of a resistance error,
   not a load. With R_apparent the "heavy load" evaporated.
3. Real cause, proven by A/B: **the R/L probe parks the rotor aligned to
   θ = 0, and an I-f start puts its current vector exactly 90° away — the
   rotor is released on the separatrix of the torque well** (undamped
   pendulum at marginal capture energy), and the frequency ramp ejects it.
   Standalone 0.3 A sweeps caught every time (v_d −0.132 V, the session-5
   value to three digits); probe-first sweeps stalled every time. Fix: the
   profile sequence runs the probe **last**. (Proper fix some day: start
   I-f with d-axis current so the rotor aligns to the frame — the canonical
   self-aligning start; noted, not done.)

**Also:** profile.py fails loudly with hints when sweep points slip or
accel plateaus are missing (the old cascade of numpy errors was awful).

**Open items:** flash persistence for params; the L probe is at its
resolution floor (HF injection would do better); d-axis-aligned I-f start;
the Stage-F leftovers (blend-kick taper, stall detector — the
fake-lock-on-L·i-artifact behavior seen while debugging is exactly what a
stall detector should catch).

**Next: MS7 — encoder as the second `AngleEstimator`, auto-calibrated
against the observer, position loop on top (sim first, then hardware).**

## 2026-07-11 — session 5: Stage F — closed-loop sensorless on hardware (MS5 complete)

**F0 — motor parameters measured from rotating I-f sweeps** (no firmware
change needed; `tools/fit_params.py`):
- Method: at steady I-f with fixed i_q, sweep speed and fit the `v_d`/`v_q`
  slopes vs ω — resistance and dead-time distortion land in the intercepts,
  physics in the slopes. Captures: 0.3 A × {150,250,350,450} rad/s el, plus
  0.45 A × {150,450} and a 0.2 A sync test (`f_paramid_*` on the dashboard).
- **The trap (documented in the script):** in I-f the frame is *forced*, not
  rotor-aligned — the rotor rides a hang angle δ ahead, so the v_d slope is
  `−(L·i_q + ψ·cos γ)` and the v_q slope is `ψ·sin γ`. The naive rotor-aligned
  fit reported Lq = 2.9 mH and ψ = 0.21 mWb — a *perfectly self-consistent
  wrong answer*. Two discriminators split it: the low-ψ model required
  pull-out at 0.2 A (sin γ = 1.45) yet the motor held sync, and the 0.45 A
  v_d slope matched the high-ψ model within 5 %.
- **Fitted (joint, all current levels):** ψ = **0.894 ± 0.04 mWb**
  (kt = 9.39 mN·m/A, bemf 6.26 mV/(rad/s mech)), R_apparent = 0.97 Ω
  (matches the 1.0 Ω locked measurement), friction ≈ 0.78 mN·m (large — the
  rotor hangs ~1.29 rad ahead at 0.3 A, pull-out margin 3.6×),
  J ≈ 0.31 µN·m·s² (rough; accel torque is 3 % of friction).
  **L is ill-conditioned in this test** (0.05 ± 0.10 mH; ψ·cos γ dominates the
  v_d slope near δ ≈ π/2) — bounded "small", design-centered at 0.1 mH; a
  locked-rotor/HF probe is the right instrument (MS6). The old assumed 0.6 mH
  was ~6× high but only biased the shadow observer ~0.05 rad (L·i ≪ ψ).

**F1 — `DriveMode::Sensorless` end to end:**
- `mmc-proto`: `Sensorless { amps, omega_e }` (wire mode 3) — startup current
  + live-retargetable speed target.
- `mmc-fw-g474` (fw v2): MS4's `Sequencer` + `SpeedLoop` in the 20 kHz ISR,
  feedforward FOC with the measured flux, current loop redesigned at
  1000 rad/s on the fitted parameters, conservative speed PI (kp 2e-4,
  ki 2e-3, ±0.8 A). Startup phases stream on the `state` channel with the
  sim's codes (Ramp 6 / Blend 7 / Closed 1).
- Host: `capture --drive sl --amp --hz` plus `--step-hz` (drive retarget at
  60 % for live speed-step traces); panel gained a "Sensorless speed" mode.

**Hardware results (dashboard `ms5-g474-bringup/g,h,i_*`):**
- Startup to 600 rad/s el: ramp 0.375 s → 50 ms blend → closed; ω̂ = 600.0
  (σ 5), **i_q settles at 77 mA — the friction current the F0 fit predicted
  (83 mA)**. Zero faults.
- Live speed step 400 → 800 rad/s el: tracks the 500 rad/s² reference slew
  (90 % in 0.72 s), 9.7 % overshoot, i_q peaks at only 0.10 A.
- Reverse to −600: **first attempt failed and taught the real lesson** — the
  sequencer's startup current was unsigned, and the instant the blend reaches
  the *rotor* angle, +q current is pure torque *against* reverse motion
  (≈ 10⁵ rad/s² el on this 0.31 µN·m·s² rotor). The rotor was flung forward
  and the loop trapped: observer in its blind zone (+12 rad/s < leak), speed
  PI railed at −0.8 A. The sim never saw it — its rotor is ~10⁴× heavier
  relative to torque. Fix in `mmc-core`: `iq_open = i_start·signum(ω_handoff)`
  through Ramp *and* Blend (test now asserts the sign). After the fix,
  reverse mirrors forward exactly: −600.0 (σ 4.6), i_q −77 mA.
- The forward handoff spike (ω̂ briefly ~1300 rad/s, caught in < 0.5 s) is the
  same max-torque blend kick pointed *with* the motion — acceptable, noted
  below as a refinement.

**Open items (new):** soften the blend kick (taper i_start toward the
friction current during Blend, or hand the speed loop over mid-blend); stall
detector (ω̂ below observer floor + railed i_q for N ms → fault) — the reverse
failure showed the trap exists; proper L measurement in MS6.

**Next:** MS6 profiler — locked-rotor R/L steps, flux + inertia sequences,
Python fitting + gain writeback (fit_params.py is the seed); the blend
refinement above; then MS7 encoder.

## 2026-07-11 — session 4: MS4 sensorless foundation + observer shadow-validated on hardware

**Landed (sim / core):**
- `mmc-core::observer::FluxObserver` — leaky voltage-model flux integrator +
  PLL (`AngleEstimator`). Key subtlety: the leaky integrator's transfer is
  `jω/(jω+leak)`, so the flux estimate **leads** by `atan(leak/ω)` — the
  compensation *subtracts* (first cut added it; the ideal-machine unit test
  caught the doubled error immediately).
- `mmc-core::sensorless` — I-f startup `Sequencer` (Ramp → Blend → Closed,
  shortest-path angle interpolation) and `SpeedLoop` with `Pi::preload` for
  bumpless transfer (preload is **sign-matched to rotation** — a +0.5 A preload
  on a reverse spin brakes through the observer's blind zone; found by test).
- `tuning::speed_pi_gains` (integral corner at bw/4 — bw/10 was too slow to
  settle inside a test run).
- `mmc-sim::SensorlessSim` — the full stack against the virtual motor, used by
  both the regression tests and `mmc-host sim --scenario sensorless-speed`.
- Regression tests: startup+handoff+load-step (2 s), 3×3 speed/load sweep
  (400/800/1200 rad/s elec × 0/0.02/0.04 N·m), negative direction. Canonical
  suite run → `testresults/ms4-sensorless/` (handoff 0.43 s, post-handoff
  angle error ≤ 0.23 rad peak / 0.08 rad RMS, speed on target).
- **Voltage-ceiling finding:** at 24 V the back-EMF meets the voltage limit at
  ~1730 rad/s elec; above ~0.7× that a small speed overshoot erases braking
  authority (feedforward saturates the voltage circle → pi_limit → 0) and the
  loop cannot recover — field weakening is future work; sweeps top out at 1200.
- Protocol channels 15–17: `theta_est`, `omega_est`, `theta_err`
  (MAX_CHANNELS 16 → 24); dashboard charts "Angle estimate error" and
  "Electrical speed estimate".

**Landed (hardware):** the G474 firmware now runs the observer in **shadow**
during volt/I-f drives (Rs = 1.0 Ω from bring-up, Ls still assumed 0.6 mH) and
streams the estimate. Stage E capture (I-f 0.3 A @ 20 Hz elec):
`omega_est` = 125.7 rad/s — *exactly* the forced frequency (σ 10.8); the
1.30 rad `theta_err` vs the forced frame is the expected I-f hang angle
(rotor d-axis aligns with the current vector ≈ π/2 ahead of the forced frame,
less ~0.27 rad of load angle), i.e. **the observer measures the true rotor
angle**; estimator noise ≈ 0.11 rad. Hardware handoff is what the Sequencer's
blend exists for.

**Next:** close the loop on hardware — port the Sequencer+SpeedLoop into
`mmc-fw-g474` as a `sensorless` drive mode (Stage F), ideally after a scripted
locked-rotor/L measurement; then MS6 profiler.

## 2026-07-10 — session 3: G474 + inverter shield preliminary motor bring-up (MS5 pulled early)

**Hardware:** G474 dev board + three-phase inverter shield + small BLDC on a
12 V supply, pre-validated sensorless with ST firmware. Schematics + a working
vendor pin-config export live in `hw/`.

**Pin map extracted from schematics** (see `mmc-fw-g474` doc header for the
full table): TIM1 CH1-3 on PA8-10 → driver IN U/V/W; EN on PB13-15; STBY PB5;
shunt amps (0.33 Ω, 0.504 V/A around a 1.558 V offset) on
PA1/PB1/PB0 = ADC1 IN2/12/15; VBUS ÷16 on PA0 = IN1; VCP = LPUART1 PA2/PA3;
current-ref PB4 held high = weakest driver limit (≈1.5 A). EN_FAULT is read on
**both PA11 and PB12 with internal pull-ups** — the shield populates different
0R routes per board variant and a floating pin false-faulted (found the hard
way).

**Landed:**
- `mmc-proto`: channels `i_a/i_b/i_c` + `state` (off/run/fault-oc/fault-drv/
  fault-vbus/cal), `SetDrive` command (Off / OpenLoopVoltage / IfCurrent).
- `mmc-fw-g474`: 170 MHz, 20 kHz center-aligned TIM1, injected ADC sequence
  (iU,iV,iW,VBUS) triggered at the counter peak via CC4 (JQDIS! the G4
  injected queue silently eats JSQR otherwise), control loop in the JEOS ISR,
  zero-current calibration at boot, software trips (|i|>1.5 A ×2 samples,
  VBUS window, gate fault, 2 s host-silence deadman), slew-limited open-loop
  voltage and I-f drives, `ISR_MAX_CYCLES` DWT diagnostic readable by probe.
- `mmc-host panel`: local web control panel (mode buttons, amplitude/Hz, live
  charts, STOP + space bar, browser-absent auto-off watchdog) over one `Link`
  — works against sim TCP and hardware serial identically. `capture` gained
  `--drive volt|if --amp --hz` and keep-alive pings.
- Staged bring-up, all curated on the dashboard (`ms5-g474-bringup/`):
  A: VBUS 12.06 V ±30 mV, zero-current σ ≈ 3 mA. B: stage live at 50/50/50,
  currents unchanged. C: open-loop 0.5 V @ 15 Hz — motor spins 129 rpm,
  i_d = +0.50 A ⇒ R ≈ 1.0 Ω and current-sense sign/scale confirmed.
  D: **I-f closed current loop — i_q 0.300 A on target, 7 mA RMS error,
  i_d = 0.000** at 129 rpm forced.

**The big find — libm trig is f64-emulated:** `libm::sinf/cosf` reduce
arguments through f64 arithmetic; on single-precision FPUs (M4F!) that is
soft-float and cost **~100 µs per control tick** — the ISR ate the whole CPU,
starved both UART tasks, the deadman fired, and I-f "died" 2 s after engage.
Diagnosed live via probe-rs memory reads (`CONTROL_TICKS` advancing,
`LAST_RX_TICK` frozen, `ISR_MAX_CYCLES` = 17 052) plus a duty-discard bisect.
Fix: fast f32 polynomial `sin_cos` + `%`-free `wrap_angle` in `mmc-core::math`
(err ≤ 3e-5, tested against libm) → ISR worst-case **13.9 µs**; 1 kHz
telemetry now lossless in every mode (was 43 % loss even in volt mode). This
also explains why the G0B1 needed its 64 MHz PLL, and it retroactively fixed
the earlier "frame loss mystery."

**Also fixed:** capture keep-alive pings (device deadman used to end long
drive captures at exactly 2 s); stale omega telemetry after stop; sim serves
the new channels.

**Open items:** TIM1_BKIN2 hardware break on PA11 (ST's ioc does this; we poll
in software), table-CRC16 if more UART headroom is ever needed, `libm::sqrtf`
is also software (≈2 calls/tick, tolerable), pole-pair count is assumed 7 for
the rpm channel until the profiler measures it.

**Next:** MS4 — flux observer + PLL in sim, then close the loop sensorless on
this hardware (MS5 completion); R/L measurement can now be scripted through
`SetDrive` + captures.

## 2026-07-10 — session 2: cleanup, dashboard, MS3 kickoff

**Context:** previous session crashed after committing the MS1+MS2 baseline
(`6bd7af9 baseline no hardware bringup`). Verified on restart: all workspace
tests pass, so MS1+MS2 stand.

**Environment changes:**
- Rust updated to **1.97.0** (was 1.84). The edition-2024 dependency workaround
  is no longer needed; `rust-version = "1.84"` stays as a floor.
- **G0B1 dev board** connected (debug-probe VCP). Purpose: early protocol bring-up
  and the hardware-modularity proof on Cortex-M0+ (no FPU). Not a motor target.

**Landed:**
- **Test-results reorg + dashboard.** Root-level `step_*.csv` strays removed;
  results now live in `testresults/<group>/` as CSV + `.meta.json` provenance
  sidecars, kept in git (`.gitignore` still ignores ad-hoc CSVs elsewhere).
  - `mmc-host suite` — runs the canonical scenario set (locked / free / loaded
    rotor current steps) into `testresults/ms2-current-loop/` and rebuilds the
    dashboard.
  - `mmc-host report` — regenerates `testresults/index.html` from whatever CSVs
    are in the tree, so hardware captures later get charts for free. Metrics
    (rise time, overshoot, steady-state error) are recomputed from the traces.
  - Dashboard is a single self-contained HTML file: step-metric tiles, dq
    current/voltage charts, speed, PWM duties, crosshair tooltips, keyboard
    navigation, light/dark, per-run data tables. Open `testresults/index.html`
    in a browser.
- **Docs:** this file + `docs/PLAN.md` (verbose in-repo plan; previously only
  in `~/.claude/plans/`).

**Suite numbers at this baseline** (2 kHz-rad/s design bandwidth, 10 kHz loop):
- Locked rotor: rise 1.00 ms (ideal 1.10), overshoot 0.0 %, SSE ≈ 0 %.
- Free rotor: rise 1.00 ms; ends voltage-limited at ~2 364 rpm (24 V bus), so
  the 1 A reference is unreachable at the end — expected without field
  weakening; SSE tile reads 99 % *by design of the scenario*.
- Loaded rotor (0.04 N·m): rise 1.00 ms; settles voltage-limited near
  2 323 rpm, current droops to torque balance (~0.5 A), SSE tile ~52 % —
  likewise physics, not a controller defect.

**MS3 landed (same session):**
- **`mmc-proto` for real**: COBS-framed, CRC16-checked messages (`0x00`
  delimited, self-resynchronizing), `no_std`/no-alloc/panic-free encode +
  incremental `Deframer`. Commands: ping, get-info, set-telemetry
  (mask + divider), stream on/off, set-iq-ref; telemetry frames carry a
  wrapping-µs device timestamp, channel mask, and packed f32 values.
  Channel registry in `mmc_proto::channel` doubles as CSV column order.
- **Sim behind the protocol**: `mmc-host serve` runs the virtual motor + FOC
  loop in real time behind a TCP server speaking mmc-proto. `mmc-host capture`
  connects over TCP *or* serial through one `Link` code path, streams to CSV +
  meta sidecar, and can command a live current step mid-capture. The suite now
  includes an end-to-end TCP leg (in-process server, 0.5 A step, ~1006 frames
  at 1 kHz, 0 rejected) → `testresults/ms3-telemetry/tcp_step.csv`.
- **G0B1 dev board bring-up (hardware modularity proof)**: new
  `crates/mmc-fw-g0b1` (outside the host workspace; embassy-stm32 0.6,
  thumbv6m-none-eabi). USART2/PA2-PA3 (debug-probe VCP) at **1 Mbaud** (divides
  16 MHz HSI and the debug-probe clock exactly). Two tasks: RX (deframe + handle
  commands) and TX (responses + telemetry ticker). No inverter attached, so it
  streams a synthetic first-order plant (τ = 20 ms) driven by the commanded
  `i_q` — mmc-core's `math` runs on the M0+ in the process. Flashed with
  probe-rs (debug probe detected on COM5).
- **Hardware capture on the dashboard**: 11 channels at ~990 Hz effective,
  **1,980 frames / 0 rejected** in 2 s; measured rise time 42.55 ms ≈ 2.2·τ,
  exactly right for the synthetic plant →
  `testresults/ms3-telemetry/g0b1_step.csv`.
- CI: thumbv6m builds of the shared crates + the G0B1 firmware build.

**Findings & fixes along the way:**
- Host serial reads were byte-per-syscall — capped ~300 frames/s; fixed with a
  4 KiB read buffer in `Link::recv`.
- The G0 at its 16 MHz reset clock is CPU-bound encoding frames (soft-float
  trig + bitwise CRC ≈ 2.3 ms/frame). Fixed: 64 MHz via PLL and one `sin_cos`
  per tick (other phases via angle-addition identities) → ~1.35 kHz max frame
  rate. Table-driven CRC16 is a future option if more headroom is needed.
- embassy-executor 0.10 renamed `arch-cortex-m` → `platform-cortex-m`; task
  instantiation now returns `Result` (arena); G0 needs the shared DMA IRQs
  (`DMA1_CHANNEL1`, `DMA1_CHANNEL2_3`) bound explicitly.

**Deferred from MS3:** a true *live* plot view (watch estimated vs. true angle
during observer tuning — needed by MS4, revisit then). Captures + dashboard
cover the current need.

**Next up (MS4):** flux observer + PLL behind `AngleEstimator` in sim, I-f
startup and handoff, sensorless speed loop, observer-vs-truth regression
sweeps. The MS3 telemetry path is how observer tuning gets watched.

## 2026-07-10 — session 1: MS1 + MS2

- Decisions: Rust, sensorless-first FOC, G474 dev board target, MS naming.
- MS1: cargo workspace (`mmc-core`, `mmc-hal`, `mmc-proto`, `mmc-sim`,
  `mmc-host`), no_std enforced via thumbv7em build, CI workflow, fmt/clippy.
- MS2: PMSM dq model + average-value inverter in `mmc-sim` (semi-implicit Euler,
  1 µs substeps), Clarke/Park, SVPWM, PI with anti-windup + feedforward
  decoupling, `Foc::step` current loop, `TruthAngle` estimator, `tuning.rs`
  bandwidth-based gain calc, step-response metrics + regression tests,
  `mmc-host sim --scenario current-step` CSV runs.
- Committed as `6bd7af9` "baseline no hardware bringup". Session crashed after.
