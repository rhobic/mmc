# Six-step, reviewed: what actually blocked it, and what fixed it

*2026-08-03. A holistic review of the six-step drive after three sessions of
contradictory verdicts, ending with closed-loop sensorless six-step locked on
the small bench motor — the motor declared incapable of it that same morning.
Every claim here is anchored to a capture in `testresults/`.*

## Why this review

Session 25 concluded the small motor **cannot** run six-step on this hardware:
30 µH was said to put unavoidable commutation transients past the 1.5 A
protection. The review re-examined that verdict against the reference
architecture — hobby-ESC practice runs six-step sensorless on exactly this
class of motor (tens of µH, tens of mΩ) at scale — and against our own raw
captures. The verdict did not survive. Neither did several other conclusions.

## Instrument errors found first

**Sector rate is not rotor speed on a forced drive.** Every "forced six-step
reaches N rad/s" claim in sessions 23–25 measured the commutation clock, which
is pinned to the command whether or not the rotor follows. The one high-speed
capture with a coast tail shows the truth: back-EMF frequency at drive-cut was
**771 rad/s el against a 1200 rad/s command** — the rotor was slipping. The
forced legs of `ms8-speed-sweep` at 597 and 898 rad/s carry this caveat in
their metadata now; they have no coast tails, so their true speeds are
unverifiable. Closed-loop runs are exempt: crossing-timed commutation *is* a
rotor measurement.

**The current telemetry sample sits at the ripple minimum.** Found in session
25, confirmed here: doubling PWM to 40 kHz moved the same commanded operating
point's reading by 1.47×. Absolute current numbers taken at 20 kHz on this
motor read low; the R/L probe is immune (differential across folded edges).

## The mechanism chain, corrected

Each row was believed at some point; only the last column survived contact
with its capture.

| symptom | first explanation | measured mechanism |
|---|---|---|
| ramps trip OC near the top | unavoidable commutation transient, `di/dt = V/2L` | **pull-out**: open-loop duty feedforward starves current as the back-EMF term dominates (±0.5 V model error ≈ the whole budget), torque collapses, the rotor slips, and the desync surge — 0.9→1.47 A sustained across ~15 windows — trips OC. No hunting resonance in the envelope spectrum; not a single-window event. |
| no zero crossings at high speed | sensing floor, back-EMF too small | the rotor was **slipping** in every high-speed forced capture; the data was garbage about sensing. On a *synchronized* rotor at 898 rad/s (carried there by FOC), the idle-phase ramps are textbook: ±1.5 V about the measured mid, crossing mid-window, every sector. |
| closed loop collapses to ~40 rad/s at any handoff | detector tuning | **low-speed only**: parity-locked artifact offsets of ±0.7–1.4 V against a 0.1–0.4 V signal (SNR < 1 below ~300 rad/s el, verified on a synced rotor at 302). The offsets are settled network properties — invariant to sample position (CCR5 40 ≡ 100) — that *grew* when PWM went 20→40 kHz, while within-window variability halved. |
| handover desyncs after 5 sectors | — | the **250 µs blank swallowed an early-arriving crossing** (21% of a 1.17 ms window; a late commutation pushes the next crossing toward window entry). Demag on this motor is ~2.5 µs; blanking was 100× oversized. At 30 µs: lock. |

## What shipped (fw v15)

- **Live FOC → six-step handover.** `SetDrive(six-cl)` while closed-loop
  sensorless FOC runs hands commutation to six-step at the observer's angle:
  sector from `sector_of(θ̂)`, detector seeded with ω̂, duty loop preloaded at
  the speed's feedforward, no open-loop ramp anywhere. The FOC and six-step
  angle conventions agree (both give `e_u = −ψω·sinθ`); verified by
  contradiction on the bench — a deliberate +π seed drew the ~1.9 A an
  antipodal pair predicts within two ticks, the direct seed starts at ~0.3 A.
- **Guarded cross-mode switching.** Previously, any live mode change
  dereferenced uninitialized control state (`None` unwrap → panic → the
  firmware halts silently). Unsupported switches now keep the running drive.
- **30 µs handover blanking** (the ramp path keeps 250 µs for low-speed
  starts).
- Host: `capture --step-kind/--step-amp` switches drive *kind* at the 60%
  mark, which is how the handover is scripted.

## The result

`sl 0.5 A → 898 rad/s el`, hand over at 60%: **locked in 6 ms, held for the
rest of the capture (8 s), zero faults, reproduced.** Sector rate 829–941
rad/s el, rotor-driven. Settled current **0.03–0.14 A** — the duty loop finds
torque balance, matching closed-loop FOC's economy. The earlier "six-step
draws 0.5 A" figures were forced-mode artifacts: a forced drive pushes
whatever it is told; any closed loop supplies what friction demands.

ISR worst case at 40 kHz PWM / 20 kHz control: idle 580 · FOC current loop
4118 · sensorless FOC 4247 · six-step locked ≤ 4247, of an 8500-cycle budget.

## The 100 kHz question, answered with data

Raising switching frequency was the review's motivating suggestion, and the
bench splits it cleanly:

| effect of f_sw ↑ | direction | evidence |
|---|---|---|
| ripple current, `V·d(1−d)/(2L·f_sw)` | ∝ 1/f_sw | 20→40 kHz halved it |
| current-measurement bias | shrinks | reading moved 1.47× toward physics |
| within-window sensing noise | shrinks | span mean 2.24→1.44 V at 302 rad/s |
| parity-locked sensing offsets | **grows** | offsets roughly doubled 20→40 kHz |
| on-time sensing window `d·T` | shrinks | duty floor for sensing rose 0.07→0.10 |
| dead-time distortion | grows | ∝ f_sw |

So f_sw is the right lever for ripple, losses, and measurement fidelity — and
the *wrong* lever for the low-speed sensing floor, which it worsens. 40 kHz is
the shipped default. 80–100 kHz becomes attractive only for ripple/acoustics,
and requires converting a single idle-phase channel per window (the 4-channel
injected sequence no longer fits the on-window) plus dead-time compensation
for open-loop voltage accuracy.

## Where six-step now stands per motor

- **Small motor (30 µH, 0.94 mWb):** closed-loop six-step runs at speed via
  handover. Its low-speed band (< ~300 rad/s el) remains closed by the
  artifact floor — standalone six-step starts are not available; the FOC
  spin-up is the startup. That is an acceptable architecture: it is how this
  drive would ship anyway, since FOC owns current control during acceleration.
- **4-pole motor (377 µH, 18.7 mWb):** 20× the flux puts its signal over the
  same artifact floor from ~50 rad/s el up, which is why the original MS8
  results happened on it. The detector improvements here (short blanking,
  handover) transfer directly and raise its speed ceiling.

## Remaining work, ranked

1. **Adaptive blanking in `mmc-core`** — a fraction of the expected interval
   with a demag-derived floor, replacing both fixed constants; regression in
   the phase-domain sim (which should also gain a handover scenario so this
   path is CI-covered).
2. **Map the lock band** — minimum sustainable speed after handover, retarget
   steps, behavior at the duty ceiling; then the honest FOC-vs-six-step
   comparison at matched *closed-loop* operating points.
3. **Per-parity offset calibration** — the low-speed offsets are settled and
   repeatable, so a learned per-sector constant could lift low-speed SNR ~3×;
   the one software lever the floor data leaves open.
4. **Current-closed ramp** for standalone six-step starts (the 4-pole motor's
   path; the feedforward-only ramp still pulls out).
5. **Hardware cycle-by-cycle limit** — the MCU's comparators can watch the
   shunt-amp pins and clamp PWM via the timer's OCREF-clear input; worth a
   pin-map check before any higher-current work.
