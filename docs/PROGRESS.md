# mmc — progress log

Newest first. Every session that changes the project appends here: what landed,
what was decided, what's next. The stable plan lives in [PLAN.md](PLAN.md).

## 2026-07-19 — session 12: sensorless generalized — QBL5704 closes the MS6 loop

**Closed-loop sensorless runs on motor 2, first attempt, and its profile is
complete.** The blockers were three compile-time constants sized for motor 1;
they are now runtime params (ids 7–9): `sl_handoff`, `omega_accel` (drives
the forced-mode slew, the sequencer ramp, and retargets — and the accel-J fit
now reads the value actually used from the capture snapshot), and `iq_limit`
(replaces both I_AMP_MAX and SL_IQ_LIMIT). Firmware bumped to v5. QBL5704
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
  l = L_ll/2). Validation both ways on the QBL5704: displayed estimate
  0.41 Ω line-line vs datasheet 0.35–0.45; entering the datasheet's
  0.35 Ω/1.0 mH produces r within 3% of the profiled value.
- Bench note: the device power-cycled again and lost its RAM params (restored
  by hand) — the flash-persistence backlog item keeps earning its place.

## 2026-07-18 — session 10: profiler output neutralized; motor 2 identified as QBL5704, datasheet cross-check

**Motor 2 is a Trinamic QMot QBL5704** (`hw/qbl5704_datasheet_v1.05.pdf`,
likely the -116-04-042 by the L comparison; check the 94 vs 116 mm body to
confirm). Full profiler-vs-datasheet comparison in
`testresults/motor2-4pole/datasheet-comparison.md`. Highlights:

- **Pole pairs = 2 confirmed twice**: the datasheet says 4 poles, and the kt
  cross-check agrees — 1.5·pp·ψ = 56.2 mN·m/A vs the datasheet's 63 (block
  convention, −11%); pp=4 would read +78%. ψ also predicts ≈3700 RPM at 36 V
  vs the 4000 RPM rating.
- **The R "discrepancy" closes exactly**: profiler R = 1.047–1.057 Ω vs
  datasheet winding 0.175–0.225 Ω. The gap is the drive path — STSPIN830
  conducting switch ≈0.5 Ω (R_DSon HS+LS = 1 Ω typ, verified from
  `hw/stspin830.pdf`) + 0.33 Ω low-side shunt (duty-weighted ≈0.32) — summing
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
- Rig context: the IHM16M1's 1.5 A limit drives this 5–6.7 A motor at ≤25%
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

## 2026-07-10 — session 3: G474 + IHM16M1 preliminary motor bring-up (MS5 pulled early)

**Hardware:** NUCLEO-G474RE + X-NUCLEO-IHM16M1 (STSPIN830) + small BLDC on a
12 V supply, pre-validated sensorless with ST firmware. Schematics + a working
CubeMX config live in `hw/`.

**Pin map extracted from schematics** (see `mmc-fw-g474` doc header for the
full table): TIM1 CH1-3 on PA8-10 → STSPIN IN U/V/W; EN on PB13-15; STBY PB5;
shunt amps (TSV994 ×2, 0.33 Ω, 0.504 V/A around a 1.558 V offset) on
PA1/PB1/PB0 = ADC1 IN2/12/15; VBUS ÷16 on PA0 = IN1; VCP = LPUART1 PA2/PA3;
current-ref PB4 held high = weakest STSPIN limit (≈1.5 A). EN_FAULT is read on
**both PA11 and PB12 with internal pull-ups** — the shield populates different
0R routes per Nucleo variant and a floating pin false-faulted (found the hard
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
- **NUCLEO-G0B1RE** connected (ST-Link VCP). Purpose: early protocol bring-up
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
- **NUCLEO-G0B1RE bring-up (hardware modularity proof)**: new
  `crates/mmc-fw-g0b1` (outside the host workspace; embassy-stm32 0.6,
  thumbv6m-none-eabi). USART2/PA2-PA3 (ST-Link VCP) at **1 Mbaud** (divides
  16 MHz HSI and the ST-Link clock exactly). Two tasks: RX (deframe + handle
  commands) and TX (responses + telemetry ticker). No inverter attached, so it
  streams a synthetic first-order plant (τ = 20 ms) driven by the commanded
  `i_q` — mmc-core's `math` runs on the M0+ in the process. Flashed with
  probe-rs (ST-LINK V2-1 detected on COM5).
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

- Decisions: Rust, sensorless-first FOC, NUCLEO-G474RE target, MS naming.
- MS1: cargo workspace (`mmc-core`, `mmc-hal`, `mmc-proto`, `mmc-sim`,
  `mmc-host`), no_std enforced via thumbv7em build, CI workflow, fmt/clippy.
- MS2: PMSM dq model + average-value inverter in `mmc-sim` (semi-implicit Euler,
  1 µs substeps), Clarke/Park, SVPWM, PI with anti-windup + feedforward
  decoupling, `Foc::step` current loop, `TruthAngle` estimator, `tuning.rs`
  bandwidth-based gain calc, step-response metrics + regression tests,
  `mmc-host sim --scenario current-step` CSV runs.
- Committed as `6bd7af9` "baseline no hardware bringup". Session crashed after.
