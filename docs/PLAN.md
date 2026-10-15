# mmc — architecture & milestone plan

The living, verbose plan. The what-happened-when log is [PROGRESS.md](PROGRESS.md);
project goals are [../overview.md](../overview.md). Update this file when a decision
changes, not just when a milestone lands.

## Decisions (confirmed with the user, 2026-07-10)

- **Language: Rust.** A `no_std`, allocation-free control core compiles unchanged
  for PC and MCU, so the whole control loop runs in `cargo test` against the
  virtual motor.
- **Motors: PMSM/BLDC, sensorless-first.** Sensorless FOC (flux observer + PLL) is
  the foundation; the encoder is added later as a second rotor-angle source that
  plugs into — and calibrates against — the sensorless estimator.
- **Hardware targets:**
  - **G474 dev board** + a three-phase inverter shield — the motor
    control target (MS5). Exact shield determines current-sense wiring at
    bring-up time, not architecture.
  - **G0B1 dev board** (Cortex-M0+, on the bench since 2026-07-10) — *not* a motor
    target yet; it exists to prove the protocol and the hardware-modularity story
    early: `mmc-proto` over UART on a core with no FPU, `thumbv6m` build of the
    shared crates. It is the low-resource scaling proof from overview.md.
- **Milestone names are `MS1…MS7`** — never "M0/M1", those collide with ARM
  Cortex-M core names (Cortex-M0 is an explicit scaling target).
- **Toolchain:** stable Rust (1.97 as of 2026-07-10; the old 1.84 pin is gone).
  `rust-version = "1.84"` in the workspace is a floor, not a pin.

## Architecture

Cargo workspace; the layering is the whole design:

```
mmc/
├── Cargo.toml           # workspace
├── crates/
│   ├── mmc-core/        # no_std, no-alloc control library — the heart
│   ├── mmc-hal/         # hardware abstraction traits (no_std)
│   ├── mmc-sim/         # virtual motor + inverter + sensor models
│   ├── mmc-proto/       # telemetry/command wire protocol (no_std, shared fw↔host)
│   ├── mmc-fw-g0b1/     # G0B1 dev board protocol/bring-up firmware (Cortex-M0+)
│   ├── mmc-fw-g474/     # STM32G474 motor firmware binary (MS5)
│   └── mmc-host/        # host CLI: sim scenarios, dashboard, telemetry capture
├── docs/                # this plan + progress log
├── testresults/         # curated CSV traces + generated dashboard (index.html)
└── tools/               # Python analysis: system-ID fitting, gain calc (MS6)
```

### mmc-core — portable control library

Pure math, no I/O, no allocation, `f32` throughout (raw arithmetic stays behind a
small `math` module so fixed-point for Cortex-M0-class targets can be introduced
later without a rewrite — no premature generics). Contents: Clarke/Park
transforms, SVPWM modulator, PI controllers with anti-windup + feedforward, dq
current loop, and (MS4) flux observer + PLL with I-f open-loop startup, speed
loop, ramp/trajectory generator. Control methodologies (FOC, six-step) gated
behind cargo features for the low-resource scaling story.

Rotor angle/velocity is consumed through the `AngleEstimator` abstraction: the
sim's `TruthAngle` today, the sensorless observer in MS4, the encoder-backed
implementation in MS7 (which auto-calibrates its offset against the observer).
That is what makes sensorless the foundation rather than a parallel path.

### mmc-hal — the abstraction that makes it portable

Small trait set sized to motor control, not a general HAL: three-phase PWM
(center-aligned, deadtime), PWM-synchronized current sense, position sensor,
bus-voltage sense. Implemented by the sim today, by real hardware at bring-up.

### mmc-sim — virtual motor

PMSM dq-frame model (Rs, Ld/Lq, flux linkage, pole pairs, inertia, friction),
average-value inverter, fixed-step integration substepped ~1 µs (≪ L/R). It
drives the *actual* `mmc-core` loop through the `mmc-hal` traits — this is what
makes CI regression tests on step responses possible.

### mmc-proto — one protocol, two transports

Compact binary telemetry frames (channel selection, decimation, timestamps) plus
a small command set. Transport-agnostic by design: the sim serves it over TCP,
firmware over UART/USB-CDC — **the same host tooling works identically against
sim and hardware**. This is the trick that keeps the PC target first-class
forever. Framing: COBS-encoded frames with CRC over lossy byte pipes; TCP can
carry the identical bytes.

### Test results & dashboard

Every scenario run lands a CSV (+ `.meta.json` provenance sidecar) under
`testresults/<group>/`; `mmc-host report` regenerates the self-contained
`testresults/index.html` dashboard from whatever it finds there, and
`mmc-host suite` runs the canonical scenario set and then rebuilds the
dashboard. Hardware captures later drop into the same tree and get the same
charts. Groups are directories (`ms2-current-loop`, `ms3-telemetry`, …), so the
dashboard grows milestone by milestone without dashboard changes.

### Profiling (host-offloaded, per overview)

Firmware only executes test sequences and streams raw samples; the host does the
math. Locked-rotor voltage steps → R, L; rotating test → flux linkage; torque
steps → inertia/friction. Python/scipy fits parameters and computes PI gains
from desired bandwidth, then writes config back over the protocol.

### Firmware runtime (MS5, decided early for direction)

`embassy-stm32` for comms/housekeeping; the FOC loop runs in a hardware ISR (ADC
end-of-conversion, triggered by TIM1 center-aligned PWM) outside the async
executor. Expect PAC-level register work for the TIM1↔ADC injected-conversion
sync — normal for motor control in any language.

## Milestones

- **MS1 — Scaffold** ✅ *(2026-07-10)*: workspace, crate skeletons, CI
  (`cargo test` + thumbv7em `no_std` build), fmt/clippy.
- **MS2 — Sim + FOC current loop** ✅ *(2026-07-10)*: PMSM model, transforms,
  SVPWM, dq current PI against sim truth angle; unit + step-response regression
  tests; CSV scenario runs; `suite`/`report` dashboard added during cleanup.
- **MS3 — Telemetry + protocol** ✅ *(2026-07-10)*: `mmc-proto` (COBS+CRC
  frames, commands, telemetry channels), sim served over TCP, host capture to
  CSV/dashboard over TCP and serial through one code path.
  **Amended 2026-07-10:** included G0B1 dev board protocol bring-up — the same
  `mmc-proto` over debug-probe VCP UART from a Cortex-M0+ (1 Mbaud, 11 channels at
  ~1 kHz, zero rejected frames) — proving protocol and hardware modularity
  before the G474 exists. `thumbv6m-none-eabi` joined the no_std CI matrix.
  *Deferred:* live plot view — revisit in MS4 where observer tuning needs it.
- **MS4 — Sensorless foundation (in sim)** ✅ *(2026-07-11)*: flux observer
  (leaky voltage model, lead-compensated) + PLL behind `AngleEstimator`, I-f
  startup `Sequencer` with blend handoff, sensorless `SpeedLoop` with bumpless
  preload. Regression sweeps (speed × load × direction) hold angle error
  < 0.3 rad vs sim truth. Observer also **shadow-validated on the real motor**
  (exact ω̂, θ̂ = true rotor angle incl. I-f hang angle, noise ≈ 0.11 rad).
  *Known limit:* no field weakening — usable ceiling ≈ 0.7 × (V_bus/√3)/ψ.
- **MS5 — G474 bring-up, sensorless** ✅ *(2026-07-11)*: clocks (170 MHz),
  TIM1 center-aligned PWM + injected-ADC shunt sensing on the inverter shield,
  zero-current calibration, protection trips (overcurrent / VBUS / gate fault
  / host deadman), open-loop spin, I-f closed current loop (0.3 A, 7 mA RMS),
  a manual web control panel (`mmc-host panel`) — and **closed-loop sensorless
  on the real motor**: motor parameters measured from rotating I-f sweeps
  (ψ = 0.894 mWb, R ≈ 1.0 Ω, friction 0.78 mN·m; `tools/fit_params.py` —
  beware the I-f hang-angle fit trap documented there), then
  `DriveMode::Sensorless` runs MS4's Sequencer + SpeedLoop in the 20 kHz ISR:
  600 rad/s el held to σ 5, live speed steps track the slew, reverse mirrors
  forward after sign-matching the startup current (the blend turns it into
  pure torque — hardware-only find, sim rotor too heavy to show it).
  *Deferred:* proper L measurement (ill-conditioned in the rotating fit),
  blend-kick softening, stall detector → MS6.
- **MS6 — Profiler/auto-tune** ✅ *(2026-07-13)*: `mmc-host profile` runs the
  measurement set (rotating I-f flux sweeps, sensorless accel run, locked-rotor
  R/L probe — a 20 kHz on-device burst capture read back over the protocol,
  square-wave excitation because τ = L/R undercuts one sample period),
  `tools/profile.py` fits R/L/ψ/friction/J and computes gains,
  `mmc-host apply` writes them to the firmware's runtime parameter table
  (RAM; flash persistence deferred) with read-back verification. Closing
  proof on the bench: the sensorless loop on its own measured parameters cut
  the handoff transient 1316 → 527 rad/s and step overshoot 9.7 → 4.8 %.
  *Sequencing lesson:* the probe parks the rotor on the I-f separatrix, so
  it runs last. *Deferred:* HF-injection L probe, d-axis-aligned I-f start,
  param flash persistence.
- **MS7 — Encoder config + position control**: encoder as second
  `AngleEstimator`, offset auto-calibrated against the observer, position loop
  on top (sim first, then hardware).
  **Deferred 2026-08-02 — no rotor-angle sensor on the bench yet.** Nothing
  blocks it architecturally (`AngleEstimator` has always been the seam), but
  the hardware leg needs an encoder or the motor 2's hall sensors wired to the
  inverter shield's connector. MS8 runs first.
- **MS8 — Six-step / trapezoidal drive** *(started 2026-08-02, out of order —
  see MS7; steps 1–3 done, theory in [SIXSTEP.md](SIXSTEP.md))*: the second control methodology from the low-resource scaling
  story, gated behind a `mmc-core` cargo feature. Sequence:
  1. **Bench experiment first** — confirm the floating phase is *readable*.
     Session 13 characterized the BEMF net with all three phases switching;
     six-step floats one, which is a different measurement. The open risk is
     the sample point: ADC2 triggers at CC4, the center-aligned counter peak,
     where all low sides conduct — so the floating terminal is sampled against
     a neutral near 0 and the clamp clamps rectify the negative half away,
     which may make the falling zero-cross invisible. Remedy if so: a second
     ADC2 trigger inside the PWM ON window, compared against VBUS/2.
  2. **`mmc-sim` phase-domain model** — the one real architectural extension.
     The sim is a dq average-value PMSM today: no floating terminal, no
     trapezoidal BEMF, so six-step is currently untestable in CI. That breaks
     the project's core invariant (every control path regression-tested in sim
     before hardware). Needs per-phase terminal states + trapezoidal BEMF
     behind the same `mmc-hal` traits.
  3. **`mmc-core/src/sixstep.rs`** — 6-sector commutation table, forced-
     commutation ramp (mirrors the I-f `Sequencer`), then zero-cross → 30° el
     delay → commutate, speed derived from ZC intervals.
  4. **Protocol + firmware** — per-phase Hi-Z is nearly free: `stage_on`/
     `stage_off` already drive EN on PB13/14/15 together, so this is a
     `stage_phases(mask)` split. *Numbering trap:* `DriveMode` wire codes run
     0–3, but firmware's `CMD_MODE` **4 is already claimed** by the R/L probe
     (which arrives via `RunTest`, not `SetDrive`). Use 5 = forced commutation,
     6 = BEMF zero-cross, keeping wire and firmware namespaces aligned.
  5. **FOC-vs-six-step bench comparison** on one motor and one profile: torque
     ripple, acoustic signature, ISR cost. *(Forced-mode comparison done
     2026-08-03 across 100–900 rad/s el; ISR cost measured. The closed-loop
     comparison unblocked by step 6 is still open.)*
  6. **FOC → six-step live handover** *(shipped 2026-08-03, fw v15 — see
     [SIXSTEP-REVIEW.md](SIXSTEP-REVIEW.md))*: closed-loop sensorless FOC
     carries the rotor to speed, six-step takes commutation at the observer's
     angle. First lock on the small motor at ~865 rad/s el. Remaining:
     adaptive blanking in `mmc-core` + a sim handover scenario (this path is
     not CI-covered yet), lock-band mapping, per-parity offset calibration
     for the low-speed floor, a current-closed ramp for standalone starts.

- **MS9 — FOC operating envelope** *(started 2026-08-08, session 27)*: make the
  FOC path good at the edges — low speed, high speed, and the limits — now that
  six-step has shown what the rig can and cannot do. Ordered by measured value,
  not by textbook glamour.

  **The envelope, in numbers, on bench motor 1** (ψ 0.937 mWb, L 30.15 µH,
  R 0.885 Ω, pp 7, 12 V bus) — because two of the obvious ideas are worthless
  here and it is better to say so up front:

  | | |
  |---|---|
  | voltage limit `V_bus/√3` | 6.93 V |
  | no-load speed ceiling | **7394 rad/s el** (~10 100 rpm mech) |
  | MS4's 0.7× usable ceiling | 5176 rad/s el |
  | best FOC speed run so far | ~600 rad/s el — **8% of the ceiling** |
  | characteristic current ψ/Ld | **31.1 A**, against a 1.2 A i_q ceiling |

  So: **FOC on this rig has never been voltage-limited**, and **field
  weakening is worth ~4% of top speed even if the entire current budget goes
  to the d axis and none to torque** (ψ/Ld is 26× the current limit). Neither
  is the constraint. Build FW in the sim if it is wanted as a capability; do
  not expect it to buy anything on this bench.

  1. **Measure `v_dead`** — *tooling shipped session 28 as profiler stage
     `vdead`; needs no new firmware (it drives `OpenLoopVoltage` at ω_e = 0)
     and is validated against `serve --deadtime` to 1.2% with a model-free
     cross-check that is exact. What remains is running it on the bench:*
     `--only rl,vdead` with the rotor clamped, then compare the ladder's R
     against the probe's — agreement is the check that the extra term is real
     rather than R being mis-assigned — then enable `Foc::deadtime` and
     re-run the low-speed legs. Note the 1.5 A trip caps the ladder near
     `2·i_thresh`, so the fully-saturated regime is not reachable on this
     motor and the fit works between the knees.
  2. **Lower the sensorless floor.** The 150 rad/s el handoff is inherited, not
     derived. With the flux magnitude now honest at low speed and the dead-time
     bias removable, find where the observer actually stops working and set the
     handoff from that. Speed-adaptive `leak` (`leak ∝ |ω̂|`) is the next lever:
     it holds the lag angle constant instead of letting it grow to the π/4 clamp.
  3. **Feed the observer the *realized* voltage.** It currently integrates the
     pre-SVPWM demand, so whenever the modulator clamps — saturation, the duty
     floor, overmodulation — the observer is lied to exactly at the limit.
     Reconstructing v from the duties actually emitted is nearly free.
  4. **Derive the correct `advance_periods`.** `Foc`'s own doc says a backend
     that latches duties a period after sampling should use 1.5; both firmware
     and sim run the 0.5 default. With PWM at 40 kHz and control at 20 kHz the
     right value needs deriving rather than assuming. Costs 0.15 rad (8.6°) of
     angle at 3000 rad/s el, ~2.5° at 865 — real only up high, which is where
     item 6 wants to go.
  5. **Limits hygiene, all in `mmc-core`:** a *circle* limit on the dq current
     reference (d and q are limited independently today, so |i| can exceed the
     hardware trip on any non-zero i_d); back-calculation anti-windup at the
     voltage circle instead of clamping (the six-step duty loop already has
     it); overmodulation to the hexagon, worth +10.3% voltage over `V_bus/√3`.
  6. **Map the real high-speed ceiling on the bench.** Nothing above
     ~600 rad/s el has been tried in FOC, and the six-step review's lesson
     applies twice over: check the coast tail, do not trust a rate that could
     be a clock. The suspects up there are current-sense window (the low-side
     shunts need a minimum low-side on-time, which sets a duty ceiling and
     therefore a speed ceiling), loop delay (item 4), and the observer's own
     `ω·dt` discretization skew, not voltage.

## Backlog (deferred, not yet scheduled)

- **Parameter flash persistence.** *(done 2026-07-20, session 14, fw v7:
  `nvparam` module — CRC32'd blob in the last page of bank 2 (0x0807_F800),
  read-while-write so the erase runs on bank 2 while the ISR executes from
  bank 1; boot range-validates every value against `param_range` so a stale or
  corrupt blob can never brick startup; `SaveParams`/`EraseParams` protocol
  messages gated on a quiet stage; `apply --persist` + panel buttons. Verified
  both directions on hardware. `sequential-storage` proved unnecessary — one
  page, one blob, rewritten whole.)* Landed alongside **VCP connect retry**
  in `link.rs` (retry open + ping-until-Pong over 6 s, killing the
  reset-then-race timeout that cost a manual retry every flash).
- **Profiler ergonomics for a new motor:** *(done 2026-07-17, sessions 8–9)*
  staged/stateful profiler (`--only/--redo/--list/--yes`, `profile_state.json`,
  per-stage bench requirements), per-motor excitation (`--rl-volts`,
  `--sweep-points`, `--accel-targets`), `pole_pairs` as runtime param id 6
  snapshotted into the state file, panel Profiler card. See
  [PROFILER.md](PROFILER.md). Proven on a second motor (4-pole, ψ 21× the
  first).
- **Runtime sensorless-startup parameters.** *(done 2026-07-19, session 12:
  `sl_handoff`/`omega_accel`/`iq_limit` = param ids 7–9, fw v5; plus the
  stall fault (state 8) and the load-adaptive blend taper. Motor 2 runs
  closed-loop sensorless on its complete measured profile — J = 25.1
  µN·m·s², 2.0% speed-step overshoot. See PROGRESS session 12.)*
- From MS5/MS6: HF-injection L probe, d-axis-aligned I-f start. *(blend-kick
  softening + stall detector done in session 12.)*
- **Six-step / trapezoidal drive** — *promoted to **MS8** on 2026-08-02 (see
  Milestones); scheduled ahead of MS7, which is blocked on encoder hardware.*
  The BEMF zero-cross front-end it needs is wired and characterized (session
  13, fw v6: `vb_u/vb_v/vb_w` channels; BEMF2=V on PC3, divider enable PC9),
  and the PWM topology already supports per-phase Hi-Z.
- ~~BEMF as observer input~~ **closed as hardware-limited** (session 13): the
  inverter shield's BEMF net is Schottky-clamped and PWM-corrupted — a coast/zero-cross
  instrument, not a live terminal-voltage sense. Lowering the observer floor
  this way would need filtered/in-line voltage sensing this shield lacks.
- **Zero-speed sensorless torque/position — saliency gate MEASURED: OPEN.**
  The flux observer is useless at ω=0 (no EMF; `observer.rs:18`) and *lies* there
  (locks to −L·i). The only physics that works at standstill is saliency (Ld≠Lq).
  The proposed L(θ) sweep gate was **built and run 2026-07-17** (`test::L_THETA`,
  `mmc-core/src/probe.rs`, `tools/saliency.py`; runbook in
  [PROFILER.md](PROFILER.md)): on the clamped bench motor
  **|ξ| = 0.0737 ± 0.0004 → Lq/Ld ≈ 1.16 — USABLE**, against a predicted ≈1.0
  (likely saturation-induced; all validity checks clean; sim controls recover
  1.5 → 1.484 and 1.0 → NOT-USABLE). *Recommended confirmation:* re-clamp the
  rotor ~45° el away and rerun — real saliency rotates with the rotor, a
  stator-locked gain artifact does not. **Still true:** a production INFORM
  estimator needs di/dt during *active* vectors — low-side shunts only sample
  at the PWM peak — so zero-speed sensorless still implies an in-line-shunt
  front-end (or clever windowing), and X/R ≈ 2 at Nyquist rules out
  rotating-carrier injection regardless. Encoder (MS7) remains the main path;
  the probe doubles as the INFORM measurement primitive if pursued. Design
  notes: `~/.claude/plans/back-to-motor-control-enumerated-hejlsberg.md`.

- **Flying start / live mode switch** — *done 2026-10-08 for a rotor the
  drive can see* (halls, or a locked observer above half the handoff speed;
  see PROGRESS session 35). Open: a hall-less rotor coasting from Off, or
  turning slower than that, needs a probing catch (zero-vector burst).
  Original note: a
  change of drive mode while running is a clean start: θ, ω and amp reset to
  0 and the new mode's blocks are rebuilt with the bridge still live and the
  rotor still turning (the one exception is the live sensorless-FOC →
  six-step handover). Open-loop at speed → I-f or sensorless then applies a
  stationary current vector against full back-EMF (current spike, OC trip or
  hard braking); into sensorless the I-f sequencer restarts from standstill
  under a spinning rotor and can trip the stall fault. The sim's `set_drive`
  does the same, so it will not flag it. Wanted: catch a spinning rotor and
  start the new mode at its speed and angle — seed θ/ω from the outgoing
  mode's angle or the observer, or measure them on entry (coast + BEMF
  zero-cross, or a short zero-vector current burst) — with the PI
  integrators preloaded for a bumpless transfer. Also covers starting into
  an already-coasting motor from Off. Until then, go through Off between
  modes. *(From a review on origin `5bc16cd`.)*
- **Online calibration on halls** *(2026-10-06/07, see
  [CALIBRATION.md](CALIBRATION.md))*: hall sector widths (fw 16), the
  online R/ψ estimator in firmware with i_d injection and dither (fw 17–18),
  flux from a coast, and the position torque around a turn measured and fed
  forward with on-board pole-pair identification (fw 19, nvparam v10).
  Next: better cancellation in reverse, or with the encoder.
- **TODO: one documentation site, with plots from the bench data.**
  *(added 2026-10-09)* The docs are spread over 13 Markdown files
  (`README.md`, `overview.md`, `docs/*.md`, `hw/README.md`,
  `testresults/…/*.md`), hand-built HTML with hand-coded charts
  (`docs/overview.html`, `docs/sixstep-conduction.html` with Chart.js,
  `testresults/index.html`, the feature report) and rustdoc, with no index,
  search or cross-links, and the ~1 000 bench CSVs in `testresults/` only
  reach the docs as numbers typed into tables. Consolidate into one
  generated site whose figures are drawn from those CSVs at build time.
  **Quarto** is the lead option: plain `.md` pages render as they are, pages
  with figures become `.qmd` with Python cells (numpy, as `tools/` already
  uses) that read the captures and draw interactive (Plotly) or static
  (matplotlib) plots with numbered, cross-referenced figures; Mermaid and
  Graphviz diagrams built in; search; PDF output; frozen outputs so CI does
  not re-run every analysis. Alternatives weighed: Sphinx + MyST-NB
  (executed notebooks, strongest API cross-referencing, most setup), MkDocs
  Material (Mermaid built in, execution via plugins), mdBook (static images
  only). Scope: one tree (design, per-subsystem notes such as HFI /
  six-step / FIXQ / calibration / profiler, hardware, bench results, the
  PROGRESS log), the hand-built HTML reports turned into pages, rustdoc
  linked in, a CI job that builds it with warnings as errors (broken links
  fail). Keep the private twin's material out of the public build.

## Verification strategy

- `cargo test` at workspace root: transform round-trips, anti-windup, sim step
  responses within tolerance bands, protocol encode/decode round-trips.
- `cargo build --target thumbv7em-none-eabihf` **and** `--target
  thumbv6m-none-eabi` for the no_std crates: embeddability proven before and
  independent of hardware.
- `mmc-host suite` → `testresults/index.html`: human-inspectable step responses.
- From MS3: host connects to sim over TCP and to the G0B1 over serial with the
  same code path; loopback/echo + telemetry-stream smoke tests.
- From MS4: observer regression tests assert estimated-vs-true angle error stays
  in bounds across speed/load sweeps, including the I-f startup handoff.
- Hardware motor verification is deferred to MS5 with its own smoke-test
  checklist (gate-driver enable, zero-current calibration, open-loop spin, R/L
  sanity) before closed-loop is attempted.
