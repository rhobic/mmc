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
  - **NUCLEO-G474RE** + ST inverter shield (IHM07M1/IHM16M1-class) — the motor
    control target (MS5). Exact shield determines current-sense wiring at
    bring-up time, not architecture.
  - **NUCLEO-G0B1RE** (Cortex-M0+, on the bench since 2026-07-10) — *not* a motor
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
│   ├── mmc-fw-g0b1/     # NUCLEO-G0B1RE protocol/bring-up firmware (Cortex-M0+)
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
  **Amended 2026-07-10:** included NUCLEO-G0B1RE protocol bring-up — the same
  `mmc-proto` over ST-Link VCP UART from a Cortex-M0+ (1 Mbaud, 11 channels at
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
  TIM1 center-aligned PWM + injected-ADC shunt sensing on the IHM16M1,
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
