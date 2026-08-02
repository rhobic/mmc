# Profiler runbook

How to measure a motor with `mmc-host profile`, fit the results, and push the
parameters back to the device. The profiler is staged and stateful: every
stage declares what it measures and what it expects from the bench, completed
stages are tracked in `<dir>/profile_state.json`, and any subset can be
(re)run. **No stage requires mechanically locking the rotor.**

## Quick start (full profile, new motor)

```
mmc-host profile --list                          # see the stages first
mmc-host profile --serial auto --baud 1000000 --dir testresults/<motor>
python tools/profile.py testresults/<motor>      # fit R, L, psi, J + gains
python tools/saliency.py testresults/<motor>     # Ld/Lq verdict
mmc-host apply --serial auto --baud 1000000 --profile testresults/<motor>/profile.json
```

`profile` prints the bench setup it needs and waits for Enter (`--yes` skips).
Parameters applied with `apply` take effect at the next clean drive start and
live in **RAM only** — re-apply after a power cycle (flash persistence is a
backlog item). The panel's parameter card does the same job interactively.

## Startup parameters (new motors without a reflash)

Sensorless startup is governed by three runtime params (set them in the panel
or via `apply`): **sl_handoff** (I-f→observer handoff speed, rad/s el — above
the observer's ~80 rad/s floor, below what the motor reaches open-loop from
rest), **omega_accel** (every commanded ramp's acceleration, rad/s² el — a
heavy/high-drag rotor needs it lowered; the accel-stage J fit reads the value
actually used from the capture snapshot), and **iq_limit** (current ceiling
for I-f/sensorless, ≤1.2 A). The QBL5704 runs at 100 / 250 / 1.0 vs the
motor-1 defaults 150 / 500 / 0.8.

Two protections ride along: a **stall fault** (state 8) trips ~100 ms after
the observer's flux magnitude collapses below 0.35·ψ in closed loop — the
"confident fake lock" a stalled rotor produces — and re-arms via STOP/Off
like any fault; and the blend now **tapers the startup current adaptively**
(the ramp's hang angle measures the load, so a light load tapers hard to kill
the handoff kick while a heavy load keeps full current).

## Persisting parameters to flash

All ten runtime params live in RAM and reset to firmware defaults on
power-cycle **unless** persisted. Once a motor's profile is dialed in:

- `mmc-host apply <profile.json> --persist` writes the applied table to flash.
- The panel's parameter card has **Save to flash** / **Erase flash** buttons.

The firmware restores the table at boot: a CRC32'd blob in the last page of
flash **bank 2** (0x0807F800). Load is a plain memory-mapped read; a corrupt,
stale-format, or absent blob is silently ignored and defaults load, and each
value is range-checked against `param_range` before it's accepted, so a bad
save can never brick startup. Erase reverts to firmware defaults on next boot.

Both operations need the **drive off** — the ~22 ms page erase runs on flash
bank 2 while the control ISR keeps executing from bank 1 (read-while-write),
but the firmware still gates save/erase on a quiet stage and NAKs otherwise.
After a reflash the blob survives (bank 2 is untouched by the bank-1 program),
so a dialed-in motor no longer needs its params re-entered by hand.

## The stages

| id | rotor | measures | needs |
|---|---|---|---|
| `sweep` | free-spinning | flux linkage ψ (→ kt) from 5 rotating I-f points | shaft spins freely, no load |
| `accel` | free-spinning | inertia J + friction from a sensorless 300→900 rad/s el step | shaft spins freely; R/L/ψ already roughly right |
| `rl` | parks rotor | R (differential) and L (folded exponential) from a d-axis square wave | nothing — aligns itself; clamp is fine |
| `saliency` | parks rotor | Ld/Lq ratio — the zero-speed-sensorless gate | nothing — ± angle pairing cancels net torque; clamp gives cleanest data |

Run order is enforced (spinning stages before parking probes): a probe-parked
rotor released straight into an I-f start sits on the torque-well separatrix
and stalls — the MS6 sequencing lesson.

`--only rl,saliency` selects a subset (e.g. with a clamped shaft);
`--redo` reruns completed stages; `--addr 127.0.0.1:7770` targets a sim
server instead of hardware.

### Per-motor excitation (`StageTuning`)

The default excitation is sized for the small bench BLDC. A different motor
usually needs different numbers — all settable per run:

- `--rl-volts a,b` — probe voltages; lower for low-R motors (trip = 1.5 A).
- `--sweep-points 0.6@60,0.9@90,…` — I-f amps@ω_e. Two constraints: the
  motor must hold sync **from rest** through the 500 rad/s² el ramp (heavier
  rotor / more friction ⇒ more amps, lower speeds), and ω·ψ must stay under
  the bus ceiling ω_max ≈ 0.7·(VBUS/√3)/ψ (the 4-pole bench motor at
  ψ≈19 mWb caps near 240 rad/s el on 12 V). Slipped points are refused by
  the fit — wrong values fail loudly.
- `--accel-targets lo,hi` — sensorless plateau speeds; recorded into
  `profile_state.json` so the fit uses what actually ran.

The same overrides ride the panel API (`"rl_volts"`, `"sweep"`, `"accel"`
keys on the profile command).

### Running from the control panel

The panel (`mmc-host panel`) has a **Profiler** card: pick stages, Run —
captures land in `--profile-dir` (default `testresults/panel-profile`), the
Python fits run automatically and stream into the card's log, and **Apply**
pushes the reviewed `profile.json` to the device. While it runs the charts
freeze and other controls are ignored (the stages command their own drives
and end with the stage off). Set **pole pairs** in the parameters card
first — it scales the kt/J fits and the speed display.

## Bootstrapping a brand-new motor (iterative)

The stages have different prerequisites: `rl` is robust on any seed, `sweep`
needs a working current loop (R, L right-ish), `accel` needs a working
sensorless startup (ψ right too). On a cold motor, iterate:

1. `profile --only rl` → `profile.py` → `apply` (R, L land; partial JSON is fine — `apply` skips absent keys)
2. `profile --only sweep --redo` → fit → `apply` (ψ lands)
3. `profile --only accel --redo` → fit → `apply` (J, friction, speed gains land)

Also update `POLE_PAIRS` in `tools/profile.py` (and the firmware telemetry
constant) for a motor with a different pole count — it scales kt and J.

## The saliency verdict

`tools/saliency.py` fits the ratio ξ = Γ1/Γ0 (in 1/L) from the cross-axis
current transient — a null channel that exists only if Ld ≠ Lq. The estimator
cancels the unknown sample latency, so the ±30% absolute-L systematic does
not touch it. Verdicts:

- **|ξ| > 0.05 — USABLE**: INFORM-style zero-speed sensing has signal on this
  motor; this probe is the measurement primitive it would use.
- **|ξ| < 0.02 — NOT USABLE**: surface-PMSM behavior; zero-speed sensorless
  is off the table, the encoder (MS7) is the path.
- **between — AMBIGUOUS**: re-clamp the rotor ~45° el away and rerun; real
  saliency rotates with the rotor, a stator-locked artifact does not. (The
  same re-clamp check is worth one run even on a clear USABLE verdict.)

With a clamped rotor at an unknown angle the fitted axis is ambiguous by 90°
el, so ξ's sign is not meaningful — only |ξ| is. Validity checks (plateau-
step spread, even/odd-cycle θ_r agreement, j-consistency) print warnings when
the rotor moved or the model misfits, and a **MEASUREMENT INVALID** verdict
(with the fix) when the square wave never settled — which says nothing about
the motor.

**Order matters:** the sweep's square-wave half-period and its post-edge fit
samples are picked from τ = L/R, the firmware from its **live parameters**
and the fit from the recorded header. Run `rl`, fit, **Apply**, *then* run
`saliency` — with stale R/L the firmware picks a half-period the motor can't
settle in (τ spans 31 µs → 360 µs across the two bench motors alone).

## Comparing to a meter or datasheet (and entering hand measurements)

The probes measure R as the **drive path**: winding plus the board's low-side
shunt (0.33 Ω, duty-weighted) and the STSPIN830's conducting switch (~0.5 Ω) —
about **0.85 Ω** on the G474+IHM16M1 rig (`R_DRIVE_PATH` in
`mmc-host/src/profile.rs`; derivation in
`testresults/motor2-4pole/datasheet-comparison.md`). That is the correct R for
the control loop, but it is several times any datasheet or multimeter figure.
The tools convert for you (wye winding assumed, line-line = 2 × per-phase):

- `profile.py` / `saliency.py` print **"At the motor"** estimates — R and L in
  line-line and per-phase form, path R subtracted — next to the drive-path
  values. The path R rides in the capture's device snapshot (0 for the sim,
  whose model has no inverter resistance), so old and sim captures normalize
  correctly.
- The panel's parameter card shows the same live estimate under the fields,
  and has a **"from a meter"** entry: type the line-line R (Ω) and/or L (mH)
  measured at the motor terminals and it sets the r/l params with the
  conversion applied (r = R_ll/2 + path, l = L_ll/2). Measuring the QBL5704's
  datasheet values this way lands within ~3% of the profiled r.

## Live resistance monitoring (panel)

The control panel shows **R̂ apparent** — `(v·i − ω_e·ψ·i_q)/|i|²` averaged
over ~1 s of telemetry — with a ΔT estimate from copper's 0.39%/°C tempco.
It is exact at standstill (no EMF term) and leans on the ψ parameter at
speed; it includes the inverter drop (this is the MS6 "apparent R", the one
the running control loop actually sees). Use it as a winding-temperature
trend indicator; the tile turns amber/red past +12%/+25%.

## Testing without hardware

The sim server implements both probes with firmware-identical schedules:

```
mmc-host serve --port 7770 --ctrl-freq 20000 --motor bench --saliency 1.5
mmc-host profile --addr 127.0.0.1:7770 --only rl,saliency --yes --dir /tmp/p
```

`--motor bench` is the profiled G474 bench motor (28 µH — the τ < Ts regime
the probes are designed for); `--saliency` sets Lq/Ld. A positive control
(1.5 → fit recovers ≈1.48) and negative control (1.0 → NOT USABLE) validate
the whole pipeline; run both whenever the fit or schedule changes — on
hardware alone, "no saliency" and "broken fit" are indistinguishable.

## Troubleshooting

- **"device refused the probe"** — the printed reason is the firmware NAK:
  still calibrating (wait ~0.5 s after power-up), busy/faulted (send STOP or
  `SetDrive Off` to re-arm), or firmware too old for the probe kind
  (`GetInfo` fw ≥ 4 has `saliency`).
- **No response at all** — after `probe-rs download`, the core may be left
  halted: `probe-rs reset --chip STM32G474RETx` and retry.
- **Probes are self-terminating** (~0.5 s of drive) and every fault/deadman
  path hands back the partial buffer, so a killed probe never hangs the host.
- The firmware clamps the saliency-sweep voltages to keep the plateau under
  0.75 × the 1.5 A trip using its live R parameter — if R is badly wrong
  (fresh board, unprofiled), run `rl` + `apply` first.
