# mmc — modular motor controller

Portable, simulation-first FOC motor control in Rust. The control core is
`no_std` and allocation-free, so the exact code that runs on the MCU also runs
on a PC against a virtual motor — including in CI.

Bench hardware and schematics: [hw/README.md](hw/README.md).

See [overview.md](overview.md) for project goals, [docs/PLAN.md](docs/PLAN.md)
for the architecture and milestone plan, and [docs/PROGRESS.md](docs/PROGRESS.md)
for the running log.

## Workspace layout

| Crate | Purpose |
|---|---|
| `mmc-core` | `no_std` control library: transforms, SVPWM, PI loops, FOC step, angle estimation |
| `mmc-hal` | Hardware abstraction traits sized to motor control (PWM, current sense, …) |
| `mmc-proto` | Telemetry/command wire protocol: COBS+CRC frames shared byte-for-byte by TCP (sim) and UART (firmware) |
| `mmc-sim` | Virtual PMSM + inverter + sensors, implementing the `mmc-hal` traits |
| `mmc-host` | Host CLI: sim scenarios, TCP sim server, telemetry capture (TCP/serial), dashboard |
| `mmc-fw-g0b1` | Protocol firmware for a Cortex-M0+ dev board (embassy) — outside the host workspace |
| `mmc-fw-g474` | Motor firmware for a Cortex-M4F dev board + three-phase inverter shield: timer PWM, injected-ADC shunt sensing, open-loop/I-f drive, protection trips |

## Quickstart

```sh
cargo test                                   # unit + sim regression tests
cargo run -p mmc-host -- suite               # canonical scenarios -> testresults/ + dashboard
cargo run -p mmc-host -- sim --scenario current-step --out step.csv   # ad-hoc run
cargo build --target thumbv7em-none-eabihf -p mmc-core -p mmc-hal -p mmc-proto  # no_std check
```

## Live telemetry (mmc-proto)

The sim serves the wire protocol over TCP; firmware serves the identical bytes
over UART — the host cannot tell the difference:

```sh
cargo run -p mmc-host -- serve                      # sim behind TCP :7770
cargo run -p mmc-host -- capture --addr 127.0.0.1:7770 --iq 0.5 --out step.csv

# M0+ board running mmc-fw-g0b1 (flash: cd crates/mmc-fw-g0b1 && cargo run --release)
cargo run -p mmc-host -- capture --serial auto --baud 1000000 --iq 1.0 --out g0b1.csv
```

## Motor bring-up

`mmc-fw-g474` drives a three-phase inverter shield with low-side shunts; every
test is instrumented through the same protocol. The capture tool can command a
drive mid-recording, and `mmc-host panel` serves a local web control panel
(mode buttons, live charts, STOP, watchdog auto-off):

```sh
cd crates/mmc-fw-g474 && cargo run --release       # flash via probe-rs
cargo run -p mmc-host -- panel --serial auto --baud 1000000   # http://127.0.0.1:8484
cargo run -p mmc-host -- capture --serial auto --baud 1000000 \
    --drive if --amp 0.3 --hz 15 --duration 6 --out testresults/ms5-g474-bringup/d_if_current.csv
```

Firmware safety: zero-current calibration at boot, |i| > 1.5 A / VBUS-window /
gate-driver-fault trips, 2 s host-silence deadman, on-device amplitude clamps.

## Test results

Curated traces live under `testresults/<group>/` as CSV plus a `.meta.json`
provenance sidecar. `mmc-host report` (run automatically by `suite`) rebuilds
the self-contained dashboard at [testresults/index.html](testresults/index.html)
from whatever CSVs are in the tree — open it in a browser. New scenario groups
(and, later, hardware captures) appear on the dashboard automatically.

## Conventions

- SI units throughout; angles in radians, electrical unless suffixed `_m`.
- Amplitude-invariant Clarke transform (the 2/3 convention).
- Torque `T = 1.5·p·(ψ·i_q + (L_d−L_q)·i_d·i_q)`.

## License

Dual-licensed under either [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE),
at your option.
