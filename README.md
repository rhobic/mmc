# mmc — modular motor controller

Portable, simulation-first FOC motor control in Rust. The control core is
`no_std` and allocation-free, so the exact code that will run on an STM32G474
runs on a PC against a virtual motor — including in CI.

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
| `mmc-fw-g0b1` | NUCLEO-G0B1RE protocol firmware (Cortex-M0+, embassy) — outside the host workspace |

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

# NUCLEO-G0B1RE running mmc-fw-g0b1 (flash: cd crates/mmc-fw-g0b1 && cargo run --release)
cargo run -p mmc-host -- capture --serial auto --baud 1000000 --iq 1.0 --out g0b1.csv
```

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
