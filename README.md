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
| `mmc-proto` | Telemetry/command wire protocol shared by firmware and host (skeleton) |
| `mmc-sim` | Virtual PMSM + inverter + sensors, implementing the `mmc-hal` traits |
| `mmc-host` | Host CLI: run simulation scenarios, capture data |

## Quickstart

```sh
cargo test                                   # unit + sim regression tests
cargo run -p mmc-host -- suite               # canonical scenarios -> testresults/ + dashboard
cargo run -p mmc-host -- sim --scenario current-step --out step.csv   # ad-hoc run
cargo build --target thumbv7em-none-eabihf -p mmc-core -p mmc-hal -p mmc-proto  # no_std check
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
