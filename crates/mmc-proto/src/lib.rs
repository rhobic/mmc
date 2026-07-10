//! Telemetry and command wire protocol shared by firmware and host.
//!
//! Skeleton crate — fleshed out in the telemetry milestone (MS3): compact
//! binary telemetry frames with channel selection, decimation and timestamps,
//! plus a small command set. Transport-agnostic by design: the simulator
//! serves it over TCP and the firmware over USB CDC / UART-DMA, so the same
//! host tooling works against both.

#![no_std]
