//! Virtual motor rig: a PMSM dq model, an ideal inverter and sensor models,
//! implementing the `mmc-hal` traits so the real `mmc-core` control loop runs
//! against it unchanged — in tests, in CI, and interactively via `mmc-host`.

pub mod analysis;
pub mod motor;
pub mod truth;
pub mod virtual_motor;

pub use motor::{PmsmModel, PmsmParams};
pub use truth::TruthAngle;
pub use virtual_motor::VirtualMotor;
