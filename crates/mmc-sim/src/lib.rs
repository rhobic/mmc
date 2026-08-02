//! Virtual motor rig: motor models, an ideal inverter and sensor models,
//! implementing the `mmc-hal` traits so the real `mmc-core` control loop runs
//! against it unchanged — in tests, in CI, and interactively via `mmc-host`.
//!
//! Two motor models live here, one per control topology. [`motor`] is the dq
//! model FOC needs, where all three phases are driven. [`phase_motor`] is the
//! phase-domain model six-step needs, where one phase floats and its terminal
//! voltage is an output rather than an input.

pub mod analysis;
pub mod motor;
pub mod phase_motor;
pub mod sensorless_rig;
pub mod sixstep_rig;
pub mod truth;
pub mod virtual_motor;

pub use motor::{PmsmModel, PmsmParams};
pub use phase_motor::{BemfShape, PhaseMotor, SamplePoint};
pub use sensorless_rig::{Sample, SensorlessRunCfg, SensorlessSim};
pub use sixstep_rig::{Mode, SixStepCfg, SixStepSample, SixStepSim};
pub use truth::TruthAngle;
pub use virtual_motor::VirtualMotor;
