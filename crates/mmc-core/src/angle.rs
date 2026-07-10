//! Rotor angle/velocity sources.
//!
//! Every consumer of the electrical rotor angle goes through this trait. The
//! sensorless flux observer is the foundation implementation (MS4); the
//! encoder-backed one (MS7) plugs in beside it and calibrates its offset
//! against the observer. `mmc-sim` provides a truth-fed implementation for
//! developing the loops before the observer exists.

use crate::transforms::AlphaBeta;

pub trait AngleEstimator {
    /// Advance the estimate by one control period. Stationary-frame currents
    /// and applied voltages are what a sensorless observer needs;
    /// sensor-backed implementations may ignore them.
    fn update(&mut self, i_ab: AlphaBeta, v_ab: AlphaBeta, dt: f32);

    /// Electrical rotor angle in radians, wrapped to `[-π, π)`.
    fn electrical_angle(&self) -> f32;

    /// Electrical angular velocity in rad/s.
    fn electrical_velocity(&self) -> f32;
}
