//! `AngleEstimator` fed directly from simulation ground truth — the stand-in
//! that lets the current/speed loops be developed before the sensorless flux
//! observer (MS4) exists, and the reference every real estimator is compared
//! against in tests.

use mmc_core::angle::AngleEstimator;
use mmc_core::transforms::AlphaBeta;

use crate::motor::PmsmModel;

#[derive(Copy, Clone, Debug, Default)]
pub struct TruthAngle {
    angle: f32,
    velocity: f32,
}

impl TruthAngle {
    /// Copy the current ground truth out of the model; call once per control
    /// period before running the control step.
    pub fn sync(&mut self, motor: &PmsmModel) {
        self.angle = motor.theta_e();
        self.velocity = motor.omega_e();
    }
}

impl AngleEstimator for TruthAngle {
    fn update(&mut self, _i_ab: AlphaBeta, _v_ab: AlphaBeta, _dt: f32) {
        // Truth needs no estimation; refreshed via `sync`.
    }

    fn electrical_angle(&self) -> f32 {
        self.angle
    }

    fn electrical_velocity(&self) -> f32 {
        self.velocity
    }
}
