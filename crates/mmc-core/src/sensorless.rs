//! Sensorless drive orchestration: I-f startup, handoff to the flux
//! observer, and the speed loop that runs on top.
//!
//! A PMSM at standstill produces no back-EMF, so the observer is blind until
//! the rotor moves. The classic fix is I-f startup: force a current vector on
//! a ramped open-loop angle (the rotor follows like a stepper), and once the
//! electrical speed is comfortably above the observer's usable floor, blend
//! from the forced angle to the estimated one and let the speed loop take
//! over. The blend is a shortest-path interpolation over a fixed time, and
//! the speed loop's integrator is preloaded with the startup current, so the
//! transfer is bumpless.

use crate::angle::AngleEstimator;
use crate::math::wrap_angle;
use crate::pi::{Pi, PiGains};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Forced-angle current ramp; observer runs in shadow.
    Ramp,
    /// Interpolating forced → estimated angle.
    Blend,
    /// Closed-loop sensorless: observer angle, speed loop commands i_q.
    Closed,
}

#[derive(Copy, Clone, Debug)]
pub struct SequencerCfg {
    /// I-f startup current [A].
    pub i_start: f32,
    /// Forced acceleration [rad/s² electrical].
    pub accel: f32,
    /// Speed at which the handoff begins [rad/s electrical]. Sign sets the
    /// startup direction; pick ≥ 3–4× the observer's `leak`.
    pub omega_handoff: f32,
    /// Blend duration [s].
    pub blend_time: f32,
}

impl Default for SequencerCfg {
    fn default() -> Self {
        Self {
            i_start: 0.5,
            accel: 400.0,
            omega_handoff: 150.0,
            blend_time: 0.05,
        }
    }
}

/// What the runner applies this control period.
#[derive(Copy, Clone, Debug)]
pub struct SequencerOut {
    /// Electrical angle for the Park transforms.
    pub theta: f32,
    /// Electrical velocity consistent with `theta` (feedforward/decoupling).
    pub omega: f32,
    /// `Some(i_q)` while the sequencer owns the current reference (startup);
    /// `None` once the speed loop should command it.
    pub iq_open: Option<f32>,
    pub phase: Phase,
}

#[derive(Copy, Clone, Debug)]
pub struct Sequencer {
    cfg: SequencerCfg,
    phase: Phase,
    theta_f: f32,
    omega_f: f32,
    blend_elapsed: f32,
}

impl Sequencer {
    pub fn new(cfg: SequencerCfg) -> Self {
        Self {
            cfg,
            phase: Phase::Ramp,
            theta_f: 0.0,
            omega_f: 0.0,
            blend_elapsed: 0.0,
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// Advance one control period against the (already-updated) observer.
    pub fn update(&mut self, observer: &impl AngleEstimator, dt: f32) -> SequencerOut {
        let target = self.cfg.omega_handoff;
        match self.phase {
            Phase::Ramp => {
                let step = self.cfg.accel * dt * target.signum();
                self.omega_f = if target > 0.0 {
                    (self.omega_f + step).min(target)
                } else {
                    (self.omega_f + step).max(target)
                };
                self.theta_f = wrap_angle(self.theta_f + self.omega_f * dt);
                if self.omega_f == target {
                    self.phase = Phase::Blend;
                    self.blend_elapsed = 0.0;
                }
                SequencerOut {
                    theta: self.theta_f,
                    omega: self.omega_f,
                    iq_open: Some(self.cfg.i_start),
                    phase: Phase::Ramp,
                }
            }
            Phase::Blend => {
                // Forced angle keeps advancing while its weight fades out.
                self.theta_f = wrap_angle(self.theta_f + self.omega_f * dt);
                self.blend_elapsed += dt;
                let alpha = (self.blend_elapsed / self.cfg.blend_time).min(1.0);
                let theta = wrap_angle(
                    self.theta_f + alpha * wrap_angle(observer.electrical_angle() - self.theta_f),
                );
                let omega = self.omega_f + alpha * (observer.electrical_velocity() - self.omega_f);
                if alpha >= 1.0 {
                    self.phase = Phase::Closed;
                }
                SequencerOut {
                    theta,
                    omega,
                    iq_open: Some(self.cfg.i_start),
                    phase: Phase::Blend,
                }
            }
            Phase::Closed => SequencerOut {
                theta: observer.electrical_angle(),
                omega: observer.electrical_velocity(),
                iq_open: None,
                phase: Phase::Closed,
            },
        }
    }
}

/// Speed loop: electrical-velocity error in, q-axis current reference out.
#[derive(Copy, Clone, Debug)]
pub struct SpeedLoop {
    pub pi: Pi,
}

impl SpeedLoop {
    pub fn new(gains: PiGains, iq_limit: f32) -> Self {
        Self {
            pi: Pi::new(gains, iq_limit),
        }
    }

    /// Bumpless takeover from the startup current.
    pub fn preload(&mut self, iq: f32) {
        self.pi.preload(iq);
    }

    pub fn update(&mut self, omega_ref: f32, omega_meas: f32, dt: f32) -> f32 {
        self.pi.update(omega_ref - omega_meas, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Stand-in estimator with fixed outputs.
    struct Fixed {
        theta: f32,
        omega: f32,
    }
    impl AngleEstimator for Fixed {
        fn update(
            &mut self,
            _: crate::transforms::AlphaBeta,
            _: crate::transforms::AlphaBeta,
            _: f32,
        ) {
        }
        fn electrical_angle(&self) -> f32 {
            self.theta
        }
        fn electrical_velocity(&self) -> f32 {
            self.omega
        }
    }

    #[test]
    fn ramps_blends_and_closes() {
        let cfg = SequencerCfg {
            i_start: 0.4,
            accel: 1000.0,
            omega_handoff: 200.0,
            blend_time: 0.02,
        };
        let mut seq = Sequencer::new(cfg);
        let obs = Fixed {
            theta: 0.3,
            omega: 210.0,
        };
        let dt = 1e-4;
        let mut t = 0.0;
        let mut reached_blend_at = None;
        for _ in 0..10_000 {
            let out = seq.update(&obs, dt);
            t += dt;
            match out.phase {
                Phase::Ramp => {
                    assert_eq!(out.iq_open, Some(0.4));
                    assert!(out.omega <= 200.0 + 1e-3);
                }
                Phase::Blend => {
                    reached_blend_at.get_or_insert(t);
                }
                Phase::Closed => {
                    assert_eq!(out.iq_open, None);
                    assert_eq!(out.theta, 0.3);
                    assert_eq!(out.omega, 210.0);
                }
            }
        }
        // Ramp to 200 rad/s at 1000 rad/s² takes 0.2 s.
        let blend_at = reached_blend_at.expect("must reach blend");
        assert!((blend_at - 0.2).abs() < 0.01, "blend at {blend_at}");
        assert_eq!(seq.phase(), Phase::Closed);
    }

    #[test]
    fn negative_direction_startup() {
        let mut seq = Sequencer::new(SequencerCfg {
            omega_handoff: -200.0,
            ..SequencerCfg::default()
        });
        let obs = Fixed {
            theta: 0.0,
            omega: -200.0,
        };
        let mut min_omega = 0.0f32;
        for _ in 0..20_000 {
            let out = seq.update(&obs, 1e-4);
            min_omega = min_omega.min(out.omega);
        }
        assert!(min_omega <= -199.0, "reached {min_omega}");
        assert_eq!(seq.phase(), Phase::Closed);
    }
}
