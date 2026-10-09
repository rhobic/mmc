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

/// Blend-taper clamp: the lowest fraction of `i_start` the blend may taper
/// to, and the safety margin over the measured load fraction.
///
/// During the blend the drive angle swings from the forced frame (where the
/// current is mostly magnetizing) to the rotor frame (where it is pure
/// torque), so full startup current arrives as a torque step — the "blend
/// kick" that spiked ω̂ ~1300 rad/s on the bench. But how much current the
/// rotor actually needs depends on load, and the ramp already measured it:
/// the current vector rides the forced frame's q-axis, so an unloaded rotor
/// aligns ~90° from the forced d-axis and load pulls it back toward 0 —
/// with γ = θ̂ − θ_f, the load's share of `i_start` is **cos γ** (motor 1's
/// recorded 1.29 rad hang at 28% load: cos 1.29 = 0.28 ✓). At the blend
/// entry the sequencer tapers toward `cos γ · MARGIN` (clamped) — a light
/// load tapers hard and kills the kick, a heavy load keeps full current and
/// holds sync. The speed loop must be preloaded with
/// `i_start · taper_end()` (signed) to stay bumpless.
pub const BLEND_TAPER_MIN: f32 = 0.4;
const BLEND_TAPER_MARGIN: f32 = 1.3;

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
    /// I-f startup current magnitude [A]. The sequencer applies it signed
    /// with the startup direction: during the blend the drive angle becomes
    /// the *rotor* angle, where q-axis current is pure torque — against the
    /// motion, an unsigned current flings a low-inertia rotor backwards
    /// (found the hard way on hardware; the sim's rotor was too heavy to
    /// show it).
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
    taper_end: f32,
    /// Observer angle minus forced angle, tracked *unwrapped* through the
    /// blend (see `update`).
    blend_diff: f32,
}

impl Sequencer {
    pub fn new(cfg: SequencerCfg) -> Self {
        Self {
            cfg,
            phase: Phase::Ramp,
            theta_f: 0.0,
            omega_f: 0.0,
            blend_elapsed: 0.0,
            taper_end: 1.0,
            blend_diff: 0.0,
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// Skip the I-f ramp and the blend: the rotor is already turning and the
    /// observer already locked (a flying start), so the observer owns the
    /// angle and the speed loop the current from the first tick.
    pub fn start_closed(&mut self) {
        self.phase = Phase::Closed;
    }

    /// Fraction of `i_start` the blend tapered to (load-adaptive, set at the
    /// blend entry; 1.0 until then). Preload the speed loop with
    /// `i_start · taper_end()` signed for a bumpless takeover.
    pub fn taper_end(&self) -> f32 {
        self.taper_end
    }

    /// Advance one control period against the (already-updated) observer.
    pub fn update(&mut self, observer: &impl AngleEstimator, dt: f32) -> SequencerOut {
        let target = self.cfg.omega_handoff;
        let iq_start = self.cfg.i_start * target.signum();
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
                    // Size the blend taper from the measured load: the hang
                    // angle γ (shadow observer vs forced angle) gives the
                    // load's share of i_start as cos γ (see the consts).
                    let gamma = wrap_angle(observer.electrical_angle() - self.theta_f);
                    self.blend_diff = gamma;
                    let load_frac = crate::math::sin_cos(gamma).1.abs();
                    self.taper_end =
                        (load_frac * BLEND_TAPER_MARGIN + 0.1).clamp(BLEND_TAPER_MIN, 1.0);
                }
                SequencerOut {
                    theta: self.theta_f,
                    omega: self.omega_f,
                    iq_open: Some(iq_start),
                    phase: Phase::Ramp,
                }
            }
            Phase::Blend => {
                // Forced angle keeps advancing while its weight fades out.
                self.theta_f = wrap_angle(self.theta_f + self.omega_f * dt);
                self.blend_elapsed += dt;
                let alpha = (self.blend_elapsed / self.cfg.blend_time).min(1.0);
                // The gap between observer and forced angle must be tracked
                // continuously, not re-wrapped each tick: once the blend
                // turns the startup current into torque a light rotor
                // outruns the still-ramping forced frame, the gap grows past
                // π, and a re-wrapped gap flips sign — jumping the drive
                // angle by α·2π in one tick. On motor 3 that was a 3.5 rad
                // jump 30 ms into the blend, i_q reversed, overcurrent
                // (session 30b). Unwrapped, it is continuous and still ends
                // on the observer angle (mod 2π) at α = 1.
                let gap = wrap_angle(observer.electrical_angle() - self.theta_f);
                self.blend_diff += wrap_angle(gap - self.blend_diff);
                let theta = wrap_angle(self.theta_f + alpha * self.blend_diff);
                let omega = self.omega_f + alpha * (observer.electrical_velocity() - self.omega_f);
                if alpha >= 1.0 {
                    self.phase = Phase::Closed;
                }
                SequencerOut {
                    theta,
                    omega,
                    // Taper toward taper_end·i_start as the angle turns
                    // torque-aligned — softens the blend kick (see the consts).
                    iq_open: Some(iq_start * (1.0 - (1.0 - self.taper_end) * alpha)),
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

    /// New gains, the integrator kept.
    pub fn set_gains(&mut self, gains: PiGains) {
        self.pi.set_gains(gains);
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
    fn blend_stays_continuous_when_the_rotor_outruns_the_forced_angle() {
        // Rotor already 1.5 rad ahead at the blend and accelerating away:
        // the observer−forced gap passes π mid-blend.
        let cfg = SequencerCfg {
            i_start: 0.6,
            accel: 1000.0,
            omega_handoff: 350.0,
            blend_time: 0.05,
        };
        let mut seq = Sequencer::new(cfg);
        let dt = 1e-4;
        let mut obs = Fixed {
            theta: 0.0,
            omega: 0.0,
        };
        let mut theta_f = 0.0f32;
        let mut omega_f = 0.0f32;
        let mut last: Option<f32> = None;
        let mut worst = 0.0f32;
        for _ in 0..8_000 {
            // Stand-in rotor: tracks the ramp 1.5 rad ahead, then pulls away
            // at 3000 rad/s² once the blend starts.
            if seq.phase() == Phase::Ramp {
                omega_f = (omega_f + cfg.accel * dt).min(cfg.omega_handoff);
                theta_f = wrap_angle(theta_f + omega_f * dt);
                obs.omega = omega_f;
                obs.theta = wrap_angle(theta_f + 1.5);
            } else {
                obs.omega += 3000.0 * dt;
                obs.theta = wrap_angle(obs.theta + obs.omega * dt);
            }
            let out = seq.update(&obs, dt);
            if let Some(prev) = last {
                // Expected per-tick motion is ω·dt < 0.1 rad; a wrap flip
                // would show as a step of ~α·2π.
                worst = worst.max(wrap_angle(out.theta - prev).abs());
            }
            last = Some(out.theta);
        }
        assert_eq!(seq.phase(), Phase::Closed);
        assert!(worst < 0.2, "drive angle stepped {worst} rad in one tick");
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
            if let Some(iq) = out.iq_open {
                // Startup torque must point in the startup direction: on the
                // blended (rotor) angle, +q against a reverse spin flings the
                // rotor forward.
                assert!(iq < 0.0, "startup current not sign-matched: {iq}");
            }
        }
        assert!(min_omega <= -199.0, "reached {min_omega}");
        assert_eq!(seq.phase(), Phase::Closed);
    }
}
