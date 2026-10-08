//! Sensorless rotor-angle estimation: stator-flux observer + PLL.
//!
//! The foundation `AngleEstimator` (see `angle`). Voltage-model observer for
//! a surface PMSM:
//!
//! ```text
//! ψ_s = ∫ (v_αβ − R·i_αβ) dt      (leaky integral, cutoff `leak`)
//! ψ_r = ψ_s − L·i_αβ              (rotor flux, aligned with the d-axis)
//! ```
//!
//! A PLL tracks the rotor-flux direction: the normalized cross product
//! `sin(θ − θ̂)` drives a PI whose integrator is the speed estimate. The leaky
//! integrator (drift protection) has transfer `jω/(jω+leak)` at speed ω, so
//! the flux estimate **leads** the true angle by `atan(leak/ω)`;
//! [`FluxObserver::electrical_angle`] subtracts that analytically, so the
//! reported angle is unbiased once the PLL is locked.
//!
//! Limits by design: useless at standstill (no EMF — that is what the I-f
//! startup in [`crate::sensorless`] is for) and increasingly noisy below
//! roughly `2·leak` electrical rad/s.

use crate::angle::AngleEstimator;
use crate::math::{sin_cos, sqrt, wrap_angle, FRAC_PI_4};
use crate::transforms::AlphaBeta;

#[derive(Copy, Clone, Debug)]
pub struct FluxObserverCfg {
    /// Stator resistance per phase [Ω].
    pub rs: f32,
    /// Stator inductance [H] (surface PMSM: Ld ≈ Lq).
    pub ls: f32,
    /// Leaky-integrator cutoff [rad/s]. Trades DC-drift rejection against
    /// low-speed phase lag (compensated, but the compensation saturates).
    pub leak: f32,
    /// PLL natural frequency [rad/s]; damping is fixed at ζ = 1.
    pub pll_bw: f32,
}

impl FluxObserverCfg {
    /// Sensible defaults for a small hobby PMSM given R and L.
    pub fn new(rs: f32, ls: f32) -> Self {
        Self {
            rs,
            ls,
            leak: 20.0,
            pll_bw: 300.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FluxObserver {
    cfg: FluxObserverCfg,
    /// Stator flux estimate [Wb].
    psi: AlphaBeta,
    /// PLL angle state (uncompensated flux angle).
    theta: f32,
    /// PLL integrator = speed estimate [rad/s electrical].
    omega: f32,
    /// Last rotor-flux magnitude [Wb] (diagnostic; ≈ motor flux linkage).
    flux_mag: f32,
}

impl FluxObserver {
    pub fn new(cfg: FluxObserverCfg) -> Self {
        Self {
            cfg,
            psi: AlphaBeta::default(),
            theta: 0.0,
            omega: 0.0,
            flux_mag: 0.0,
        }
    }

    /// Re-seed the PLL (e.g. at I-f handoff the forced angle is a better
    /// initial guess than wherever the PLL wandered during startup).
    pub fn seed(&mut self, theta: f32, omega: f32) {
        self.theta = wrap_angle(theta + self.lead_compensation_at(omega));
        self.omega = omega;
    }

    /// Put the observer straight into the state it converges to for a rotor
    /// at electrical angle `theta`, speed `omega` and flux linkage `flux`,
    /// with stator current `i_ab` flowing: the leaky integral leads the rotor
    /// flux by `atan(leak/ω)` and is attenuated by the matching factor. A
    /// flying start (catching a rotor that is already turning, from halls or
    /// another estimate) then runs closed loop from the first tick instead
    /// of waiting out the integrator's ~1/leak settling with a wrong angle.
    pub fn prime(&mut self, theta: f32, omega: f32, flux: f32, i_ab: AlphaBeta) {
        let att = self.leak_attenuation_at(omega);
        let (s, c) = sin_cos(theta + self.lead_compensation_at(omega));
        let mag = flux * att;
        self.psi = AlphaBeta {
            alpha: mag * c + self.cfg.ls * i_ab.alpha,
            beta: mag * s + self.cfg.ls * i_ab.beta,
        };
        self.flux_mag = mag;
        self.seed(theta, omega);
    }

    /// Rotor-flux magnitude [Wb]; converges to the magnet flux linkage and
    /// doubles as the lock-quality indicator the stall detector trips on.
    ///
    /// Corrected for the leaky integrator's attenuation, for the same reason
    /// [`Self::electrical_angle`] is corrected for its phase lead: the
    /// transfer `jω/(jω+leak)` costs `|ω|/√(ω²+leak²)` of magnitude, which is
    /// 11% at `ω = 2·leak` and 29% at `ω = leak`. Reporting the raw integral
    /// would make a healthy drive look progressively less healthy the slower
    /// it ran — precisely backwards for a detector whose job is to catch
    /// low-speed stalls.
    pub fn flux_mag(&self) -> f32 {
        self.flux_mag / self.leak_attenuation_at(self.omega)
    }

    /// The uncompensated integral, for diagnostics and for anything that
    /// needs the same quantity the PLL normalizes by.
    pub fn flux_mag_raw(&self) -> f32 {
        self.flux_mag
    }

    /// How far the raw PLL angle leads the true rotor angle: `atan(leak/|ω|)`,
    /// sign-following; clamped to π/4 — below |ω| ≈ leak the estimate is not
    /// trustworthy anyway.
    fn lead_compensation_at(&self, omega: f32) -> f32 {
        let w = omega.abs().max(1e-3);
        let x = self.cfg.leak / w;
        if x >= 1.0 {
            return FRAC_PI_4 * omega.signum();
        }
        // atan(x) ≈ x·(π/4 + 0.273·(1−x)) for x ∈ [0,1], err < 0.005 rad.
        (x * (FRAC_PI_4 + 0.273 * (1.0 - x))) * omega.signum()
    }

    /// Magnitude counterpart of [`Self::lead_compensation_at`]: the leaky
    /// integrator's gain `|ω|/√(ω²+leak²)` = `cos(atan(leak/ω))`. Clamped at
    /// the same place the angle compensation clamps — below `|ω| ≈ leak` the
    /// estimate is not trustworthy, so the correction stops growing rather
    /// than dividing a small number by a smaller one.
    fn leak_attenuation_at(&self, omega: f32) -> f32 {
        let w = omega.abs().max(1e-3);
        let x = self.cfg.leak / w;
        if x >= 1.0 {
            // cos(π/4), matching the lead compensation's own clamp.
            return core::f32::consts::FRAC_1_SQRT_2;
        }
        1.0 / sqrt(1.0 + x * x)
    }
}

impl AngleEstimator for FluxObserver {
    fn update(&mut self, i_ab: AlphaBeta, v_ab: AlphaBeta, dt: f32) {
        let cfg = &self.cfg;
        // Leaky back-EMF integration.
        self.psi.alpha += (v_ab.alpha - cfg.rs * i_ab.alpha - cfg.leak * self.psi.alpha) * dt;
        self.psi.beta += (v_ab.beta - cfg.rs * i_ab.beta - cfg.leak * self.psi.beta) * dt;

        // Rotor flux.
        let ra = self.psi.alpha - cfg.ls * i_ab.alpha;
        let rb = self.psi.beta - cfg.ls * i_ab.beta;
        self.flux_mag = sqrt(ra * ra + rb * rb);

        // PLL: normalized cross product ≈ sin(θ_flux − θ̂).
        let (s, c) = sin_cos(self.theta);
        let err = (rb * c - ra * s) / self.flux_mag.max(1e-6);
        let kp = 2.0 * cfg.pll_bw; // ζ = 1
        let ki = cfg.pll_bw * cfg.pll_bw;
        self.omega += ki * err * dt;
        self.theta = wrap_angle(self.theta + (self.omega + kp * err) * dt);
    }

    fn electrical_angle(&self) -> f32 {
        wrap_angle(self.theta - self.lead_compensation_at(self.omega))
    }

    fn electrical_velocity(&self) -> f32 {
        self.omega
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::PI;
    use crate::transforms::{inverse_park, Dq};

    /// A primed observer is locked from the first tick: angle within the
    /// steady-state bound throughout, where an unprimed one starts a radian
    /// or more off.
    #[test]
    fn primed_observer_is_locked_from_the_start() {
        let (rs, ls, flux) = (0.5, 0.6e-3, 0.008);
        for &omega in &[150.0f32, 400.0, -400.0] {
            let dt = 5e-5;
            let iq = 1.0f32;
            let theta0: f32 = 2.0;
            let run = |primed: bool| {
                let mut obs = FluxObserver::new(FluxObserverCfg::new(rs, ls));
                let mut theta = theta0;
                if primed {
                    let i_ab = inverse_park(Dq { d: 0.0, q: iq }, sin_cos(theta));
                    obs.prime(theta, omega, flux, i_ab);
                }
                let mut worst = 0.0f32;
                for _ in 0..(0.02 / dt) as usize {
                    theta = wrap_angle(theta + omega * dt);
                    let sc = sin_cos(theta);
                    let v_dq = Dq {
                        d: -omega * ls * iq,
                        q: rs * iq + omega * flux,
                    };
                    obs.update(
                        inverse_park(Dq { d: 0.0, q: iq }, sc),
                        inverse_park(v_dq, sc),
                        dt,
                    );
                    worst = worst.max(wrap_angle(obs.electrical_angle() - theta).abs());
                }
                worst
            };
            let (p, u) = (run(true), run(false));
            let bound = 0.03 + 1.5 * omega.abs() * dt;
            assert!(
                p < bound,
                "omega {omega}: primed error {p} rad (bound {bound})"
            );
            assert!(
                u > 5.0 * bound,
                "omega {omega}: unprimed error only {u} rad?"
            );
        }
    }

    /// Ideal SPMSM electrical steady state at constant speed: the observer
    /// must lock to the rotor angle within tolerance, including the leak
    /// compensation.
    #[test]
    fn locks_to_ideal_machine() {
        let (rs, ls, flux) = (0.5, 0.6e-3, 0.008);
        for &omega in &[150.0f32, 400.0, 1200.0, -400.0] {
            let mut obs = FluxObserver::new(FluxObserverCfg::new(rs, ls));
            let dt = 5e-5;
            let iq = 1.0f32;
            let mut theta: f32 = 1.0; // arbitrary start, PLL starts at 0
            let mut worst = 0.0f32;
            let steps = (1.0 / dt) as usize;
            for k in 0..steps {
                theta = wrap_angle(theta + omega * dt);
                let sc = sin_cos(theta);
                // v = R·i + dψ/dt with ψ_r rotating at ω:
                let v_dq = Dq {
                    d: -omega * ls * iq,
                    q: rs * iq + omega * flux,
                };
                let i_ab = inverse_park(Dq { d: 0.0, q: iq }, sc);
                let v_ab = inverse_park(v_dq, sc);
                obs.update(i_ab, v_ab, dt);
                if k > steps / 2 {
                    worst = worst.max(wrap_angle(obs.electrical_angle() - theta).abs());
                }
            }
            // Bias must vanish; what remains is discrete-integration skew of
            // order ω·dt (the FOC's ZOH advance covers this in the loop).
            let bound = 0.03 + 1.5 * omega.abs() * dt;
            assert!(
                worst < bound,
                "omega {omega}: angle error {worst} rad (bound {bound})"
            );
            assert!(
                (obs.electrical_velocity() - omega).abs() < 0.02 * omega.abs(),
                "omega {omega}: speed estimate {}",
                obs.electrical_velocity()
            );
            assert!(
                (obs.flux_mag() - flux).abs() < 0.15 * flux,
                "omega {omega}: flux magnitude {}",
                obs.flux_mag()
            );
        }
    }

    /// The atan approximation used for lead compensation stays within
    /// 0.01 rad over its valid range (|ω| > leak; below that it clamps).
    #[test]
    fn lead_compensation_accuracy() {
        let obs = FluxObserver::new(FluxObserverCfg::new(0.5, 0.6e-3));
        for i in 3..100 {
            let omega = i as f32 * 10.0;
            let exact = libm::atanf(obs.cfg.leak / omega);
            let approx = obs.lead_compensation_at(omega);
            assert!(
                (approx - exact).abs() < 0.01,
                "omega {omega}: approx {approx} exact {exact}"
            );
        }
        assert!((obs.lead_compensation_at(-400.0) + obs.lead_compensation_at(400.0)).abs() < 1e-6);
        assert!(obs.lead_compensation_at(1.0) <= PI / 4.0 + 1e-6);
    }
}
