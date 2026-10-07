//! Online estimation of stator resistance and magnet flux.
//!
//! The steady dq voltage equations of a PMSM are linear in `R` and `ψ`:
//!
//! ```text
//! v_d − L_d·di_d/dt + ω·L_q·i_q = R·i_d
//! v_q − L_q·di_q/dt − ω·L_d·i_d = R·i_q + ψ·ω
//! ```
//!
//! so a Kalman filter on those rows (the parameters modelled as a slow random
//! walk) tracks both while the drive runs. Both drift with temperature (copper +0.39 %/K, NdFeB about
//! −0.1 %/K), and both feed the flux observer and the current loop, so a
//! drive that tracks them stays calibrated as it warms.
//!
//! # What makes them observable
//!
//! The q row alone cannot separate them on a fast motor: at speed `ψ·ω`
//! dwarfs `R·i_q` (motor 3 at 400 rad/s el: 2.7 V against 0.16 V), so a 1 %
//! error in the voltage is a 17 % error in `R`. The d row carries `R` with
//! no `ψ` in it, but only when `i_d ≠ 0`, which an `i_d = 0` FOC never
//! provides. A small injected `i_d` (no torque on a surface magnet) makes `R`
//! directly observable, so each parameter gets the row that sees it best:
//! `R` from the d row alone, `ψ` from the q row with `R̂·i_q` (a small term)
//! subtracted. Letting the q row also pull on `R` hands every q-axis
//! transient to the parameter it says least about. Rows are only used when
//! they carry information (see [`RpsiCfg::omega_min`] and
//! [`RpsiCfg::i_min`]).
//!
//! On a real bridge the d row also carries an offset (dead-time
//! compensation residue, voltage-angle error), which a constant `i_d`
//! cannot tell from `R`. A third state absorbs it, and `R` then comes from
//! *steps* in the injected `i_d`: the slope, whatever the offset.
//!
//! # Feed it averages
//!
//! Sample by sample, the current loop's own dynamics make `i_d` and `v_d`
//! wiggle together with a slope that is not `R` (the PI answers a current
//! error with voltage), and a filter fitting raw samples learns that slope
//! too: on motor 3 it read as low as −0.27 Ω against a static 1.4 Ω.
//! [`RpsiAverager`] hands the estimator block means instead (tens of ms:
//! long against the current loop, short against the dither), which keeps the
//! static relationship and drops the dynamic one. Bench results in
//! `docs/CALIBRATION.md`.
//!
//! The random walk, rather than exponential forgetting, is deliberate: with
//! forgetting, a direction the data does not excite (R and ψ along a fixed
//! operating point) inflates exponentially and has to be clamped, and the
//! clamp then freezes the excited direction too. A random walk grows the
//! uncertainty linearly, at a rate stated in physical units, so the estimate
//! holds still when it has nothing to learn and tracks when it does.
//!
//! `R` here is whatever sits between the commanded voltage and the back-EMF:
//! winding plus bridge path, the same quantity the R/L probe measures.

use crate::transforms::Dq;

#[derive(Copy, Clone, Debug)]
pub struct RpsiCfg {
    pub ld: f32,
    pub lq: f32,
    /// Random-walk variance added per sample: how far R [Ω], ψ [Wb] and the
    /// d-axis voltage bias [V] may drift between samples, squared.
    pub q_r: f32,
    pub q_psi: f32,
    pub q_bias: f32,
    /// Variance of one row's voltage error [V²] (noise, dead-time residue).
    pub noise: f32,
    /// Below this |ω_e| [rad/s] the q row carries no ψ information and is
    /// skipped.
    pub omega_min: f32,
    /// Below this |i_d| [A] the d row is skipped.
    pub i_min: f32,
    /// Use `L·di/dt` (needs samples well inside L/R); otherwise the rows are
    /// treated as steady state.
    pub use_derivative: bool,
    /// Initial and maximum variance, in the scaled units (`R` in Ω², `ψ` in
    /// units of [`PSI_SCALE`]², bias in V²).
    pub p0: f32,
}

/// ψ is estimated as `ψ·PSI_SCALE` so the parameters are of order one.
pub const PSI_SCALE: f32 = 1000.0;

const N: usize = 3;

/// A Kalman-filter estimate of `R`, `ψ`, and a d-axis voltage bias.
///
/// The bias absorbs what the d row cannot attribute to `R·i_d`: dead-time
/// compensation residue and any error in the voltage angle (at speed, a
/// small angle error moves `ω·ψ·sin δ` onto the d axis). With a constant
/// injected `i_d` it is indistinguishable from `R`, so the estimate of `R`
/// comes from *changes* in `i_d`: step the injection between two levels
/// and `R` is the slope, whatever the offset.
#[derive(Copy, Clone, Debug)]
pub struct RpsiEstimator {
    pub cfg: RpsiCfg,
    /// `[R, ψ·PSI_SCALE, b_d]`.
    theta: [f32; N],
    p: [[f32; N]; N],
    prev_i: Option<Dq>,
    /// Prediction error of the last q and d rows [V], before the update.
    pub residual: Dq,
    /// Samples that carried information.
    pub updates: u32,
}

impl RpsiEstimator {
    /// Start from prior values (e.g. the profiler's) and zero bias.
    pub fn new(cfg: RpsiCfg, r0: f32, psi0: f32) -> Self {
        let mut p = [[0.0; N]; N];
        for (k, row) in p.iter_mut().enumerate() {
            row[k] = cfg.p0;
        }
        Self {
            cfg,
            theta: [r0, psi0 * PSI_SCALE, 0.0],
            p,
            prev_i: None,
            residual: Dq::default(),
            updates: 0,
        }
    }

    pub fn r(&self) -> f32 {
        self.theta[0]
    }

    pub fn psi(&self) -> f32 {
        self.theta[1] / PSI_SCALE
    }

    /// The d-axis voltage the model could not explain by `R·i_d` [V].
    pub fn bias_d(&self) -> f32 {
        self.theta[2]
    }

    /// 1-σ uncertainty of `R` [Ω] and `ψ` [Wb].
    pub fn sigma(&self) -> (f32, f32) {
        (
            libm::sqrtf(self.p[0][0]),
            libm::sqrtf(self.p[1][1]) / PSI_SCALE,
        )
    }

    /// One sample: commanded voltage, measured current, electrical speed,
    /// time since the previous sample. Returns whether anything was learned.
    pub fn update(&mut self, v: Dq, i: Dq, omega_e: f32, dt: f32) -> bool {
        let c = self.cfg;
        let (didt, dqdt) = match (self.prev_i, c.use_derivative && dt > 0.0) {
            (Some(p), true) => ((i.d - p.d) / dt, (i.q - p.q) / dt),
            _ => (0.0, 0.0),
        };
        self.prev_i = Some(i);
        let mut used = false;
        // q row: y = ψ·ω, with the (small) resistive drop taken from R̂.
        if libm::fabsf(omega_e) >= c.omega_min {
            let y = v.q - c.lq * dqdt - omega_e * c.ld * i.d - self.theta[0] * i.q;
            let phi = [0.0, omega_e / PSI_SCALE, 0.0];
            self.residual.q = y - dot(&phi, &self.theta);
            self.measure(phi, y);
            used = true;
        }
        // d row: y = R·i_d + b_d.
        if libm::fabsf(i.d) >= c.i_min {
            let y = v.d - c.ld * didt + omega_e * c.lq * i.q;
            let phi = [i.d, 0.0, 1.0];
            self.residual.d = y - dot(&phi, &self.theta);
            self.measure(phi, y);
            used = true;
        }
        // The parameters may have drifted since the last sample.
        self.p[0][0] += c.q_r;
        self.p[1][1] += c.q_psi * PSI_SCALE * PSI_SCALE;
        self.p[2][2] += c.q_bias;
        if used {
            self.updates += 1;
        }
        // Bound the covariance over a long quiet stretch, scaling the whole
        // matrix (never one element) so it stays positive definite.
        let big = (0..N).map(|k| self.p[k][k]).fold(0.0f32, f32::max);
        if big > c.p0 {
            let s = c.p0 / big;
            for row in self.p.iter_mut() {
                for x in row.iter_mut() {
                    *x *= s;
                }
            }
        }
        used
    }

    /// Kalman measurement update for one row `y = φ·θ + noise`.
    fn measure(&mut self, phi: [f32; N], y: f32) {
        let p = self.p;
        let pphi: [f32; N] = core::array::from_fn(|r| dot(&p[r], &phi));
        let denom = self.cfg.noise + dot(&phi, &pphi);
        let k: [f32; N] = core::array::from_fn(|r| pphi[r] / denom);
        let e = y - dot(&phi, &self.theta);
        for r in 0..N {
            self.theta[r] += k[r] * e;
            for c in 0..N {
                self.p[r][c] = p[r][c] - k[r] * pphi[c];
            }
        }
    }
}

/// Block-averages samples for an [`RpsiEstimator`]: sums over `block`
/// seconds of running, then one estimator update with the means.
#[derive(Copy, Clone, Debug)]
pub struct RpsiAverager {
    pub est: RpsiEstimator,
    /// Block length [s].
    pub block: f32,
    sum: [f32; 5],
    elapsed: f32,
    n: u32,
}

impl RpsiAverager {
    pub fn new(est: RpsiEstimator, block: f32) -> Self {
        Self {
            est,
            block,
            sum: [0.0; 5],
            elapsed: 0.0,
            n: 0,
        }
    }

    /// One sample. `running` false (drive off, faulted, starting) discards
    /// the block in progress, so a block never mixes regimes. Returns true
    /// when a block was completed and fed to the estimator.
    pub fn push(&mut self, v: Dq, i: Dq, omega_e: f32, dt: f32, running: bool) -> bool {
        if !running {
            self.reset_block();
            return false;
        }
        for (s, x) in self.sum.iter_mut().zip([v.d, v.q, i.d, i.q, omega_e]) {
            *s += x;
        }
        self.n += 1;
        self.elapsed += dt;
        if self.elapsed < self.block {
            return false;
        }
        let k = 1.0 / self.n as f32;
        let m: [f32; 5] = core::array::from_fn(|j| self.sum[j] * k);
        let dt_block = self.elapsed;
        self.reset_block();
        self.est.update(
            Dq { d: m[0], q: m[1] },
            Dq { d: m[2], q: m[3] },
            m[4],
            dt_block,
        )
    }

    fn reset_block(&mut self) {
        self.sum = [0.0; 5];
        self.elapsed = 0.0;
        self.n = 0;
    }
}

fn dot(a: &[f32; N], b: &[f32; N]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: f32 = 0.356e-3;

    fn cfg() -> RpsiCfg {
        RpsiCfg {
            ld: L,
            lq: L,
            q_r: 1e-8,
            q_psi: 1e-14,
            q_bias: 1e-9,
            noise: 1e-4,
            omega_min: 50.0,
            i_min: 0.05,
            use_derivative: false,
            p0: 1.0,
        }
    }

    /// Steady operating points from an exact motor (R 1.42, ψ 6.645 mWb):
    /// from wrong priors the estimate converges once speed and i_d vary.
    #[test]
    fn converges_on_exact_steady_data() {
        let (r, psi) = (1.42f32, 6.645e-3f32);
        let mut e = RpsiEstimator::new(cfg(), 1.0, 5.0e-3);
        for k in 0..20_000 {
            let w = 200.0 + 400.0 * ((k / 2000) % 3) as f32;
            let id = if (k / 500) % 2 == 0 { -0.2 } else { -0.6 };
            let iq = 0.11 + 0.0001 * w;
            let v = Dq {
                d: r * id - w * L * iq,
                q: r * iq + w * (L * id + psi),
            };
            e.update(v, Dq { d: id, q: iq }, w, 1e-3);
        }
        assert!((e.r() - r).abs() < 1e-3, "R {}", e.r());
        assert!((e.psi() - psi).abs() < 1e-6, "psi {}", e.psi());
    }

    /// With i_d = 0 at one fixed operating point nothing separates R from ψ:
    /// the estimate must not wander off while it waits for excitation.
    #[test]
    fn holds_still_without_excitation() {
        let (r, psi) = (1.42f32, 6.645e-3f32);
        let mut e = RpsiEstimator::new(cfg(), r, psi);
        for _ in 0..50_000 {
            let (w, iq) = (400.0, 0.11);
            let v = Dq {
                d: -w * L * iq,
                q: r * iq + w * psi,
            };
            e.update(v, Dq { d: 0.0, q: iq }, w, 1e-3);
        }
        assert!((e.r() - r).abs() < 1e-3 && (e.psi() - psi).abs() < 1e-6);
    }

    /// A constant d-axis voltage error (dead-time residue) would read as a
    /// resistance error at fixed i_d; stepping i_d between two levels
    /// recovers R exactly and puts the offset in the bias.
    #[test]
    fn stepped_injection_separates_r_from_a_voltage_offset() {
        let (r, psi, off) = (1.42f32, 6.645e-3f32, 0.12f32);
        let mut e = RpsiEstimator::new(cfg(), 1.0, psi);
        for k in 0..20_000 {
            let w = 400.0;
            let id = if (k / 250) % 2 == 0 { -0.2 } else { -0.6 };
            let iq = 0.11;
            let v = Dq {
                d: r * id - w * L * iq + off,
                q: r * iq + w * (L * id + psi),
            };
            e.update(v, Dq { d: id, q: iq }, w, 1e-3);
        }
        assert!((e.r() - r).abs() < 2e-3, "R {}", e.r());
        assert!((e.bias_d() - off).abs() < 2e-3, "bias {}", e.bias_d());
    }

    /// R rises 6 % mid-run (the winding warming): the estimate follows.
    #[test]
    fn tracks_a_resistance_step() {
        let psi = 6.645e-3f32;
        let mut e = RpsiEstimator::new(cfg(), 1.42, psi);
        let mut r = 1.42f32;
        for k in 0..30_000 {
            if k == 10_000 {
                r *= 1.06;
            }
            let w = 400.0;
            let id = if (k / 300) % 2 == 0 { -0.2 } else { -0.5 };
            let iq = 0.11;
            let v = Dq {
                d: r * id - w * L * iq,
                q: r * iq + w * (L * id + psi),
            };
            e.update(v, Dq { d: id, q: iq }, w, 1e-3);
        }
        assert!((e.r() - 1.42 * 1.06).abs() < 3e-3, "R {}", e.r());
    }
}
