//! Continuous PMSM model in the rotor (dq) frame, integrated with
//! semi-implicit Euler substeps.

use mmc_core::math::{sin_cos, wrap_angle};
use mmc_core::transforms::{inverse_clarke, inverse_park, park, Abc, AlphaBeta, Dq};

/// Physical parameters of a PMSM, SI units.
#[derive(Copy, Clone, Debug)]
pub struct PmsmParams {
    /// Stator resistance per phase [Ω].
    pub rs: f32,
    /// d-axis inductance [H].
    pub ld: f32,
    /// q-axis inductance [H].
    pub lq: f32,
    /// Permanent-magnet flux linkage [Wb].
    pub flux: f32,
    pub pole_pairs: u32,
    /// Rotor inertia [kg·m²].
    pub inertia: f32,
    /// Viscous friction [N·m·s/rad].
    pub viscous: f32,
}

impl PmsmParams {
    /// A small hobby BLDC, roughly the class of motor used with hobby inverter
    /// inverter shields.
    pub fn small_bldc() -> Self {
        Self {
            rs: 0.5,
            ld: 0.6e-3,
            lq: 0.6e-3,
            flux: 0.008,
            pole_pairs: 7,
            inertia: 1.0e-5,
            viscous: 2.0e-6,
        }
    }

    /// The MS6-profiled bench motor on the G474 + IHM16M1 rig: a
    /// low-inductance surface BLDC with τ = L/R ≈ 31 µs — *under* the 50 µs
    /// control period, which is the regime the R/L and saliency probes are
    /// designed for (`small_bldc`'s τ = 1.2 ms is the opposite regime and
    /// cannot exercise them).
    pub fn bench_g474() -> Self {
        Self {
            rs: 0.904,
            ld: 28e-6,
            lq: 28e-6,
            flux: 0.894e-3,
            pole_pairs: 7,
            inertia: 1.75e-6,
            viscous: 1.0e-6,
        }
    }

    /// Torque per amp of i_q (surface PMSM, i_d = 0) [N·m/A].
    pub fn torque_constant(&self) -> f32 {
        1.5 * self.pole_pairs as f32 * self.flux
    }
}

#[derive(Copy, Clone, Debug)]
pub struct PmsmModel {
    pub params: PmsmParams,
    /// Stator currents in the rotor frame [A].
    pub i_dq: Dq,
    /// Mechanical rotor angle [rad], wrapped.
    pub theta_m: f32,
    /// Mechanical angular velocity [rad/s].
    pub omega_m: f32,
    /// Hold the rotor at standstill (locked-rotor test bench).
    pub locked: bool,
}

impl PmsmModel {
    pub fn new(params: PmsmParams) -> Self {
        Self {
            params,
            i_dq: Dq::default(),
            theta_m: 0.0,
            omega_m: 0.0,
            locked: false,
        }
    }

    /// Electrical rotor angle [rad], wrapped to [-π, π).
    pub fn theta_e(&self) -> f32 {
        wrap_angle(self.theta_m * self.params.pole_pairs as f32)
    }

    /// Electrical angular velocity [rad/s].
    pub fn omega_e(&self) -> f32 {
        self.omega_m * self.params.pole_pairs as f32
    }

    /// Electromagnetic torque [N·m].
    pub fn torque(&self) -> f32 {
        let p = &self.params;
        1.5 * p.pole_pairs as f32
            * (p.flux * self.i_dq.q + (p.ld - p.lq) * self.i_dq.d * self.i_dq.q)
    }

    /// Instantaneous phase currents [A].
    pub fn phase_currents(&self) -> Abc {
        let sc = sin_cos(self.theta_e());
        inverse_clarke(inverse_park(self.i_dq, sc))
    }

    /// One integration substep with the given stator-frame terminal voltage
    /// and external load torque. `dt` must be small relative to L/R
    /// (~1 µs for small motors).
    pub fn step(&mut self, v_ab: AlphaBeta, load_torque: f32, dt: f32) {
        let p = self.params;
        let sc = sin_cos(self.theta_e());
        let v = park(v_ab, sc);
        let we = self.omega_e();

        // dq voltage equations (motor convention).
        let did = (v.d - p.rs * self.i_dq.d + we * p.lq * self.i_dq.q) / p.ld;
        let diq = (v.q - p.rs * self.i_dq.q - we * (p.ld * self.i_dq.d + p.flux)) / p.lq;
        self.i_dq.d += did * dt;
        self.i_dq.q += diq * dt;

        if self.locked {
            self.omega_m = 0.0;
        } else {
            // Semi-implicit: mechanics see the updated currents.
            let acc = (self.torque() - load_torque - p.viscous * self.omega_m) / p.inertia;
            self.omega_m += acc * dt;
            self.theta_m = wrap_angle(self.theta_m + self.omega_m * dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mmc_core::math::sin_cos;
    use mmc_core::transforms::inverse_park;

    /// Locked rotor + DC voltage on the d axis must settle at i_d = v/R.
    #[test]
    fn locked_rotor_dc_settles_at_v_over_r() {
        let mut m = PmsmModel::new(PmsmParams::small_bldc());
        m.locked = true;
        let v_ab = inverse_park(Dq { d: 1.0, q: 0.0 }, sin_cos(m.theta_e()));
        // 20 ms ≫ L/R = 1.2 ms.
        for _ in 0..20_000 {
            m.step(v_ab, 0.0, 1e-6);
        }
        assert!(
            (m.i_dq.d - 1.0 / m.params.rs).abs() < 0.01,
            "i_d = {}",
            m.i_dq.d
        );
        assert!(m.i_dq.q.abs() < 0.01);
    }

    /// Positive q-axis voltage produces positive torque and spins the rotor
    /// in the positive direction.
    #[test]
    fn positive_vq_spins_forward() {
        let mut m = PmsmModel::new(PmsmParams::small_bldc());
        for _ in 0..50_000 {
            let v_ab = inverse_park(Dq { d: 0.0, q: 2.0 }, sin_cos(m.theta_e()));
            m.step(v_ab, 0.0, 1e-6);
        }
        assert!(m.omega_m > 10.0, "omega_m = {}", m.omega_m);
    }

    /// Zero terminal volts is a short across the stator: a spinning rotor
    /// generates braking current and settles to standstill. The low-inertia
    /// electromechanical mode is underdamped, so it may ring through zero on
    /// the way — the invariant is that the stored energy dissipates.
    #[test]
    fn shorted_terminals_brake_to_standstill() {
        let mut m = PmsmModel::new(PmsmParams::small_bldc());
        m.omega_m = 100.0;
        // 200 ms ≫ both electrical and electromechanical time constants.
        for _ in 0..200_000 {
            m.step(AlphaBeta::default(), 0.0, 1e-6);
        }
        assert!(m.omega_m.abs() < 0.5, "omega_m = {}", m.omega_m);
        assert!(
            m.i_dq.d.abs() < 0.1 && m.i_dq.q.abs() < 0.1,
            "currents should decay, i_dq = {:?}",
            m.i_dq
        );
    }
}
