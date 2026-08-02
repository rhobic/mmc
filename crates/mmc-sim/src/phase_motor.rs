//! Phase-domain motor model: two phases conducting, one floating.
//!
//! The dq model in [`crate::motor`] assumes all three phases are driven, which
//! is exactly the assumption six-step breaks. This model keeps the same
//! [`PmsmParams`] but carries a different state — one line current through a
//! conducting pair — and can therefore answer the question the dq model cannot:
//! *what voltage appears on the terminal that is not connected to anything?*
//!
//! Both models describe the same machine; they are different coordinate
//! choices for different drive topologies, and each is the natural one for its
//! control scheme. Derivations live in `docs/SIXSTEP.md`.

use mmc_core::math::{sin_cos, wrap_angle};
use mmc_core::sixstep;

use crate::motor::PmsmParams;

/// Shape of the phase back-EMF over one electrical revolution, normalised to
/// unit peak.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BemfShape {
    /// A distributed winding: what the dq model assumes, and what the bench
    /// motors actually are. Six-step on a sinusoidal machine works, but the
    /// torque per amp varies across each sector — that ripple is inherent, not
    /// a tuning defect.
    Sinusoidal,
    /// A concentrated winding with a 120° flat top: the machine six-step was
    /// designed for, where the conducting pair sits on constant back-EMF and
    /// torque per amp is flat within a sector.
    Trapezoidal,
}

impl BemfShape {
    /// Normalised back-EMF of phase `k` (U = 0, V = 1, W = 2) at `theta_e`.
    ///
    /// Sign convention matches the dq model: for the sinusoidal case this is
    /// `−sin(θ − k·120°)`, which reproduces `T = 1.5·p·ψ·i_q` exactly.
    pub fn shape(&self, k: usize, theta_e: f32) -> f32 {
        let off = k as f32 * core::f32::consts::TAU / 3.0;
        let th = theta_e - off;
        match self {
            BemfShape::Sinusoidal => -sin_cos(th).0,
            BemfShape::Trapezoidal => {
                // Put the positive flat top at φ = 0 to match −sin's peak.
                let phi = wrap_angle(th + core::f32::consts::FRAC_PI_2).abs();
                let sixth = core::f32::consts::FRAC_PI_3;
                if phi <= sixth {
                    1.0
                } else if phi <= 2.0 * sixth {
                    1.0 - 2.0 * (phi - sixth) / sixth
                } else {
                    -1.0
                }
            }
        }
    }
}

/// Conducting-switch drops, used only to place the *terminal* voltages.
///
/// Their effect on current is already inside [`PmsmParams::rs`], which in this
/// project is the whole drive-path resistance the profiler measures, not the
/// winding alone — so these must not be added to the circuit equation again.
/// What they change is where `v_hi` and `v_lo` actually sit, and hence what the
/// correct comparison reference is.
#[derive(Copy, Clone, Debug)]
pub struct Bridge {
    /// High-side conducting resistance [Ω]; pulls `v_hi` below the rail.
    pub r_hs: f32,
    /// Low-side conducting resistance plus the current-sense shunt [Ω]; lifts
    /// `v_lo` above ground. On the bench rig this is the larger of the two,
    /// because the shunt is in it.
    pub r_ls: f32,
    /// Sense-network full scale at the terminal [V]; readings clip above it.
    /// A fixed divider ratio against a high bus is what makes this bite.
    pub sense_max: f32,
}

impl Default for Bridge {
    fn default() -> Self {
        // Ideal bridge: no drops, no clipping.
        Self {
            r_hs: 0.0,
            r_ls: 0.0,
            sense_max: f32::INFINITY,
        }
    }
}

impl Bridge {
    /// The bench rig: ~0.5 Ω of conducting switch each side, plus the 0.33 Ω
    /// shunt in the low leg, and a divider that saturates at 18.3 V.
    pub fn bench() -> Self {
        Self {
            r_hs: 0.5,
            r_ls: 0.83,
            sense_max: 18.3,
        }
    }
}

/// Where in the PWM period the idle terminal is sampled.
///
/// This is not a modelling detail — it is the design decision MS8 step 1 was
/// about. The same machine, the same instant of rotor motion, gives a usable
/// zero-cross at one sample point and nothing at the other.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SamplePoint {
    /// High-side conducting: `v_hi = V_bus`, `v_lo = 0`, so the idle terminal
    /// swings about `V_bus/2`.
    OnTime,
    /// Both driven legs low — where a low-side shunt must sample phase
    /// current. The idle terminal swings about 0, so half of it is below
    /// ground.
    Freewheel,
}

/// A star-connected machine driven two phases at a time.
#[derive(Copy, Clone, Debug)]
pub struct PhaseMotor {
    pub params: PmsmParams,
    pub shape: BemfShape,
    /// Current through the conducting pair [A]: into `hi`, out of `lo`.
    pub i_line: f32,
    pub theta_m: f32,
    pub omega_m: f32,
    /// Sector currently energised.
    pub sector: usize,
    /// Conducting-switch drops and sense-network limits.
    pub bridge: Bridge,
    /// Remaining freewheel time on the phase that most recently opened [s].
    flyback: f32,
    /// Sign of the trapped current that is freewheeling.
    flyback_sign: f32,
    pub locked: bool,
}

impl PhaseMotor {
    pub fn new(params: PmsmParams, shape: BemfShape) -> Self {
        Self {
            params,
            shape,
            i_line: 0.0,
            theta_m: 0.0,
            omega_m: 0.0,
            sector: 0,
            bridge: Bridge::default(),
            flyback: 0.0,
            flyback_sign: 0.0,
            locked: false,
        }
    }

    pub fn theta_e(&self) -> f32 {
        wrap_angle(self.theta_m * self.params.pole_pairs as f32)
    }

    pub fn omega_e(&self) -> f32 {
        self.omega_m * self.params.pole_pairs as f32
    }

    /// Back-EMF of phase `k` [V]: `e = ω_e · ψ · shape(θ_e)`.
    pub fn bemf(&self, k: usize) -> f32 {
        self.omega_e() * self.params.flux * self.shape.shape(k, self.theta_e())
    }

    /// Electromagnetic torque [N·m].
    ///
    /// `T = p·ψ·Σ sₖ·iₖ` — the general expression, valid for any back-EMF
    /// shape. Only the two conducting phases contribute, so it collapses to
    /// `p·ψ·i·(s_hi − s_lo)`.
    pub fn torque(&self) -> f32 {
        let (hi, lo, _) = sixstep::TABLE[self.sector % sixstep::SECTORS];
        let th = self.theta_e();
        let p = &self.params;
        p.pole_pairs as f32
            * p.flux
            * self.i_line
            * (self.shape.shape(hi as usize, th) - self.shape.shape(lo as usize, th))
    }

    /// Instantaneous phase currents [A]; the idle phase carries exactly zero,
    /// which is what makes it a sense node.
    pub fn phase_currents(&self) -> [f32; 3] {
        let (hi, lo, _) = sixstep::TABLE[self.sector % sixstep::SECTORS];
        let mut i = [0.0; 3];
        i[hi as usize] = self.i_line;
        i[lo as usize] = -self.i_line;
        i
    }

    /// Voltage on the idle terminal [V], as the sense network would see it.
    ///
    /// Exact for any back-EMF shape:
    ///
    /// ```text
    /// v_f = (v_hi + v_lo)/2 − (e_hi + e_lo)/2 + e_f
    /// ```
    ///
    /// The `(v_hi + v_lo)/2` term is set by `at` — half the bus during the
    /// on-time, zero in the freewheel. The rest is the machine. Two shapes
    /// collapse it differently, and the difference is instructive:
    ///
    /// - **Sinusoidal**: the three back-EMFs sum to zero, so
    ///   `−(e_hi + e_lo)/2 = e_f/2` and `v_f = mid + 1.5·e_f`.
    /// - **Trapezoidal**: the conducting pair sits on opposite flat tops for
    ///   the whole window, so `e_hi + e_lo = 0` exactly and `v_f = mid + e_f`.
    ///   Lower gain, but the flat top is precisely what keeps the crossing
    ///   free of the machine's third harmonic.
    ///
    /// Either way the crossing of `mid` happens when `e_f` does, which is what
    /// the commutation timer needs.
    ///
    /// While the freshly-opened phase is still freewheeling its trapped
    /// current, the terminal is clamped to a rail and carries no rotor
    /// information — that is what blanking exists to skip.
    pub fn idle_terminal(&self, at: SamplePoint, v_bus: f32) -> f32 {
        let mid = match at {
            SamplePoint::OnTime => v_bus * 0.5,
            SamplePoint::Freewheel => 0.0,
        };
        if self.flyback > 0.0 {
            return if self.flyback_sign > 0.0 { v_bus } else { 0.0 };
        }
        let (hi, lo, f) = sixstep::TABLE[self.sector % sixstep::SECTORS];
        // `mid` here is the *true* mid-point of the driven terminals, drops
        // included — the identity is about physical node voltages, not about
        // what the rail nominally is.
        let mid = mid + 0.5 * (self.i_line * self.bridge.r_ls - self.i_line * self.bridge.r_hs);
        self.clip(
            mid - 0.5 * (self.bemf(hi as usize) + self.bemf(lo as usize)) + self.bemf(f as usize),
        )
    }

    /// Voltages on the two *driven* terminals during `at` [V].
    ///
    /// This is what a real bridge presents, and it is not `(V_bus, 0)`: the
    /// conducting switches and the shunt move both ends. Their average is the
    /// reference the idle phase actually swings about.
    pub fn driven_terminals(&self, at: SamplePoint, v_bus: f32) -> (f32, f32) {
        let i = self.i_line;
        let (hi, lo) = match at {
            SamplePoint::OnTime => (v_bus - i * self.bridge.r_hs, i * self.bridge.r_ls),
            // Freewheeling: the high leg's low-side switch is on too, so both
            // ends sit near ground, lifted by their own conduction drops.
            SamplePoint::Freewheel => (-i * self.bridge.r_ls, i * self.bridge.r_ls),
        };
        (self.clip(hi), self.clip(lo))
    }

    /// The reference the idle terminal swings about: the mean of the driven
    /// terminals, as a controller could actually measure it.
    pub fn measured_mid(&self, at: SamplePoint, v_bus: f32) -> f32 {
        let (hi, lo) = self.driven_terminals(at, v_bus);
        0.5 * (hi + lo)
    }

    fn clip(&self, v: f32) -> f32 {
        v.min(self.bridge.sense_max)
    }

    /// Commutate to `sector`, starting a freewheel on the phase that opens.
    ///
    /// The trapped current decays at `di/dt = V_bus/L`, so the freewheel lasts
    /// about `L·|i|/V_bus`. It is short — tens of microseconds here — but it is
    /// exactly the interval in which a naive detector sees a rail-to-rail edge
    /// and calls it a zero-cross.
    pub fn commutate(&mut self, sector: usize, v_bus: f32) {
        if sector % sixstep::SECTORS != self.sector {
            let l = self.params.ld.max(1e-9);
            self.flyback = l * self.i_line.abs() / v_bus.max(1.0);
            self.flyback_sign = self.i_line.signum();
            self.sector = sector % sixstep::SECTORS;
        }
    }

    /// One integration substep.
    ///
    /// The conducting pair is a series circuit: `v_line = 2R·i + 2L·di/dt +
    /// (e_hi − e_lo)`. Two phases in series is why the resistance and
    /// inductance that matter here are twice the per-phase values.
    pub fn step(&mut self, v_line: f32, load_torque: f32, dt: f32) {
        let p = self.params;
        let th = self.theta_e();
        let (hi, lo, _) = sixstep::TABLE[self.sector % sixstep::SECTORS];
        let e_line = self.omega_e()
            * p.flux
            * (self.shape.shape(hi as usize, th) - self.shape.shape(lo as usize, th));

        let di = (v_line - 2.0 * p.rs * self.i_line - e_line) / (2.0 * p.ld);
        self.i_line += di * dt;
        self.flyback = (self.flyback - dt).max(0.0);

        if self.locked {
            self.omega_m = 0.0;
        } else {
            let acc = (self.torque() - load_torque - p.viscous * self.omega_m) / p.inertia;
            self.omega_m += acc * dt;
            self.theta_m = wrap_angle(self.theta_m + self.omega_m * dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mmc_core::sixstep::SECTORS;

    fn bench() -> PmsmParams {
        PmsmParams::bench_g474()
    }

    /// A sinusoidal machine's three back-EMFs sum to zero at every angle.
    #[test]
    fn sinusoidal_bemf_sums_to_zero() {
        for i in 0..360 {
            let th = i as f32 * core::f32::consts::TAU / 360.0;
            let s: f32 = (0..3).map(|k| BemfShape::Sinusoidal.shape(k, th)).sum();
            assert!(s.abs() < 1e-3, "at {th}: sum {s}");
        }
    }

    /// An ideal 120°-flat trapezoid does *not* sum to zero — it carries a
    /// third harmonic, and the sum is that harmonic: a triangle wave at three
    /// times the electrical frequency. Anything that assumes the sum vanishes
    /// (a resistor virtual-neutral, or the `1.5·e_f` shortcut) inherits this
    /// as an error term.
    #[test]
    fn trapezoidal_bemf_sum_is_pure_third_harmonic() {
        let sum = |th: f32| -> f32 { (0..3).map(|k| BemfShape::Trapezoidal.shape(k, th)).sum() };
        let third = core::f32::consts::TAU / 3.0;
        for i in 0..360 {
            let th = i as f32 * core::f32::consts::TAU / 360.0;
            assert!(
                (sum(th) - sum(th + third)).abs() < 1e-3,
                "sum is not periodic at 120° near {th}"
            );
        }
        // And it is genuinely present, not a rounding artefact.
        let peak = (0..360)
            .map(|i| sum(i as f32 * core::f32::consts::TAU / 360.0).abs())
            .fold(0.0f32, f32::max);
        assert!(peak > 0.5, "third harmonic peak {peak}");
    }

    /// The flat top earns its keep: through every float window the conducting
    /// pair sits on opposite tops, so their back-EMFs cancel exactly and the
    /// idle terminal is `mid + e_f` with no third-harmonic offset at all.
    #[test]
    fn trapezoidal_conducting_pair_cancels_across_each_window() {
        let m = PhaseMotor::new(bench(), BemfShape::Trapezoidal);
        for k in 0..SECTORS {
            let centre = sixstep::SECTOR0_CENTRE + k as f32 * sixstep::SECTOR_RAD;
            for step in -9..=9 {
                let th = centre + step as f32 * sixstep::SECTOR_RAD / 20.0;
                let (hi, lo, _) = sixstep::TABLE[k];
                let s = m.shape.shape(hi as usize, th) + m.shape.shape(lo as usize, th);
                assert!(s.abs() < 1e-3, "sector {k} at {th}: pair sums {s}");
            }
        }
    }

    /// Both shapes must be normalised to unit peak, or `flux` means something
    /// different for each and the torque constants stop being comparable.
    #[test]
    fn shapes_have_unit_peak() {
        for shape in [BemfShape::Sinusoidal, BemfShape::Trapezoidal] {
            let peak = (0..720)
                .map(|i| {
                    shape
                        .shape(0, i as f32 * core::f32::consts::TAU / 720.0)
                        .abs()
                })
                .fold(0.0f32, f32::max);
            assert!((peak - 1.0).abs() < 1e-3, "{shape:?} peak {peak}");
        }
    }

    /// Locked rotor, DC on the line: current settles at v/(2R), because two
    /// phases are in series.
    #[test]
    fn locked_rotor_settles_at_v_over_two_r() {
        let mut m = PhaseMotor::new(bench(), BemfShape::Trapezoidal);
        m.locked = true;
        for _ in 0..200_000 {
            m.step(1.0, 0.0, 1e-7);
        }
        let expect = 1.0 / (2.0 * m.params.rs);
        assert!(
            (m.i_line - expect).abs() < expect * 0.01,
            "i {} vs {expect}",
            m.i_line
        );
    }

    /// The idle terminal must be an affine image of its own back-EMF, with the
    /// offset set by the sample point. This is the model's central claim.
    #[test]
    fn idle_terminal_follows_the_sensing_identity() {
        let mut m = PhaseMotor::new(bench(), BemfShape::Sinusoidal);
        m.omega_m = 200.0;
        for k in 0..SECTORS {
            m.sector = k;
            m.theta_m = 0.3 * k as f32;
            // Sinusoidal: the sum vanishes, so the gain is exactly 1.5.
            let e = m.bemf(sixstep::floating_phase(k));
            let on = m.idle_terminal(SamplePoint::OnTime, 24.0);
            let fw = m.idle_terminal(SamplePoint::Freewheel, 24.0);
            assert!((on - (12.0 + 1.5 * e)).abs() < 1e-3, "on-time sector {k}");
            assert!((fw - 1.5 * e).abs() < 1e-3, "freewheel sector {k}");
        }
    }

    /// Sampled in the freewheel, half of every electrical revolution sits
    /// below ground — the bench result, reproduced from first principles.
    #[test]
    fn freewheel_sampling_puts_half_the_waveform_below_ground() {
        let mut m = PhaseMotor::new(bench(), BemfShape::Sinusoidal);
        m.omega_m = 300.0;
        let mut negative = 0;
        let n = 600;
        for i in 0..n {
            m.theta_m = i as f32 * core::f32::consts::TAU / n as f32;
            m.sector = sixstep::sector_of(m.theta_e());
            if m.idle_terminal(SamplePoint::Freewheel, 24.0) < 0.0 {
                negative += 1;
            }
        }
        let frac = negative as f32 / n as f32;
        assert!(
            (0.3..0.7).contains(&frac),
            "expected about half below ground, got {frac}"
        );
    }

    /// Sampled during the on-time, the idle terminal never leaves the rails,
    /// so no clamp can eat any of it.
    #[test]
    fn on_time_sampling_stays_inside_the_rails() {
        let mut m = PhaseMotor::new(bench(), BemfShape::Sinusoidal);
        m.omega_m = 300.0;
        for i in 0..600 {
            m.theta_m = i as f32 * core::f32::consts::TAU / 600.0;
            m.sector = sixstep::sector_of(m.theta_e());
            let v = m.idle_terminal(SamplePoint::OnTime, 24.0);
            assert!((0.0..=24.0).contains(&v), "v_f = {v} at step {i}");
        }
    }

    /// Torque must agree with the dq convention for the sinusoidal shape, so
    /// `flux` keeps one meaning across both models.
    #[test]
    fn torque_matches_dq_convention() {
        // At a sector centre the conducting pair's shape difference is
        // sqrt(3) for the sinusoidal machine, giving T = sqrt(3)·p·ψ·i.
        let mut m = PhaseMotor::new(bench(), BemfShape::Sinusoidal);
        m.i_line = 1.0;
        m.sector = 0;
        m.theta_m = sixstep::SECTOR0_CENTRE / m.params.pole_pairs as f32;
        let t = m.torque();
        let expect = 3.0f32.sqrt() * m.params.pole_pairs as f32 * m.params.flux;
        assert!((t - expect).abs() < expect * 0.02, "T {t} vs {expect}");
    }

    /// Driving the sector that leads the rotor must accelerate it forward.
    #[test]
    fn energising_the_leading_sector_spins_forward() {
        let mut m = PhaseMotor::new(bench(), BemfShape::Trapezoidal);
        for _ in 0..400_000 {
            let s = sixstep::sector_of(m.theta_e());
            m.commutate(s, 24.0);
            m.step(2.0, 0.0, 1e-7);
        }
        assert!(m.omega_m > 20.0, "omega_m = {}", m.omega_m);
    }
}
