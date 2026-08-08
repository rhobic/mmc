//! Inverter voltage error: the gap between the voltage a controller commands
//! and the voltage the winding actually sees.
//!
//! While both switches in a leg are off — the dead time the gate driver
//! inserts, plus any turn-on/turn-off delay mismatch — the leg output is
//! clamped by whichever freewheeling diode the phase current forward-biases.
//! The lost volt-seconds therefore depend on the **sign** of the phase
//! current, not on its magnitude:
//!
//! ```text
//! v_applied = v_commanded − v_dead · sign(i_phase)
//! v_dead ≈ (t_dead + t_on − t_off)/T_pwm · V_bus + V_diode
//! ```
//!
//! One model, two users, so they cannot drift apart: the simulator's inverter
//! *subtracts* it (that is the defect), and [`crate::foc::Foc`] *adds* it back
//! before modulating (that is the fix). The same split the commutation table
//! got in `sixstep`.
//!
//! **Why it matters here and not in a textbook.** This project's `rs` is the
//! whole drive-path resistance the profiler measures — 0.885 Ω on the bench
//! motor against a ~0.1 Ω winding — so every *resistive* drop in the bridge is
//! already inside the model the flux observer uses. This term is the one that
//! is left over, and it has two properties that make it the natural suspect
//! for the sensorless low-speed floor:
//!
//! - **The R/L probe cannot see it.** That probe differences across folded
//!   square-wave edges, where the current sign is unchanged, so a `sign(i)`
//!   term cancels exactly. It has never appeared in a fit.
//! - **It does not shrink with speed, and the back-EMF does.** On the bench
//!   motor `ψ·ω` is 141 mV at the 150 rad/s el handoff and 37 mV at 40 rad/s,
//!   while a 250 ns dead time on a 40 kHz / 12 V bridge is ~120 mV flat.
//!
//! The observer integrates `v − R·i` using the *commanded* `v`, so this error
//! lands directly on its flux estimate — worst exactly where the signal is
//! smallest.

use crate::transforms::{clarke, Abc, AlphaBeta};

/// Dead-time and diode-drop voltage error, as a function of phase current.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct DeadtimeModel {
    /// Sign-dependent voltage error per phase [V]. A measured quantity —
    /// nothing in the profiler measures it yet (see the module docs).
    pub v_dead: f32,
    /// Half-width of the zero-current clamping band [A]. Ripple carries the
    /// real current back and forth across zero within a PWM period, so the
    /// transition is a ramp rather than a step; below this the error is
    /// linear in current. Also keeps the model differentiable where a hard
    /// `sign` would chatter, on hardware as much as in the sim.
    pub i_thresh: f32,
}

impl DeadtimeModel {
    /// The bench rig's order of magnitude. **Neither number is measured** —
    /// this is the estimate that motivates measuring them.
    ///
    /// `v_dead`: 250 ns of dead time on a 40 kHz bridge at 12 V → 0.12 V.
    ///
    /// `i_thresh`: the ripple *amplitude*, because that is precisely the band
    /// within which the true current changes sign inside a PWM period, so
    /// both diodes take a share and the error scales with how far off centre
    /// the ripple triangle sits. `V_bus/(8·L·f_pwm)` on 30 µH at 40 kHz is
    /// ~1.2 A pk-pk, ~0.5 A amplitude once the star connection's effective
    /// inductance is taken into account.
    ///
    /// That ratio is the uncomfortable part: `v_dead/i_thresh` = 0.24 Ω of
    /// *apparent* resistance sitting on top of a fitted 0.885 Ω, everywhere
    /// the drive actually operates (0.08–0.5 A). On this rig the dead-time
    /// error does not look like zero-crossing distortion at all — it looks
    /// like the observer's R being ~25% too small.
    pub const fn bench_g474_estimate() -> Self {
        Self {
            v_dead: 0.12,
            i_thresh: 0.5,
        }
    }

    /// True when this model does nothing — an ideal inverter.
    #[inline]
    pub fn is_ideal(&self) -> bool {
        self.v_dead == 0.0
    }

    /// Error voltage for one leg carrying current `i` [V].
    #[inline]
    pub fn phase_error(&self, i: f32) -> f32 {
        if self.i_thresh > 0.0 {
            self.v_dead * (i / self.i_thresh).clamp(-1.0, 1.0)
        } else if i >= 0.0 {
            self.v_dead
        } else {
            -self.v_dead
        }
    }

    /// The three legs' error as a stationary-frame vector.
    ///
    /// Clarke drops the common mode, which is the physically right thing: a
    /// star-connected winding only sees the differential part. An error that
    /// happened to be equal on all three legs would vanish here — this one
    /// does not, because the three currents carry independent signs.
    #[inline]
    pub fn error_ab(&self, i_abc: Abc) -> AlphaBeta {
        clarke(Abc {
            a: self.phase_error(i_abc.a),
            b: self.phase_error(i_abc.b),
            c: self.phase_error(i_abc.c),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::sin_cos;
    use crate::transforms::{inverse_clarke, inverse_park, Dq};

    #[test]
    fn saturates_outside_the_band_and_is_linear_inside() {
        let m = DeadtimeModel {
            v_dead: 0.12,
            i_thresh: 0.1,
        };
        assert!((m.phase_error(1.0) - 0.12).abs() < 1e-6);
        assert!((m.phase_error(-1.0) + 0.12).abs() < 1e-6);
        assert!((m.phase_error(0.05) - 0.06).abs() < 1e-6);
        assert_eq!(m.phase_error(0.0), 0.0);
        // Odd symmetry: a reversed current reverses the error exactly.
        for k in -20..=20 {
            let i = k as f32 * 0.02;
            assert!((m.phase_error(i) + m.phase_error(-i)).abs() < 1e-7);
        }
    }

    #[test]
    fn hard_sign_when_the_band_is_zero() {
        let m = DeadtimeModel {
            v_dead: 0.5,
            i_thresh: 0.0,
        };
        assert_eq!(m.phase_error(1e-9), 0.5);
        assert_eq!(m.phase_error(-1e-9), -0.5);
    }

    /// The whole reason this term is dangerous: it opposes the current vector
    /// no matter where the rotor is, so in the rotor frame it is a **bias**,
    /// not a zero-mean ripple. Averaged over an electrical revolution with the
    /// current on the q axis, it leaves a standing −q voltage error.
    #[test]
    fn averages_to_a_bias_against_the_current_vector() {
        let m = DeadtimeModel::bench_g474_estimate();
        let iq = 0.5;
        let n = 720;
        let mut sum_q = 0.0f32;
        let mut sum_d = 0.0f32;
        for k in 0..n {
            let theta = k as f32 * core::f32::consts::TAU / n as f32;
            let sc = sin_cos(theta);
            let i_abc = inverse_clarke(inverse_park(Dq { d: 0.0, q: iq }, sc));
            let e = crate::transforms::park(m.error_ab(i_abc), sc);
            sum_d += e.d;
            sum_q += e.q;
        }
        let (mean_d, mean_q) = (sum_d / n as f32, sum_q / n as f32);
        // Aligned with i_q, and a large fraction of v_dead — for reference,
        // the bench motor's back-EMF at its 150 rad/s el handoff is 0.141 V.
        assert!(mean_q > 0.5 * m.v_dead, "mean q error {mean_q}");
        assert!(mean_d.abs() < 0.05 * m.v_dead, "mean d error {mean_d}");
    }
}
