//! One complete FOC torque-control step: phase currents and rotor angle in,
//! PWM duties out. This is the function the ADC interrupt calls on hardware
//! and the simulator calls per control period.

use crate::current_loop::CurrentLoop;
use crate::math::{sin_cos, sqrt, FRAC_1_SQRT_3};
use crate::pi::PiGains;
use crate::svpwm::svpwm;
use crate::transforms::{clarke, inverse_park, park, Abc, AlphaBeta, Dq};

/// Cross-coupling and back-EMF decoupling feedforward.
///
/// A PI alone leaves a steady-state current error of `(dE/dt)/(R·ω_bw)`
/// against the ramping back-EMF of an accelerating motor; feeding the known
/// motor voltages forward removes that burden from the PI entirely.
#[derive(Copy, Clone, Debug, Default)]
pub struct Decoupling {
    pub ld: f32,
    pub lq: f32,
    pub flux: f32,
}

impl Decoupling {
    /// Steady-state dq voltage at the given operating point (resistive drop
    /// excluded — the PI integrator owns that).
    #[inline]
    pub fn voltage(&self, i_dq: Dq, omega_e: f32) -> Dq {
        Dq {
            d: -omega_e * self.lq * i_dq.q,
            q: omega_e * (self.ld * i_dq.d + self.flux),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Foc {
    pub current_loop: CurrentLoop,
    /// `None` runs plain PI — the state before motor parameters are known
    /// (the profiler, MS6, exists to fill this in).
    pub feedforward: Option<Decoupling>,
    /// Voltage-vector angle advance, in control periods. The inverter holds
    /// the commanded vector fixed in the stationary frame while the rotor
    /// keeps moving, so without compensation the effective dq voltage lags
    /// and cross-couples — at ω_e approaching the loop bandwidth this limit-
    /// cycles. 0.5 compensates the zero-order hold itself; hardware backends
    /// where duties latch a full PWM period after sampling should use 1.5.
    pub advance_periods: f32,
}

/// Everything a step produced, for telemetry as much as for actuation.
#[derive(Copy, Clone, Debug, Default)]
pub struct FocOutput {
    pub duties: [f32; 3],
    /// Measured currents in the rotor frame.
    pub i_dq: Dq,
    /// Voltage demand in the rotor frame (PI + feedforward).
    pub v_dq: Dq,
    /// Voltage demand in the stator frame (what an observer needs).
    pub v_ab: AlphaBeta,
    /// Measured currents in the stator frame (what an observer needs).
    pub i_ab: AlphaBeta,
}

impl Foc {
    pub fn new(current_gains: PiGains) -> Self {
        Self {
            current_loop: CurrentLoop::new(current_gains),
            feedforward: None,
            advance_periods: 0.5,
        }
    }

    pub fn with_feedforward(current_gains: PiGains, decoupling: Decoupling) -> Self {
        Self {
            current_loop: CurrentLoop::new(current_gains),
            feedforward: Some(decoupling),
            advance_periods: 0.5,
        }
    }

    /// One control period. `theta_e`/`omega_e` come from an
    /// [`AngleEstimator`](crate::angle::AngleEstimator).
    pub fn step(
        &mut self,
        i_abc: Abc,
        theta_e: f32,
        omega_e: f32,
        i_ref: Dq,
        v_bus: f32,
        dt: f32,
    ) -> FocOutput {
        let sc = sin_cos(theta_e);
        let i_ab = clarke(i_abc);
        let i_dq = park(i_ab, sc);
        let v_limit = v_bus * FRAC_1_SQRT_3;

        // Feedforward first; the PI pair gets the remaining voltage authority
        // so the summed demand stays on the voltage circle and the
        // anti-windup clamps remain truthful.
        let mut v_ff = self
            .feedforward
            .map(|f| f.voltage(i_dq, omega_e))
            .unwrap_or_default();
        let ff_mag = sqrt(v_ff.d * v_ff.d + v_ff.q * v_ff.q);
        let pi_limit = if ff_mag > v_limit {
            let scale = v_limit / ff_mag;
            v_ff.d *= scale;
            v_ff.q *= scale;
            0.0
        } else {
            v_limit - ff_mag
        };

        let v_pi = self.current_loop.update(i_dq, i_ref, pi_limit, dt);
        let v_dq = Dq {
            d: v_ff.d + v_pi.d,
            q: v_ff.q + v_pi.q,
        };
        // Command the vector where the rotor will be, not where it was.
        let sc_out = sin_cos(theta_e + self.advance_periods * omega_e * dt);
        let v_ab = inverse_park(v_dq, sc_out);
        FocOutput {
            duties: svpwm(v_ab, v_bus),
            i_dq,
            v_dq,
            v_ab,
            i_ab,
        }
    }
}
