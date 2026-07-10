//! Synchronous-frame (dq) current regulator: one PI per axis with a shared
//! bus-voltage-dependent limit. The d axis gets priority when the combined
//! demand exceeds the voltage circle (flux/field-weakening authority first,
//! torque takes what remains).

use crate::math::sqrt;
use crate::pi::{Pi, PiGains};
use crate::transforms::Dq;

#[derive(Copy, Clone, Debug)]
pub struct CurrentLoop {
    pub pi_d: Pi,
    pub pi_q: Pi,
}

impl CurrentLoop {
    pub fn new(gains: PiGains) -> Self {
        // Real limits are set per-update from the bus voltage.
        Self {
            pi_d: Pi::new(gains, 0.0),
            pi_q: Pi::new(gains, 0.0),
        }
    }

    pub fn reset(&mut self) {
        self.pi_d.reset();
        self.pi_q.reset();
    }

    /// One control period: measured and reference dq currents in, dq voltage
    /// demand out, constrained to |v| ≤ `v_limit`.
    pub fn update(&mut self, i_meas: Dq, i_ref: Dq, v_limit: f32, dt: f32) -> Dq {
        self.pi_d.set_limit(v_limit);
        let d = self.pi_d.update(i_ref.d - i_meas.d, dt);
        let q_limit = sqrt((v_limit * v_limit - d * d).max(0.0));
        self.pi_q.set_limit(q_limit);
        let q = self.pi_q.update(i_ref.q - i_meas.q, dt);
        Dq { d, q }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn respects_voltage_circle() {
        let mut cl = CurrentLoop::new(PiGains { kp: 100.0, ki: 0.0 });
        let v = cl.update(Dq::default(), Dq { d: 10.0, q: 10.0 }, 5.0, 1e-4);
        assert!(sqrt(v.d * v.d + v.q * v.q) <= 5.0 + 1e-4);
        // d axis kept its full authority.
        assert!((v.d - 5.0).abs() < 1e-5);
    }
}
