//! PI controller with symmetric output saturation and integrator anti-windup
//! (integrator clamping).

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PiGains {
    pub kp: f32,
    pub ki: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct Pi {
    pub gains: PiGains,
    limit: f32,
    integral: f32,
}

impl Pi {
    pub fn new(gains: PiGains, limit: f32) -> Self {
        Self {
            gains,
            limit: limit.max(0.0),
            integral: 0.0,
        }
    }

    /// Symmetric output limit `[-limit, +limit]`. May change every cycle
    /// (e.g. a voltage limit that follows the bus voltage).
    pub fn set_limit(&mut self, limit: f32) {
        self.limit = limit.max(0.0);
        self.integral = self.integral.clamp(-self.limit, self.limit);
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    /// Preload the integrator (clamped to the output limit) — bumpless
    /// transfer, e.g. handing the I-f startup current to the speed loop
    /// without a torque step.
    pub fn preload(&mut self, value: f32) {
        self.integral = value.clamp(-self.limit, self.limit);
    }

    pub fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.integral = (self.integral + self.gains.ki * error * dt).clamp(-self.limit, self.limit);
        (self.gains.kp * error + self.integral).clamp(-self.limit, self.limit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn output_saturates() {
        let mut pi = Pi::new(PiGains { kp: 100.0, ki: 0.0 }, 1.0);
        assert_eq!(pi.update(10.0, 1e-3), 1.0);
        assert_eq!(pi.update(-10.0, 1e-3), -1.0);
    }

    #[test]
    fn integrator_does_not_wind_up() {
        let mut pi = Pi::new(PiGains { kp: 0.0, ki: 10.0 }, 1.0);
        // Drive hard into saturation for a long time.
        for _ in 0..10_000 {
            pi.update(100.0, 1e-3);
        }
        // Must recover within ~2·limit/(ki·error·dt) steps, not thousands.
        let mut steps = 0;
        while pi.update(-1.0, 1e-3) > 0.0 {
            steps += 1;
            assert!(steps < 250, "integrator wound up beyond its clamp");
        }
    }

    #[test]
    fn tracks_integral_of_error() {
        let mut pi = Pi::new(PiGains { kp: 0.0, ki: 2.0 }, 10.0);
        for _ in 0..1000 {
            pi.update(0.5, 1e-3);
        }
        // ∫ ki·e dt = 2.0 · 0.5 · 1.0 = 1.0
        assert!((pi.update(0.0, 0.0) - 1.0).abs() < 1e-3);
    }
}
