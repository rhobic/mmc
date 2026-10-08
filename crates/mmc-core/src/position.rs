//! Position control building blocks: a trapezoidal reference generator and
//! an unwrapping position accumulator.
//!
//! Positions here are in whatever angular unit the caller uses consistently
//! (the drive uses electrical radians internally).

use crate::math::{sqrt as sqrtf, wrap_angle};

/// Trapezoidal motion reference: accelerates at `accel` toward the target,
/// cruises at `vmax`, and decelerates so as to arrive with zero velocity.
/// Retargetable at any time; the velocity stays continuous.
#[derive(Copy, Clone, Debug, Default)]
pub struct TrapRef {
    pub pos: f32,
    pub vel: f32,
}

impl TrapRef {
    pub fn new(pos: f32) -> Self {
        Self { pos, vel: 0.0 }
    }

    /// Advance one period toward `target`.
    pub fn update(&mut self, target: f32, vmax: f32, accel: f32, dt: f32) {
        let d = target - self.pos;
        let dv = accel * dt;
        // Close enough and slow enough to land in one step: snap, so the
        // reference does not dither around the target.
        if d.abs() <= 0.5 * dv * dt + 1e-6 && self.vel.abs() <= dv {
            self.pos = target;
            self.vel = 0.0;
            return;
        }
        // Fastest speed from which the remaining distance still stops in
        // time: v² = 2·a·d.
        let v_stop = sqrtf(2.0 * accel * d.abs()).min(vmax.abs());
        let want = if d >= 0.0 { v_stop } else { -v_stop };
        self.vel += (want - self.vel).clamp(-dv, dv);
        self.pos += self.vel * dt;
    }
}

/// Accumulates a wrapped angle into an unwrapped position.
#[derive(Copy, Clone, Debug, Default)]
pub struct Unwrap {
    last: Option<f32>,
    pub pos: f32,
}

impl Unwrap {
    pub const fn new() -> Self {
        Self {
            last: None,
            pos: 0.0,
        }
    }

    /// Feed the current wrapped angle; returns the unwrapped position
    /// (0 at the first sample).
    pub fn update(&mut self, theta: f32) -> f32 {
        if let Some(last) = self.last {
            self.pos += wrap_angle(theta - last);
        }
        self.last = Some(theta);
        self.pos
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trapezoid_arrives_at_rest_without_overshoot() {
        for &target in &[10.0f32, -3.0, 0.2] {
            let mut r = TrapRef::new(0.0);
            let (dt, vmax, accel) = (1e-4, 20.0, 100.0);
            let mut peak_v = 0.0f32;
            for _ in 0..40_000 {
                r.update(target, vmax, accel, dt);
                peak_v = peak_v.max(r.vel.abs());
                assert!(r.pos.abs() <= target.abs() + 1e-3, "overshoot to {}", r.pos);
            }
            assert!((r.pos - target).abs() < 1e-4, "ended at {}", r.pos);
            assert_eq!(r.vel, 0.0);
            assert!(peak_v <= vmax + 1e-3);
        }
    }

    #[test]
    fn unwrap_counts_turns() {
        let mut u = Unwrap::new();
        let mut theta = 0.0f32;
        for _ in 0..1000 {
            theta = wrap_angle(theta + 0.1);
            u.update(theta);
        }
        assert!((u.pos - 99.9).abs() < 1e-2, "{}", u.pos);
    }
}
