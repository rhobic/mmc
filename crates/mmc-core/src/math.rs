//! Scalar math for the control core.
//!
//! Everything is `f32` today, and all transcendental calls route through this
//! module, so a fixed-point or CORDIC implementation for Cortex-M0-class
//! targets can be introduced later without touching the control code.

pub use core::f32::consts::PI;

pub const TWO_PI: f32 = 2.0 * PI;
pub const SQRT_3: f32 = 1.732_050_8;
pub const FRAC_1_SQRT_3: f32 = 0.577_350_26;

/// `(sin, cos)` of an angle in radians.
#[inline]
pub fn sin_cos(angle: f32) -> (f32, f32) {
    (libm::sinf(angle), libm::cosf(angle))
}

#[inline]
pub fn sqrt(x: f32) -> f32 {
    libm::sqrtf(x)
}

/// Wrap an angle to `[-PI, PI)`.
#[inline]
pub fn wrap_angle(angle: f32) -> f32 {
    let mut a = angle % TWO_PI;
    if a >= PI {
        a -= TWO_PI;
    } else if a < -PI {
        a += TWO_PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_angle_stays_in_range() {
        for i in -100..100 {
            let a = i as f32 * 0.37;
            let w = wrap_angle(a);
            assert!((-PI..PI).contains(&w), "wrap({a}) = {w}");
            // Same direction on the unit circle.
            let (s0, c0) = sin_cos(a);
            let (s1, c1) = sin_cos(w);
            assert!((s0 - s1).abs() < 1e-4 && (c0 - c1).abs() < 1e-4);
        }
    }
}
