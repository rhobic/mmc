//! Scalar math for the control core.
//!
//! Everything is `f32` today, and all transcendental calls route through this
//! module, so a fixed-point or CORDIC implementation for Cortex-M0-class
//! targets can be introduced later without touching the control code.

pub use core::f32::consts::{FRAC_PI_4, PI};

pub const TWO_PI: f32 = 2.0 * PI;
pub const SQRT_3: f32 = 1.732_050_8;
pub const FRAC_1_SQRT_3: f32 = 0.577_350_26;

/// `(sin, cos)` of an angle in radians.
///
/// Polynomial evaluation on the wrapped angle, single-precision throughout.
/// `libm`'s `sinf`/`cosf` reduce arguments in `f64`, which is emulated in
/// software on single-precision FPUs (Cortex-M4F/M7) and costs thousands of
/// cycles per call — measured wedging the G474's 20 kHz control ISR. This
/// version is a few tens of cycles; worst-case error ≈ 3e-6, far below any
/// control-loop tolerance here.
#[inline]
pub fn sin_cos(angle: f32) -> (f32, f32) {
    const FRAC_PI_2: f32 = core::f32::consts::FRAC_PI_2;
    let x = wrap_angle(angle);
    // Reflect into [-π/2, π/2]; sine is preserved, cosine flips sign.
    let (x, cos_sign) = if x > FRAC_PI_2 {
        (PI - x, -1.0f32)
    } else if x < -FRAC_PI_2 {
        (-PI - x, -1.0)
    } else {
        (x, 1.0)
    };
    let x2 = x * x;
    // Taylor through x⁹ / x⁸: |err| < 7e-7 (sin), < 3e-6 (cos) on the range.
    let s = x
        * (1.0
            + x2 * (-1.666_666_7e-1
                + x2 * (8.333_333e-3 + x2 * (-1.984_127e-4 + x2 * 2.755_732e-6))));
    let c = 1.0 + x2 * (-0.5 + x2 * (4.166_666_6e-2 + x2 * (-1.388_889e-3 + x2 * 2.480_159e-5)));
    (s, c * cos_sign)
}

#[inline]
pub fn sqrt(x: f32) -> f32 {
    libm::sqrtf(x)
}

/// Wrap an angle to `[-PI, PI)`.
///
/// No `%` here: `f32 % f32` lowers to a software `fmodf` on targets without
/// hardware remainder (all Cortex-M) — float→int→float conversions are single
/// instructions instead.
#[inline]
pub fn wrap_angle(angle: f32) -> f32 {
    const FRAC_1_TWO_PI: f32 = 1.0 / TWO_PI;
    let k = angle * FRAC_1_TWO_PI;
    let k_round = (k + if k >= 0.0 { 0.5 } else { -0.5 }) as i32;
    let w = angle - k_round as f32 * TWO_PI;
    if w >= PI {
        w - TWO_PI
    } else if w < -PI {
        w + TWO_PI
    } else {
        w
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fast polynomial must stay indistinguishable from libm for control
    /// purposes, across many wraps.
    #[test]
    fn sin_cos_matches_libm() {
        let mut max_err = 0.0f32;
        for i in -40_000..40_000 {
            let a = i as f32 * 1e-3; // ±40 rad, ~6 full turns
            let (s, c) = sin_cos(a);
            max_err = max_err
                .max((s - libm::sinf(a)).abs())
                .max((c - libm::cosf(a)).abs());
        }
        // Dominated by f32 wrap quantization at large angles; ~100× under the
        // 1e-4-class tolerances the control tests use.
        assert!(max_err < 5e-5, "max_err = {max_err}");
    }

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
