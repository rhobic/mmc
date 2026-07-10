//! Space-vector PWM by min/max common-mode injection (equivalent to
//! third-harmonic injection). Linear range extends to |v| = v_bus/√3.

use crate::transforms::{inverse_clarke, AlphaBeta};

/// Convert a stationary-frame voltage demand into three duty cycles in `[0, 1]`.
///
/// Demands beyond the hexagon simply clamp per-phase (no overmodulation
/// strategy yet); keep |v| ≤ v_bus/√3 for undistorted output.
#[inline]
pub fn svpwm(v: AlphaBeta, v_bus: f32) -> [f32; 3] {
    if v_bus <= 0.0 {
        return [0.5; 3];
    }
    let p = inverse_clarke(v);
    let vmax = p.a.max(p.b).max(p.c);
    let vmin = p.a.min(p.b).min(p.c);
    let common = -0.5 * (vmax + vmin);
    let inv = 1.0 / v_bus;
    [
        ((p.a + common) * inv + 0.5).clamp(0.0, 1.0),
        ((p.b + common) * inv + 0.5).clamp(0.0, 1.0),
        ((p.c + common) * inv + 0.5).clamp(0.0, 1.0),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{sin_cos, FRAC_1_SQRT_3};
    use crate::transforms::{clarke, Abc};

    /// Reconstruct the line-to-neutral voltage a balanced load would see:
    /// v_x = (d_x − mean(d)) · v_bus, then Clarke back to αβ.
    fn reconstruct(duties: [f32; 3], v_bus: f32) -> AlphaBeta {
        let mean = (duties[0] + duties[1] + duties[2]) / 3.0;
        clarke(Abc {
            a: (duties[0] - mean) * v_bus,
            b: (duties[1] - mean) * v_bus,
            c: (duties[2] - mean) * v_bus,
        })
    }

    #[test]
    fn reproduces_demand_in_linear_region() {
        let v_bus = 24.0;
        let mag = v_bus * FRAC_1_SQRT_3 * 0.99;
        for i in 0..36 {
            let (s, c) = sin_cos(i as f32 * 0.1745);
            let v = AlphaBeta {
                alpha: mag * c,
                beta: mag * s,
            };
            let duties = svpwm(v, v_bus);
            for d in duties {
                assert!((0.0..=1.0).contains(&d));
            }
            let r = reconstruct(duties, v_bus);
            assert!(
                (r.alpha - v.alpha).abs() < 0.02 && (r.beta - v.beta).abs() < 0.02,
                "angle step {i}: {r:?} vs {v:?}"
            );
        }
    }

    #[test]
    fn zero_demand_centers_duties() {
        let duties = svpwm(AlphaBeta::default(), 24.0);
        for d in duties {
            assert!((d - 0.5).abs() < 1e-6);
        }
    }
}
