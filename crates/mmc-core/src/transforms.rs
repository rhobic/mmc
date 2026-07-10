//! Clarke/Park reference-frame transforms, amplitude-invariant convention
//! (a 1 A peak phase current maps to |i_αβ| = 1 A).

use crate::math::{FRAC_1_SQRT_3, SQRT_3};

/// Three-phase quantities (currents or voltages).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Abc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

/// Stationary two-phase (stator) frame.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AlphaBeta {
    pub alpha: f32,
    pub beta: f32,
}

/// Rotating (rotor) frame; d aligned with the PM flux axis.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Dq {
    pub d: f32,
    pub q: f32,
}

/// abc → αβ. Discards any zero-sequence component.
#[inline]
pub fn clarke(x: Abc) -> AlphaBeta {
    AlphaBeta {
        alpha: (2.0 * x.a - x.b - x.c) * (1.0 / 3.0),
        beta: (x.b - x.c) * FRAC_1_SQRT_3,
    }
}

/// αβ → abc (balanced, zero-sequence free).
#[inline]
pub fn inverse_clarke(x: AlphaBeta) -> Abc {
    let ha = -0.5 * x.alpha;
    let hb = 0.5 * SQRT_3 * x.beta;
    Abc {
        a: x.alpha,
        b: ha + hb,
        c: ha - hb,
    }
}

/// αβ → dq at rotor angle θ, passed as `(sin θ, cos θ)`.
#[inline]
pub fn park(x: AlphaBeta, (sin, cos): (f32, f32)) -> Dq {
    Dq {
        d: x.alpha * cos + x.beta * sin,
        q: -x.alpha * sin + x.beta * cos,
    }
}

/// dq → αβ at rotor angle θ, passed as `(sin θ, cos θ)`.
#[inline]
pub fn inverse_park(x: Dq, (sin, cos): (f32, f32)) -> AlphaBeta {
    AlphaBeta {
        alpha: x.d * cos - x.q * sin,
        beta: x.d * sin + x.q * cos,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::sin_cos;

    fn assert_close(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "{a} vs {b}");
    }

    #[test]
    fn clarke_round_trip_balanced() {
        for i in 0..20 {
            let theta = i as f32 * 0.33;
            let (s, c) = sin_cos(theta);
            // Balanced three-phase set with amplitude 2.5.
            let x = inverse_clarke(AlphaBeta {
                alpha: 2.5 * c,
                beta: 2.5 * s,
            });
            assert_close(x.a + x.b + x.c, 0.0);
            let back = clarke(x);
            assert_close(back.alpha, 2.5 * c);
            assert_close(back.beta, 2.5 * s);
        }
    }

    #[test]
    fn park_round_trip() {
        let sc = sin_cos(1.234);
        let dq = Dq { d: -0.7, q: 1.9 };
        let back = park(inverse_park(dq, sc), sc);
        assert_close(back.d, dq.d);
        assert_close(back.q, dq.q);
    }

    #[test]
    fn amplitude_invariant() {
        // Phase A current 1.0 at θ = 0 maps to α = 1.0.
        let ab = clarke(Abc {
            a: 1.0,
            b: -0.5,
            c: -0.5,
        });
        assert_close(ab.alpha, 1.0);
        assert_close(ab.beta, 0.0);
    }
}
