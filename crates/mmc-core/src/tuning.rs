//! Model-based gain calculation. The motor profiler (MS6) measures the
//! parameters these formulas need; until then they come from a datasheet or
//! the simulator's ground truth.

use crate::pi::PiGains;

/// Current-loop PI gains by pole-zero cancellation of the R–L plant:
/// `kp = L·ω`, `ki = R·ω` yields a first-order closed loop with bandwidth
/// `ω` rad/s (time constant 1/ω). Keep ω well below the control rate,
/// e.g. ω ≤ 2π·f_ctrl/10.
pub fn current_pi_gains(r: f32, l: f32, bandwidth_rad: f32) -> PiGains {
    PiGains {
        kp: l * bandwidth_rad,
        ki: r * bandwidth_rad,
    }
}
