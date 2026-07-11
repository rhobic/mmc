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

/// Speed-loop PI gains from the mechanical plant `J·ω̇ = kt·i_q`, for a loop
/// commanding i_q from *electrical* velocity error (hence `pole_pairs`).
/// `kp = J·ω_bw/(kt·p)`; the integral corner sits at `ω_bw/4` (≈ 15°
/// phase cost at crossover, settles in a few 1/ω_bw). Keep `ω_bw` at least
/// 5–10× below the current-loop bandwidth.
pub fn speed_pi_gains(inertia: f32, kt: f32, pole_pairs: u32, bandwidth_rad: f32) -> PiGains {
    let kp = inertia * bandwidth_rad / (kt * pole_pairs as f32);
    PiGains {
        kp,
        ki: kp * bandwidth_rad / 4.0,
    }
}
