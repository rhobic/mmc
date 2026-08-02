//! Saliency (Ld/Lq) probe schedule — shared verbatim by the G474 firmware ISR
//! and the host simulator so the two can never drift apart (the host fit,
//! `tools/saliency.py`, reads the actual schedule from the header the device
//! records, so a silent mismatch would turn the sim's positive control into
//! noise).
//!
//! ## The measurement
//!
//! With the machine at standstill, excite with a square-wave d-axis voltage
//! along a commanded electrical angle θx and record the current in the
//! *excitation* frame. At DC the current settles to V/R along the applied
//! axis at **every** angle (R is isotropic), so the cross-axis current i_q is
//! a null channel: its post-edge transient exists **only** if Ld ≠ Lq. The
//! fit works in 1/L (`1/L(δ) = Γ0 + Γ1·cos 2δ`, δ = θx − θr) and estimates
//! the ratio ξ = Γ1/Γ0 in a form where the unknown effective sample latency
//! cancels — the same systematic that limits the absolute L measurement to
//! ±30% does not touch ξ.
//!
//! ## Why the schedule looks like this
//!
//! - **±paired angles** (`+δ, −δ, …`): magnet torque is ∝ sin δ (odd), so
//!   consecutive ± blocks cancel the net torque impulse. With short blocks an
//!   unclamped rotor dithers by a few electrical degrees instead of chasing
//!   the vector — no mechanical clamping is required.
//! - **τ-adaptive half-period**: each square-wave half must settle (≥ ~8·τ)
//!   for the plateau normalization to mean anything, and τ = L/R spans an
//!   order of magnitude across motors (31 µs bench BLDC → 360 µs 4-pole).
//!   [`sal_half_ticks`] picks the half-period from the device's live R/L
//!   parameters; the block is always 4 half-periods and the cycle count
//!   shrinks to keep the total exactly [`SAL_TICKS`]. **Profile R/L and
//!   apply it before running the saliency stage** — a stale τ picks a wrong
//!   half-period, which the host fit detects as unsettled plateaus.
//! - **Interleaved cycles**: sweeping all angles once per cycle and repeating
//!   turns slow thermal R drift into common-mode across the angle axis
//!   instead of a fake cos 2δ signature.
//! - **One burst, one recording**: the whole sweep fits the existing burst
//!   buffer, so R cannot drift between angles waiting on protocol readback.

use core::f32::consts::PI;

/// Signed excitation angles per cycle (8 magnitudes × ±).
pub const SAL_SLOTS: usize = 16;
/// Total recorded ticks (excludes the align phase). Always exactly this many
/// regardless of the picked half-period: cycles = SAL_TICKS/(SLOTS·4·half).
pub const SAL_TICKS: usize = 4096;
/// f32s of self-describing header preceding the (i_d, i_q) pairs in the
/// burst buffer: `[kind, slots, cycles, block_ticks, half_ticks, v_low,
/// v_high, ctrl_freq]`. The host fit reads everything from here — it
/// hard-codes no firmware constants.
pub const SAL_HDR: usize = 8;
/// Allowed half-period lengths [ticks]. Powers of two so the cycle count
/// stays integral: 8→8 cycles, 16→4, 32→2, 64→1.
pub const SAL_HALF_CHOICES: [usize; 4] = [8, 16, 32, 64];

/// Pick the half-period for a motor with electrical time constant
/// `tau_ticks` (= (L/R)·ctrl_freq): the smallest choice giving ≥ 8·τ of
/// settling, saturating at the largest. At the ceiling a τ > 8 ticks motor
/// no longer fully settles — the host fit's plateau checks will say so.
pub fn sal_half_ticks(tau_ticks: f32) -> usize {
    let need = 8.0 * tau_ticks;
    for &h in &SAL_HALF_CHOICES {
        if h as f32 >= need {
            return h;
        }
    }
    SAL_HALF_CHOICES[SAL_HALF_CHOICES.len() - 1]
}

/// Block length (one slot visit) for a half-period: 4 square-wave halves.
pub fn sal_block_ticks(half_ticks: usize) -> usize {
    4 * half_ticks
}

/// Interleaved repeats of the full slot cycle for a half-period.
pub fn sal_cycles(half_ticks: usize) -> usize {
    SAL_TICKS / (SAL_SLOTS * sal_block_ticks(half_ticks))
}

/// Signed excitation angle for a recorded tick (0-based, align excluded),
/// in electrical radians relative to the align axis (θ = 0).
///
/// Slot s: magnitude index m = s/2 → δ = (m + 0.5)·π/SLOTS (5.625°…84.375°
/// in 11.25° steps), sign + for even s, − for odd s. 2δ then covers the
/// circle uniformly at 22.5° spacing — exactly what the cos 2δ / sin 2δ fit
/// wants.
pub fn sal_angle(tick: usize, half_ticks: usize) -> f32 {
    let slot = (tick / sal_block_ticks(half_ticks)) % SAL_SLOTS;
    let mag = (slot / 2) as f32 + 0.5;
    let delta = mag * (PI / SAL_SLOTS as f32);
    if slot.is_multiple_of(2) {
        delta
    } else {
        -delta
    }
}

/// Square-wave level for a recorded tick: half-periods run low, high, low,
/// high within each block, so every block starts settled at `v_low` after
/// its angle change and contains one falling and two rising edges.
pub fn sal_level_is_high(tick: usize, half_ticks: usize) -> bool {
    let half = (tick % sal_block_ticks(half_ticks)) / half_ticks;
    half % 2 == 1
}

/// Header block written at the front of the burst buffer.
pub fn sal_header(
    kind: u8,
    half_ticks: usize,
    v_low: f32,
    v_high: f32,
    ctrl_freq: f32,
) -> [f32; SAL_HDR] {
    [
        kind as f32,
        SAL_SLOTS as f32,
        sal_cycles(half_ticks) as f32,
        sal_block_ticks(half_ticks) as f32,
        half_ticks as f32,
        v_low,
        v_high,
        ctrl_freq,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_half_choice_tiles_the_buffer_exactly() {
        for &h in &SAL_HALF_CHOICES {
            let cycles = sal_cycles(h);
            assert!(cycles >= 1, "half {h} leaves no full cycle");
            assert_eq!(
                SAL_SLOTS * cycles * sal_block_ticks(h),
                SAL_TICKS,
                "half {h} does not tile SAL_TICKS"
            );
        }
    }

    #[test]
    fn half_picker_covers_both_bench_motors() {
        // Bench BLDC: tau = 31 us at 20 kHz = 0.62 ticks -> 8 (8 cycles).
        assert_eq!(sal_half_ticks(0.62), 8);
        // 4-pole motor: tau = 360 us = 7.2 ticks -> 64 (1 cycle).
        assert_eq!(sal_half_ticks(7.2), 64);
        // Saturates instead of overflowing the buffer.
        assert_eq!(sal_half_ticks(100.0), 64);
    }

    #[test]
    fn angles_pair_off_to_zero_net_torque() {
        // Magnet torque ∝ sin δ: over one cycle the signed angles must sum
        // their sines to zero (± pairing), or an unclamped rotor walks.
        for &h in &SAL_HALF_CHOICES {
            let mut sum = 0.0f64;
            for slot in 0..SAL_SLOTS {
                sum += libm::sin(sal_angle(slot * sal_block_ticks(h), h) as f64);
            }
            assert!(sum.abs() < 1e-6, "net sin(delta) = {sum} at half {h}");
        }
    }

    #[test]
    fn two_delta_covers_the_circle_uniformly() {
        // Uniform coverage of the 2δ circle ⇔ the first few harmonics of the
        // sample phasor sum vanish: Σ e^{i·k·2δ} = 0 for k = 1, 2, 3.
        for k in 1..=3 {
            let (mut sc, mut ss) = (0.0f64, 0.0f64);
            for slot in 0..SAL_SLOTS {
                let a = 2.0 * k as f64 * sal_angle(slot * sal_block_ticks(8), 8) as f64;
                sc += libm::cos(a);
                ss += libm::sin(a);
            }
            assert!(
                sc.abs() < 1e-5 && ss.abs() < 1e-5,
                "harmonic {k}: ({sc}, {ss})"
            );
        }
    }

    #[test]
    fn level_pattern_starts_low_and_alternates() {
        for &h in &[8usize, 64] {
            for block in 0..2 {
                let t0 = block * sal_block_ticks(h);
                for j in 0..h {
                    assert!(!sal_level_is_high(t0 + j, h), "half 0 must be v_low");
                    assert!(sal_level_is_high(t0 + h + j, h));
                    assert!(!sal_level_is_high(t0 + 2 * h + j, h));
                    assert!(sal_level_is_high(t0 + 3 * h + j, h));
                }
            }
        }
    }

    #[test]
    fn header_roundtrip() {
        let h = sal_header(1, 64, 0.3, 0.9, 20_000.0);
        assert_eq!(h[0], 1.0);
        assert_eq!(h[1] as usize, SAL_SLOTS);
        assert_eq!(h[2] as usize, 1); // 64-tick halves -> single cycle
        assert_eq!(h[3] as usize, 256);
        assert_eq!(h[4] as usize, 64);
        assert_eq!(h[5], 0.3);
        assert_eq!(h[6], 0.9);
        assert_eq!(h[7], 20_000.0);
    }
}
