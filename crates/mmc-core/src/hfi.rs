//! High-frequency injection at standstill: the on-board sweep schedule and
//! its burst layout, shared by the firmware and the host tools.
//!
//! ## The measurement
//!
//! A square-wave voltage `±V_h` alternating every control period (a carrier
//! at half the control rate) is applied along a test angle θx. At that
//! frequency the winding is mostly inductive, so the current answers with a
//! ripple of amplitude ∝ `V_h/L(θx)`. With saliency (`Ld ≠ Lq`, from the
//! rotor's geometry or from saturation along the magnet axis) the ripple
//! along the test axis varies as `cos 2(θx − θr)`, and the ripple *across*
//! it — a null channel on a round rotor — as `sin 2(θx − θr)`. Both are
//! demodulated against the carrier sign and accumulated per angle on the
//! device, so a sweep fits a few hundred bytes of burst buffer however long
//! it runs (the F302's buffer could not hold the raw-sample saliency probe).
//!
//! The carrier has zero mean, so the drive puts no net torque on the rotor:
//! the sweep measures a rotor that stays where it is. An optional align
//! (a DC current vector at a given angle, held, then released) parks the
//! rotor at a known angle first — a ground truth finer than a hall sector.
//!
//! Burst layout: [`HDR`] header floats `[kind, angles, cycles, dwell, skip,
//! v_h, align, hall, ctrl_hz, samples_per_bin]` then per angle
//! [`PER_ANGLE`] floats `[mean i_d, mean i_q, demod Δi_d, demod Δi_q]`, all
//! in the excitation frame (the demodulated sums are of the per-period
//! current *change*, weighted by [`weight`]). `align`/`hall` are [`NONE`]
//! when absent.

use core::f32::consts::PI;

use crate::math::{sin_cos, wrap_angle};
use crate::transforms::{park, AlphaBeta};

/// Test angles over one electrical turn (the 2θ saliency term repeats every
/// half turn; a full turn also shows a 1θ polarity term if there is one).
pub const ANGLES: usize = 24;
/// Interleaved sweeps: slow drift (winding temperature) spreads evenly over
/// the angles instead of faking a pattern.
pub const CYCLES: usize = 6;
/// Ticks per visit to an angle, and how many of them settle unrecorded.
pub const DWELL: usize = 80;
pub const SKIP: usize = 16;
pub const HDR: usize = 10;
pub const PER_ANGLE: usize = 4;
/// Burst floats a sweep fills.
pub const LEN: usize = HDR + ANGLES * PER_ANGLE;
/// Recorded ticks after the align.
pub const TICKS: usize = ANGLES * CYCLES * DWELL;
/// Marker for an absent align or hall angle.
pub const NONE: f32 = 99.0;
/// Align current [A] (as volts over the live R) and its hold and release.
pub const ALIGN_A: f32 = 0.8;
pub const ALIGN_S: f32 = 0.3;
pub const RELEASE_S: f32 = 0.05;

/// Carrier sign at tick `t`: `++−−`, a quarter of the control rate. A sign
/// held for two periods is seen whole by at least one sample interval
/// whatever the duty's load latency; flipping every period, the sample
/// interval straddles the flip on a board that loads the duty mid-period
/// (the F302's) and the response cancels.
pub fn carrier(t: usize) -> f32 {
    if (t / 2).is_multiple_of(2) {
        1.0
    } else {
        -1.0
    }
}

/// Demodulation weight for the current *change* into the sample at tick
/// `t`: the mean of the two carriers before it, `+1, 0, −1, 0`. Exact for a
/// mid-period duty load, and still right for a full-period one (non-zero
/// only where both candidates agree).
pub fn weight(t: usize) -> f32 {
    if t < 2 {
        return 0.0;
    }
    0.5 * (carrier(t - 1) + carrier(t - 2))
}

/// Test angle index, carrier sign and demodulation weight at sweep tick
/// `t`, and whether the tick is recorded (past the visit's settling).
pub fn schedule(t: usize) -> (usize, f32, f32, bool) {
    let visit = t / DWELL;
    let k = visit % ANGLES;
    let within = t % DWELL;
    (k, carrier(t), weight(t), within >= SKIP)
}

/// Electrical test angle of index `k` [rad].
pub fn angle(k: usize) -> f32 {
    2.0 * PI * k as f32 / ANGLES as f32
}

/// Samples accumulated per angle over a whole sweep.
pub fn samples_per_bin() -> usize {
    CYCLES * (DWELL - SKIP)
}

/// Real-time pulsating square-wave HFI: the rotor axis tracked at any speed
/// down to zero from saliency alone.
///
/// Every control period the drive adds `±v_h` (alternating) along the
/// estimate's d axis ([`Tracker::carrier`]). The current answers along d
/// with a ripple ∝ 1/L; across it (q̂) only if the estimate is off the
/// rotor's axis, as `−ξ·sin 2(θ̂ − θr)` relative to the d ripple (ξ the
/// saliency ratio `(Lq − Ld)/(Lq + Ld)`; the sign measured on motor 3 and on
/// a salient simulated motor with the standstill sweep). Both are demodulated by
/// differencing consecutive samples against the carrier sign, which cancels
/// the drive's own (slow) current, and a PLL turns the normalised cross
/// ripple into angle and speed. Its gains are scaled by ξ so `bandwidth`
/// is what the loop actually gets.
///
/// The axis comes out modulo π: saliency cannot tell the magnet's north
/// from its south. A start must settle the polarity separately.
#[derive(Copy, Clone, Debug)]
pub struct Tracker {
    v_h: f32,
    kp: f32,
    ki: f32,
    theta: f32,
    omega: f32,
    /// Carrier phase (0..4, `++−−`) and the last two carriers commanded.
    phase: usize,
    c1: f32,
    c2: f32,
    prev: Option<AlphaBeta>,
    /// Low-passed d-axis demodulated ripple [A] (the normaliser; its sign
    /// is the carrier-to-sample latency's).
    pub d_amp: f32,
    /// This period's unfiltered d response [A], when the period refreshed
    /// it (every other one): a short polarity pulse is read from these, as
    /// `d_amp` still carries the previous pulse.
    pub d_fresh: Option<f32>,
    /// Last normalised error (≈ 2ξ·(θr − θ̂) when small).
    pub err: f32,
}

impl Tracker {
    /// `v_h` carrier amplitude [V], `xi` the saliency ratio (from the
    /// sweep), `bandwidth` the PLL's natural frequency [rad/s].
    pub fn new(v_h: f32, xi: f32, bandwidth: f32, theta0: f32) -> Self {
        let g = 2.0 * xi.abs().max(1e-3);
        Self {
            v_h,
            kp: 2.0 * bandwidth / g,
            ki: bandwidth * bandwidth / g,
            theta: wrap_angle(theta0),
            omega: 0.0,
            phase: 0,
            c1: 0.0,
            c2: 0.0,
            prev: None,
            d_amp: 0.0,
            d_fresh: None,
            err: 0.0,
        }
    }

    /// The carrier voltage to add along the estimate's d axis this period
    /// (`++−−` at a quarter of the control rate, see [`carrier`]).
    pub fn carrier(&self) -> f32 {
        carrier(self.phase) * self.v_h
    }

    /// Feed this period's measured current (it answers the carriers of the
    /// previous periods); returns the updated (angle, speed) estimate and
    /// advances the carrier to the one to command now.
    pub fn update(&mut self, i_ab: AlphaBeta, dt: f32) -> (f32, f32) {
        let w = 0.5 * (self.c1 + self.c2);
        if let (Some(prev), true) = (self.prev, w != 0.0) {
            let sc = sin_cos(self.theta);
            let now = park(i_ab, sc);
            let before = park(prev, sc);
            let dd = w * (now.d - before.d);
            let dq = w * (now.q - before.q);
            // ~10-update average: the normaliser need not be fast.
            self.d_fresh = Some(dd);
            self.d_amp += 0.1 * (dd - self.d_amp);
            if self.d_amp.abs() > 1e-4 {
                self.err = (dq / self.d_amp).clamp(-1.0, 1.0);
            }
        } else {
            self.d_fresh = None;
            self.err = 0.0;
        }
        // The PLL runs every period; the error refreshes every other one, so
        // its gains (for a per-period error) are doubled.
        self.omega += 2.0 * self.ki * self.err * dt;
        self.theta = wrap_angle(self.theta + (self.omega + 2.0 * self.kp * self.err) * dt);
        self.prev = Some(i_ab);
        self.phase = (self.phase + 1) % 4;
        self.c2 = self.c1;
        self.c1 = carrier(self.phase);
        (self.theta, self.omega)
    }

    pub fn theta(&self) -> f32 {
        self.theta
    }

    pub fn omega(&self) -> f32 {
        self.omega
    }

    /// Take over a known speed (a handover from another estimator), so the
    /// PLL does not have to acquire it.
    pub fn set_omega(&mut self, omega: f32) {
        self.omega = omega;
    }

    /// Turn the estimate by π (after a polarity check said it locked onto
    /// the magnet's south).
    pub fn flip(&mut self) {
        self.theta = wrap_angle(self.theta + PI);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::{inverse_park, Dq};

    /// A salient RL winding at standstill or slow rotation, voltage applied
    /// for one period with a one-period delay, sampled at the period end:
    /// the tracker must lock onto the rotor axis (mod π) from 60° away and
    /// follow a slow turn.
    #[test]
    fn tracker_locks_onto_a_salient_rotor() {
        let (r, ld, lq) = (1.4f32, 0.33e-3f32, 0.40e-3f32);
        let xi = (lq - ld) / (lq + ld);
        let dt = 1e-4;
        for &(theta_r0, omega_r) in &[(0.5f32, 0.0f32), (-2.0, 0.0), (1.0, 30.0), (0.3, -20.0)] {
            let mut tr = Tracker::new(2.0, xi, 300.0, theta_r0 + 1.0);
            let mut theta_r = theta_r0;
            let mut i = Dq { d: 0.0, q: 0.0 }; // rotor frame
            let mut v_pending = AlphaBeta {
                alpha: 0.0,
                beta: 0.0,
            };
            let mut worst_late = 0.0f32;
            for n in 0..6000 {
                // Plant: apply last period's command in the rotor frame.
                let sc_r = sin_cos(theta_r);
                let v = park(v_pending, sc_r);
                i.d += (v.d - r * i.d) / ld * dt;
                i.q += (v.q - r * i.q) / lq * dt;
                theta_r = wrap_angle(theta_r + omega_r * dt);
                let i_ab = inverse_park(i, sin_cos(theta_r));
                let (th, _) = tr.update(i_ab, dt);
                // Command for next period: the carrier on the estimate's d.
                // (`update` flipped the sign, so this is the next carrier.)
                v_pending = inverse_park(
                    Dq {
                        d: tr.carrier(),
                        q: 0.0,
                    },
                    sin_cos(th),
                );
                if n > 3000 {
                    let e = wrap_angle(2.0 * (th - theta_r)).abs() / 2.0;
                    worst_late = worst_late.max(e);
                }
            }
            assert!(
                worst_late < 0.05,
                "rotor {theta_r0} at {omega_r} rad/s: error {worst_late} rad (mod π)"
            );
        }
    }

    #[test]
    fn every_angle_gets_the_same_samples_and_a_balanced_carrier() {
        let mut n = [0usize; ANGLES];
        let mut sum = [0.0f32; ANGLES];
        for t in 0..TICKS {
            let (k, s, _, rec) = schedule(t);
            if rec {
                n[k] += 1;
                sum[k] += s;
            }
        }
        assert!(n.iter().all(|&c| c == samples_per_bin()));
        assert!(
            sum.iter().all(|&s| s == 0.0),
            "carrier mean per angle: {sum:?}"
        );
    }
}
