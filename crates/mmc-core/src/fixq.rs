//! Integer-only low-speed sensorless drive, for MCUs without an FPU
//! (Cortex-M0+): the FOC current loop, pulsating-carrier HFI tracking, its
//! start from standstill (lock, polarity pulses) and the speed loop, the
//! same algorithms as [`crate::foc`], [`crate::hfi`] and the drive's HFI
//! start, in fixed point.
//!
//! The per-tick path ([`FixHfi::step`]) is i32 multiplies, adds and shifts,
//! one i32 division (the tracker's normalisation), one u32 division (the bus
//! voltage reciprocal) and two integer square roots — no float. Setup
//! ([`FixHfi::new`]) converts the f32 parameters once.
//!
//! Units:
//! - current: Q15 of `i_base` amps (32768 = `i_base`)
//! - voltage: Q15 of `v_base` volts
//! - electrical angle: u32, 2³² = one revolution (wraps for free)
//! - speed: angle units per control tick (`w_u`); the tracker and the speed
//!   reference keep [`W_FRAC`] extra fraction bits, the speed loop works on
//!   `w_u >> W_SHIFT`
//! - duty: Q15, 0..=32768
//!
//! Every product is i32 × i32 with both factors bounded to ±2¹⁵ (or a gain
//! below 2¹⁵), so nothing needs a 64-bit multiply, which the M0+ lacks.

/// Q15 one.
pub const ONE: i32 = 1 << 15;
/// Extra fraction bits on the tracker speed and the speed reference.
pub const W_FRAC: u32 = 4;
/// The speed loop's speed unit: `w_u >> W_SHIFT`.
pub const W_SHIFT: u32 = 10;
/// Angle units per radian.
const UNITS_PER_RAD: f32 = 4_294_967_296.0 / (2.0 * core::f32::consts::PI);

/// sin over one revolution, 256 steps + the wrap point, Q15.
static SIN: [i16; 257] = [
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787,
    21403, 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790, 27245, 27683,
    28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113, 31356, 31580, 31785,
    31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678,
    32609, 32521, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571, 30273,
    29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25329, 24811,
    24279, 23731, 23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846,
    16151, 15446, 14732, 14010, 13279, 12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179, 6393,
    5602, 4808, 4011, 3212, 2410, 1608, 804, 0, -804, -1608, -2410, -3212, -4011, -4808, -5602,
    -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793, -12539, -13279, -14010, -14732,
    -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105, -28510,
    -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757, -32767, -32757, -32728, -32678,
    -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832,
    -25329, -24811, -24279, -23731, -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279, -12539, -11793, -11039, -10278,
    -9512, -8739, -7962, -7179, -6393, -5602, -4808, -4011, -3212, -2410, -1608, -804, 0,
];

/// (sin, cos) of a binary angle, Q15, linear interpolation (error ≤ ~3 LSB).
#[inline]
pub fn sin_cos(theta: u32) -> (i32, i32) {
    (sin(theta), sin(theta.wrapping_add(1 << 30)))
}

#[inline]
fn sin(theta: u32) -> i32 {
    let i = (theta >> 24) as usize;
    let frac = ((theta >> 8) & 0xFFFF) as i32;
    let a = SIN[i] as i32;
    let b = SIN[i + 1] as i32;
    a + (((b - a) * frac) >> 16)
}

#[inline]
fn clamp(x: i32, lim: i32) -> i32 {
    x.clamp(-lim, lim)
}

/// Integer square root (floor), bitwise: 16 iterations, no division.
pub fn isqrt(x: u32) -> u32 {
    let mut rem = x;
    let mut root = 0u32;
    let mut bit = 1u32 << 30;
    while bit > rem {
        bit >>= 2;
    }
    while bit != 0 {
        if rem >= root + bit {
            rem -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    root
}

/// A real gain as `x·m >> s`, for |x| ≤ 2¹⁵: `m` in [2¹⁴, 2¹⁵) where the
/// gain allows, so the product stays under 2³⁰.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Gain {
    m: i32,
    s: u32,
}

impl Gain {
    pub fn new(g: f32) -> Self {
        if g == 0.0 || !g.is_finite() {
            return Self { m: 0, s: 0 };
        }
        let mut m = g;
        let mut s = 0;
        while m.abs() < 16384.0 && s < 62 {
            m *= 2.0;
            s += 1;
        }
        Self {
            m: (m + 0.5 * m.signum()).clamp(-32767.0, 32767.0) as i32,
            s,
        }
    }

    /// `x·g`, `x` clamped to ±2¹⁵.
    #[inline]
    pub fn apply(self, x: i32) -> i32 {
        if self.s >= 31 {
            return 0;
        }
        (clamp(x, ONE) * self.m) >> self.s
    }
}

/// PI with a Q30 integrator (Q15 output and 15 extra bits of integration
/// resolution) and clamping anti-windup.
#[derive(Copy, Clone, Debug, Default)]
pub struct Pi {
    kp: Gain,
    /// ki·dt per tick, scaled into the Q30 integrator.
    ki: Gain,
    integ: i32,
}

impl Pi {
    /// `kp` output Q15 per input LSB; `ki_dt` the same per tick.
    pub fn new(kp: f32, ki_dt: f32) -> Self {
        Self {
            kp: Gain::new(kp),
            ki: Gain::new(ki_dt * ONE as f32),
            integ: 0,
        }
    }

    pub fn set_gains(&mut self, kp: f32, ki_dt: f32) {
        self.kp = Gain::new(kp);
        self.ki = Gain::new(ki_dt * ONE as f32);
    }

    /// Output Q15 clamped to ±`lim` (≤ 2¹⁵).
    #[inline]
    pub fn update(&mut self, err: i32, lim: i32) -> i32 {
        let e = clamp(err, ONE);
        let ilim = lim << 15;
        self.integ = clamp(self.integ.saturating_add(self.ki.apply(e)), ilim);
        clamp(self.kp.apply(e) + (self.integ >> 15), lim)
    }

    pub fn preload(&mut self, out: i32) {
        self.integ = out << 15;
    }
}

/// Pulsating square-wave HFI tracker (see [`crate::hfi::Tracker`]), integer.
#[derive(Copy, Clone, Debug)]
pub struct FixTracker {
    vh: i32,
    /// Angle units per err LSB, and `w` (×2^W_FRAC) per err LSB per tick.
    kp: i32,
    ki: i32,
    pub theta: u32,
    /// Speed [angle units per tick × 2^W_FRAC].
    pub omega_f: i32,
    omega_max_f: i32,
    seq: crate::hfi::CarrierSeq,
    c1: i32,
    c2: i32,
    prev: Option<(i32, i32)>,
    /// Low-passed demodulated d ripple (Q15 A), the normaliser.
    pub d_amp: i32,
    /// This tick's unfiltered d response, when it refreshed.
    pub d_fresh: Option<i32>,
    pub err: i32,
}

impl FixTracker {
    /// `vh` carrier [Q15 V]; `xi`, `bandwidth` [rad/s] as the float
    /// tracker; `dt` the control period; `omega_max` [rad/s el] the speed
    /// clamp (keeps the integer state in range).
    pub fn new(vh: i32, xi: f32, bandwidth: f32, dt: f32, omega_max: f32) -> Self {
        let g = 2.0 * xi.abs().max(1e-3);
        let kp = 2.0 * bandwidth / g;
        let ki = bandwidth * bandwidth / g;
        let per_lsb = 1.0 / ONE as f32;
        let wf = (1u32 << W_FRAC) as f32;
        Self {
            vh,
            // θ += 2·kp·err·dt; ω += 2·ki·err·dt (gains doubled: the error
            // refreshes every other tick).
            kp: (2.0 * kp * dt * UNITS_PER_RAD * per_lsb) as i32,
            ki: (2.0 * ki * dt * dt * UNITS_PER_RAD * wf * per_lsb + 0.5) as i32,
            theta: 0,
            omega_f: 0,
            omega_max_f: (omega_max * dt * UNITS_PER_RAD * wf).min(i32::MAX as f32 / 2.0) as i32,
            seq: crate::hfi::CarrierSeq::new(0),
            c1: 0,
            c2: 0,
            prev: None,
            d_amp: 0,
            d_fresh: None,
            err: 0,
        }
    }

    /// Spread the carrier (see [`crate::hfi::CarrierSeq`]).
    pub fn with_spread(mut self, spread: u8) -> Self {
        self.seq = crate::hfi::CarrierSeq::new(spread);
        self
    }

    /// Carrier voltage to add along the tracker's d axis now [Q15 V].
    #[inline]
    pub fn carrier(&self) -> i32 {
        self.c1 * self.vh
    }

    /// Speed [angle units per tick].
    #[inline]
    pub fn omega_u(&self) -> i32 {
        self.omega_f >> W_FRAC
    }

    /// Feed this tick's measured αβ current (Q15).
    pub fn update(&mut self, i_ab: (i32, i32)) {
        let w = (self.c1 + self.c2) / 2;
        match self.prev {
            Some(prev) if w != 0 => {
                let (s, c) = sin_cos(self.theta);
                let (nd, nq) = park(i_ab, s, c);
                let (bd, bq) = park(prev, s, c);
                let dd = clamp(w * (nd - bd), ONE);
                let dq = clamp(w * (nq - bq), ONE);
                self.d_fresh = Some(dd);
                // ~0.1 per update, as the float tracker.
                self.d_amp += ((dd - self.d_amp) * 13) >> 7;
                if self.d_amp.abs() > 2 {
                    self.err = clamp((dq << 15) / self.d_amp, ONE);
                }
            }
            _ => {
                self.d_fresh = None;
                self.err = 0;
            }
        }
        self.omega_f = clamp(self.omega_f + self.ki * self.err, self.omega_max_f);
        self.theta = self
            .theta
            .wrapping_add(((self.omega_f >> W_FRAC) + self.kp * self.err) as u32);
        self.prev = Some(i_ab);
        self.c2 = self.c1;
        self.c1 = self.seq.advance() as i32;
    }

    /// Turn the estimate onto the other pole.
    pub fn flip(&mut self) {
        self.theta = self.theta.wrapping_add(1 << 31);
    }
}

#[inline]
fn park((a, b): (i32, i32), s: i32, c: i32) -> (i32, i32) {
    let a = clamp(a, ONE - 1);
    let b = clamp(b, ONE - 1);
    ((a * c + b * s) >> 15, (b * c - a * s) >> 15)
}

#[inline]
fn inverse_park((d, q): (i32, i32), s: i32, c: i32) -> (i32, i32) {
    let d = clamp(d, ONE - 1);
    let q = clamp(q, ONE - 1);
    ((d * c - q * s) >> 15, (d * s + q * c) >> 15)
}

/// abc → αβ, amplitude-invariant (as [`crate::transforms::clarke`]).
#[inline]
pub fn clarke(a: i32, b: i32, c: i32) -> (i32, i32) {
    (((2 * a - b - c) * 10923) >> 15, ((b - c) * 18919) >> 15)
}

/// Min-max (SVPWM-equivalent) modulation of an αβ voltage [Q15 V] on a bus
/// whose reciprocal `recip` = 2²⁸ / vbus is given; duties Q15.
#[inline]
fn modulate((al, be): (i32, i32), recip: i32) -> [i32; 3] {
    let al = clamp(al, ONE);
    let be = clamp(be, ONE);
    let h = (be * 28378) >> 15; // β·√3/2
    let a = al;
    let b = -(al >> 1) + h;
    let c = -(al >> 1) - h;
    let common = -((a.max(b).max(c) + a.min(b).min(c)) >> 1);
    let duty = |p: i32| ((((p + common) * recip) >> 13) + (ONE >> 1)).clamp(0, ONE);
    [duty(a), duty(b), duty(c)]
}

/// The drive's parameters, in SI units, converted once.
#[derive(Copy, Clone, Debug)]
pub struct FixParams {
    pub i_base: f32,
    pub v_base: f32,
    pub dt: f32,
    pub r: f32,
    pub l: f32,
    pub flux: f32,
    pub cur_bw: f32,
    /// Speed-loop gains [A per rad/s el, A per rad el].
    pub speed_kp: f32,
    pub speed_ki: f32,
    pub iq_limit: f32,
    pub omega_accel: f32,
    pub hfi_v: f32,
    pub hfi_xi: f32,
    pub hfi_bw: f32,
    pub hfi_xsat: f32,
    /// Carrier spreading, 0..=2 (`hfi_spread`).
    pub hfi_spread: u8,
    pub id_inject: f32,
    pub pol_a: f32,
    pub pol_s: f32,
    pub pol_n: u32,
    pub lock_s: f32,
    /// The lock's d bias ramps in over this long.
    pub lock_ramp_s: f32,
    pub stuck_s: f32,
    /// Ticks the duty lands after the sample (advance of the output angle).
    pub advance_periods: f32,
    /// Speed clamp [rad/s el].
    pub omega_max: f32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Phase {
    Lock,
    PolPos,
    PolNeg,
    Run,
}

/// Everything a tick produced, integer, for actuation and telemetry.
#[derive(Copy, Clone, Debug, Default)]
pub struct FixOut {
    pub duties: [i32; 3],
    pub i_dq: (i32, i32),
    pub v_dq: (i32, i32),
    /// Commanded αβ voltage without the carrier (what an observer needs).
    pub v_ab: (i32, i32),
    pub i_ab: (i32, i32),
    pub iq_cmd: i32,
    /// Angle the FOC used, and the tracker's speed [units/tick].
    pub theta: u32,
    pub omega_u: i32,
    /// The speed loop has held its limit on a rotor that is not following.
    pub stuck: bool,
}

/// The integer HFI sensorless drive: start sequence, tracker, speed loop,
/// current loop, modulation.
#[derive(Copy, Clone, Debug)]
pub struct FixHfi {
    pub tracker: FixTracker,
    pi_d: Pi,
    pi_q: Pi,
    speed: Pi,
    /// Decoupling: ψ per speed-loop unit, L per (w_s·i Q15).
    ff_flux: Gain,
    ff_l: Gain,
    /// θ advance of the output, ×8 per `w_u >> 3`.
    adv_x8: i32,
    xsat: i32,
    iq_limit: i32,
    id_inject: i32,
    pol_a: i32,
    accel_f: i32,
    stuck_w_min: i32,
    lock_ticks: u32,
    lock_ramp_ticks: u32,
    pol_ticks: u32,
    pol_n: u32,
    stuck_ticks: u32,
    pub phase: Phase,
    ticks: u32,
    pairs: u32,
    acc: [i32; 2],
    n: [i32; 2],
    stuck: u32,
    /// Speed reference [units/tick × 2^W_FRAC].
    pub w_ref_f: i32,
}

impl FixHfi {
    pub fn new(p: &FixParams) -> Self {
        let iq = ONE as f32 / p.i_base; // Q15 per A
        let vq = ONE as f32 / p.v_base; // Q15 per V
        let ticks = |s: f32| (s / p.dt + 0.5) as u32;
        // Speed-loop unit per rad/s el.
        let ws_per_rad_s = p.dt * UNITS_PER_RAD / (1u32 << W_SHIFT) as f32;
        let wf = (1u32 << W_FRAC) as f32;
        // Current PI: V per A → Q15 V per Q15 A.
        let (kp_c, ki_c) = (p.l * p.cur_bw, p.r * p.cur_bw);
        let cv = vq / iq;
        Self {
            tracker: FixTracker::new((p.hfi_v * vq) as i32, p.hfi_xi, p.hfi_bw, p.dt, p.omega_max)
                .with_spread(p.hfi_spread),
            pi_d: Pi::new(kp_c * cv, ki_c * p.dt * cv),
            pi_q: Pi::new(kp_c * cv, ki_c * p.dt * cv),
            speed: Pi::new(
                p.speed_kp * iq / ws_per_rad_s,
                p.speed_ki * p.dt * iq / ws_per_rad_s,
            ),
            ff_flux: Gain::new(p.flux * vq / ws_per_rad_s),
            // ω·L·i: (w_s · i_Q15) >> 15 is rad/s·A scaled by ws_per_rad_s/iq·2^-15.
            ff_l: Gain::new(p.l * vq * ONE as f32 / (ws_per_rad_s * iq)),
            adv_x8: (p.advance_periods * 8.0 + 0.5) as i32,
            xsat: (p.hfi_xsat / iq * UNITS_PER_RAD) as i32,
            iq_limit: (p.iq_limit * iq) as i32,
            id_inject: (p.id_inject * iq) as i32,
            pol_a: (p.pol_a * iq) as i32,
            accel_f: (p.omega_accel * p.dt * p.dt * UNITS_PER_RAD * wf + 0.5) as i32,
            stuck_w_min: (5.0 * ws_per_rad_s) as i32,
            lock_ticks: ticks(p.lock_s),
            lock_ramp_ticks: ticks(p.lock_ramp_s).max(1),
            pol_ticks: ticks(p.pol_s).max(2),
            pol_n: p.pol_n.max(1),
            stuck_ticks: ticks(p.stuck_s),
            phase: Phase::Lock,
            ticks: 0,
            pairs: 0,
            acc: [0; 2],
            n: [0; 2],
            stuck: 0,
            w_ref_f: 0,
        }
    }

    /// New speed-loop gains (same units as [`FixParams`]).
    pub fn set_speed_gains(&mut self, p: &FixParams, kp: f32, ki: f32) {
        let iq = ONE as f32 / p.i_base;
        let ws_per_rad_s = p.dt * UNITS_PER_RAD / (1u32 << W_SHIFT) as f32;
        self.speed
            .set_gains(kp * iq / ws_per_rad_s, ki * p.dt * iq / ws_per_rad_s);
    }

    /// The speed target [rad/s el] in reference units (setup-side helper).
    pub fn target_units(p: &FixParams, omega: f32) -> i32 {
        (omega * p.dt * UNITS_PER_RAD * (1u32 << W_FRAC) as f32) as i32
    }

    /// One control tick. `i_abc` measured phase currents [Q15 A], `vbus`
    /// [Q15 V], `target_f` the speed command [units/tick × 2^W_FRAC].
    #[inline(never)]
    pub fn step(&mut self, i_abc: [i32; 3], vbus: i32, target_f: i32) -> FixOut {
        let tr = self.tracker;
        let w_u = tr.omega_u();
        let w_s = tr.omega_f >> (W_FRAC + W_SHIFT);

        // --- start sequence / speed loop → (id, iq) commands.
        self.ticks += 1;
        let mut iq_cmd = 0;
        let id_cmd = match self.phase {
            Phase::Lock => {
                if self.ticks >= self.lock_ticks {
                    self.phase = Phase::PolPos;
                    self.ticks = 0;
                }
                if self.ticks >= self.lock_ramp_ticks {
                    self.id_inject
                } else {
                    // ticks < ramp ≤ 2¹⁵ for any sane ramp: no overflow.
                    self.id_inject * self.ticks as i32 / self.lock_ramp_ticks as i32
                }
            }
            Phase::PolPos | Phase::PolNeg => {
                let k = (self.phase == Phase::PolNeg) as usize;
                if let Some(d) = tr.d_fresh.filter(|_| 2 * self.ticks > self.pol_ticks) {
                    self.acc[k] += d.abs();
                    self.n[k] += 1;
                }
                if self.ticks >= self.pol_ticks {
                    self.ticks = 0;
                    if k == 0 {
                        self.phase = Phase::PolNeg;
                    } else if self.pairs + 1 < self.pol_n {
                        self.pairs += 1;
                        self.phase = Phase::PolPos;
                    } else {
                        // Compare the means without dividing:
                        // neg/n1 > pos/n0 ⇔ neg·n0 > pos·n1 (both small).
                        let pos = self.acc[0] >> 4;
                        let neg = self.acc[1] >> 4;
                        if neg * self.n[0] > pos * self.n[1] {
                            self.tracker.flip();
                        }
                        self.phase = Phase::Run;
                    }
                }
                if k == 0 {
                    self.pol_a
                } else {
                    -self.pol_a
                }
            }
            Phase::Run => {
                let d = clamp(target_f - self.w_ref_f, self.accel_f);
                self.w_ref_f += d;
                let ref_s = self.w_ref_f >> (W_FRAC + W_SHIFT);
                iq_cmd = self.speed.update(ref_s - w_s, self.iq_limit);
                let stuck = iq_cmd.abs() >= self.iq_limit - (self.iq_limit >> 3)
                    && w_s.abs() < (ref_s.abs().max(self.stuck_w_min) >> 1);
                self.stuck = if stuck {
                    self.stuck + 1
                } else {
                    self.stuck.saturating_sub(1)
                };
                self.id_inject
            }
        };
        let running = self.phase == Phase::Run;

        // --- FOC on the tracker's angle, one tick ahead, plus the
        // cross-saturation correction.
        let theta = tr
            .theta
            .wrapping_add(w_u as u32)
            .wrapping_add((self.xsat * iq_cmd) as u32);
        let (s, c) = sin_cos(theta);
        let i_ab = clarke(i_abc[0], i_abc[1], i_abc[2]);
        let (id, iq) = park(i_ab, s, c);
        let vlim = (clamp(vbus, ONE) * 18919) >> 15; // vbus/√3

        // Decoupling (off until running: the speed is noise at a standstill).
        let w_ff = if running { clamp(w_s, ONE) } else { 0 };
        let wi_d = (w_ff * clamp(id, ONE)) >> 15;
        let wi_q = (w_ff * clamp(iq, ONE)) >> 15;
        // Each under 2¹⁵/√2, so the squared magnitude fits an i32.
        let mut ff_d = clamp(-self.ff_l.apply(wi_q), 23170);
        let mut ff_q = clamp(self.ff_flux.apply(w_ff) + self.ff_l.apply(wi_d), 23170);
        let ff_mag = isqrt((ff_d * ff_d + ff_q * ff_q) as u32) as i32;
        let pi_lim = if ff_mag > vlim {
            ff_d = ff_d * vlim / ff_mag;
            ff_q = ff_q * vlim / ff_mag;
            0
        } else {
            vlim - ff_mag
        };
        let vd_pi = self.pi_d.update(id_cmd - id, pi_lim);
        let q_lim = isqrt((pi_lim * pi_lim - vd_pi * vd_pi).max(0) as u32) as i32;
        let vq_pi = self.pi_q.update(iq_cmd - iq, q_lim);
        let v_dq = (ff_d + vd_pi, ff_q + vq_pi);
        let (so, co) = sin_cos(theta.wrapping_add(((w_u >> 3) * self.adv_x8) as u32));
        let v_ab = inverse_park(v_dq, so, co);

        // --- HFI: update on this sample, inject along the new estimate.
        self.tracker.update(i_ab);
        if !running {
            // The rotor should be still: no co-rotating tracker.
            self.tracker.omega_f = 0;
        }
        let (st, ct) = sin_cos(self.tracker.theta);
        let vh = self.tracker.carrier();
        let v_mod = (v_ab.0 + ((vh * ct) >> 15), v_ab.1 + ((vh * st) >> 15));
        let recip = ((1u32 << 28) / (vbus.max(ONE >> 4) as u32)) as i32;
        FixOut {
            duties: modulate(v_mod, recip),
            i_dq: (id, iq),
            v_dq,
            v_ab,
            i_ab,
            iq_cmd,
            theta,
            omega_u: w_u,
            stuck: self.stuck >= self.stuck_ticks,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math;

    #[test]
    fn sin_cos_matches_float_to_a_few_lsb() {
        let mut worst = 0;
        for k in 0..4096u32 {
            let th = k.wrapping_mul(1_048_573);
            let (s, c) = sin_cos(th);
            let x = th as f32 / UNITS_PER_RAD;
            let (fs, fc) = (libm::sinf(x), libm::cosf(x));
            worst = worst
                .max((s - (fs * 32767.0) as i32).abs())
                .max((c - (fc * 32767.0) as i32).abs());
        }
        assert!(worst <= 4, "worst {worst} LSB");
    }

    #[test]
    fn isqrt_is_floor_sqrt() {
        for x in [
            0u32,
            1,
            2,
            3,
            4,
            15,
            16,
            17,
            1 << 20,
            999_999,
            (1 << 30) + 12345,
            u32::MAX >> 1,
        ] {
            let r = isqrt(x) as u64;
            assert!(r * r <= x as u64 && (r + 1) * (r + 1) > x as u64, "{x}");
        }
    }

    #[test]
    fn gain_scales_across_magnitudes() {
        for g in [1e-6f32, 3.3e-3, 0.0445, 0.7, 1.0, 13.0, 11300.0] {
            let gn = Gain::new(g);
            for x in [1000i32, -20000, 32768] {
                let want = x as f32 * g;
                let got = gn.apply(x) as f32;
                assert!(
                    (got - want).abs() <= want.abs() * 2e-4 + 1.0,
                    "g {g} x {x}: {got} vs {want}"
                );
            }
        }
    }

    #[test]
    fn pi_integrates_below_one_lsb_per_tick() {
        // ki·dt far under one output LSB per tick still integrates.
        let mut pi = Pi::new(0.0, 1e-3);
        let mut out = 0;
        for _ in 0..10_000 {
            out = pi.update(100, ONE);
        }
        assert!((out - 1000).abs() <= 2, "{out}");
    }

    #[test]
    fn clarke_park_round_trip_match_float() {
        for k in 0..64 {
            let th = (k as u32).wrapping_mul(67_108_864) + 12345;
            let x = th as f32 / UNITS_PER_RAD;
            let (fa, fb, fc) = (0.3f32, -0.1, -0.2);
            let q = |v: f32| (v * ONE as f32) as i32;
            let (al, be) = clarke(q(fa), q(fb), q(fc));
            let (s, c) = sin_cos(th);
            let (d, qq) = park((al, be), s, c);
            let ab = crate::transforms::clarke(crate::transforms::Abc {
                a: fa,
                b: fb,
                c: fc,
            });
            let dq = crate::transforms::park(ab, math::sin_cos(x));
            assert!(
                (d - q(dq.d)).abs() <= 6 && (qq - q(dq.q)).abs() <= 6,
                "{k}: {d},{qq} vs {:?}",
                dq
            );
        }
    }

    #[cfg(feature = "foc")]
    #[test]
    fn modulation_matches_float_svpwm() {
        let vbus = 18.0f32;
        let vbase = 32.0f32;
        let q = |v: f32| (v / vbase * ONE as f32) as i32;
        let recip = ((1u32 << 28) / q(vbus) as u32) as i32;
        for k in 0..36 {
            let a = k as f32 * 0.1745;
            let (al, be) = (6.0 * libm::cosf(a), 6.0 * libm::sinf(a));
            let d = modulate((q(al), q(be)), recip);
            let f = crate::svpwm::svpwm(
                crate::transforms::AlphaBeta {
                    alpha: al,
                    beta: be,
                },
                vbus,
            );
            for i in 0..3 {
                let got = d[i] as f32 / ONE as f32;
                assert!((got - f[i]).abs() < 2e-3, "{k}/{i}: {got} vs {}", f[i]);
            }
        }
    }

    #[test]
    fn tracker_matches_float_on_the_same_currents() {
        // A salient-motor response synthesised from the float tracker's own
        // model: both trackers fed identical currents from a rotor at 0.6 rad.
        let dt = 1e-4;
        let (ld, lq) = (0.33e-3f32, 0.39e-3);
        let xi = (lq - ld) / (lq + ld);
        let mut fl = crate::hfi::Tracker::new(1.0, xi, 150.0, 0.0);
        let mut fx = FixTracker::new((1.0 / 32.0 * ONE as f32) as i32, xi, 150.0, dt, 2000.0);
        let theta_r = 0.6f32;
        let (mut ia, mut ib) = (0.0f32, 0.0f32);
        for _ in 0..4000 {
            // Integrate the carrier the float tracker commands on its axis.
            let v = fl.carrier();
            let th = fl.theta();
            let (va, vb) = (v * libm::cosf(th), v * libm::sinf(th));
            let (sr, cr) = (libm::sinf(theta_r), libm::cosf(theta_r));
            let vd = va * cr + vb * sr;
            let vq = -va * sr + vb * cr;
            let (dd, dq) = (vd / ld * dt, vq / lq * dt);
            ia += dd * cr - dq * sr;
            ib += dd * sr + dq * cr;
            fl.update(
                crate::transforms::AlphaBeta {
                    alpha: ia,
                    beta: ib,
                },
                dt,
            );
            let q = |x: f32| (x / 4.0 * ONE as f32) as i32;
            fx.update((q(ia), q(ib)));
        }
        let fx_th = fx.theta as f32 / UNITS_PER_RAD;
        let err = |a: f32| {
            let e = (a - theta_r).rem_euclid(core::f32::consts::PI);
            e.min(core::f32::consts::PI - e)
        };
        assert!(err(fl.theta()) < 0.05, "float {}", fl.theta());
        assert!(err(fx_th) < 0.05, "fixed {fx_th}");
    }
}
