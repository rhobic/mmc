//! Six-step (trapezoidal) commutation and back-EMF zero-cross sensing.
//!
//! The second control methodology alongside [`crate::foc`]. Where FOC
//! continuously orients a current vector, six-step energises exactly two
//! phases at a time and leaves the third open. That idle phase is the whole
//! point: with no current in it, its terminal carries the rotor's back-EMF, so
//! the machine tells you where it is without an observer, a model, or any
//! trigonometry. The cost is torque ripple and a startup that has no
//! measurement to work from.
//!
//! Theory and derivations: `docs/SIXSTEP.md`.
//!
//! # The sensing identity
//!
//! With phases `hi` and `lo` conducting and `f` open, writing each phase as
//! `v_x = v_n + R·i_x + L·di_x/dt + e_x` and summing the two conducting ones
//! (whose currents are equal and opposite) eliminates every resistive and
//! inductive term, leaving the star-point voltage and hence:
//!
//! ```text
//! v_f = (v_hi + v_lo)/2 − (e_hi + e_lo)/2 + e_f
//! ```
//!
//! exactly, for any back-EMF shape. It collapses two ways: on a sinusoidal
//! machine the three back-EMFs sum to zero, giving `mid + 1.5·e_f`; on a
//! trapezoidal one the conducting pair sits on opposite flat tops through the
//! whole window, giving `mid + e_f` with the machine's third harmonic
//! cancelled rather than assumed away.
//!
//! So the idle terminal is an *affine* image of its own back-EMF, and the
//! reference it swings about is set entirely by what the bridge is doing:
//!
//! - sampled while the high side is on (`v_hi = V_bus`, `v_lo = 0`), the idle
//!   terminal crosses **`V_bus/2`** exactly when `e_f` crosses zero;
//! - sampled while both driven legs are low (the freewheel, which is where a
//!   low-side shunt must sample current), the reference is **0**, so half the
//!   waveform sits below ground and any clamp on the sense network eats it.
//!
//! That second case is not hypothetical — it is what the bench measured
//! before this module existed (see `docs/PROGRESS.md`, MS8 step 1).
//!
//! # Why 30 degrees
//!
//! Commutation should happen when the rotor reaches the boundary of the
//! present sector. The idle phase's back-EMF crosses zero at the *centre* of
//! its 60° window, so every zero-cross is exactly 30° electrical ahead of the
//! commutation it schedules. Since zero-crosses recur every 60°, the time for
//! 30° is half the last zero-cross interval — no angle estimate, no speed
//! loop, just a stopwatch.

use crate::math::wrap_angle;

/// Number of commutation sectors per electrical revolution.
pub const SECTORS: usize = 6;

/// Electrical radians spanned by one sector (60°).
pub const SECTOR_RAD: f32 = core::f32::consts::TAU / SECTORS as f32;

/// Rotor angle of sector 0's centre, in electrical radians.
///
/// Sector 0 drives U high and V low. That pair's torque per amp is
/// `1.5·p·ψ·(s_u − s_v)`, which for the sinusoidal shape `s_x = −sin(θ − x·120°)`
/// peaks at `θ = −120°`; the sector therefore spans ±30° about that angle.
/// Getting this constant wrong costs torque as `cos(error)` and, worse, moves
/// the zero-cross off the window centre where the 30° rule assumes it.
pub const SECTOR0_CENTRE: f32 = -2.0 * core::f32::consts::PI / 3.0;

/// What the bridge does with one phase during a sector.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PhaseDrive {
    /// Switching against the bus: this leg carries the PWM.
    High,
    /// Held at the negative rail.
    Low,
    /// Both switches open — the back-EMF sense node.
    Float,
}

/// Commutation table: `(high, low, floating)` phase indices per sector, with
/// U = 0, V = 1, W = 2. Advancing the index advances the rotor.
///
/// Firmware and simulator both index this table, so a change to the
/// commutation order cannot desynchronise them.
pub const TABLE: [(u8, u8, u8); SECTORS] = [
    (0, 1, 2), // U+ V−, W idle
    (0, 2, 1), // U+ W−, V idle
    (1, 2, 0), // V+ W−, U idle
    (1, 0, 2), // V+ U−, W idle
    (2, 0, 1), // W+ U−, V idle
    (2, 1, 0), // W+ V−, U idle
];

/// The sector whose energised pair produces the most torque at `theta_e`.
pub fn sector_of(theta_e: f32) -> usize {
    // Shift so sector 0 spans [centre − 30°, centre + 30°), then floor-divide.
    let x = wrap_angle(theta_e - SECTOR0_CENTRE + SECTOR_RAD * 0.5);
    let k = (x + core::f32::consts::TAU) / SECTOR_RAD;
    (k as usize) % SECTORS
}

/// Per-phase bridge states for a sector.
pub fn drive_states(sector: usize) -> [PhaseDrive; 3] {
    let (hi, lo, fl) = TABLE[sector % SECTORS];
    let mut out = [PhaseDrive::Float; 3];
    out[hi as usize] = PhaseDrive::High;
    out[lo as usize] = PhaseDrive::Low;
    out[fl as usize] = PhaseDrive::Float;
    out
}

/// Index of the phase left floating in `sector`.
pub fn floating_phase(sector: usize) -> usize {
    TABLE[sector % SECTORS].2 as usize
}

/// Duties for a sector at `duty` on the high leg. The floating phase's entry
/// is zero but meaningless — the caller must also open that leg's enable, or
/// the phase is driven low instead of floating.
pub fn duties(sector: usize, duty: f32) -> [f32; 3] {
    let (hi, _, _) = TABLE[sector % SECTORS];
    let mut d = [0.0; 3];
    d[hi as usize] = duty;
    d
}

/// Does the idle phase's back-EMF rise or fall through its window?
///
/// It alternates with sector parity: each commutation swaps which of the two
/// remaining phases opens, and consecutive idle phases sit on opposite slopes
/// of the back-EMF waveform. The detector needs this to reject the wrong
/// crossing — noise and flyback can produce either direction.
pub fn rising_edge(sector: usize) -> bool {
    sector % 2 == 1
}

/// Zero-cross detector configuration.
#[derive(Copy, Clone, Debug)]
pub struct ZcCfg {
    /// Ignore the idle terminal for this long after each commutation [s].
    ///
    /// When a phase opens, its trapped current keeps flowing through a
    /// freewheel diode until it decays; until then the terminal is clamped to
    /// a rail and says nothing about the rotor. This blanking is the single
    /// most important number in a real six-step drive: too short and flyback
    /// is mistaken for a crossing (commutation runs away), too long and the
    /// crossing itself is masked at high speed, where the window is short.
    pub blank: f32,
    /// Consecutive plausible crossings required before reporting lock.
    pub lock_count: u32,
    /// Reject an interval differing from the running estimate by more than
    /// this factor — a cheap guard against a missed or doubled detection.
    pub interval_tol: f32,
    /// Commutate anyway after this multiple of the expected interval, so a
    /// missed crossing coasts instead of stalling the commutator.
    pub timeout_factor: f32,
}

impl Default for ZcCfg {
    fn default() -> Self {
        Self {
            blank: 200e-6,
            lock_count: 6,
            interval_tol: 2.5,
            timeout_factor: 3.0,
        }
    }
}

/// What the detector wants the commutator to do this tick.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ZcEvent {
    /// Nothing to do.
    None,
    /// A crossing was accepted; commutation is now scheduled.
    Detected,
    /// Advance to the next sector now.
    Commutate,
}

/// Back-EMF zero-cross detector and commutation timer.
///
/// Feeds on one number per control tick — the idle terminal voltage — plus the
/// reference it should be compared against. It owns no angle: sector timing
/// comes from measured crossings alone.
pub struct ZeroCross {
    cfg: ZcCfg,
    /// Seconds since the last commutation.
    since_comm: f32,
    /// Seconds between the last two accepted crossings (60° electrical).
    interval: f32,
    /// Time from the accepted crossing to the commutation it schedules.
    pending: Option<f32>,
    last_sign: bool,
    good: u32,
    locked: bool,
}

impl ZeroCross {
    pub fn new(cfg: ZcCfg) -> Self {
        Self {
            cfg,
            since_comm: 0.0,
            interval: 0.0,
            pending: None,
            last_sign: false,
            good: 0,
            locked: false,
        }
    }

    /// True once enough consecutive crossings agree that commutation can be
    /// handed over from the forced ramp.
    pub fn locked(&self) -> bool {
        self.locked
    }

    /// Electrical speed implied by the crossing interval [rad/s].
    ///
    /// One interval is exactly 60°, so `ω = (π/3)/T`. This is a *measurement*,
    /// not an estimate from a model — its noise is the timing jitter of the
    /// crossings and nothing else.
    pub fn omega_e(&self) -> f32 {
        if self.interval > 1e-6 {
            SECTOR_RAD / self.interval
        } else {
            0.0
        }
    }

    /// Seed the timer from the forced ramp so the handoff is not a cold start.
    pub fn seed(&mut self, omega_e: f32) {
        if omega_e.abs() > 1e-3 {
            self.interval = SECTOR_RAD / omega_e.abs();
        }
    }

    /// Call once per control period.
    ///
    /// `v_float` is the idle terminal voltage and `v_ref` the reference it
    /// swings about — `V_bus/2` when sampled during the high-side on-time.
    /// Returns [`ZcEvent::Commutate`] on the tick the caller should advance.
    pub fn update(&mut self, sector: usize, v_float: f32, v_ref: f32, dt: f32) -> ZcEvent {
        self.since_comm += dt;

        // A commutation already scheduled by an accepted crossing wins: the
        // 30° timer is the control law, and re-detecting inside it would let
        // noise retrigger the same edge.
        if let Some(remaining) = self.pending.as_mut() {
            *remaining -= dt;
            if *remaining <= 0.0 {
                self.pending = None;
                self.commutated();
                return ZcEvent::Commutate;
            }
            return ZcEvent::None;
        }

        // Blanking: the idle terminal is still unwinding the freewheel.
        if self.since_comm < self.cfg.blank {
            self.last_sign = v_float >= v_ref;
            return ZcEvent::None;
        }

        // Fallback so a missed crossing does not stall the commutator.
        if self.interval > 1e-6 && self.since_comm > self.interval * self.cfg.timeout_factor {
            self.good = 0;
            self.locked = false;
            self.commutated();
            return ZcEvent::Commutate;
        }

        let sign = v_float >= v_ref;
        let crossed = sign != self.last_sign && sign == rising_edge(sector);
        self.last_sign = sign;
        if !crossed {
            return ZcEvent::None;
        }

        // Interval between crossings is one sector; the previous crossing sat
        // 30° before the last commutation, so add that half-sector back.
        let measured = self.since_comm + self.interval * 0.5;
        let plausible = self.interval <= 1e-6
            || (measured < self.interval * self.cfg.interval_tol
                && measured > self.interval / self.cfg.interval_tol);
        if !plausible {
            self.good = 0;
            self.locked = false;
            return ZcEvent::None;
        }

        self.interval = if self.interval <= 1e-6 {
            measured
        } else {
            // Light smoothing: enough to ride out one noisy crossing, not so
            // much that the commutator lags a genuine acceleration.
            0.5 * self.interval + 0.5 * measured
        };
        self.good = self.good.saturating_add(1);
        if self.good >= self.cfg.lock_count {
            self.locked = true;
        }
        // Commutate 30° after the crossing = half a sector interval.
        self.pending = Some(self.interval * 0.5);
        ZcEvent::Detected
    }

    /// Tell the detector a commutation happened (forced ramp or fallback), so
    /// blanking restarts.
    pub fn commutated(&mut self) {
        self.since_comm = 0.0;
    }
}

/// Forced-commutation startup: step the sector on a rising frequency ramp
/// until the rotor is turning fast enough to be sensed.
///
/// A stationary rotor produces no back-EMF, so nothing can be measured and the
/// commutator has to guess. It guesses by imposing a rotating sequence and
/// accelerating it slowly enough that the rotor is dragged along — the same
/// bargain the I-f startup in [`crate::sensorless`] makes, and it fails the
/// same way, by pulling out under load.
#[derive(Copy, Clone, Debug)]
pub struct RampCfg {
    /// Electrical speed the ramp starts from [rad/s].
    pub omega_start: f32,
    /// Electrical speed at which sensing takes over [rad/s].
    pub omega_handoff: f32,
    /// Ramp acceleration [rad/s² electrical].
    pub accel: f32,
    /// High-side duty during the ramp.
    pub duty: f32,
}

impl Default for RampCfg {
    fn default() -> Self {
        Self {
            omega_start: 20.0,
            omega_handoff: 200.0,
            accel: 400.0,
            duty: 0.15,
        }
    }
}

/// Startup ramp state.
pub struct Ramp {
    cfg: RampCfg,
    omega: f32,
    /// Electrical angle accumulated by the forced sequence.
    theta: f32,
}

impl Ramp {
    pub fn new(cfg: RampCfg) -> Self {
        Self {
            cfg,
            omega: cfg.omega_start,
            theta: 0.0,
        }
    }

    pub fn omega_e(&self) -> f32 {
        self.omega
    }

    /// True once the ramp has reached the handoff speed.
    pub fn done(&self) -> bool {
        self.omega >= self.cfg.omega_handoff
    }

    /// Advance the ramp; returns the sector it now demands.
    pub fn update(&mut self, dt: f32) -> usize {
        self.omega = (self.omega + self.cfg.accel * dt).min(self.cfg.omega_handoff);
        self.theta = wrap_angle(self.theta + self.omega * dt);
        sector_of(self.theta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::sin_cos;

    /// Normalised sinusoidal back-EMF shape for phase `k`, matching the sign
    /// convention the dq model uses (verified by `torque_matches_dq_convention`).
    fn shape(k: usize, theta_e: f32) -> f32 {
        let off = k as f32 * core::f32::consts::TAU / 3.0;
        -sin_cos(theta_e - off).0
    }

    /// The table must energise the pair with the largest back-EMF difference
    /// at the centre of every sector — that is what "six-step" means.
    #[test]
    fn table_picks_max_torque_pair() {
        for (k, &(hi, lo, _)) in TABLE.iter().enumerate() {
            let centre = SECTOR0_CENTRE + k as f32 * SECTOR_RAD;
            let chosen = shape(hi as usize, centre) - shape(lo as usize, centre);
            for a in 0..3u8 {
                for b in 0..3u8 {
                    if a == b {
                        continue;
                    }
                    let other = shape(a as usize, centre) - shape(b as usize, centre);
                    assert!(
                        chosen >= other - 1e-4,
                        "sector {k}: ({hi},{lo}) gives {chosen}, ({a},{b}) gives {other}"
                    );
                }
            }
        }
    }

    /// `sector_of` must return the sector whose centre the angle is nearest.
    #[test]
    fn sector_of_is_consistent_with_centres() {
        for k in 0..SECTORS {
            let centre = SECTOR0_CENTRE + k as f32 * SECTOR_RAD;
            assert_eq!(sector_of(centre), k, "centre of {k}");
            // Just inside either edge.
            assert_eq!(sector_of(centre - SECTOR_RAD * 0.49), k, "low edge of {k}");
            assert_eq!(sector_of(centre + SECTOR_RAD * 0.49), k, "high edge of {k}");
        }
    }

    /// Advancing the rotor must advance the sector by one, all the way round.
    #[test]
    fn sectors_advance_monotonically_with_rotation() {
        let mut prev = sector_of(0.0);
        let mut seen = 0;
        let steps = 600;
        for i in 1..=steps {
            let s = sector_of(i as f32 * core::f32::consts::TAU / steps as f32);
            if s != prev {
                assert_eq!(s, (prev + 1) % SECTORS, "step {i}: {prev} -> {s}");
                prev = s;
                seen += 1;
            }
        }
        assert_eq!(
            seen, SECTORS,
            "one full revolution must commutate six times"
        );
    }

    /// The idle phase's back-EMF must cross zero at the centre of its window —
    /// the assumption the whole 30° rule rests on.
    #[test]
    fn idle_phase_crosses_zero_at_window_centre() {
        for k in 0..SECTORS {
            let centre = SECTOR0_CENTRE + k as f32 * SECTOR_RAD;
            let f = floating_phase(k);
            assert!(
                shape(f, centre).abs() < 1e-3,
                "sector {k}: idle phase {f} reads {} at centre",
                shape(f, centre)
            );
            // And it must be on the slope the detector expects.
            let ahead = shape(f, centre + 0.05);
            assert_eq!(
                ahead > 0.0,
                rising_edge(k),
                "sector {k}: slope disagrees with rising_edge"
            );
        }
    }

    /// Exactly one phase floats, one is high and one is low, in every sector.
    #[test]
    fn drive_states_are_well_formed() {
        for k in 0..SECTORS {
            let s = drive_states(k);
            assert_eq!(s.iter().filter(|d| **d == PhaseDrive::High).count(), 1);
            assert_eq!(s.iter().filter(|d| **d == PhaseDrive::Low).count(), 1);
            assert_eq!(s.iter().filter(|d| **d == PhaseDrive::Float).count(), 1);
        }
    }

    /// A clean synthetic crossing must be accepted and scheduled 30° later,
    /// and the recovered speed must match the one that generated it.
    #[test]
    fn detector_times_commutation_thirty_degrees_after_crossing() {
        let dt = 50e-6;
        let omega = 300.0; // rad/s electrical
        let mut zc = ZeroCross::new(ZcCfg {
            blank: 100e-6,
            ..Default::default()
        });
        zc.seed(omega);
        let mut sector = 1usize; // rising idle phase
        let mut theta = SECTOR0_CENTRE + sector as f32 * SECTOR_RAD - SECTOR_RAD * 0.5;
        let mut commutations = 0;
        let mut t_since_cross = None;
        let mut delay_seen = 0.0;
        for _ in 0..4000 {
            theta = wrap_angle(theta + omega * dt);
            let v = 12.0 + 1.5 * shape(floating_phase(sector), theta) * omega * 0.01;
            match zc.update(sector, v, 12.0, dt) {
                ZcEvent::Detected => t_since_cross = Some(0.0),
                ZcEvent::Commutate => {
                    if let Some(t) = t_since_cross.take() {
                        delay_seen = t;
                        commutations += 1;
                    }
                    sector = (sector + 1) % SECTORS;
                }
                ZcEvent::None => {
                    if let Some(t) = t_since_cross.as_mut() {
                        *t += dt;
                    }
                }
            }
        }
        assert!(commutations > 20, "only {commutations} commutations");
        assert!(zc.locked(), "detector never locked");
        let expect = SECTOR_RAD * 0.5 / omega;
        assert!(
            (delay_seen - expect).abs() < expect * 0.25,
            "delay {delay_seen} vs expected {expect}"
        );
        assert!(
            (zc.omega_e() - omega).abs() < omega * 0.1,
            "omega {} vs {omega}",
            zc.omega_e()
        );
    }

    /// Flyback right after a commutation must not be mistaken for a crossing.
    #[test]
    fn blanking_rejects_post_commutation_transient() {
        let dt = 50e-6;
        let mut zc = ZeroCross::new(ZcCfg {
            blank: 500e-6,
            ..Default::default()
        });
        zc.commutated();
        // A hard rail-to-rail transient inside the blanking window.
        for i in 0..8 {
            let v = if i % 2 == 0 { 24.0 } else { 0.0 };
            assert_eq!(zc.update(1, v, 12.0, dt), ZcEvent::None, "tick {i}");
        }
        assert!(!zc.locked());
    }

    /// The ramp must sweep every sector in order and reach its handoff speed.
    #[test]
    fn ramp_reaches_handoff_and_sweeps_all_sectors() {
        let cfg = RampCfg {
            omega_start: 10.0,
            omega_handoff: 200.0,
            accel: 500.0,
            duty: 0.1,
        };
        let mut r = Ramp::new(cfg);
        let mut seen = [false; SECTORS];
        for _ in 0..40_000 {
            seen[r.update(50e-6)] = true;
        }
        assert!(r.done(), "ramp stalled at {}", r.omega_e());
        assert!(seen.iter().all(|s| *s), "sectors visited: {seen:?}");
    }
}
