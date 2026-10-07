//! Feed-forward of the torque a rotor meets around a turn (cogging, detent,
//! anything conservative that repeats with mechanical position).
//!
//! The torque is a short Fourier series in the mechanical angle, measured
//! off-line from steady hall runs (`tools/drag_profile.py`):
//!
//! ```text
//! τ(θ) = Σ amp_i · sin(order_i · θ + phase_i)     [N·m, + = along SEQUENCE]
//! ```
//!
//! θ is measured from a hall edge: the boundary into [`SEQUENCE`]`[0]`, on
//! one particular pole pair. Halls carry no index, so which of the `p` pole
//! pairs that is has to be found every power-up. [`CogComp`] counts hall
//! states from power-up (its own mechanical frame), and while the rotor
//! turns it measures the external torque at every state boundary the way
//! the off-line tool does — the kinetic-energy change between neighbouring
//! states minus the motor's own torque — and correlates it with the series
//! rotated by each pole pair. The best match is the shift; until then the
//! feed-forward is zero.
//!
//! Cost per control tick: one series evaluation (`TERMS` `sin_cos` calls)
//! for the drive's feed-forward and tracker (the engine alternates the two
//! angles it needs), plus one while a match is being scored (one candidate
//! at one boundary per tick, so a 7-pole-pair motor takes 294 ticks).

use crate::hall::{index_of, HallMap};
use crate::math::{sin_cos, wrap_angle, TWO_PI};

/// Series terms the drive carries.
pub const TERMS: usize = 4;
/// Largest pole-pair count the state counter handles.
pub const MAX_POLE_PAIRS: usize = 8;
const MAX_SLICES: usize = 6 * MAX_POLE_PAIRS;

/// One term of the torque series: `amp·sin(order·θ_m + phase)` [N·m].
/// `order` 0 or `amp` 0 is an unused slot.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Term {
    pub order: f32,
    pub amp: f32,
    pub phase: f32,
}

/// Torque of the series at mechanical angle `theta_m` (table frame).
pub fn series(terms: &[Term; TERMS], theta_m: f32) -> f32 {
    let mut t = 0.0;
    for term in terms {
        if term.order != 0.0 && term.amp != 0.0 {
            t += term.amp * sin_cos(term.order * theta_m + term.phase).0;
        }
    }
    t
}

/// How the identification is going.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Ident {
    /// Collecting torque samples around the turn.
    Collecting,
    /// Correlating: candidate `k`, boundary `b`.
    Scoring { k: u8, b: u8 },
    /// Done (the shift is in [`CogComp::shift`]).
    Found,
}

/// Mechanical position from counted hall states, plus the pole-pair
/// identification. Feed it every tick.
#[derive(Clone, Debug)]
pub struct CogComp {
    p: u8,
    n: u8,
    dir: f32,
    /// Electrical widths in SEQUENCE order and their cumulative starts.
    widths: [f32; 6],
    starts: [f32; 6],
    /// Firmware-frame electrical angle of the boundary into SEQUENCE[0].
    edge0: f32,
    /// State index around the turn, 0..6p, counted since power-up; `None`
    /// until the first valid state, and again after a skipped state.
    slice: Option<u8>,
    seq_last: Option<usize>,
    /// Time in the current state, and i_q summed over it.
    t_in: f32,
    iq_sum: f32,
    iq_n: u32,
    /// The state finished last: (slice, signed mechanical speed, mean i_q).
    prev: Option<(u8, f32, f32)>,
    /// Pole-pair shift: table pole pair = counted pole pair + shift.
    pub shift: Option<u8>,
    pub ident: Ident,
    tau_sum: [f32; MAX_SLICES],
    tau_n: [u16; MAX_SLICES],
    score: [f32; MAX_POLE_PAIRS],
    tab_sq: f32,
    /// Σ and Σ² of the measured boundary torques, gathered on the first
    /// candidate's pass so the decision itself is a few operations (it runs
    /// inside the control interrupt).
    meas_sum: f32,
    meas_sq: f32,
}

impl CogComp {
    /// Direction, widths, their starts and the boundary into SEQUENCE[0].
    fn geometry(map: &HallMap) -> (f32, [f32; 6], [f32; 6], f32) {
        let widths = HallMap::normalized_widths(map.widths);
        let mut starts = [0.0; 6];
        for k in 1..6 {
            starts[k] = starts[k - 1] + widths[k - 1];
        }
        let dir = if map.dir < 0.0 { -1.0 } else { 1.0 };
        (
            dir,
            widths,
            starts,
            wrap_angle(map.offset - dir * 0.5 * widths[0]),
        )
    }

    pub fn new(map: &HallMap, pole_pairs: u8) -> Self {
        let p = pole_pairs.clamp(1, MAX_POLE_PAIRS as u8);
        let (dir, widths, starts, edge0) = Self::geometry(map);
        Self {
            p,
            n: 6 * p,
            dir,
            widths,
            starts,
            edge0,
            slice: None,
            seq_last: None,
            t_in: 0.0,
            iq_sum: 0.0,
            iq_n: 0,
            prev: None,
            shift: None,
            ident: Ident::Collecting,
            tau_sum: [0.0; MAX_SLICES],
            tau_n: [0; MAX_SLICES],
            score: [0.0; MAX_POLE_PAIRS],
            tab_sq: 0.0,
            meas_sum: 0.0,
            meas_sq: 0.0,
        }
    }

    /// New hall calibration or pole-pair count. The count survives a
    /// recalibration of the same motor; a different pole-pair count starts
    /// it again.
    pub fn retune(&mut self, map: &HallMap, pole_pairs: u8) {
        if pole_pairs.clamp(1, MAX_POLE_PAIRS as u8) != self.p {
            *self = Self::new(map, pole_pairs);
            return;
        }
        (self.dir, self.widths, self.starts, self.edge0) = Self::geometry(map);
    }

    /// Forget the pole-pair match and everything collected for it (the
    /// counter lost a state, or the series changed).
    pub fn reset_ident(&mut self) {
        self.shift = None;
        self.ident = Ident::Collecting;
        self.tau_sum = [0.0; MAX_SLICES];
        self.tau_n = [0; MAX_SLICES];
        self.prev = None;
    }

    /// The counted state index, 0..6p (`None` while untracked).
    pub fn slice(&self) -> Option<u8> {
        self.slice
    }

    /// Mechanical angle of state boundary `b` in the counted frame [rad].
    fn boundary_angle(&self, b: u8) -> f32 {
        let rev = (b / 6) as f32;
        (rev * TWO_PI + self.starts[(b % 6) as usize]) / self.p as f32
    }

    /// One control tick. `state`: raw hall state; `iq`: measured i_q [A];
    /// `collect`: the drive is in a steady closed-loop hall mode where the
    /// energy balance holds (speed loop on, no big accel); `kt` [N·m/A] and
    /// `inertia` [kg·m²] turn i_q and speed into torque. `terms` is the
    /// series to identify against. Returns true when the counter lost track
    /// (the caller should forget any shift it published).
    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        state: Option<u8>,
        iq: f32,
        dt: f32,
        collect: bool,
        kt: f32,
        inertia: f32,
        terms: &[Term; TERMS],
    ) -> bool {
        let mut lost = false;
        self.t_in += dt;
        self.iq_sum += iq;
        self.iq_n += 1;
        if let Some(k) = state.and_then(index_of) {
            match (self.seq_last, self.slice) {
                (Some(last), Some(s)) if last != k => {
                    let step = (k + 6 - last) % 6;
                    let n = self.n;
                    let next = match step {
                        1 => Some((s + 1) % n),
                        5 => Some((s + n - 1) % n),
                        _ => None,
                    };
                    match next {
                        Some(ns) => {
                            self.edge(s, step == 1, collect, kt, inertia);
                            self.slice = Some(ns);
                        }
                        None => {
                            // Skipped a state: the count is no longer the
                            // rotor's. Start again from this state.
                            self.slice = Some(k as u8);
                            self.reset_ident();
                            lost = true;
                        }
                    }
                    self.t_in = 0.0;
                    self.iq_sum = 0.0;
                    self.iq_n = 0;
                }
                (None, _) | (_, None) => {
                    self.slice = Some(k as u8);
                    self.t_in = 0.0;
                    self.iq_sum = 0.0;
                    self.iq_n = 0;
                }
                _ => {}
            }
            self.seq_last = Some(k);
        }
        if !collect {
            self.prev = None;
        }
        self.advance_ident(terms);
        lost
    }

    /// The rotor left state `s` for `ns` (forward = along SEQUENCE).
    fn edge(&mut self, s: u8, forward: bool, collect: bool, kt: f32, inertia: f32) {
        let w = self.widths[(s % 6) as usize] / self.p as f32;
        let sign = if forward { 1.0 } else { -1.0 };
        let omega = sign * w / self.t_in.max(1e-6);
        // i_q in the counted frame: + torque drives along SEQUENCE only if
        // SEQUENCE is + electrical rotation.
        let iq = self.dir * self.iq_sum / self.iq_n.max(1) as f32;
        // A state only counts if the rotor crossed it (entered from the
        // neighbour on the other side): one the rotor reversed inside
        // gives no speed.
        if let (true, Some((ps, pw, piq))) = (collect, self.prev) {
            let crossed = if forward {
                (ps + 1) % self.n == s
            } else {
                (s + 1) % self.n == ps
            };
            if crossed && pw.signum() == omega.signum() && self.ident == Ident::Collecting {
                // Boundary between the two finished states, in the
                // direction of travel.
                let b = if forward { s } else { ps };
                let wp = self.widths[(ps % 6) as usize] / self.p as f32;
                let dth = sign * 0.5 * (w + wp);
                let tau = inertia * (omega * omega - pw * pw) / (2.0 * dth) - kt * 0.5 * (iq + piq);
                if tau.is_finite() {
                    self.tau_sum[b as usize] += tau;
                    self.tau_n[b as usize] = self.tau_n[b as usize].saturating_add(1);
                }
            }
        }
        self.prev = Some((s, omega, iq));
    }

    /// Score one (candidate, boundary) pair per call once every boundary
    /// has `MIN_SAMPLES`.
    fn advance_ident(&mut self, terms: &[Term; TERMS]) {
        const MIN_SAMPLES: u16 = 3;
        let n = self.n as usize;
        match self.ident {
            Ident::Found => {}
            Ident::Collecting => {
                if self.tau_n[..n].iter().all(|&c| c >= MIN_SAMPLES) {
                    self.score = [0.0; MAX_POLE_PAIRS];
                    self.tab_sq = 0.0;
                    self.meas_sum = 0.0;
                    self.meas_sq = 0.0;
                    self.ident = Ident::Scoring { k: 0, b: 0 };
                }
            }
            Ident::Scoring { k, b } => {
                let meas = self.tau_sum[b as usize] / self.tau_n[b as usize] as f32;
                let th = self.boundary_angle(b) + k as f32 * TWO_PI / self.p as f32;
                let tab = series(terms, th);
                self.score[k as usize] += meas * tab;
                if k == 0 {
                    self.tab_sq += tab * tab;
                    self.meas_sum += meas;
                    self.meas_sq += meas * meas;
                }
                let (mut k2, mut b2) = (k, b + 1);
                if b2 as usize == n {
                    b2 = 0;
                    k2 += 1;
                }
                if k2 < self.p {
                    self.ident = Ident::Scoring { k: k2, b: b2 };
                } else {
                    self.decide();
                }
            }
        }
    }

    fn decide(&mut self) {
        let n = self.n as f32;
        let p = self.p as usize;
        // Correlation against a zero-mean series: the measured side's spread
        // about its mean (friction).
        let mean = self.meas_sum / n;
        let meas_sq = (self.meas_sq - n * mean * mean).max(0.0);
        let norm = crate::math::sqrt(meas_sq * self.tab_sq).max(1e-12);
        // The series sums to ~0 over the boundaries, so the mean term drops
        // out of the score up to the uneven-width residue.
        let mut best = (0usize, f32::MIN);
        let mut second = f32::MIN;
        for k in 0..p {
            let c = self.score[k] / norm;
            if c > best.1 {
                second = best.1;
                best = (k, c);
            } else if c > second {
                second = c;
            }
        }
        if best.1 > 0.5 && best.1 - second > 0.15 {
            self.shift = Some(best.0 as u8);
            self.ident = Ident::Found;
        } else {
            // Not conclusive: collect afresh.
            self.tau_sum = [0.0; MAX_SLICES];
            self.tau_n = [0; MAX_SLICES];
            self.ident = Ident::Collecting;
        }
    }

    /// The raw correlation sums of the last scoring pass, per candidate.
    pub fn scores(&self) -> &[f32] {
        &self.score[..self.p as usize]
    }

    /// Mechanical angle in the series' frame for firmware electrical angle
    /// `theta_e`, or `None` without a counted state and a shift.
    pub fn theta_m(&self, theta_e: f32) -> Option<f32> {
        let s = self.slice?;
        let shift = self.shift?;
        let k = (s % 6) as usize;
        // Electrical angle past the boundary into SEQUENCE[0], in SEQUENCE
        // direction, kept next to the counted state (the interpolated angle
        // and the hall edge disagree by the hysteresis near a boundary).
        let x = wrap_angle(self.dir * (theta_e - self.edge0));
        let c = self.starts[k] + 0.5 * self.widths[k];
        let x = c + wrap_angle(x - c);
        let rev = ((s / 6) + shift) % self.p;
        Some((rev as f32 * TWO_PI + x) / self.p as f32)
    }

    /// Feed-forward i_q [A] cancelling the series at `theta_e` (the drive's
    /// electrical angle, already advanced for its current-loop lag).
    pub fn feedforward(&self, terms: &[Term; TERMS], theta_e: f32, kt: f32) -> f32 {
        match self.theta_m(theta_e) {
            Some(th) if kt > 0.0 => -self.dir * series(terms, th) / kt,
            _ => 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hall::SEQUENCE;
    use crate::math::PI;

    fn map() -> HallMap {
        HallMap {
            offset: 0.3,
            dir: 1.0,
            hyst: 0.0,
            widths: [63.3f32, 60.0, 56.3, 62.9, 60.7, 56.9].map(|d| d.to_radians()),
        }
    }

    fn terms() -> [Term; TERMS] {
        [
            Term {
                order: 12.0,
                amp: 8.1e-3,
                phase: -3.108,
            },
            Term {
                order: 24.0,
                amp: 4.7e-3,
                phase: 3.096,
            },
            Term {
                order: 15.0,
                amp: 2.07e-3,
                phase: -2.475,
            },
            Term {
                order: 14.0,
                amp: 1.81e-3,
                phase: 3.068,
            },
        ]
    }

    /// Hall state at counted-frame mechanical angle `th` (rad), for a rotor
    /// whose table pole pair is the counted one + `shift`.
    fn state_at(c: &CogComp, th_e: f32) -> u8 {
        let x = th_e.rem_euclid(TWO_PI);
        let mut k = 0;
        for j in 0..6 {
            if x >= c.starts[j] {
                k = j;
            }
        }
        SEQUENCE[k]
    }

    #[test]
    fn counts_states_both_ways_and_wraps() {
        let mut c = CogComp::new(&map(), 7);
        let t = terms();
        let seq: [u8; 50] = core::array::from_fn(|i| SEQUENCE[i % 6]);
        for s in &seq {
            c.tick(Some(*s), 0.0, 1e-4, false, 0.07, 4.4e-6, &t);
        }
        assert_eq!(c.slice(), Some(49 % 42));
        for s in seq.iter().rev().skip(1) {
            c.tick(Some(*s), 0.0, 1e-4, false, 0.07, 4.4e-6, &t);
        }
        assert_eq!(c.slice(), Some(0));
        // A skipped state loses the count (and says so).
        assert!(c.tick(Some(SEQUENCE[2]), 0.0, 1e-4, false, 0.07, 4.4e-6, &t));
    }

    #[test]
    fn angle_is_continuous_across_edges() {
        let m = map();
        let mut c = CogComp::new(&m, 7);
        c.shift = Some(0);
        let t = terms();
        let mut last: Option<f32> = None;
        // Rotor sweeps 2 electrical turns from just after the boundary.
        for i in 0..4000 {
            let th_rel = 0.01 + i as f32 * 4.0 * PI / 4000.0;
            c.tick(
                Some(state_at(&c, th_rel)),
                0.0,
                1e-4,
                false,
                0.07,
                4.4e-6,
                &t,
            );
            let theta_e = wrap_angle(th_rel + c.edge0);
            let th = c.theta_m(theta_e).unwrap();
            assert!(
                (th - th_rel / 7.0).abs() < 1e-3,
                "{i}: {th} vs {}",
                th_rel / 7.0
            );
            if let Some(l) = last {
                assert!((th - l).abs() < 0.01);
            }
            last = Some(th);
        }
    }

    /// A rotor spinning against the series alone (no motor torque) speeds
    /// up and slows down with position; the identification must find the
    /// pole pair it was started on, from any of them.
    #[test]
    fn finds_the_pole_pair() {
        let m = map();
        let t = terms();
        let j = 4.4e-6;
        for true_shift in 0..7u8 {
            let mut c = CogComp::new(&m, 7);
            // Counted frame angle 0 is table pole pair `true_shift`.
            let off = true_shift as f32 * TWO_PI / 7.0;
            let mut th = 0.01f32; // mechanical, counted frame
            let e0 = 0.5 * j * 20.0f32.powi(2); // 20 rad/s mech
            let dt = 1e-4;
            let mut ticks = 0;
            while c.ident != Ident::Found && ticks < 400_000 {
                // Energy conservation: ½Jω² = E0 − U(θ), U = −∫τ.
                let mut u = 0.0;
                for term in &t {
                    if term.amp != 0.0 {
                        u +=
                            term.amp / term.order * sin_cos(term.order * (th + off) + term.phase).1;
                    }
                }
                let w = crate::math::sqrt(2.0 * (e0 - u) / j);
                th += w * dt;
                let st = state_at(&c, (th * 7.0) % TWO_PI);
                c.tick(Some(st), 0.0, dt, true, 0.07, j, &t);
                ticks += 1;
            }
            assert_eq!(c.shift, Some(true_shift), "scores {:?}", c.scores());
        }
    }

    #[test]
    fn feedforward_cancels_the_series() {
        let m = map();
        let mut c = CogComp::new(&m, 7);
        let t = terms();
        c.tick(Some(SEQUENCE[0]), 0.0, 1e-4, false, 0.07, 4.4e-6, &t);
        c.shift = Some(3);
        let theta_e = wrap_angle(c.edge0 + 0.2);
        let th = c.theta_m(theta_e).unwrap();
        let ff = c.feedforward(&t, theta_e, 0.07);
        assert!((ff * 0.07 + series(&t, th)).abs() < 1e-7);
        assert!((th - (3.0 * TWO_PI + 0.2) / 7.0).abs() < 1e-5);
    }
}
