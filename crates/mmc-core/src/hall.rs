//! Three-channel 120° hall sensors: state decoding, sector, direction and
//! speed from edge timing.
//!
//! Hall sensors report the rotor's electrical position to 60° — six valid
//! states out of eight (`0b000` and `0b111` mean a missing supply or a broken
//! wire). In one direction of rotation the states follow the Gray-code cycle
//! [`SEQUENCE`]; which physical direction that is, and where electrical zero
//! sits, depend on how the sensors are mounted and wired — that is the
//! calibration ([`HallMap`]), measured on the bench, not assumed.
//!
//! Speed comes from the time between edges: each edge is 60° electrical, so a
//! single edge interval gives ω = (π/3)/Δt. The estimate is held between
//! edges and decays to zero once the rotor has been quiet longer than the
//! last interval would allow, so a stalled rotor reads 0, not its last speed.

use core::f32::consts::PI;

use crate::math::wrap_angle;

/// The six valid states in forward Gray-code order (each step flips one bit).
pub const SEQUENCE: [u8; 6] = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101];

/// Position of `state` in [`SEQUENCE`], `None` for the two invalid states.
pub fn index_of(state: u8) -> Option<usize> {
    SEQUENCE.iter().position(|&s| s == state & 0b111)
}

/// Electrical angle at the *center* of each [`SEQUENCE`] step — the hall
/// calibration. `offset` is the electrical angle of the center of step 0;
/// `dir` is +1 when [`SEQUENCE`] order is positive electrical rotation.
/// `hyst` [rad] is the sensors' switching hysteresis: each edge fires that
/// far past its magnetic midpoint *in the direction of travel*. A slow
/// forward/reverse calibration cannot see it (it lands in the averaged-out
/// lag); a speed sweep against the flux observer can (`tools/hall_ref.py`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HallMap {
    pub offset: f32,
    pub dir: f32,
    pub hyst: f32,
}

impl HallMap {
    /// Electrical angle [rad] of the center of `state`'s 60° sector.
    pub fn angle(&self, state: u8) -> Option<f32> {
        index_of(state).map(|k| wrap_angle(self.offset + self.dir * k as f32 * PI / 3.0))
    }
}

/// Hall-edge speed estimator.
#[derive(Clone, Debug)]
pub struct HallSpeed {
    last_idx: Option<usize>,
    /// Time since the last edge [s].
    since_edge: f32,
    /// Last full edge interval [s] (0 until two edges were seen).
    interval: f32,
    /// +1 / −1 in [`SEQUENCE`] order, 0 when unknown.
    dir: f32,
    /// Invalid states seen (wiring / supply diagnostic).
    pub invalid: u32,
    /// Edges that skipped a state (noise, or faster than the sample rate).
    pub skips: u32,
}

impl Default for HallSpeed {
    fn default() -> Self {
        Self::new()
    }
}

impl HallSpeed {
    pub const fn new() -> Self {
        Self {
            last_idx: None,
            since_edge: 0.0,
            interval: 0.0,
            dir: 0.0,
            invalid: 0,
            skips: 0,
        }
    }

    /// Feed one sample of the raw state; `dt` is the sample period. Returns
    /// true on an edge.
    pub fn update(&mut self, state: u8, dt: f32) -> bool {
        self.since_edge += dt;
        let Some(idx) = index_of(state) else {
            self.invalid = self.invalid.wrapping_add(1);
            return false;
        };
        let Some(last) = self.last_idx else {
            self.last_idx = Some(idx);
            self.since_edge = 0.0;
            return false;
        };
        if idx == last {
            return false;
        }
        let step = (idx + 6 - last) % 6;
        let dir = match step {
            1 => 1.0,
            5 => -1.0,
            _ => {
                // Two or three steps at once: direction is ambiguous, and the
                // interval no longer spans 60°. Resync without a speed.
                self.skips = self.skips.wrapping_add(1);
                self.last_idx = Some(idx);
                self.since_edge = 0.0;
                self.interval = 0.0;
                return true;
            }
        };
        // A reversal breaks the interval too: the rotor went back over the
        // edge it last crossed, not through a full sector.
        self.interval = if dir == self.dir {
            self.since_edge
        } else {
            0.0
        };
        self.dir = dir;
        self.last_idx = Some(idx);
        self.since_edge = 0.0;
        true
    }

    /// Electrical speed [rad/s] in [`SEQUENCE`] direction (multiply by
    /// [`HallMap::dir`] for the motor's convention). Zero until two
    /// consecutive same-direction edges, and decays as 1/t once the current
    /// sector has lasted longer than the last interval — a stopping rotor
    /// cannot be faster than "one sector in the time since the last edge".
    pub fn omega(&self) -> f32 {
        if self.interval <= 0.0 {
            return 0.0;
        }
        let t = self.interval.max(self.since_edge);
        // Treat a rotor quiet for 4× its last interval as stopped.
        if self.since_edge > 4.0 * self.interval {
            return 0.0;
        }
        self.dir * (PI / 3.0) / t
    }

    /// Current position in [`SEQUENCE`], if a valid state has been seen.
    pub fn index(&self) -> Option<usize> {
        self.last_idx
    }

    /// Direction of the last edge in [`SEQUENCE`] order: +1, −1, or 0 before
    /// the first one.
    pub fn seq_dir(&self) -> f32 {
        self.dir
    }
}

/// Rotor electrical angle from calibrated halls, good enough to run FOC on.
///
/// Between edges the only information is "somewhere in this 60° sector", so
/// the estimate is: on an edge, the edge's own angle (the boundary the rotor
/// just crossed, in its direction of travel); between edges, that plus the
/// edge-timed speed × time, clamped to the sector. With no speed estimate
/// (at rest) it is the sector centre — at worst 30° from the truth, which
/// still leaves cos 30° = 87% of the torque, so a sensored drive starts from
/// rest without the I-f ramp.
#[derive(Clone, Debug)]
pub struct HallAngle {
    pub map: HallMap,
    speed: HallSpeed,
    theta: f32,
    have: bool,
}

impl HallAngle {
    pub const fn new(map: HallMap) -> Self {
        Self {
            map,
            speed: HallSpeed::new(),
            theta: 0.0,
            have: false,
        }
    }

    /// Feed one sample; returns the angle estimate [rad], or `None` until a
    /// valid state has been seen (and on an invalid one: a sensor fault).
    pub fn update(&mut self, state: u8, dt: f32) -> Option<f32> {
        let edge = self.speed.update(state, dt);
        let center = self.map.angle(state)?;
        let half = PI / 6.0;
        if !self.have {
            self.theta = center;
            self.have = true;
        } else if edge {
            // Entered this sector across the boundary on the side we came
            // from: the trailing edge in the direction of travel, plus the
            // hysteresis the sensor switched late by, plus half a sample —
            // the edge happened somewhere in the last period, on average in
            // its middle.
            let travel = self.speed.seq_dir() * self.map.dir;
            self.theta = if travel == 0.0 {
                center
            } else {
                wrap_angle(
                    center - travel * half + travel * self.map.hyst + 0.5 * self.omega() * dt,
                )
            };
        } else if self.omega() == 0.0 {
            // No speed estimate (rest, or before two same-direction edges):
            // the sector centre, never more than 30° wrong. Holding the
            // entry-edge angle instead stalls six-step — the energised pair
            // parks the rotor on the far hall edge, 60° from the held
            // estimate, and nothing ever commutates (session 30, sim).
            self.theta = center;
        } else {
            self.theta = wrap_angle(self.theta + self.omega() * dt);
        }
        // Never leave the sector the sensors say the rotor is in (widened by
        // the hysteresis: the rotor really is that far past the boundary).
        let lim = half + self.map.hyst.abs();
        let d = wrap_angle(self.theta - center).clamp(-lim, lim);
        self.theta = wrap_angle(center + d);
        Some(self.theta)
    }

    /// Electrical speed [rad/s] in the motor's convention.
    pub fn omega(&self) -> f32 {
        self.speed.omega() * self.map.dir
    }

    /// Edge statistics (invalid states, skipped edges) for diagnostics.
    pub fn speed(&self) -> &HallSpeed {
        &self.speed
    }
}

/// Model-based rotor tracker for low speed on halls.
///
/// Between hall edges the sensors only say "somewhere in this 60° sector",
/// and an edge-timed speed is stale or zero — no good for damping a position
/// loop. This tracker predicts with the mechanical model instead — the
/// commanded torque and the rotor inertia, plus a learned load term — and
/// treats the halls as constraints:
///
/// - **on an edge** the rotor position is known exactly (the boundary, plus
///   hysteresis in the direction of travel): snap to it, and fold the drift
///   accumulated since the last edge into the speed and load estimates;
/// - **between edges** the rotor is inside the sector: if the prediction
///   leaves it, the rotor is slower than predicted — clamp to the boundary
///   and drop the outward velocity.
///
/// The position is kept unwrapped (electrical radians since the first
/// sample), for position control.
#[derive(Clone, Debug)]
pub struct HallTracker {
    map: HallMap,
    /// Electrical acceleration per amp of i_q [rad/s² per A]:
    /// `1.5·p²·ψ / J`.
    accel_per_amp: f32,
    theta: f32,
    omega: f32,
    /// Learned load acceleration [rad/s², electrical] (friction, cogging
    /// average, external load).
    load: f32,
    last_idx: Option<usize>,
    since_edge: f32,
    /// Direction of the last edge (+1/−1 electrical, 0 before one).
    last_travel: f32,
}

impl HallTracker {
    /// Load-estimate gain per edge (fraction of the implied acceleration).
    const LOAD_GAIN: f32 = 0.2;
    /// Shortest edge interval used to infer a speed correction [s] (guards
    /// against a sensor bounce reading as a huge speed).
    const MIN_INTERVAL: f32 = 10e-3;

    pub fn new(map: HallMap, accel_per_amp: f32) -> Self {
        Self {
            map,
            accel_per_amp,
            theta: 0.0,
            omega: 0.0,
            load: 0.0,
            last_idx: None,
            since_edge: 0.0,
            last_travel: 0.0,
        }
    }

    /// Learned load acceleration [rad/s² electrical] (diagnostic).
    pub fn load(&self) -> f32 {
        self.load
    }

    /// Feed the raw hall state and the q-axis current applied over the last
    /// period. Returns (unwrapped electrical position, electrical speed), or
    /// `None` on an invalid state (sensor fault).
    pub fn update(&mut self, state: u8, iq: f32, dt: f32) -> Option<(f32, f32)> {
        let idx = index_of(state)?;
        let center = self.map.angle(state)?;
        let half = PI / 6.0;
        let Some(last) = self.last_idx else {
            self.theta = center;
            self.last_idx = Some(idx);
            return Some((self.theta, 0.0));
        };
        // Predict.
        self.theta += self.omega * dt;
        self.omega += (self.accel_per_amp * iq + self.load) * dt;
        self.since_edge += dt;
        // The sector centre nearest the (unwrapped) estimate.
        let c = self.theta + wrap_angle(center - self.theta);
        if idx != last {
            let step = (idx + 6 - last) % 6;
            let seq = match step {
                1 => 1.0,
                5 => -1.0,
                _ => 0.0,
            };
            let travel = seq * self.map.dir;
            if travel == 0.0 {
                // Skipped a state: no direction, no exact edge.
                self.theta = c;
                self.omega = 0.0;
            } else {
                let edge = c - travel * half + travel * self.map.hyst;
                let e = edge - self.theta;
                self.theta = edge;
                let t = self.since_edge.max(Self::MIN_INTERVAL);
                if self.last_travel != 0.0 && travel != self.last_travel {
                    // Back across the edge it just crossed: the rotor passed
                    // through zero speed, and the time since that edge says
                    // nothing about its speed. A rotor resting on a boundary
                    // chatters like this (motor 3: H3 flipping every few ms),
                    // and reading each flip as e/t put 70-170 rad/s kicks on
                    // the position loop's D term.
                    self.omega = 0.0;
                    self.last_travel = travel;
                    self.since_edge = 0.0;
                    self.last_idx = Some(idx);
                    return Some((self.theta, self.omega));
                }
                // The drift since the last edge says the speed was off by
                // about e/t, and the acceleration by about e/t².
                self.omega += e / t;
                if travel == self.last_travel {
                    // Two edges the same way: one sector in t is a direct
                    // speed measurement; average it with the model.
                    self.omega = 0.5 * self.omega + 0.5 * travel * (PI / 3.0) / t;
                }
                let lim = self.accel_per_amp;
                self.load = (self.load + Self::LOAD_GAIN * e / (t * t)).clamp(-lim, lim);
                self.last_travel = travel;
            }
            self.since_edge = 0.0;
            self.last_idx = Some(idx);
        } else {
            let lim = half + self.map.hyst.abs();
            let bound = if self.theta > c + lim {
                Some(c + lim)
            } else if self.theta < c - lim {
                Some(c - lim)
            } else {
                None
            };
            if let Some(b) = bound {
                // Out of the sector without an edge: the model ran ahead.
                // No edge for `since_edge` caps the speed at one sector in
                // that time; whatever the model held above that cap it
                // over-predicted, so learn it into the load term.
                self.theta = b;
                let t = self.since_edge.max(Self::MIN_INTERVAL);
                let cap = (PI / 3.0) / t;
                let capped = self.omega.clamp(-cap, cap);
                let excess = self.omega - capped;
                self.omega = capped;
                let lim_a = self.accel_per_amp;
                self.load = (self.load - Self::LOAD_GAIN * excess / t).clamp(-lim_a, lim_a);
            }
        }
        Some((self.theta, self.omega))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ideal halls for an electrical angle, in the convention where
    /// SEQUENCE order is positive rotation with step 0 centered on 0.
    fn ideal(theta: f32) -> u8 {
        let k = ((wrap_angle(theta) + PI / 6.0).rem_euclid(2.0 * PI) / (PI / 3.0)) as usize % 6;
        SEQUENCE[k]
    }

    #[test]
    fn sequence_is_gray_and_complete() {
        for k in 0..6 {
            let a = SEQUENCE[k];
            let b = SEQUENCE[(k + 1) % 6];
            assert_eq!((a ^ b).count_ones(), 1, "{a:03b} -> {b:03b}");
            assert!(a != 0 && a != 7);
        }
        assert_eq!(index_of(0), None);
        assert_eq!(index_of(7), None);
    }

    #[test]
    fn speed_and_direction_from_edges() {
        for &w in &[100.0f32, -250.0] {
            let mut h = HallSpeed::new();
            let dt = 1e-4;
            let mut theta = 0.3;
            for _ in 0..5000 {
                theta += w * dt;
                h.update(ideal(theta), dt);
            }
            let est = h.omega();
            assert!((est - w).abs() < 0.05 * w.abs(), "w {w} est {est}");
            assert_eq!(h.skips, 0);
        }
    }

    #[test]
    fn stopping_rotor_reads_zero() {
        let mut h = HallSpeed::new();
        let dt = 1e-4;
        let mut theta = 0.0;
        for _ in 0..2000 {
            theta += 200.0 * dt;
            h.update(ideal(theta), dt);
        }
        assert!(h.omega() > 150.0);
        for _ in 0..2000 {
            h.update(ideal(theta), dt);
        }
        assert_eq!(h.omega(), 0.0);
    }

    #[test]
    fn angle_tracks_a_spinning_rotor_within_a_few_degrees() {
        // A map with a non-trivial offset and reversed wiring: the true
        // halls are `ideal(dir*(theta - offset))`.
        let map = HallMap {
            offset: 1.0,
            dir: -1.0,
            hyst: 0.0,
        };
        for &w in &[300.0f32, -300.0] {
            let mut est = HallAngle::new(map);
            let dt = 1e-4;
            let mut theta = 0.2f32;
            let mut worst = 0.0f32;
            for k in 0..20_000 {
                theta = wrap_angle(theta + w * dt);
                let a = est
                    .update(ideal(map.dir * (theta - map.offset)), dt)
                    .unwrap();
                if k > 2000 {
                    worst = worst.max(wrap_angle(a - theta).abs());
                }
            }
            assert!(worst < 0.1, "w {w}: worst error {worst} rad");
            assert!((est.omega() - w).abs() < 0.05 * w.abs());
        }
    }

    #[test]
    fn angle_at_standstill_is_inside_the_sector() {
        let map = HallMap {
            offset: 0.0,
            dir: 1.0,
            hyst: 0.0,
        };
        let mut est = HallAngle::new(map);
        for &theta in &[0.0f32, 1.0, 2.5, -2.0] {
            for _ in 0..100 {
                est.update(ideal(theta), 1e-4);
            }
            let a = est.update(ideal(theta), 1e-4).unwrap();
            assert!(wrap_angle(a - theta).abs() <= PI / 3.0 + 1e-4);
        }
        assert_eq!(est.update(0b111, 1e-4), None, "invalid state is a fault");
    }

    #[test]
    fn hysteresis_correction_cancels_late_switching() {
        // Sensors that switch `h` late in the direction of travel: without
        // the correction the estimate lags by ~h both ways; with it, not.
        let h = 0.08f32;
        let late = |theta: f32, dir: f32| ideal(theta - dir * h);
        for &w in &[300.0f32, -300.0] {
            for (hyst, expect_lag) in [(0.0f32, true), (h, false)] {
                let map = HallMap {
                    offset: 0.0,
                    dir: 1.0,
                    hyst,
                };
                let mut est = HallAngle::new(map);
                let dt = 1e-4;
                let mut theta = 0.0f32;
                let mut sum = 0.0f32;
                let mut n = 0;
                for k in 0..20_000 {
                    theta = wrap_angle(theta + w * dt);
                    let a = est.update(late(theta, w.signum()), dt).unwrap();
                    if k > 2000 {
                        sum += wrap_angle(a - theta) * w.signum();
                        n += 1;
                    }
                }
                let lag = -sum / n as f32;
                if expect_lag {
                    assert!((lag - h).abs() < 0.03, "w {w}: uncorrected lag {lag}");
                } else {
                    assert!(lag.abs() < 0.03, "w {w}: corrected lag {lag}");
                }
            }
        }
    }

    #[test]
    fn tracker_follows_a_slow_rotor_between_edges() {
        // A rotor creeping at 5 rad/s el (an edge every 0.2 s), driven by a
        // known current against a known load: the tracker must hold the
        // speed between edges, where an edge-timed estimate has nothing.
        let map = HallMap {
            offset: 0.0,
            dir: 1.0,
            hyst: 0.0,
        };
        let g = 1.0e5;
        let mut tr = HallTracker::new(map, g);
        let dt = 1e-4;
        let (mut theta, mut omega) = (0.0f32, 5.0f32);
        let load = -2000.0; // rad/s² el, friction-like
        let iq = -load / g; // just balances it
        let mut worst_w = 0.0f32;
        let mut worst_th = 0.0f32;
        for k in 0..60_000 {
            omega += (g * iq + load) * dt;
            theta += omega * dt;
            let (th, w) = tr.update(ideal(theta), iq, dt).unwrap();
            // After the load estimate has converged (a few seconds at an edge
            // every 0.2 s).
            if k > 40_000 {
                worst_w = worst_w.max((w - omega).abs());
                worst_th = worst_th.max(wrap_angle(th - theta).abs());
            }
        }
        assert!(worst_w < 1.0, "speed error {worst_w} rad/s el");
        assert!(worst_th < 0.1, "angle error {worst_th} rad el");
    }

    #[test]
    fn map_recovers_sector_centers() {
        let map = HallMap {
            offset: 0.0,
            dir: 1.0,
            hyst: 0.0,
        };
        for k in 0..6 {
            let center = k as f32 * PI / 3.0;
            let a = map.angle(ideal(center)).unwrap();
            assert!(wrap_angle(a - center).abs() < 1e-5);
        }
        assert_eq!(map.angle(0), None);
    }
}
