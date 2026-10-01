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
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HallMap {
    pub offset: f32,
    pub dir: f32,
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
    fn map_recovers_sector_centers() {
        let map = HallMap {
            offset: 0.0,
            dir: 1.0,
        };
        for k in 0..6 {
            let center = k as f32 * PI / 3.0;
            let a = map.angle(ideal(center)).unwrap();
            assert!(wrap_angle(a - center).abs() < 1e-5);
        }
        assert_eq!(map.angle(0), None);
    }
}
