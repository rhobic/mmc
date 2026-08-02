//! The six-step stack running against the phase-domain motor: forced
//! commutation ramp, back-EMF zero-cross sensing, and a duty (voltage) speed
//! loop closed on the interval between crossings.
//!
//! This is to [`mmc_core::sixstep`] what [`crate::sensorless_rig`] is to the
//! FOC stack — one `step()` per control period, returning the controller's
//! view beside the simulator's ground truth, so a regression test can assert
//! on the gap between them.

use mmc_core::pi::{Pi, PiGains};
use mmc_core::sixstep::{self, Ramp, RampCfg, ZcCfg, ZcEvent, ZeroCross};

use crate::motor::PmsmParams;
use crate::phase_motor::{BemfShape, Bridge, PhaseMotor, SamplePoint};

/// What the detector compares the idle terminal against.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RefSource {
    /// Half the bus voltage. Correct only for an ideal bridge: on a real one
    /// the conducting switches and the shunt move both driven terminals, and
    /// the error is a fixed voltage that does not shrink with the signal.
    VbusHalf,
    /// The measured average of the two driven terminals. The drops appear in
    /// both and cancel.
    MeasuredMid,
}

/// Which commutation source is in charge.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Forced ramp: the rotor is being dragged, nothing is being measured.
    Ramp,
    /// Commutating on measured zero-crossings.
    Sensing,
    /// Sensing was lost after having locked — the drive is freewheeling on the
    /// timeout fallback and no longer trustworthy.
    Lost,
}

#[derive(Copy, Clone, Debug)]
pub struct SixStepCfg {
    pub params: PmsmParams,
    pub shape: BemfShape,
    pub vbus: f32,
    pub ctrl_freq: f32,
    /// Where in the PWM period the idle terminal is sampled.
    pub sample_at: SamplePoint,
    /// Model a sense network that cannot read below ground (the clamped
    /// divider on the bench shield). Set with [`SamplePoint::Freewheel`] to
    /// reproduce the MS8 step 1 hardware result.
    pub clamp_negative: bool,
    /// Conducting-switch drops and sense clipping.
    pub bridge: Bridge,
    /// Which reference the detector compares against.
    pub ref_source: RefSource,
    pub ramp: RampCfg,
    pub zc: ZcCfg,
    /// Speed target [rad/s electrical]; the duty loop drives towards it once
    /// sensing has locked.
    pub omega_ref: f32,
    /// Duty-loop gains, output in duty per rad/s electrical.
    ///
    /// The plant from duty to electrical acceleration is
    /// `p·(2ψ)·V_bus/(2R·J)`, about 5.8e4 rad/s² per unit duty on the bench
    /// machine, so `kp` sets a crossover near `kp·5.8e4`. The integral corner
    /// `ki/kp` must sit well *below* that crossover — putting it above is what
    /// makes the loop hunt.
    pub speed_gains: PiGains,
    /// Maximum high-side duty.
    pub duty_max: f32,
    /// Minimum high-side duty. A six-step bridge with no braking quadrant
    /// cannot command negative duty, so the loop saturates here on
    /// deceleration and needs back-calculation to avoid winding up.
    pub duty_min: f32,
}

impl SixStepCfg {
    /// A trapezoidal machine of the bench motor's electrical size, on a 24 V
    /// bus — the configuration the regression tests use.
    pub fn bench(omega_ref: f32) -> Self {
        Self {
            params: PmsmParams {
                // Give the model a rotor big enough that commutation, not
                // inertia, sets the timescale.
                inertia: 2.0e-5,
                viscous: 5.0e-6,
                ..PmsmParams::bench_g474()
            },
            shape: BemfShape::Trapezoidal,
            // 12 V, not 24: with a fixed divider ratio the driven terminals
            // must stay inside the sense range, or the measured mid-point
            // cannot be read and the reference has to fall back to V_bus/2.
            // The bench hit exactly this and dropped its supply for the same
            // reason.
            vbus: 12.0,
            ctrl_freq: 20_000.0,
            sample_at: SamplePoint::OnTime,
            clamp_negative: false,
            bridge: Bridge::bench(),
            ref_source: RefSource::MeasuredMid,
            ramp: RampCfg {
                omega_start: 30.0,
                omega_handoff: 400.0,
                accel: 1500.0,
                duty: 0.25,
            },
            zc: ZcCfg::default(),
            omega_ref,
            // Crossover ~23 rad/s, integral corner ~2.5 rad/s: a factor of
            // nine apart, which is the usual margin for a loop whose plant is
            // a pure integrator with lag.
            speed_gains: PiGains {
                kp: 4.0e-4,
                ki: 1.0e-3,
            },
            duty_max: 0.9,
            duty_min: 0.02,
        }
    }
}

/// One control period, controller view plus ground truth.
#[derive(Copy, Clone, Debug)]
pub struct SixStepSample {
    pub t: f32,
    pub mode: Mode,
    pub sector: usize,
    /// Sector the rotor's true angle says should be energised.
    pub sector_true: usize,
    pub duty: f32,
    pub i_line: f32,
    pub v_float: f32,
    /// Reference the detector compares `v_float` against.
    pub v_ref: f32,
    /// Speed measured from the crossing interval [rad/s electrical].
    pub omega_zc: f32,
    /// Simulator truth [rad/s electrical].
    pub omega_e_true: f32,
    pub theta_e_true: f32,
    /// Sectors of commutation error, signed and wrapped to ±3.
    pub sector_err: i32,
    pub locked: bool,
}

pub struct SixStepSim {
    pub motor: PhaseMotor,
    cfg: SixStepCfg,
    ramp: Ramp,
    zc: ZeroCross,
    speed: Pi,
    mode: Mode,
    sector: usize,
    duty: f32,
    ctrl_dt: f32,
    t: f32,
    load: f32,
    physics_dt: f32,
}

impl SixStepSim {
    pub fn new(cfg: SixStepCfg) -> Self {
        let mut motor = PhaseMotor::new(cfg.params, cfg.shape);
        motor.bridge = cfg.bridge;
        Self {
            motor,
            ramp: Ramp::new(cfg.ramp),
            zc: ZeroCross::new(cfg.zc),
            speed: Pi::new(cfg.speed_gains, cfg.duty_max),
            mode: Mode::Ramp,
            sector: 0,
            duty: cfg.ramp.duty,
            ctrl_dt: 1.0 / cfg.ctrl_freq,
            t: 0.0,
            load: 0.0,
            physics_dt: 2e-7,
            cfg,
        }
    }

    pub fn set_load(&mut self, nm: f32) {
        self.load = nm;
    }

    pub fn set_speed_ref(&mut self, omega_e: f32) {
        self.cfg.omega_ref = omega_e;
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Sample the idle terminal the way the hardware front end would.
    fn sense(&self) -> f32 {
        let v = self.motor.idle_terminal(self.cfg.sample_at, self.cfg.vbus);
        if self.cfg.clamp_negative && v < 0.0 {
            0.0
        } else {
            v
        }
    }

    pub fn step(&mut self) -> SixStepSample {
        let dt = self.ctrl_dt;
        let v_ref = match (self.cfg.ref_source, self.cfg.sample_at) {
            (RefSource::MeasuredMid, at) => self.motor.measured_mid(at, self.cfg.vbus),
            (RefSource::VbusHalf, SamplePoint::OnTime) => self.cfg.vbus * 0.5,
            (RefSource::VbusHalf, SamplePoint::Freewheel) => 0.0,
        };
        let v_float = self.sense();

        match self.mode {
            Mode::Ramp => {
                let want = self.ramp.update(dt);
                if want != self.sector {
                    self.sector = want;
                    self.motor.commutate(self.sector, self.cfg.vbus);
                    self.zc.commutated();
                }
                // Run the detector during the ramp so it can lock before the
                // ramp finishes; its output is watched, not obeyed.
                self.zc.update(self.sector, v_float, v_ref, dt);
                self.duty = self.cfg.ramp.duty;
                if self.ramp.done() && self.zc.locked() {
                    self.speed.preload(self.duty);
                    self.mode = Mode::Sensing;
                } else if self.ramp.done() {
                    // Reached handoff speed without confident sensing: seed
                    // the timer from the forced frequency and hand over
                    // anyway, which is what a real drive must do.
                    self.zc.seed(self.ramp.omega_e());
                    self.speed.preload(self.duty);
                    self.mode = Mode::Sensing;
                }
            }
            Mode::Sensing | Mode::Lost => {
                if self.zc.update(self.sector, v_float, v_ref, dt) == ZcEvent::Commutate {
                    self.sector = (self.sector + 1) % sixstep::SECTORS;
                    self.motor.commutate(self.sector, self.cfg.vbus);
                }
                self.mode = if self.zc.locked() {
                    Mode::Sensing
                } else {
                    Mode::Lost
                };
                let err = self.cfg.omega_ref - self.zc.omega_e();
                let raw = self.speed.update(err, dt);
                self.duty = raw.clamp(self.cfg.duty_min, self.cfg.duty_max);
                if raw != self.duty {
                    // Back-calculation: hold the integrator at what was
                    // actually applied. Without this the loop winds down
                    // through the whole deceleration and overshoots on the
                    // way back.
                    self.speed.preload(self.duty);
                }
            }
        }

        // Average-value bridge: the conducting pair sees duty × bus.
        let v_line = self.duty * self.cfg.vbus;
        let n = ((dt / self.physics_dt).round() as usize).max(1);
        let sub = dt / n as f32;
        for _ in 0..n {
            self.motor.step(v_line, self.load, sub);
        }
        self.t += dt;

        let sector_true = sixstep::sector_of(self.motor.theta_e());
        let mut d = self.sector as i32 - sector_true as i32;
        while d > 3 {
            d -= 6;
        }
        while d < -3 {
            d += 6;
        }
        SixStepSample {
            t: self.t,
            mode: self.mode,
            sector: self.sector,
            sector_true,
            duty: self.duty,
            i_line: self.motor.i_line,
            v_float,
            v_ref,
            omega_zc: self.zc.omega_e(),
            omega_e_true: self.motor.omega_e(),
            theta_e_true: self.motor.theta_e(),
            sector_err: d,
            locked: self.zc.locked(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(cfg: SixStepCfg, secs: f32) -> (SixStepSim, Vec<SixStepSample>) {
        let mut sim = SixStepSim::new(cfg);
        let n = (secs * cfg.ctrl_freq) as usize;
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            out.push(sim.step());
        }
        (sim, out)
    }

    /// The headline: forced ramp, handoff to sensing, and the drive holds a
    /// speed it measures itself.
    #[test]
    fn starts_locks_and_holds_speed() {
        let cfg = SixStepCfg::bench(600.0);
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() * 3 / 4..];
        assert!(
            tail.iter().all(|x| x.mode == Mode::Sensing),
            "lost sensing in the tail"
        );
        let mean: f32 = tail.iter().map(|x| x.omega_e_true).sum::<f32>() / tail.len() as f32;
        assert!(
            (mean - 600.0).abs() < 30.0,
            "held {mean} rad/s el, wanted 600"
        );
    }

    /// Commutation must stay aligned with the rotor: the sector the drive has
    /// energised is the sector the rotor's true angle asks for, within one.
    #[test]
    fn commutation_tracks_the_rotor() {
        let cfg = SixStepCfg::bench(600.0);
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() / 2..];
        let worst = tail.iter().map(|x| x.sector_err.abs()).max().unwrap();
        assert!(worst <= 1, "commutation drifted {worst} sectors from truth");
    }

    /// The speed read off the crossing interval must agree with the truth it
    /// never sees. This is the measurement the loop is closed on.
    #[test]
    fn measured_speed_matches_truth() {
        let cfg = SixStepCfg::bench(600.0);
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() * 3 / 4..];
        let err: f32 = tail
            .iter()
            .map(|x| (x.omega_zc - x.omega_e_true).abs() / x.omega_e_true.max(1.0))
            .sum::<f32>()
            / tail.len() as f32;
        assert!(err < 0.05, "mean speed error {:.1}%", err * 100.0);
    }

    /// Sampling in the freewheel with a clamped sense network must fail — this
    /// is the bench result of MS8 step 1, reproduced in simulation. If this
    /// test ever passes, the model has stopped describing the hardware.
    #[test]
    fn freewheel_sampling_with_a_clamped_network_cannot_lock() {
        let cfg = SixStepCfg {
            sample_at: SamplePoint::Freewheel,
            clamp_negative: true,
            ..SixStepCfg::bench(600.0)
        };
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() / 2..];
        let good =
            tail.iter().filter(|x| x.sector_err.abs() <= 1).count() as f32 / tail.len() as f32;
        assert!(
            good < 0.9,
            "clamped freewheel sampling tracked {:.0}% of the time — it should not work",
            good * 100.0
        );
    }

    /// Moving the sample into the on-time is the fix, on the same machine and
    /// the same clamp. Paired with the test above, this is the whole argument
    /// for the firmware change.
    #[test]
    fn on_time_sampling_rescues_the_same_clamped_network() {
        let cfg = SixStepCfg {
            sample_at: SamplePoint::OnTime,
            clamp_negative: true,
            ..SixStepCfg::bench(600.0)
        };
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() / 2..];
        let good =
            tail.iter().filter(|x| x.sector_err.abs() <= 1).count() as f32 / tail.len() as f32;
        assert!(
            good > 0.95,
            "on-time sampling only tracked {:.0}%",
            good * 100.0
        );
    }

    /// A load step must be absorbed by the duty loop without losing sync.
    #[test]
    fn holds_sync_through_a_load_step() {
        let cfg = SixStepCfg::bench(600.0);
        let mut sim = SixStepSim::new(cfg);
        for _ in 0..(2.2 * cfg.ctrl_freq) as usize {
            sim.step();
        }
        sim.set_load(3.0e-3);
        let mut worst = 0;
        let mut last = None;
        for _ in 0..(0.6 * cfg.ctrl_freq) as usize {
            let s = sim.step();
            worst = worst.max(s.sector_err.abs());
            last = Some(s);
        }
        let last = last.unwrap();
        assert!(worst <= 1, "load step cost {worst} sectors of alignment");
        assert_eq!(last.mode, Mode::Sensing, "lost sensing under load");
    }

    /// The bench's own failure, reproduced: with real switch and shunt drops,
    /// comparing against V_bus/2 puts a fixed offset on the threshold that is
    /// a large fraction of the back-EMF ramp, and the drive cannot hold
    /// commutation. Asserts failure on purpose — see the on-time/freewheel
    /// pair above for the same pattern.
    #[test]
    fn vbus_half_reference_fails_against_real_bridge_drops() {
        let cfg = SixStepCfg {
            ref_source: RefSource::VbusHalf,
            bridge: Bridge::bench(),
            ..SixStepCfg::bench(600.0)
        };
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() / 2..];
        let good =
            tail.iter().filter(|x| x.sector_err.abs() <= 1).count() as f32 / tail.len() as f32;
        assert!(
            good < 0.9,
            "V_bus/2 tracked {:.0}% against real drops — it should not work",
            good * 100.0
        );
    }

    /// Same machine, same drops, reference taken from the measured driven
    /// terminals: the drive holds. This pair is the argument for the firmware
    /// change made in session 19.
    #[test]
    fn measured_midpoint_reference_survives_real_bridge_drops() {
        let cfg = SixStepCfg {
            ref_source: RefSource::MeasuredMid,
            bridge: Bridge::bench(),
            ..SixStepCfg::bench(600.0)
        };
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() / 2..];
        let good =
            tail.iter().filter(|x| x.sector_err.abs() <= 1).count() as f32 / tail.len() as f32;
        assert!(
            good > 0.95,
            "measured mid-point only tracked {:.0}%",
            good * 100.0
        );
    }

    /// A sinusoidal machine also runs, with more sector-to-sector ripple —
    /// six-step is not restricted to trapezoidal machines, it is just less
    /// efficient on them.
    #[test]
    fn sinusoidal_machine_also_runs() {
        let cfg = SixStepCfg {
            shape: BemfShape::Sinusoidal,
            ..SixStepCfg::bench(600.0)
        };
        let (_, s) = run(cfg, 2.5);
        let tail = &s[s.len() * 3 / 4..];
        assert!(tail.iter().all(|x| x.mode == Mode::Sensing));
        let mean: f32 = tail.iter().map(|x| x.omega_e_true).sum::<f32>() / tail.len() as f32;
        assert!((mean - 600.0).abs() < 30.0, "held {mean} rad/s el");
    }
}
