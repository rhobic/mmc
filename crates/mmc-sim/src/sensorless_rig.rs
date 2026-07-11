//! The full sensorless stack running against the virtual motor: FOC current
//! loop + flux observer + I-f startup sequencer + speed loop. One step()
//! per control period, returning both the controller's view and the sim's
//! ground truth — this is what the MS4 regression tests assert against and
//! what `mmc-host sim --scenario sensorless-speed` records.

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc, FocOutput};
use mmc_core::math::wrap_angle;
use mmc_core::observer::{FluxObserver, FluxObserverCfg};
use mmc_core::sensorless::{Phase, Sequencer, SequencerCfg, SpeedLoop};
use mmc_core::transforms::{Abc, Dq};
use mmc_core::tuning::{current_pi_gains, speed_pi_gains};
use mmc_hal::{BusVoltageSense, CurrentSense, PwmOutput};

use crate::{PmsmParams, VirtualMotor};

#[derive(Copy, Clone, Debug)]
pub struct SensorlessRunCfg {
    pub params: PmsmParams,
    pub vbus: f32,
    pub ctrl_freq: f32,
    /// Current-loop bandwidth [rad/s].
    pub current_bw: f32,
    /// Speed-loop bandwidth [rad/s].
    pub speed_bw: f32,
    /// Speed-loop i_q limit [A].
    pub iq_limit: f32,
    /// Final speed target [rad/s electrical].
    pub omega_ref: f32,
    /// Speed-reference slew after handoff [rad/s² electrical].
    pub ref_accel: f32,
    pub seq: SequencerCfg,
}

impl SensorlessRunCfg {
    pub fn small_bldc(omega_ref: f32) -> Self {
        Self {
            params: PmsmParams::small_bldc(),
            vbus: 24.0,
            ctrl_freq: 10_000.0,
            current_bw: 2000.0,
            speed_bw: 40.0,
            iq_limit: 1.5,
            omega_ref,
            ref_accel: 2500.0,
            seq: SequencerCfg::default(),
        }
    }
}

/// One control period's worth of results, controller view + ground truth.
#[derive(Copy, Clone, Debug)]
pub struct Sample {
    pub t: f32,
    pub iq_ref: f32,
    pub out: FocOutput,
    pub i_abc: [f32; 3],
    /// Sim truth.
    pub omega_m_true: f32,
    pub theta_e_true: f32,
    /// Observer view.
    pub theta_est: f32,
    pub omega_est: f32,
    /// wrap(θ̂ − θ_true) — the number MS4 lives or dies by.
    pub theta_err: f32,
    pub phase: Phase,
}

pub struct SensorlessSim {
    pub rig: VirtualMotor,
    cfg: SensorlessRunCfg,
    foc: Foc,
    obs: FluxObserver,
    seq: Sequencer,
    speed: SpeedLoop,
    omega_ref_cur: f32,
    speed_preloaded: bool,
    ctrl_dt: f32,
}

impl SensorlessSim {
    pub fn new(cfg: SensorlessRunCfg) -> Self {
        let p = cfg.params;
        let mut rig = VirtualMotor::new(p, cfg.vbus);
        rig.enable();
        Self {
            rig,
            foc: Foc::with_feedforward(
                current_pi_gains(p.rs, p.lq, cfg.current_bw),
                Decoupling {
                    ld: p.ld,
                    lq: p.lq,
                    flux: p.flux,
                },
            ),
            obs: FluxObserver::new(FluxObserverCfg::new(p.rs, p.lq)),
            seq: Sequencer::new(cfg.seq),
            speed: SpeedLoop::new(
                speed_pi_gains(p.inertia, p.torque_constant(), p.pole_pairs, cfg.speed_bw),
                cfg.iq_limit,
            ),
            omega_ref_cur: cfg.seq.omega_handoff,
            speed_preloaded: false,
            ctrl_dt: 1.0 / cfg.ctrl_freq,
            cfg,
        }
    }

    /// External load torque [N·m].
    pub fn set_load(&mut self, nm: f32) {
        self.rig.load_torque = nm;
    }

    pub fn phase(&self) -> Phase {
        self.seq.phase()
    }

    pub fn step(&mut self) -> Sample {
        let dt = self.ctrl_dt;
        let t = self.rig.time() as f32;
        let [ia, ib, ic] = self.rig.phase_currents();
        let vbus = self.rig.vbus();

        let seq_out = self.seq.update(&self.obs, dt);
        let iq_ref = match seq_out.iq_open {
            Some(iq) => iq,
            None => {
                if !self.speed_preloaded {
                    // Bumpless transfer from the startup current, torque
                    // aligned with the rotation direction.
                    self.speed
                        .preload(self.cfg.seq.i_start * self.cfg.seq.omega_handoff.signum());
                    self.speed_preloaded = true;
                }
                // Slew the reference from the handoff speed to the target.
                let d = (self.cfg.omega_ref - self.omega_ref_cur)
                    .clamp(-self.cfg.ref_accel * dt, self.cfg.ref_accel * dt);
                self.omega_ref_cur += d;
                self.speed.update(self.omega_ref_cur, seq_out.omega, dt)
            }
        };

        let out = self.foc.step(
            Abc {
                a: ia,
                b: ib,
                c: ic,
            },
            seq_out.theta,
            seq_out.omega,
            Dq { d: 0.0, q: iq_ref },
            vbus,
            dt,
        );
        self.rig.set_duties(out.duties);
        self.rig.advance(dt);

        // Observer sees what the controller applied and measured.
        self.obs.update(out.i_ab, out.v_ab, dt);

        let theta_est = self.obs.electrical_angle();
        Sample {
            t,
            iq_ref,
            out,
            i_abc: [ia, ib, ic],
            omega_m_true: self.rig.motor.omega_m,
            theta_e_true: self.rig.motor.theta_e(),
            theta_est,
            omega_est: self.obs.electrical_velocity(),
            theta_err: wrap_angle(theta_est - self.rig.motor.theta_e()),
            phase: seq_out.phase,
        }
    }
}
