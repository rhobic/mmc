//! The control tick: everything the control interrupt does, against a
//! [`MotorBoard`].

use core::sync::atomic::Ordering;

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc};
use mmc_core::hall::{HallAngle, HallMap, HallSpeed};
use mmc_core::inverter::DeadtimeModel;
use mmc_core::math::{sin_cos, wrap_angle};
use mmc_core::observer::{FluxObserver, FluxObserverCfg};
use mmc_core::pi::{Pi, PiGains};
use mmc_core::probe;
use mmc_core::sensorless::{Phase, Sequencer, SequencerCfg, SpeedLoop};
use mmc_core::sixstep::{self, Ramp, RampCfg, ZcCfg, ZcEvent, ZeroCross};
use mmc_core::svpwm::svpwm;
use mmc_core::transforms::{clarke, inverse_park, park, Abc, AlphaBeta, Dq};
use mmc_core::tuning::current_pi_gains;
use mmc_hal::MotorBoard;
use mmc_proto::{channel, param, test};

use crate::{mode, Shared, BURST_DONE, ST_FAULT_DRV, ST_FAULT_OC, ST_FAULT_VBUS, ST_OFF};
use crate::{ST_FAULT_HALL, ST_RUN, ST_SL_BLEND, ST_SL_RAMP, ST_SS_UNLOCKED, ST_STALL};

const V_SLEW: f32 = 5.0; // V/s
const I_SLEW: f32 = 2.0; // A/s
/// Boot zero-current calibration length [ticks] — a sample count, not a
/// time: it sets the offset-estimate noise, whatever the loop rate.
const CAL_TICKS: u32 = 8192;
/// Drive shuts off if the host goes silent this long (capture keep-alives).
const DEADMAN_S: f32 = 2.0;
/// Consecutive low-observer-flux time before the stall fault trips.
const STALL_S: f32 = 0.1;
/// Invalid hall states tolerated in a row (a glitch) before the hall fault.
const HALL_BAD_S: f32 = 0.003;

// Locked-rotor R/L probe (RunTest RL_STEP): rotor-align time at the first
// voltage level, then square-wave excitation between the two levels. τ=L/R
// sits near the sample period on the low-inductance bench motor, so a single
// edge has ~2 usable samples; folding many edges recovers it. The plateau
// half-period (32 ticks) is long against τ, so the plateaus settle and R
// comes out differentially (dead time cancels).
const PROBE_ALIGN_S: f32 = 0.3;
const PROBE_HALF_TICKS: u32 = 32;
/// Pre-roll recorded before the excitation starts, in pairs.
const PROBE_PRE_PAIRS: usize = 256;

/// State owned by the control interrupt.
pub struct Engine {
    // calibration
    cal_count: u32,
    cal_sum: [f32; 3],
    offset_v: [f32; 3], // amp output at zero current
    // drive
    epoch_seen: u32,
    mode: u8,
    theta: f32,
    omega: f32,
    amp: f32,
    foc: Option<Foc>,
    /// Flux observer: shadow-instrumented during the forced-angle drives,
    /// the angle source in sensorless mode.
    obs: Option<FluxObserver>,
    // Sensorless mode (MS4 stack): startup sequencer + speed loop.
    seq: Option<Sequencer>,
    speed: Option<SpeedLoop>,
    /// Slewed speed-loop reference [rad/s electrical].
    omega_ref_cur: f32,
    /// Signed startup current, consumed as the speed-PI preload on the
    /// first closed-loop tick (0.0 = already consumed).
    sl_preload: f32,
    /// Six-step: commutation sector, zero-cross detector and startup ramp.
    ss_sector: usize,
    ss_zc: Option<ZeroCross>,
    ss_ramp: Option<Ramp>,
    /// True once commutation is timed from measured crossings, not the ramp.
    ss_sensing: bool,
    /// Duty→speed loop for six-step, closed on the crossing interval.
    ss_speed: Option<Pi>,
    /// Speed target while sensing [rad/s electrical].
    ss_target: f32,
    /// Probe tick counter (mode 4).
    probe_ticks: u32,
    oc_strikes: u8,
    /// Consecutive low-observer-flux ticks in closed-loop sensorless.
    stall_strikes: u32,
    vbus_filt: f32,
    /// Hall edge timing, fed every tick whether or not anything uses it.
    hall: HallSpeed,
    /// Calibrated hall angle: the angle source of the hall drive modes, and
    /// the reference every other mode is scored against on a board with
    /// halls.
    hall_angle: Option<HallAngle>,
    /// Consecutive invalid hall states while a hall mode runs.
    hall_bad: u32,
    /// The voltage command of the previous tick: what the bridge actually
    /// applied over the period the current sample just closed. The observer
    /// integrates this, not the command computed this tick.
    v_applied: AlphaBeta,
    /// The command before that (still on the winding for the first
    /// `pwm_latency` of the period).
    v_applied2: AlphaBeta,
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    pub const fn new() -> Self {
        Self {
            cal_count: 0,
            cal_sum: [0.0; 3],
            offset_v: [0.0; 3],
            epoch_seen: 0,
            mode: 0,
            theta: 0.0,
            omega: 0.0,
            amp: 0.0,
            foc: None,
            obs: None,
            seq: None,
            speed: None,
            omega_ref_cur: 0.0,
            sl_preload: 0.0,
            ss_sector: 0,
            ss_zc: None,
            ss_ramp: None,
            ss_sensing: false,
            ss_speed: None,
            ss_target: 0.0,
            probe_ticks: 0,
            oc_strikes: 0,
            stall_strikes: 0,
            vbus_filt: 0.0,
            hall: HallSpeed::new(),
            hall_angle: None,
            hall_bad: 0,
            v_applied: AlphaBeta {
                alpha: 0.0,
                beta: 0.0,
            },
            v_applied2: AlphaBeta {
                alpha: 0.0,
                beta: 0.0,
            },
        }
    }

    /// Calibrated zero-current amplifier outputs [V] (diagnostic).
    pub fn current_offsets(&self) -> [f32; 3] {
        self.offset_v
    }

    fn stage_off(b: &mut impl MotorBoard) {
        b.set_phase_enables(0);
        b.set_duties([0.0; 3]);
    }

    /// Abort the drive into `state`, from inside the tick.
    fn trip<const N: usize>(&mut self, sh: &Shared<N>, b: &mut impl MotorBoard, state: u8) {
        self.mode = mode::OFF;
        self.omega = 0.0;
        self.amp = 0.0;
        Self::stage_off(b);
        sh.burst_abort();
        sh.state.store(state, Ordering::Relaxed);
        sh.cmd_mode.store(mode::OFF, Ordering::Relaxed);
    }

    /// One control period. Call from the control interrupt, once per
    /// [`mmc_hal::BoardSpec::ctrl_hz`] period, after the synchronous sample
    /// has converted.
    pub fn tick<const N: usize>(&mut self, sh: &Shared<N>, b: &mut impl MotorBoard) {
        let t0 = b.cycles();
        let cfg = &sh.cfg;
        let spec = &cfg.spec;
        let dt = cfg.dt();
        let ticks = sh.control_ticks.fetch_add(1, Ordering::Relaxed) + 1;
        let p = |id| sh.param(id);

        let sample = b.sample();
        let volts = sample.phase_volts;
        self.vbus_filt += 0.05 * (sample.vbus - self.vbus_filt);

        // --- zero-current calibration (stage is off; measure amp offsets).
        if self.cal_count < CAL_TICKS {
            for (sum, &v) in self.cal_sum.iter_mut().zip(&volts) {
                *sum += v;
            }
            self.cal_count += 1;
            if self.cal_count == CAL_TICKS {
                for (offset, &sum) in self.offset_v.iter_mut().zip(&self.cal_sum) {
                    *offset = sum / CAL_TICKS as f32;
                }
                sh.state.store(ST_OFF, Ordering::Relaxed);
            }
            return;
        }

        // Positive phase current (into the motor) pulls the amp output below
        // its zero-current offset.
        let k = spec.cur_volts_per_amp;
        let i_abc = Abc {
            a: (self.offset_v[0] - volts[0]) / k,
            b: (self.offset_v[1] - volts[1]) / k,
            c: (self.offset_v[2] - volts[2]) / k,
        };
        let vt = b.terminal_volts();
        let hall_state = b.hall_state();
        if let Some(h) = hall_state {
            self.hall.update(h, dt);
        }
        // The calibrated hall angle, whenever a drive is built on a board
        // with halls: the angle source of the hall modes and the reference
        // for all the others.
        let hall_ref = match (hall_state, self.hall_angle.as_mut()) {
            (Some(h), Some(ha)) => ha.update(h, dt),
            _ => None,
        };

        // --- pick up new host commands.
        let epoch = sh.cmd_epoch.load(Ordering::Acquire);
        if epoch != self.epoch_seen {
            self.epoch_seen = epoch;
            self.command(sh, b);
        }

        // --- protection trips (only meaningful once running).
        if self.mode != mode::OFF {
            let fault = if b.driver_fault() {
                Some(ST_FAULT_DRV)
            } else if self.vbus_filt > spec.vbus_max {
                Some(ST_FAULT_VBUS)
            } else {
                let lim = spec.i_trip;
                let oc = i_abc.a.abs() > lim || i_abc.b.abs() > lim || i_abc.c.abs() > lim;
                self.oc_strikes = if oc { self.oc_strikes + 1 } else { 0 };
                (self.oc_strikes >= 2).then_some(ST_FAULT_OC)
            };
            if let Some(f) = fault {
                self.trip(sh, b, f);
            }
            // Deadman: host silent too long with the stage live.
            let last = sh.last_rx_tick.load(Ordering::Relaxed);
            if self.mode != mode::OFF && ticks.wrapping_sub(last) > cfg.ticks(DEADMAN_S) {
                self.trip(sh, b, ST_OFF);
            }
        }

        // --- drive.
        let mut duties = [0.0f32; 3];
        let mut v_dq = Dq::default();
        let mut i_dq = Dq::default();
        let mut iq_ref = 0.0f32;
        let mut theta_est = 0.0f32;
        let mut omega_est = 0.0f32;
        let mut theta_err = 0.0f32;

        if self.mode != mode::OFF {
            // Sample point is a runtime param so it can be swept over the wire.
            b.set_terminal_sample_offset(p(param::ONTIME_CCR5));
            // Six-step drives the enables itself; everything else wants all
            // three phases live (a previous six-step run may have left one
            // Hi-Z).
            let six_step = matches!(
                self.mode,
                mode::SS_FORCED | mode::SS_SENSORLESS | mode::SS_HALL
            );
            if !six_step {
                b.set_phase_enables(0b111);
            }
            let omega_target = f32::from_bits(sh.cmd_omega.load(Ordering::Relaxed));
            let amp_target = f32::from_bits(sh.cmd_amp.load(Ordering::Relaxed));
            let i_ab = clarke(i_abc);
            let vbus = self.vbus_filt.max(1.0);

            let v_ab = if self.mode == mode::PROBE
                && sh.probe_kind.load(Ordering::Relaxed) == test::L_THETA
            {
                // Saliency sweep: align at v_low on θ = 0 (parks a free
                // rotor; a clamped one just stays put and the fit recovers its
                // angle), then run the shared `mmc_core::probe` schedule —
                // square-wave v_d along ±paired electrical angles, recording
                // (i_d, i_q) in the excitation frame behind a self-describing
                // header.
                self.probe_ticks += 1;
                let align = cfg.ticks(PROBE_ALIGN_S);
                let n = sh.burst_len.load(Ordering::Relaxed) as usize;
                if n >= probe::SAL_HDR + probe::SAL_TICKS * 2 {
                    // Recording complete: stage off, hand the buffer over.
                    self.finish_probe(sh, b);
                    AlphaBeta::default()
                } else {
                    let v_low = f32::from_bits(sh.probe_v_align.load(Ordering::Relaxed));
                    let v_high = f32::from_bits(sh.probe_v_step.load(Ordering::Relaxed));
                    let half = sh.probe_half.load(Ordering::Relaxed) as usize;
                    let (theta_x, v) = if self.probe_ticks <= align {
                        (0.0, v_low)
                    } else {
                        let t = (self.probe_ticks - align - 1) as usize;
                        if t == 0 {
                            let hdr = probe::sal_header(
                                test::L_THETA,
                                half,
                                v_low,
                                v_high,
                                spec.ctrl_hz as f32,
                            );
                            // Safety: the host only reads once DONE.
                            let buf = unsafe { &mut *sh.burst.0.get() };
                            buf[..probe::SAL_HDR].copy_from_slice(&hdr);
                        }
                        let high = probe::sal_level_is_high(t, half);
                        (probe::sal_angle(t, half), if high { v_high } else { v_low })
                    };
                    let sc = sin_cos(theta_x);
                    i_dq = park(i_ab, sc);
                    if self.probe_ticks > align {
                        let t = (self.probe_ticks - align - 1) as usize;
                        let idx = probe::SAL_HDR + 2 * t;
                        // Safety: the host only reads once DONE.
                        let buf = unsafe { &mut *sh.burst.0.get() };
                        buf[idx] = i_dq.d;
                        buf[idx + 1] = i_dq.q;
                        sh.burst_len.store((idx + 2) as u32, Ordering::Relaxed);
                    }
                    v_dq = Dq { d: v, q: 0.0 };
                    let v_ab = inverse_park(v_dq, sc);
                    duties = svpwm(v_ab, vbus);
                    v_ab
                }
            } else if self.mode == mode::PROBE {
                // Locked-rotor R/L probe: θ held at 0 (rotor aligned during
                // the first phase), then unslewed square-wave v_d between the
                // two levels, recording (i_d, v_d) per tick into the burst
                // buffer at the full control rate.
                self.probe_ticks += 1;
                let align = cfg.ticks(PROBE_ALIGN_S);
                let n = sh.burst_len.load(Ordering::Relaxed) as usize;
                if n + 2 > N {
                    self.finish_probe(sh, b);
                    AlphaBeta::default()
                } else {
                    let v = f32::from_bits(if self.probe_ticks <= align {
                        sh.probe_v_align.load(Ordering::Relaxed)
                    } else {
                        let half = (self.probe_ticks - align - 1) / PROBE_HALF_TICKS;
                        if half.is_multiple_of(2) {
                            sh.probe_v_step.load(Ordering::Relaxed)
                        } else {
                            sh.probe_v_align.load(Ordering::Relaxed)
                        }
                    });
                    let sc = sin_cos(0.0);
                    i_dq = park(i_ab, sc);
                    // The pre-roll scales down on a small buffer so most of it
                    // still records excitation edges.
                    let pre = PROBE_PRE_PAIRS.min(N / 16) as u32;
                    if self.probe_ticks + pre > align {
                        // Safety: the host only reads once DONE.
                        let buf = unsafe { &mut *sh.burst.0.get() };
                        buf[n] = i_dq.d;
                        buf[n + 1] = v;
                        sh.burst_len.store((n + 2) as u32, Ordering::Relaxed);
                    }
                    v_dq = Dq { d: v, q: 0.0 };
                    let v_ab = inverse_park(v_dq, sc);
                    duties = svpwm(v_ab, vbus);
                    v_ab
                }
            } else if self.mode == mode::HALL_FOC || self.mode == mode::SS_HALL {
                // Hall-sensored: the calibrated halls own the angle. A short
                // run of invalid states (a glitch) holds the last estimate;
                // a longer one is a lost sensor and trips.
                match hall_ref {
                    Some(theta) => {
                        self.hall_bad = 0;
                        self.theta = theta;
                        self.omega = self.hall_angle.as_ref().unwrap().omega();
                    }
                    None => {
                        self.hall_bad += 1;
                        if self.hall_bad >= cfg.ticks(HALL_BAD_S) {
                            self.trip(sh, b, ST_FAULT_HALL);
                        }
                    }
                }
                if self.mode == mode::OFF {
                    AlphaBeta::default()
                } else if self.mode == mode::HALL_FOC {
                    // Speed loop on the hall speed, FOC on the hall angle:
                    // the closed-loop half of the sensorless stack, without
                    // the I-f ramp in front of it.
                    let accel = p(param::OMEGA_ACCEL);
                    let d = (omega_target - self.omega_ref_cur).clamp(-accel * dt, accel * dt);
                    self.omega_ref_cur += d;
                    iq_ref =
                        self.speed
                            .as_mut()
                            .unwrap()
                            .update(self.omega_ref_cur, self.omega, dt);
                    let out = self.foc.as_mut().unwrap().step(
                        i_abc,
                        self.theta,
                        self.omega,
                        Dq { d: 0.0, q: iq_ref },
                        vbus,
                        dt,
                    );
                    duties = out.duties;
                    v_dq = out.v_dq;
                    i_dq = out.i_dq;
                    out.v_ab
                } else {
                    duties = self.sixstep_hall(sh, b, omega_target, amp_target);
                    i_dq = park(i_ab, sin_cos(self.theta));
                    AlphaBeta::default()
                }
            } else if self.mode == mode::SENSORLESS {
                // Sensorless: the sequencer owns the angle (I-f ramp → blend
                // → observer), the speed loop owns i_q once closed. The
                // observer state is one tick old here; it is fed below, same
                // as the rig.
                let seq_out = self
                    .seq
                    .as_mut()
                    .unwrap()
                    .update(self.obs.as_ref().unwrap(), dt);
                iq_ref = match seq_out.iq_open {
                    Some(iq) => iq,
                    None => {
                        let speed = self.speed.as_mut().unwrap();
                        if self.sl_preload != 0.0 {
                            // Bumpless takeover from the (blend-tapered)
                            // startup current the sequencer actually ended on.
                            let taper = self.seq.as_ref().map_or(1.0, |q| q.taper_end());
                            speed.preload(self.sl_preload * taper);
                            self.sl_preload = 0.0;
                        }
                        // Slew the reference from the handoff speed toward the
                        // (live-retargetable) command.
                        let accel = p(param::OMEGA_ACCEL);
                        let d = (omega_target - self.omega_ref_cur).clamp(-accel * dt, accel * dt);
                        self.omega_ref_cur += d;
                        speed.update(self.omega_ref_cur, seq_out.omega, dt)
                    }
                };
                // Closed loop: the observer was last updated on the previous
                // tick's sample, so bring its angle forward to this one before
                // the Park transforms use it.
                self.theta = if seq_out.phase == Phase::Closed {
                    wrap_angle(seq_out.theta + seq_out.omega * dt)
                } else {
                    seq_out.theta
                };
                self.omega = seq_out.omega;
                let out = self.foc.as_mut().unwrap().step(
                    i_abc,
                    self.theta,
                    self.omega,
                    Dq { d: 0.0, q: iq_ref },
                    vbus,
                    dt,
                );
                duties = out.duties;
                v_dq = out.v_dq;
                i_dq = out.i_dq;
                out.v_ab
            } else {
                // Forced-frame modes: ramp the electrical frequency + amplitude.
                let accel = p(param::OMEGA_ACCEL);
                let d_omega = (omega_target - self.omega).clamp(-accel * dt, accel * dt);
                self.omega += d_omega;
                let slew = if self.mode == mode::IF {
                    I_SLEW
                } else {
                    V_SLEW
                };
                let d_amp = (amp_target - self.amp).clamp(-slew * dt, slew * dt);
                self.amp += d_amp;
                self.theta = wrap_angle(self.theta + self.omega * dt);

                let sc = sin_cos(self.theta);
                i_dq = park(i_ab, sc);

                if self.mode == mode::VOLT {
                    // Open-loop rotating voltage vector.
                    v_dq = Dq {
                        d: self.amp,
                        q: 0.0,
                    };
                    let v_ab = inverse_park(v_dq, sc);
                    duties = svpwm(v_ab, vbus);
                    v_ab
                } else if self.mode == mode::SS_SENSORLESS {
                    duties = self.sixstep_sensorless(sh, b, vt);
                    AlphaBeta::default()
                } else if self.mode == mode::SS_FORCED {
                    // Forced six-step commutation: two phases conduct, the
                    // third is Hi-Z. `amp` is the high-side duty (0..1), not
                    // volts. Sector advances with the forced angle, so the
                    // rotor is dragged exactly as in open-loop voltage mode.
                    // Sector mapping and table are shared with the simulator
                    // via mmc_core::sixstep, so commutation order and
                    // alignment cannot drift apart between the two.
                    let sector = sixstep::sector_of(self.theta);
                    let (hi, lo, float) = sixstep::TABLE[sector];
                    let duty = self.amp.clamp(0.0, spec.max_duty);
                    duties = [0.0; 3];
                    duties[hi as usize] = duty;
                    duties[lo as usize] = 0.0;
                    b.set_phase_enables(!(1u8 << float) & 0b111);
                    sh.sixstep_sector.store(sector as u8, Ordering::Relaxed);
                    // The observer has no meaningful input here (one phase
                    // current is unmeasured by construction), so feed it zero
                    // and let the shadow estimate go stale rather than lie.
                    AlphaBeta::default()
                } else {
                    // I-f: closed current loop on the forced angle.
                    iq_ref = self.amp;
                    let out = self.foc.as_mut().unwrap().step(
                        i_abc,
                        self.theta,
                        self.omega,
                        Dq { d: 0.0, q: iq_ref },
                        vbus,
                        dt,
                    );
                    duties = out.duties;
                    v_dq = out.v_dq;
                    i_dq = out.i_dq;
                    out.v_ab
                }
            };
            if self.mode != mode::OFF {
                b.set_duties(duties);
            }

            // Observer update from what was applied and measured. In the
            // forced modes it runs in shadow and theta_err is the
            // (wrong-frame) hang angle; in sensorless mode theta_err is the
            // one-tick innovation.
            //
            // Six-step never feeds it: one phase current is unmeasured by
            // construction and the applied vector is not a rotating one, so
            // the estimate would be meaningless. Skipping it is also most of
            // why six-step costs less per tick than FOC.
            let observer_runs = !six_step && self.mode != mode::OFF;
            // The command just computed only reaches the winding from the
            // next PWM update on; the current sampled this tick was driven by
            // the previous command. Integrating this tick's command made the
            // observer lead the rotor by about one tick — measured against the
            // halls on motor 3 as part of a ~2-tick odd-in-ω angle error
            // (session 30b).
            // On hardware the command also lands `pwm_latency` of a tick late,
            // so the period just closed was driven partly by the one before.
            let lat = spec.pwm_latency;
            let v_obs = AlphaBeta {
                alpha: (1.0 - lat) * self.v_applied.alpha + lat * self.v_applied2.alpha,
                beta: (1.0 - lat) * self.v_applied.beta + lat * self.v_applied2.beta,
            };
            self.v_applied2 = self.v_applied;
            self.v_applied = v_ab;
            if let Some(obs) = self.obs.as_mut().filter(|_| observer_runs) {
                obs.update(i_ab, v_obs, dt);
                theta_est = obs.electrical_angle();
                omega_est = obs.electrical_velocity();
                theta_err = wrap_angle(theta_est - hall_ref.unwrap_or(self.theta));

                // Stall detector (closed-loop sensorless only): a stalled
                // rotor makes no back-EMF, but the observer still "locks"
                // onto the rotating L·i artifact and reports a confident,
                // wrong speed — with flux magnitude ≈ L·|i| instead of ψ (an
                // order of magnitude low; the MS4/MS5 fake-lock lesson).
                // 100 ms below 0.35·ψ while nominally closed-loop trips a
                // stall fault.
                //
                // `flux_mag()` is leak-compensated, so this threshold means
                // the same thing at every speed.
                let closed = self.mode == mode::SENSORLESS
                    && self
                        .seq
                        .as_ref()
                        .is_some_and(|q| q.phase() == Phase::Closed);
                if closed && obs.flux_mag() < 0.35 * p(param::FLUX) {
                    self.stall_strikes += 1;
                    if self.stall_strikes >= cfg.ticks(STALL_S) {
                        self.trip(sh, b, ST_STALL);
                    }
                } else {
                    self.stall_strikes = 0;
                }
            }
        }

        // --- telemetry snapshot (seqlock).
        let seq = sh.telem_seq.load(Ordering::Relaxed);
        sh.telem_seq.store(seq.wrapping_add(1), Ordering::Release);
        let put = |id: u8, v: f32| sh.telem[id as usize].store(v.to_bits(), Ordering::Relaxed);
        put(channel::IQ_REF, iq_ref);
        put(channel::I_D, i_dq.d);
        put(channel::I_Q, i_dq.q);
        put(channel::V_D, v_dq.d);
        put(channel::V_Q, v_dq.q);
        put(channel::DUTY_A, duties[0]);
        put(channel::DUTY_B, duties[1]);
        put(channel::DUTY_C, duties[2]);
        put(channel::OMEGA_M, self.omega / p(param::POLE_PAIRS).max(1.0));
        put(channel::THETA_E, self.theta);
        put(channel::VBUS, self.vbus_filt);
        put(channel::I_A, i_abc.a);
        put(channel::I_B, i_abc.b);
        put(channel::I_C, i_abc.c);
        // While sensorless runs, the state channel reports the startup phase.
        let state_telem = match (self.mode, self.seq.as_ref().map(|q| q.phase())) {
            (mode::SENSORLESS, Some(Phase::Ramp)) => ST_SL_RAMP,
            (mode::SENSORLESS, Some(Phase::Blend)) => ST_SL_BLEND,
            // Six-step: distinguish the forced ramp, confident sensing, and
            // commutating-but-unlocked, which otherwise all look like
            // "running".
            (mode::SS_SENSORLESS, _) if !self.ss_sensing => ST_SL_RAMP,
            (mode::SS_SENSORLESS, _) if !self.ss_zc.as_ref().is_some_and(|z| z.locked()) => {
                ST_SS_UNLOCKED
            }
            _ => sh.state.load(Ordering::Relaxed),
        };
        put(channel::STATE, state_telem as f32);
        put(channel::THETA_EST, theta_est);
        put(channel::OMEGA_EST, omega_est);
        put(channel::THETA_ERR, theta_err);
        // Terminal voltages via the BEMF dividers. On the shields so far the
        // clamp diodes rectify these, so they read 0..peak (a zero-cross /
        // coast-down instrument, not a live terminal-voltage sense under PWM).
        put(channel::VB_U, vt[0]);
        put(channel::VB_V, vt[1]);
        put(channel::VB_W, vt[2]);
        let six_step = matches!(
            self.mode,
            mode::SS_FORCED | mode::SS_SENSORLESS | mode::SS_HALL
        );
        put(
            channel::SECTOR,
            if six_step {
                sh.sixstep_sector.load(Ordering::Relaxed) as f32
            } else {
                0.0
            },
        );
        put(channel::HALL, hall_state.unwrap_or(0) as f32);
        put(channel::OMEGA_HALL, self.hall.omega());
        sh.telem_seq.store(seq.wrapping_add(2), Ordering::Release);

        let dur = b.cycles().wrapping_sub(t0);
        sh.isr_max_cycles.fetch_max(dur, Ordering::Relaxed);
    }

    fn finish_probe<const N: usize>(&mut self, sh: &Shared<N>, b: &mut impl MotorBoard) {
        self.mode = mode::OFF;
        Self::stage_off(b);
        sh.state.store(ST_OFF, Ordering::Relaxed);
        sh.cmd_mode.store(mode::OFF, Ordering::Relaxed);
        sh.burst_state.store(BURST_DONE, Ordering::Release);
    }

    /// Apply a new host command (the epoch moved).
    fn command<const N: usize>(&mut self, sh: &Shared<N>, b: &mut impl MotorBoard) {
        let cfg = &sh.cfg;
        let spec = &cfg.spec;
        let p = |id| sh.param(id);
        let mode = sh.cmd_mode.load(Ordering::Relaxed);
        let state = sh.state.load(Ordering::Relaxed);
        if mode == mode::OFF {
            self.mode = mode::OFF;
            self.omega = 0.0;
            self.amp = 0.0;
            Self::stage_off(b);
            sh.burst_abort();
            if state >= ST_FAULT_OC || state == ST_RUN {
                sh.state.store(ST_OFF, Ordering::Relaxed); // (fault re-arm)
            }
            return;
        }
        if state != ST_OFF && state != ST_RUN {
            return; // calibrating or faulted: Off first
        }
        if self.vbus_filt < spec.vbus_min_run || self.vbus_filt > spec.vbus_max {
            self.mode = mode::OFF;
            Self::stage_off(b);
            sh.burst_abort();
            sh.state.store(ST_FAULT_VBUS, Ordering::Relaxed);
            return;
        }
        if self.mode == mode::SENSORLESS
            && mode == mode::SS_SENSORLESS
            && matches!(self.seq.as_ref().map(|q| q.phase()), Some(Phase::Closed))
            && self.omega > 40.0
        {
            // Live handover: closed-loop sensorless FOC has the rotor at a
            // verified speed, and six-step takes over commutation from the
            // observer's angle. Open-loop six-step ramps lose a low-inductance
            // rotor (pull-out) well below any speed worth handing off at —
            // carrying it up under current control is both the robust startup
            // and the only way to put a *synchronized* rotor under the
            // back-EMF detector at speed. Forward spin only: the sector table
            // runs one direction.
            let w = self.omega;
            // The FOC angle and the six-step table share one convention
            // (inverse_park and the sixstep shape both give
            // e_u = -psi*omega*sin(theta)), so the observer angle seeds the
            // sector directly. Verified on the bench by contradiction: a
            // deliberate +pi seed drew the ~1.9 A an antipodal pair predicts
            // within two control ticks, while this seed starts at the ~0.3 A
            // an aligned one does.
            self.ss_sector = sixstep::sector_of(self.theta);
            // Blanking here is NOT the ramp path's 250 us: it only has to
            // clear the freewheel demag (L*I/V ~ 2.5 us on the bench motor)
            // and the divider settle. At speed the window is ~1 ms and a late
            // commutation pushes the next crossing toward window entry — a
            // long blank then swallows it, the timeout fires, and one missed
            // window is desync (measured: tracked 5 sectors at 898 rad/s,
            // then lost exactly the window whose crossing arrived early).
            let mut zc = ZeroCross::new(ZcCfg {
                blank: 30e-6,
                ..Default::default()
            });
            zc.seed(w);
            self.ss_zc = Some(zc);
            let mut pi = Pi::new(
                PiGains {
                    kp: p(param::SS_KP),
                    ki: p(param::SS_KI),
                },
                spec.max_duty,
            );
            // Seed the duty loop at this speed's feedforward so the modulation
            // change is bumpless.
            let ff = (p(param::FLUX) * w + p(param::IQ_LIMIT) * 2.0 * p(param::R))
                / self.vbus_filt.max(1.0);
            pi.preload(ff.clamp(0.01, spec.max_duty));
            self.ss_speed = Some(pi);
            self.ss_ramp = None;
            self.ss_sensing = true;
            self.ss_target = w;
            // `amp` becomes the duty ceiling now; slewing down from the FOC
            // current amplitude would cap harder than commanded.
            self.amp = f32::from_bits(sh.cmd_amp.load(Ordering::Relaxed)).clamp(0.0, spec.max_duty);
            self.mode = mode::SS_SENSORLESS;
            return;
        }
        // A new mode starts clean, whether from Off or straight from another
        // running mode; only a repeat of the same mode is a live retarget.
        // Each mode unwraps its own control blocks, so switching without
        // rebuilding them would panic the ISR with the bridge live (merged
        // from origin `40d502c`, which fixed the same rule in the pre-split
        // G474 firmware).
        if mode != self.mode {
            if self.mode == mode::PROBE {
                // Leaving a probe mid-recording: hand back the partial buffer
                // rather than leave it owned by the ISR.
                sh.burst_abort();
            }
            self.clean_start(sh, mode);
        }
        self.mode = mode;
        sh.state.store(ST_RUN, Ordering::Relaxed);
    }

    /// Build every control block for a drive starting from rest, from the
    /// runtime parameter table (profiler-writable).
    fn clean_start<const N: usize>(&mut self, sh: &Shared<N>, mode: u8) {
        let spec = &sh.cfg.spec;
        let p = |id| sh.param(id);
        self.theta = 0.0;
        self.omega = 0.0;
        self.amp = 0.0;
        self.v_applied = AlphaBeta::default();
        self.v_applied2 = AlphaBeta::default();
        self.oc_strikes = 0;
        self.stall_strikes = 0;
        self.probe_ticks = 0;
        let (rs, ls) = (p(param::R), p(param::L));
        let gains = current_pi_gains(rs, ls, p(param::CUR_BW));
        self.obs = Some(FluxObserver::new(FluxObserverCfg::new(rs, ls)));
        self.ss_sector = 0;
        self.ss_sensing = false;
        self.ss_zc = None;
        self.ss_ramp = None;
        self.ss_speed = None;
        self.ss_target = 0.0;
        if mode == mode::SS_SENSORLESS {
            // Blanking has to clear the freewheel of the phase that just
            // opened. Handoff speed comes from the command.
            self.ss_zc = Some(ZeroCross::new(ZcCfg {
                blank: 250e-6,
                ..Default::default()
            }));
            // Duty per rad/s electrical. The plant from duty to electrical
            // acceleration is p·(2ψ)·V_bus/(2R·J); the integral corner must
            // stay well under the crossover or the loop hunts, the lesson the
            // simulator taught.
            self.ss_speed = Some(Pi::new(
                PiGains {
                    kp: p(param::SS_KP),
                    ki: p(param::SS_KI),
                },
                spec.max_duty,
            ));
            self.ss_ramp = Some(Ramp::new(RampCfg {
                omega_start: 20.0,
                omega_handoff: f32::from_bits(sh.cmd_omega.load(Ordering::Relaxed))
                    .abs()
                    .max(40.0),
                accel: p(param::OMEGA_ACCEL),
                duty: 0.0, // duty comes from CMD_AMP
            }));
        }
        if mode == mode::SENSORLESS {
            // Sensorless: feedforward FOC (flux is measured now), I-f
            // sequencer toward the commanded direction, speed loop preloaded
            // with the (blend-tapered) startup current at handoff. Handoff
            // speed, ramp accel, and the current ceiling are runtime params
            // so a new motor tunes without a reflash.
            self.foc = Some(Foc::with_feedforward(
                gains,
                Decoupling {
                    ld: ls,
                    lq: ls,
                    flux: p(param::FLUX),
                },
            ));
            let omega_t = f32::from_bits(sh.cmd_omega.load(Ordering::Relaxed));
            let dir = if omega_t < 0.0 { -1.0 } else { 1.0 };
            let iq_lim = p(param::IQ_LIMIT);
            let handoff = p(param::SL_HANDOFF);
            let i_start = f32::from_bits(sh.cmd_amp.load(Ordering::Relaxed))
                .abs()
                .clamp(0.1, iq_lim);
            self.seq = Some(Sequencer::new(SequencerCfg {
                i_start,
                accel: p(param::OMEGA_ACCEL),
                omega_handoff: handoff * dir,
                ..SequencerCfg::default()
            }));
            // SPEED_KP/SPEED_KI, in amps per rad/s el — the FOC speed loop's
            // own gains, which `tools/profile.py` fits from J and kt and
            // `mmc-host apply` writes. NOT ss_kp/ss_ki: those are six-step's
            // duty→speed loop, a different plant in different units
            // (`0882ae7` bled them in here once; see PROGRESS session 27).
            self.speed = Some(SpeedLoop::new(
                PiGains {
                    kp: p(param::SPEED_KP),
                    ki: p(param::SPEED_KI),
                },
                iq_lim,
            ));
            self.omega_ref_cur = handoff * dir;
            self.sl_preload = i_start * dir;
        } else if mode == mode::HALL_FOC {
            // Same feedforward FOC and speed loop as sensorless, with the
            // amplitude of the command as the i_q authority (under
            // iq_limit) and the reference ramping from rest.
            self.foc = Some(Foc::with_feedforward(
                gains,
                Decoupling {
                    ld: ls,
                    lq: ls,
                    flux: p(param::FLUX),
                },
            ));
            let authority = f32::from_bits(sh.cmd_amp.load(Ordering::Relaxed))
                .abs()
                .clamp(0.05, p(param::IQ_LIMIT));
            self.speed = Some(SpeedLoop::new(
                PiGains {
                    kp: p(param::SPEED_KP),
                    ki: p(param::SPEED_KI),
                },
                authority,
            ));
            self.seq = None;
            self.omega_ref_cur = 0.0;
            self.sl_preload = 0.0;
        } else {
            self.foc = Some(Foc::new(gains));
            self.seq = None;
            self.speed = None;
            self.sl_preload = 0.0;
        }
        // Every drive on a board with halls carries the calibrated hall
        // angle: the hall modes run on it, the rest are scored against it.
        self.hall_angle = spec.has_halls.then(|| {
            HallAngle::new(HallMap {
                offset: p(param::HALL_OFFSET),
                dir: if p(param::HALL_DIR) < 0.0 { -1.0 } else { 1.0 },
                hyst: p(param::HALL_HYST),
            })
        });
        self.hall_bad = 0;
        if mode == mode::SS_HALL {
            // Duty per rad/s el, preloaded with the duty that drives the
            // current ceiling through two windings at standstill (the
            // breakaway duty), so the loop does not integrate up to it.
            let mut pi = Pi::new(
                PiGains {
                    kp: p(param::SS_KP),
                    ki: p(param::SS_KI),
                },
                spec.max_duty,
            );
            let ff = p(param::IQ_LIMIT) * 2.0 * p(param::R) / self.vbus_filt.max(1.0);
            pi.preload(ff.clamp(0.0, spec.max_duty));
            self.ss_speed = Some(pi);
            self.ss_target = 0.0;
        }
        // Dead-time compensation applies to every FOC-modulated mode, not just
        // sensorless: the bridge takes its cut from an I-f current vector
        // exactly the same way. `v_dead = 0` (the default until the rig is
        // measured) leaves the modulator untouched.
        if let Some(foc) = self.foc.as_mut() {
            // Command the voltage vector where the rotor will be while it is
            // applied: half a period for the hold itself, plus however late
            // this board's duties land. Left at the sim's 0.5 on hardware,
            // the field lagged ~0.1-0.2 rad at 1200 rad/s el on motor 3 and
            // i_d rippled ±0.28 A (session 30b).
            foc.advance_periods = 0.5 + spec.pwm_latency;
            foc.deadtime = Some(DeadtimeModel {
                v_dead: p(param::V_DEAD),
                i_thresh: p(param::I_THRESH),
            });
        }
    }

    /// Hall six-step: the sector from the hall angle, a duty→speed loop on
    /// the hall speed. Both directions: reverse torque at the same rotor
    /// angle is the same pair with the current flipped, i.e. the sector
    /// three steps on.
    fn sixstep_hall<const N: usize>(
        &mut self,
        sh: &Shared<N>,
        b: &mut impl MotorBoard,
        omega_target: f32,
        amp_target: f32,
    ) -> [f32; 3] {
        let spec = &sh.cfg.spec;
        let dt = sh.cfg.dt();
        let p = |id| sh.param(id);
        let dir = if omega_target < 0.0 { -1.0 } else { 1.0 };
        let mut sector = sixstep::sector_of(self.theta);
        if dir < 0.0 {
            sector = (sector + 3) % sixstep::SECTORS;
        }
        self.ss_sector = sector;
        // Slew the target so a retarget does not step the duty.
        let accel = p(param::OMEGA_ACCEL);
        let want = omega_target.abs();
        self.ss_target += (want - self.ss_target).clamp(-accel * dt, accel * dt);
        let measured = self.omega * dir;
        let ceiling = amp_target.clamp(0.0, spec.max_duty);
        let pi = self.ss_speed.as_mut().unwrap();
        let raw = pi.update(self.ss_target - measured, dt);
        let duty = raw.max(0.0).min(ceiling);
        if raw != duty {
            // Back-calculation: no braking quadrant, so it saturates low on
            // every deceleration and would otherwise wind up.
            pi.preload(duty);
        }
        let (hi, lo, fl) = sixstep::TABLE[sector];
        let mut duties = [0.0; 3];
        duties[hi as usize] = duty;
        duties[lo as usize] = 0.0;
        b.set_phase_enables(!(1u8 << fl) & 0b111);
        sh.sixstep_sector.store(sector as u8, Ordering::Relaxed);
        duties
    }

    /// Sensorless six-step: one tick of commutation + the duty→speed loop.
    /// The idle phase is sampled during the PWM on-time, where it swings
    /// about the driven-pair midpoint — see docs/SIXSTEP.md for the identity.
    fn sixstep_sensorless<const N: usize>(
        &mut self,
        sh: &Shared<N>,
        b: &mut impl MotorBoard,
        vt: [f32; 3],
    ) -> [f32; 3] {
        let spec = &sh.cfg.spec;
        let dt = sh.cfg.dt();
        let p = |id| sh.param(id);
        // Reference is the MEASURED mid-point of the two driven terminals,
        // not V_bus/2. The identity is v_f = (v_hi + v_lo)/2 + (back-EMF
        // term), and on a real bridge v_lo is not 0 — the low-side switch and
        // the shunt put it at i·(R_dson + R_shunt), 1.6 V at 1 A on the first
        // bench. Using V_bus/2 leaves that as a reference error of ~0.7 V,
        // which is half the back-EMF ramp at low speed and is what made one
        // sector parity undetectable. Measuring both driven terminals cancels
        // the drops exactly.
        let (hi_i, lo_i, fl_i) = sixstep::TABLE[self.ss_sector];
        // A driven-high terminal can clip at the divider's full scale (the
        // shields' 10k/2.2k reads at most 18.3 V): on motor 3 at an 18 V bus
        // exactly a third of the samples — the driven-high one — were
        // clipped, the midpoint sat low, crossings alternated early/late and
        // the measured speed read 5% high against the halls (session 30b).
        // The clipped terminal is at the bus, less one switch drop.
        let v_hi = if vt[hi_i as usize] >= 0.995 * spec.terminal_full_scale {
            self.vbus_filt
        } else {
            vt[hi_i as usize]
        };
        let v_ref = 0.5 * (v_hi + vt[lo_i as usize]);
        let v_float = vt[fl_i as usize];
        // `amp` is a duty ceiling in both phases: the ramp feeds forward the
        // duty its commanded speed needs, and once sensing the speed loop sets
        // it. Each branch below assigns it.
        let duty;

        if !self.ss_sensing {
            let want = self.ss_ramp.as_mut().unwrap().update(dt);
            if want != self.ss_sector {
                self.ss_sector = want;
                self.ss_zc.as_mut().unwrap().commutated();
            }
            // Watch the detector during the ramp without obeying it.
            self.ss_zc
                .as_mut()
                .unwrap()
                .update(self.ss_sector, v_float, v_ref, dt);
            let r = self.ss_ramp.as_ref().unwrap();
            // Feedforward the ramp duty instead of applying `amp` flat. A
            // fixed duty is worst at standstill, where there is no back-EMF to
            // oppose it and the whole voltage lands on the winding. Asking
            // instead for the duty that produces a chosen current — bus volts
            // to cover the back-EMF the ramp speed implies, plus i·R across
            // the two conducting phases — holds current roughly flat all the
            // way up, so `amp` becomes a ceiling the ramp rarely reaches.
            let i_ramp = p(param::IQ_LIMIT);
            let r_path = 2.0 * p(param::R);
            let ff =
                (p(param::FLUX) * r.omega_e().abs() + i_ramp * r_path) / self.vbus_filt.max(1.0);
            // max/min rather than clamp: `clamp` panics when min > max, and
            // the ceiling is below the 0.01 floor for one tick if a mode
            // arrives before its amplitude. A panic halts the firmware.
            duty = ff.max(0.01).min(self.amp.clamp(0.0, spec.max_duty));
            if r.done() {
                let zc = self.ss_zc.as_mut().unwrap();
                if !zc.locked() {
                    zc.seed(r.omega_e());
                }
                // Bumpless: start the duty loop from the ramp's duty.
                self.ss_speed.as_mut().unwrap().preload(duty);
                self.ss_target = r.omega_e();
                self.ss_sensing = true;
            }
            self.omega = r.omega_e();
        } else {
            let zc = self.ss_zc.as_mut().unwrap();
            if zc.update(self.ss_sector, v_float, v_ref, dt) == ZcEvent::Commutate {
                self.ss_sector = (self.ss_sector + 1) % sixstep::SECTORS;
            }
            let measured = zc.omega_e();
            self.omega = measured;
            // Slew the target so a retarget does not step the duty.
            let want = f32::from_bits(sh.cmd_omega.load(Ordering::Relaxed)).abs();
            let accel = p(param::OMEGA_ACCEL);
            self.ss_target += (want - self.ss_target).clamp(-accel * dt, accel * dt);
            let pi = self.ss_speed.as_mut().unwrap();
            let raw = pi.update(self.ss_target - measured, dt);
            duty = raw.max(0.01).min(self.amp.clamp(0.0, spec.max_duty));
            if raw != duty {
                // Back-calculation: a six-step bridge has no braking quadrant,
                // so the loop saturates low on every deceleration and would
                // otherwise wind up.
                pi.preload(duty);
            }
        }

        let mut duties = [0.0; 3];
        duties[hi_i as usize] = duty;
        duties[lo_i as usize] = 0.0;
        b.set_phase_enables(!(1u8 << fl_i) & 0b111);
        sh.sixstep_sector
            .store(self.ss_sector as u8, Ordering::Relaxed);
        duties
    }
}
