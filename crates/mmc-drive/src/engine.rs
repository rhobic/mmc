//! The control tick: everything the control interrupt does, against a
//! [`MotorBoard`].

use core::sync::atomic::Ordering;

use mmc_core::angle::AngleEstimator;
use mmc_core::cogging::{CogComp, Ident, Term, TERMS};
use mmc_core::estim::{RpsiAverager, RpsiCfg, RpsiEstimator};
#[cfg(feature = "fixq")]
use mmc_core::fixq::{FixHfi, FixParams};
use mmc_core::foc::{Decoupling, Foc};
use mmc_core::hall::{HallAngle, HallMap, HallSpeed, HallTracker};
use mmc_core::hfi::Tracker as HfiTracker;
use mmc_core::inverter::DeadtimeModel;
use mmc_core::math::{sin_cos, wrap_angle};
use mmc_core::observer::{FluxObserver, FluxObserverCfg};
use mmc_core::pi::{Pi, PiGains};
use mmc_core::position::TrapRef;
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
/// Speed floor of the online R/ψ estimator [rad/s el]; the i_d dither runs
/// only above it too (below, it would only heat the winding).
const ESTIM_OMEGA_MIN: f32 = 100.0;
/// Speed band [rad/s el] where the pole-pair identification collects: fast
/// enough to cross states steadily, slow enough that state times are many
/// control ticks long and the position torque shows in the speed.
const COG_ID_OMEGA: (f32, f32) = (50.0, 600.0);
/// Above this speed [rad/s el] the position-torque feed-forward is off: it
/// measured neutral on motor 3 from 400 rad/s el up (testresults/motor3-cogff).
const COG_FF_OMEGA_MAX: f32 = 500.0;
/// Slowest rotor [rad/s el] a drive start catches on the fly; below it the
/// rotor is treated as at rest and the mode starts as it always has.
const CATCH_OMEGA_MIN: f32 = 30.0;
/// HFI sensorless start (`hfi_v` > 0): time for the tracker to lock, and
/// the pulse current along ±d. Saturation makes the +d (magnet north) pulse
/// answer the carrier more strongly (motor 3: 2-20 % at ±0.7 A). The pulse
/// length and the number of +d/−d pairs are params (`hfi_pol_s`,
/// `hfi_pol_n`); the first half of each pulse settles unrecorded.
///
/// One sign of pulse always pushes against the magnet: an unstable
/// equilibrium that turns the rotor half a revolution if it lasts. Motor 3
/// at 0.6 A: offsets grow with τ = 1/√(1.5·p²·ψ·i/J) ≈ 4 ms, so 25 ms pulses
/// flipped it back and forth (1–3 el revs per start) and tripped the 1.5 A
/// limit in ~1 start in 10 (session 39). Pulses of a few τ cannot.
const HFI_LOCK_S: f32 = 0.3;
/// The lock's d bias ramps in over this long. Stepped on, with the tracker
/// free to build speed, the rotor swinging onto the bias dragged the
/// tracker, which dragged the bias: in ~1 start in 11 the pair ran away
/// into a full electrical revolution or more inside the lock, once into the
/// overcurrent limit (session 41). The tracker's speed is also held at zero
/// until the run phase: the rotor is meant to be still.
const HFI_LOCK_RAMP_S: f32 = 0.15;

const HFI_POL_A: f32 = 0.6;
/// HFI lock-up: q current at 90 % of its limit with the tracker below half
/// the reference (at least [`FRIC_OMEGA`]) for this long, net of the ticks it
/// was not (tracker noise at a few rad/s must not reset it), trips a stall. Cross-saturation
/// drags the HFI angle with q current (motor 3: about −25° el/A, running
/// away past ~0.6 A); far enough off, q current makes no torque and the speed
/// loop winds to its limit holding a stopped rotor (session 39).
const HFI_STUCK_S: f32 = 0.5;
/// Below this fraction of `sl_handoff` a sensorless drive running on the
/// flux observer hands back to HFI (`hfi_v` > 0); hysteresis against the
/// handover at `sl_handoff`.
const HFI_HANDBACK: f32 = 0.6;
/// Speed [rad/s el] over which the friction feed-forward (`sl_fric`) fades
/// in from zero, so it changes sign smoothly through a zero crossing.
const FRIC_OMEGA: f32 = 5.0;

/// Phases of the HFI sensorless start.
#[derive(Copy, Clone, Debug, PartialEq)]
enum HfiPhase {
    Lock,
    PolPos,
    PolNeg,
    Run,
}

#[derive(Copy, Clone, Debug)]
struct HfiStart {
    phase: HfiPhase,
    ticks: u32,
    acc: [f32; 2],
    n: [u32; 2],
    pairs: u32,
}

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
    // Hall position mode: measured position (electrical, unwrapped from the
    // hall angle), the trapezoidal reference, and the PID integrator.
    tracker: Option<HallTracker>,
    /// Online R/ψ estimate in the closed-loop FOC modes.
    rpsi: Option<RpsiAverager>,
    /// i_d dither: time into the current level, and which level.
    dither_t: f32,
    dither_hi: bool,
    /// Hall states counted since power-up, the pole-pair identification and
    /// the position-torque feed-forward (boards with halls).
    cog: Option<CogComp>,
    /// The shift last published to `cog_shift`.
    cog_published: Option<u8>,
    /// Hall angle of the last tick (boards with halls; every mode).
    hall_last: Option<f32>,
    /// High-frequency injection tracker (`hfi_v` > 0), and its error against
    /// the hall angle sampled at the last hall edge (where the halls are
    /// exact), for scoring it.
    hfi: Option<HfiTracker>,
    /// Sensorless start on HFI (`hfi_v` > 0): lock, polarity, run, then
    /// hand over to the flux observer. `None` once handed over.
    hfi_start: Option<HfiStart>,
    hfi_edge_err: f32,
    hfi_stuck: u32,
    /// The integer HFI drive (feature `fixq`), replacing the float HFI path.
    #[cfg(feature = "fixq")]
    fixq: Option<(FixHfi, FixParams)>,
    hall_state_prev: Option<u8>,
    /// Measured stator current of the last tick, αβ: a flying start reads
    /// the torque current the rotor was carrying in the rotor's own frame
    /// (the outgoing mode's i_q may be in a forced frame).
    i_ab_last: AlphaBeta,
    /// Angle and speed [rad, rad/s el] the position-torque feed-forward is
    /// placed by: the hall tracker's, which predicts between edges with the
    /// known torque (the plain hall interpolation assumes constant speed and
    /// is several degrees off while the rotor hunts).
    ff_theta: Option<(f32, f32)>,
    /// The position torque as cancelling i_q [A], held between updates: at
    /// the tracker's angle (its model) and advanced for the current loop
    /// (the feed-forward). Each is re-evaluated every other tick, in turn,
    /// so a tick pays for one series (the F302's interrupt budget).
    cog_now: f32,
    cog_ff: f32,
    cog_odd: bool,
    pos_origin: Option<f32>,
    pos_meas: f32,
    pos_ref: TrapRef,
    pos_int: f32,
    /// Measured i_q of the previous tick: the torque the tracker's model
    /// integrates.
    iq_last: f32,
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
            tracker: None,
            rpsi: None,
            dither_t: 0.0,
            dither_hi: false,
            cog: None,
            cog_published: None,
            hall_last: None,
            hfi: None,
            hfi_start: None,
            hfi_edge_err: 0.0,
            hfi_stuck: 0,
            #[cfg(feature = "fixq")]
            fixq: None,
            hall_state_prev: None,
            i_ab_last: AlphaBeta {
                alpha: 0.0,
                beta: 0.0,
            },
            ff_theta: None,
            cog_now: 0.0,
            cog_ff: 0.0,
            cog_odd: false,
            pos_origin: None,
            pos_meas: 0.0,
            pos_ref: TrapRef { pos: 0.0, vel: 0.0 },
            pos_int: 0.0,
            iq_last: 0.0,
        }
    }

    /// Calibrated zero-current amplifier outputs [V] (diagnostic).
    /// The hall-state counter and position-torque identification (boards
    /// with halls), for diagnostics.
    pub fn cog(&self) -> Option<&CogComp> {
        self.cog.as_ref()
    }

    /// The angle [rad el, unwrapped] the position-torque feed-forward used
    /// last tick (the hall tracker's), if any.
    pub fn ff_angle(&self) -> Option<f32> {
        self.ff_theta.map(|t| t.0)
    }

    /// The drive's electrical angle [rad] as of the last tick.
    pub fn theta(&self) -> f32 {
        self.theta
    }

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
                // A persisted shift belongs to a count that restarted.
                sh.params[param::COG_SHIFT as usize].store((-1.0f32).to_bits(), Ordering::Relaxed);
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
        self.hall_last = hall_ref;
        let cog_terms = cog_terms(&p);
        let kt = 1.5 * p(param::POLE_PAIRS) * p(param::FLUX);
        if spec.has_halls {
            let pp = p(param::POLE_PAIRS).max(1.0) as u8;
            let cog = match self.cog {
                Some(ref mut cog) => cog,
                None => {
                    // First tick after calibration. The hall angle is tracked
                    // from here on, not from the first drive build: a first
                    // start on a rotor that is already turning (a reset while
                    // it coasts) needs it to catch the rotor.
                    let map = hall_map(&p);
                    self.hall_angle = Some(HallAngle::new(map));
                    self.cog.insert(CogComp::new(&map, pp))
                }
            };
            // Collect torque samples only in steady hall FOC: the energy
            // balance needs a speed loop holding the rotor near a setpoint.
            // Gated on the setpoint, not the measured speed: a hunting rotor
            // is slowest exactly where the torque is largest (motor 3: 27
            // rad/s el at its worst state for a 100 setpoint), and gating
            // on speed starved that boundary forever.
            let collect = self.mode == mode::HALL_FOC
                && sh.state.load(Ordering::Relaxed) == ST_RUN
                && (COG_ID_OMEGA.0..COG_ID_OMEGA.1).contains(&self.omega_ref_cur.abs())
                && self.omega_ref_cur == f32::from_bits(sh.cmd_omega.load(Ordering::Relaxed));
            let lost = cog.tick(
                hall_state,
                self.iq_last,
                dt,
                collect,
                kt,
                p(param::INERTIA),
                &cog_terms,
            );
            let host = p(param::COG_SHIFT);
            if lost {
                self.cog_published = None;
                sh.params[param::COG_SHIFT as usize].store((-1.0f32).to_bits(), Ordering::Relaxed);
            } else if host >= 0.0 {
                // Forced, or ours echoed back.
                let s = (host as u8) % pp.max(1);
                if cog.shift != Some(s) {
                    cog.shift = Some(s);
                    cog.ident = Ident::Found;
                }
                self.cog_published = Some(s);
            } else if self.cog_published.is_some() {
                // The host cleared it: identify afresh.
                self.cog_published = None;
                cog.reset_ident();
            } else if let Some(s) = cog.shift {
                self.cog_published = Some(s);
                sh.params[param::COG_SHIFT as usize].store((s as f32).to_bits(), Ordering::Relaxed);
            }
        }

        // --- pick up new host commands.
        let epoch = sh.cmd_epoch.load(Ordering::Acquire);
        if epoch != self.epoch_seen {
            self.epoch_seen = epoch;
            self.command(sh, b);
        }
        let (cog_now, cog_unit) = self.cog_unit(&p, &cog_terms, kt, dt);

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
        let mut hfi_out: Option<(f32, f32)> = None;

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

            let mut v_ab = if self.mode == mode::PROBE
                && sh.probe_kind.load(Ordering::Relaxed) == test::HFI_SWEEP
            {
                // HFI sweep (`mmc_core::hfi`): optional align, a short
                // release, then a ±V_h carrier along each test angle, the
                // response demodulated and accumulated into the burst.
                use mmc_core::hfi;
                self.probe_ticks += 1;
                let v_h = f32::from_bits(sh.probe_v_step.load(Ordering::Relaxed));
                let align_at = f32::from_bits(sh.probe_v_align.load(Ordering::Relaxed));
                let aligning = align_at.abs() <= 7.0;
                let (t_align, t_rel) = if aligning {
                    (cfg.ticks(hfi::ALIGN_S), cfg.ticks(hfi::RELEASE_S))
                } else {
                    (0, 0)
                };
                // Safety: the host only reads once DONE.
                let buf = unsafe { &mut *sh.burst.0.get() };
                if self.probe_ticks <= t_align {
                    // Park the rotor: a DC current vector at the align angle.
                    let sc = sin_cos(align_at);
                    i_dq = park(i_ab, sc);
                    v_dq = Dq {
                        d: hfi::ALIGN_A * p(param::R),
                        q: 0.0,
                    };
                    let v_ab = inverse_park(v_dq, sc);
                    duties = svpwm(v_ab, vbus);
                    v_ab
                } else if self.probe_ticks <= t_align + t_rel {
                    // Released: zero volts while the align current decays.
                    duties = svpwm(AlphaBeta::default(), vbus);
                    AlphaBeta::default()
                } else {
                    let t = (self.probe_ticks - t_align - t_rel - 1) as usize;
                    if t == 0 {
                        buf[..hfi::LEN].fill(0.0);
                        let hdr = [
                            test::HFI_SWEEP as f32,
                            hfi::ANGLES as f32,
                            hfi::CYCLES as f32,
                            hfi::DWELL as f32,
                            hfi::SKIP as f32,
                            v_h,
                            if aligning { align_at } else { hfi::NONE },
                            self.hall_last.unwrap_or(hfi::NONE),
                            spec.ctrl_hz as f32,
                            hfi::samples_per_bin() as f32,
                        ];
                        buf[..hfi::HDR].copy_from_slice(&hdr);
                    }
                    if t >= hfi::TICKS {
                        sh.burst_len.store(hfi::LEN as u32, Ordering::Relaxed);
                        self.finish_probe(sh, b);
                        AlphaBeta::default()
                    } else {
                        let (k, sign, w, rec) = hfi::schedule(t);
                        let sc = sin_cos(hfi::angle(k));
                        i_dq = park(i_ab, sc);
                        if rec {
                            // The change since the last sample, in this
                            // angle's frame (the visit's first ticks settle
                            // unrecorded, so the last sample was this angle).
                            let before = park(self.i_ab_last, sc);
                            let at = hfi::HDR + k * hfi::PER_ANGLE;
                            buf[at] += i_dq.d;
                            buf[at + 1] += i_dq.q;
                            buf[at + 2] += w * (i_dq.d - before.d);
                            buf[at + 3] += w * (i_dq.q - before.q);
                        }
                        // `id_inject` doubles as a DC bias along each test
                        // axis (as volts over R): saturation then lowers L
                        // toward the magnet's north only, which puts a 1θ
                        // term (the polarity) beside the 2θ one.
                        v_dq = Dq {
                            d: sign * v_h + p(param::ID_INJECT) * p(param::R),
                            q: 0.0,
                        };
                        let v_ab = inverse_park(v_dq, sc);
                        duties = svpwm(v_ab, vbus);
                        v_ab
                    }
                }
            } else if self.mode == mode::PROBE
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
            } else if matches!(self.mode, mode::HALL_FOC | mode::SS_HALL | mode::HALL_POS) {
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
                } else if self.mode == mode::HALL_POS {
                    // Position loop on the hall tracker: every hall edge is
                    // an exact position update, and between edges the
                    // rotor is predicted from the commanded torque and the
                    // inertia — so the D term has a velocity where the
                    // edge-timed speed is stale or zero. Without it this
                    // loop limit-cycled ±10° mechanical in the sim. The
                    // target arrives in mechanical radians.
                    let pp = p(param::POLE_PAIRS).max(1.0);
                    if let Some((pos, w)) = hall_state
                        .and_then(|h| self.tracker.as_mut()?.update(h, self.iq_last - cog_now, dt))
                    {
                        // Positions are relative to where the drive started.
                        if self.pos_origin.is_none() {
                            self.pos_origin = Some(pos);
                        }
                        self.pos_meas = pos - self.pos_origin.unwrap_or(0.0);
                        self.omega = w;
                        self.ff_theta = Some((pos, w));
                    }
                    let measured = self.pos_meas;
                    let target = omega_target * pp;
                    self.pos_ref
                        .update(target, p(param::POS_VMAX), p(param::OMEGA_ACCEL), dt);
                    let err = self.pos_ref.pos - measured;
                    let auth = amp_target.abs().clamp(0.05, p(param::IQ_LIMIT));
                    let ki = p(param::POS_KI);
                    // Integrate only outside half a hall sector of the
                    // reference: inside it the position is not measurable,
                    // and integrating there against stiction (motor 3 breaks
                    // away at ~0.22 A but runs at ~0.11 A) is what made the
                    // hold hunt ±27°. Frozen, not cleared: it keeps whatever
                    // holding torque it built.
                    //
                    // And only while the reference holds still: during a move
                    // P + D + the friction feedforward carry it, and
                    // integrating the move's tracking lag wound up into ~20°
                    // overshoots and, on a full turn, a stick-slip cycle
                    // larger than a sector.
                    let band = core::f32::consts::FRAC_PI_6 + p(param::HALL_HYST);
                    if self.pos_ref.vel == 0.0 && err.abs() > band {
                        self.pos_int += err * dt;
                    }
                    if ki > 0.0 {
                        self.pos_int = self.pos_int.clamp(-auth / ki, auth / ki);
                    }
                    // Running friction fed forward while the reference moves.
                    let fric = if self.pos_ref.vel.abs() > 1e-3 {
                        p(param::I_FRIC) * self.pos_ref.vel.signum()
                    } else {
                        0.0
                    };
                    iq_ref = (p(param::POS_KP) * err
                        + ki * self.pos_int
                        + p(param::POS_KD) * (self.pos_ref.vel - self.omega)
                        + fric)
                        .clamp(-auth, auth);
                    iq_ref = (iq_ref + p(param::COG_FF) * cog_unit)
                        .clamp(-p(param::IQ_LIMIT), p(param::IQ_LIMIT));
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
                    if let Some(h) = hall_state {
                        self.ff_theta = self
                            .tracker
                            .as_mut()
                            .and_then(|t| t.update(h, self.iq_last - cog_now, dt));
                    }
                    iq_ref = (iq_ref + p(param::COG_FF) * cog_unit)
                        .clamp(-p(param::IQ_LIMIT), p(param::IQ_LIMIT));
                    // An injected i_d (normally 0) makes R observable to an
                    // online estimator; no torque on a surface magnet. The
                    // dither steps it so R shows as a slope, which voltage
                    // offsets cannot fake (docs/CALIBRATION.md).
                    let lim = p(param::IQ_LIMIT);
                    let mut id_ref = p(param::ID_INJECT);
                    let dither = p(param::ID_DITHER);
                    if dither != 0.0 && self.omega.abs() >= ESTIM_OMEGA_MIN {
                        let half = 0.5 * p(param::ID_DITHER_PERIOD);
                        self.dither_t += dt;
                        if self.dither_t >= half {
                            self.dither_t -= half;
                            self.dither_hi = !self.dither_hi;
                        }
                        if self.dither_hi {
                            id_ref += dither;
                        }
                    } else {
                        self.dither_t = 0.0;
                        self.dither_hi = false;
                    }
                    let id_ref = id_ref.clamp(-lim, lim);
                    let out = self.foc.as_mut().unwrap().step(
                        i_abc,
                        self.theta,
                        self.omega,
                        Dq {
                            d: id_ref,
                            q: iq_ref,
                        },
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
            } else if self.mode == mode::SENSORLESS && self.fixq_active() {
                let o = self.fixq_tick(b, i_abc, vbus, omega_target);
                duties = o.duties;
                v_dq = o.v_dq;
                i_dq = o.i_dq;
                iq_ref = o.iq_ref;
                hfi_out = Some((o.theta_tr, o.omega));
                o.v_ab
            } else if self.mode == mode::SENSORLESS && self.hfi_start.is_some() {
                // Sensorless from standstill on high-frequency injection:
                // the tracker owns the angle (one tick old; it updates in
                // the injection hook below), the start sequence the d-axis
                // current, the speed loop q once running.
                let tr = *self.hfi.as_ref().unwrap();
                let w = tr.omega();
                let st = self.hfi_start.as_mut().unwrap();
                st.ticks += 1;
                let pol_a = HFI_POL_A.min(0.5 * spec.i_trip);
                let mut iq_cmd = 0.0;
                let id_cmd = match st.phase {
                    HfiPhase::Lock => {
                        if st.ticks >= cfg.ticks(HFI_LOCK_S) {
                            st.phase = HfiPhase::PolPos;
                            st.ticks = 0;
                        }
                        // The d bias (`id_inject`) keeps the phase currents
                        // off zero, where the dead time would flip with the
                        // carrier and swamp it. Off the axis it is torque,
                        // so the rotor turns a little toward the estimate as
                        // the estimate turns onto the rotor; ramped, so it
                        // settles rather than runs away (HFI_LOCK_RAMP_S).
                        p(param::ID_INJECT)
                            * (st.ticks as f32 / cfg.ticks(HFI_LOCK_RAMP_S).max(1) as f32).min(1.0)
                    }
                    HfiPhase::PolPos | HfiPhase::PolNeg => {
                        let k = (st.phase == HfiPhase::PolNeg) as usize;
                        let pulse = cfg.ticks(p(param::HFI_POL_S));
                        if let Some(d) = tr.d_fresh.filter(|_| 2 * st.ticks > pulse) {
                            st.acc[k] += d.abs();
                            st.n[k] += 1;
                        }
                        if st.ticks >= pulse {
                            st.ticks = 0;
                            if k == 0 {
                                st.phase = HfiPhase::PolNeg;
                            } else if st.pairs + 1 < p(param::HFI_POL_N) as u32 {
                                st.pairs += 1;
                                st.phase = HfiPhase::PolPos;
                            } else {
                                // The axis the tracker found is the magnet's
                                // south if −d answered the stronger.
                                let pos = st.acc[0] / st.n[0].max(1) as f32;
                                let neg = st.acc[1] / st.n[1].max(1) as f32;
                                if neg > pos {
                                    self.hfi.as_mut().unwrap().flip();
                                }
                                st.phase = HfiPhase::Run;
                            }
                        }
                        if k == 0 {
                            pol_a
                        } else {
                            -pol_a
                        }
                    }
                    HfiPhase::Run => {
                        let accel = p(param::OMEGA_ACCEL);
                        let d = (omega_target - self.omega_ref_cur).clamp(-accel * dt, accel * dt);
                        self.omega_ref_cur += d;
                        let lim = p(param::IQ_LIMIT);
                        iq_cmd = (self
                            .speed
                            .as_mut()
                            .unwrap()
                            .update(self.omega_ref_cur, w, dt)
                            + fric_ff(p(param::SL_FRIC), self.omega_ref_cur))
                        .clamp(-lim, lim);
                        let stuck = iq_cmd.abs() >= 0.9 * lim
                            && w.abs() < 0.5 * self.omega_ref_cur.abs().max(FRIC_OMEGA);
                        self.hfi_stuck = if stuck {
                            self.hfi_stuck + 1
                        } else {
                            self.hfi_stuck.saturating_sub(1)
                        };
                        // Fast enough for the flux observer, which has been
                        // integrating all along: hand over, stop injecting.
                        if w.abs() >= p(param::SL_HANDOFF) && w * omega_target > 0.0 {
                            if let Some(q) = self.seq.as_mut() {
                                q.start_closed();
                            }
                            self.speed.as_mut().unwrap().set_gains(PiGains {
                                kp: p(param::SPEED_KP),
                                ki: p(param::SPEED_KI),
                            });
                            self.hfi_start = None;
                            self.hfi = None;
                        }
                        p(param::ID_INJECT)
                    }
                };
                iq_ref = iq_cmd;
                // Cross-saturation: q current turns the saliency axis the
                // tracker locks onto; the rotor's d axis is that plus
                // `hfi_xsat`·i_q. The carrier stays on the tracker's own axis.
                let th = wrap_angle(tr.theta() + w * dt + p(param::HFI_XSAT) * iq_cmd);
                // Locking and polarity happen on a rotor meant to stand
                // still: the tracker's speed is noise there, and fed to the
                // FOC's back-EMF and cross-coupling feedforward it becomes q
                // voltage, torque and motion.
                let w_ff = if self
                    .hfi_start
                    .as_ref()
                    .is_some_and(|s| s.phase != HfiPhase::Run)
                {
                    0.0
                } else {
                    w
                };
                self.theta = th;
                self.omega = w;
                let out = self.foc.as_mut().unwrap().step(
                    i_abc,
                    th,
                    w_ff,
                    Dq {
                        d: id_cmd,
                        q: iq_cmd,
                    },
                    vbus,
                    dt,
                );
                duties = out.duties;
                v_dq = out.v_dq;
                i_dq = out.i_dq;
                out.v_ab
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
                // Slow enough that the observer is losing the rotor: hand
                // back to HFI (from next tick), starting the tracker on the
                // observer's angle and speed — the polarity is known, so no
                // lock and no pulses. Hysteresis against the handover.
                if seq_out.phase == Phase::Closed
                    && p(param::HFI_V) > 0.0
                    && !self.fixq_active()
                    && seq_out.omega.abs() < HFI_HANDBACK * p(param::SL_HANDOFF)
                {
                    self.hfi_handback(&p, self.theta, self.omega, iq_ref);
                }
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
            // High-frequency injection: track the rotor's axis from its
            // saliency, the carrier riding on whatever the mode applied. In
            // the hall modes this is a shadow, scored at hall edges.
            let hfi_mode = matches!(self.mode, mode::HALL_FOC | mode::HALL_POS)
                || (self.mode == mode::SENSORLESS && self.hfi_start.is_some());
            if let Some(tr) = self.hfi.as_mut().filter(|_| hfi_mode) {
                let (th, w) = tr.update(i_ab, dt);
                // Locking and polarity: the rotor should be still, and a
                // tracker free to build speed can co-rotate with it.
                let (th, w) = if self
                    .hfi_start
                    .as_ref()
                    .is_some_and(|s| s.phase != HfiPhase::Run)
                {
                    tr.set_omega(0.0);
                    (th, 0.0)
                } else {
                    (th, w)
                };
                let inj = inverse_park(
                    Dq {
                        d: tr.carrier(),
                        q: 0.0,
                    },
                    sin_cos(th),
                );
                v_ab = AlphaBeta {
                    alpha: v_ab.alpha + inj.alpha,
                    beta: v_ab.beta + inj.beta,
                };
                // Re-modulating here must keep the FOC's dead-time
                // compensation, which its duties carried and `v_ab` (the
                // voltage that lands, for the observer) does not. Until
                // session 41 it was dropped whenever HFI ran.
                let mut v_mod = v_ab;
                if let Some(m) = self
                    .foc
                    .as_ref()
                    .and_then(|f| f.deadtime)
                    .filter(|m| !m.is_ideal())
                {
                    let c = m.error_ab(i_abc);
                    v_mod.alpha += c.alpha;
                    v_mod.beta += c.beta;
                }
                duties = svpwm(v_mod, vbus);
                hfi_out = Some((th, w));
            }
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
                    && self.hfi_start.is_none()
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
                if self.hfi_stuck >= cfg.ticks(HFI_STUCK_S) {
                    self.hfi_stuck = 0;
                    self.trip(sh, b, ST_STALL);
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
        if let Some(e) = self.rpsi.as_mut() {
            let running = sh.state.load(Ordering::Relaxed) == ST_RUN
                && matches!(self.mode, mode::HALL_FOC | mode::SENSORLESS);
            e.push(v_dq, i_dq, self.omega, cfg.dt(), running);
            put(channel::R_HAT, e.est.r());
            put(channel::PSI_HAT, e.est.psi());
        } else {
            put(channel::R_HAT, 0.0);
            put(channel::PSI_HAT, 0.0);
        }
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
            // On HFI (its start, or handed back from the observer).
            (mode::SENSORLESS, _) if self.hfi_start.is_some() => ST_SL_RAMP,
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
        if let Some((th, w)) = hfi_out {
            // A hall edge is where the hall angle is exact: score the HFI
            // axis (mod π) there and hold it until the next edge.
            if let (Some(h), Some(prev), Some(href)) =
                (hall_state, self.hall_state_prev, self.hall_last)
            {
                if h != prev {
                    self.hfi_edge_err = 0.5 * wrap_angle(2.0 * (th - href));
                }
            }
            theta_est = th;
            omega_est = w;
            theta_err = self.hfi_edge_err;
        }
        self.hall_state_prev = hall_state;
        put(
            channel::HFI_D,
            self.hfi
                .as_ref()
                .filter(|_| hfi_out.is_some())
                .map_or(0.0, |t| t.d_amp),
        );
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
        let pp = p(param::POLE_PAIRS).max(1.0);
        let in_pos = self.mode == mode::HALL_POS;
        put(
            channel::POS_M,
            if in_pos { self.pos_meas / pp } else { 0.0 },
        );
        self.iq_last = i_dq.q;
        self.i_ab_last = clarke(i_abc);
        put(
            channel::POS_REF,
            if in_pos { self.pos_ref.pos / pp } else { 0.0 },
        );
        put(channel::HALL, hall_state.unwrap_or(0) as f32);
        put(channel::OMEGA_HALL, self.hall.omega());
        sh.telem_seq.store(seq.wrapping_add(2), Ordering::Release);

        let dur = b.cycles().wrapping_sub(t0);
        sh.isr_max_cycles.fetch_max(dur, Ordering::Relaxed);
    }

    /// The i_q [A] that cancels the position torque at the tracker's angle,
    /// and at that angle advanced by the current loop's lag (its bandwidth)
    /// plus a control period; zeros before the pole pair is known. One
    /// series pass a tick: the tracker's model subtracts the first from the
    /// measured current (the torque acts on the rotor whether or not it is
    /// cancelled), the feed-forward adds `cog_ff` times the second.
    fn cog_unit(
        &mut self,
        p: &impl Fn(u8) -> f32,
        terms: &[Term; TERMS],
        kt: f32,
        dt: f32,
    ) -> (f32, f32) {
        let active = self.cog.as_ref().is_some_and(|c| c.shift.is_some())
            && matches!(self.mode, mode::HALL_FOC | mode::HALL_POS)
            // Above this the inertia filters the position torque and the
            // feed-forward measured neutral on the bench; skip it there
            // (and its cycles, where the interrupt is busiest).
            && self.omega.abs() <= COG_FF_OMEGA_MAX;
        if !active {
            (self.cog_now, self.cog_ff) = (0.0, 0.0);
            return (0.0, 0.0);
        }
        let c = self.cog.as_ref().unwrap();
        let (th, w) = self.ff_theta.unwrap_or((self.theta, self.omega));
        self.cog_odd = !self.cog_odd;
        if self.cog_odd {
            // Held for two ticks: lead by the current loop's lag, one
            // period of PWM latency and the extra period it is held.
            let lead = 1.0 / p(param::CUR_BW).max(1.0) + 2.0 * dt;
            self.cog_ff = c.feedforward(terms, th + w * lead, kt);
        } else {
            self.cog_now = c.feedforward(terms, th + w * dt, kt);
        }
        (self.cog_now, self.cog_ff)
    }

    /// The rotor's electrical angle and speed if it is turning: from the
    /// halls when the board has them (they run in every mode, from Off too)
    /// at [`CATCH_OMEGA_MIN`] and up, else from the flux observer if the
    /// outgoing mode fed it, it is locked (flux within ±50 % of the profile)
    /// and the rotor is at `obs_min` or faster (half the sensorless handoff:
    /// slower, the observer's angle is not to be trusted). The flag says the
    /// observer was the source.
    fn catch_rotor(&self, flux: f32, obs_min: f32) -> Option<(f32, f32, bool)> {
        if let Some(ha) = self.hall_angle.as_ref() {
            let w = ha.omega();
            return match self.hall_last {
                Some(th) if w.abs() >= CATCH_OMEGA_MIN => Some((th, w, false)),
                // The halls say it is slow or at rest: believe them.
                _ => None,
            };
        }
        let fed = matches!(
            self.mode,
            mode::VOLT | mode::IF | mode::SENSORLESS | mode::HALL_FOC | mode::HALL_POS
        );
        let o = self.obs.as_ref().filter(|_| fed)?;
        let w = o.electrical_velocity();
        let locked = (0.5 * flux..1.5 * flux).contains(&o.flux_mag());
        (w.abs() >= CATCH_OMEGA_MIN.max(obs_min) && locked).then(|| (o.electrical_angle(), w, true))
    }

    /// Start `mode` on a rotor already at electrical angle `th` and speed
    /// `w` (after `clean_start` built its blocks for a start from rest).
    fn seed_flying<const N: usize>(
        &mut self,
        sh: &Shared<N>,
        mode: u8,
        th: f32,
        w: f32,
        from_obs: bool,
        old_obs: Option<FluxObserver>,
    ) {
        let p = |id| sh.param(id);
        let flux = p(param::FLUX);
        // The observer: keep the locked one, or prime the fresh one at the
        // halls' angle so it is locked from the first tick.
        if from_obs {
            self.obs = old_obs;
        } else if let Some(o) = self.obs.as_mut() {
            o.prime(th, w, flux, AlphaBeta::default());
        }
        let target = f32::from_bits(sh.cmd_omega.load(Ordering::Relaxed));
        // The torque current the rotor carried, on its own q axis.
        let iq_rotor = park(self.i_ab_last, sin_cos(th)).q;
        match mode {
            mode::SENSORLESS => {
                // Same direction and fast enough for the observer: no I-f
                // ramp from standstill under a turning rotor, closed loop at
                // once, speed reference from where the rotor is.
                let fast = w.abs() >= 0.5 * p(param::SL_HANDOFF);
                if fast && w * target > 0.0 {
                    if let Some(q) = self.seq.as_mut() {
                        q.start_closed();
                    }
                    self.theta = th;
                    self.omega = w;
                    self.omega_ref_cur = w;
                    self.sl_preload = iq_rotor;
                }
            }
            mode::HALL_FOC => {
                // The reference ramps from the rotor's speed, not from rest
                // (which would brake it first), with the current it carried.
                self.omega_ref_cur = w;
                if let Some(s) = self.speed.as_mut() {
                    s.preload(iq_rotor);
                }
            }
            mode::IF => {
                // The forced frame where steady I-f would hold this rotor:
                // the current mostly on its d axis, leaning toward q only by
                // the torque it was carrying (the hang angle), at the
                // commanded amplitude at once. Full current on q would be a
                // torque step; slewing up from zero lets the rotor slip out
                // of the frame before the current can hold it.
                let amp = f32::from_bits(sh.cmd_amp.load(Ordering::Relaxed));
                let frac = (iq_rotor / amp.abs().max(1e-3)).clamp(-1.0, 1.0);
                // Along +d in either direction of travel: that is the stable
                // hang (the rotor's d axis follows the current vector); −d
                // would put the full current against the magnet.
                let lean = mmc_core::math::sqrt(1.0 - frac * frac);
                // Current angle in the rotor frame, then the forced frame
                // whose q axis carries it.
                let phi = mmc_core::math::atan2(frac, lean);
                self.theta = wrap_angle(th + phi - core::f32::consts::FRAC_PI_2);
                self.omega = w;
                self.amp = amp;
            }
            mode::VOLT => {
                // The voltage vector on the back-EMF (a quarter turn ahead
                // of the rotor flux in the direction of travel) at its
                // magnitude: no current at entry. Starting from zero volts
                // would short the windings against the back-EMF.
                self.theta = wrap_angle(th + core::f32::consts::FRAC_PI_2 * w.signum());
                self.omega = w;
                self.amp = flux * w.abs();
            }
            _ => {}
        }
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
            // Only a start from Off clears the overcurrent debounce: a live
            // switch keeps a pending strike, so the 2-strike debounce spans
            // the switch transient (review follow-up, origin `5bc16cd`).
            if self.mode == mode::OFF {
                self.oc_strikes = 0;
                // The same rule for the board's latched driver fault.
                b.clear_driver_fault();
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
        // Flying start: where the rotor is and how fast it turns, before
        // the resets below forget it. `self.mode` is still the outgoing mode.
        let catch = self.catch_rotor(p(param::FLUX), 0.5 * p(param::SL_HANDOFF));
        let old_obs = self.obs;
        self.theta = 0.0;
        self.omega = 0.0;
        self.amp = 0.0;
        self.v_applied = AlphaBeta::default();
        self.v_applied2 = AlphaBeta::default();
        self.stall_strikes = 0;
        self.hfi_stuck = 0;
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
        } else if mode == mode::HALL_POS {
            self.foc = Some(Foc::with_feedforward(
                gains,
                Decoupling {
                    ld: ls,
                    lq: ls,
                    flux: p(param::FLUX),
                },
            ));
            self.seq = None;
            self.speed = None;
            self.sl_preload = 0.0;
            // Electrical acceleration per amp: 1.5·p²·ψ / J.
            let pp = p(param::POLE_PAIRS).max(1.0);
            let g = 1.5 * pp * pp * p(param::FLUX) / p(param::INERTIA);
            self.tracker = Some(HallTracker::new(hall_map(&p), g));
            self.pos_origin = None;
            self.pos_meas = 0.0;
            self.pos_ref = TrapRef::new(0.0);
            self.pos_int = 0.0;
            self.iq_last = 0.0;
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
        if let Some((th, w, from_obs)) = catch {
            self.seed_flying(sh, mode, th, w, from_obs, old_obs);
        }
        // Every drive on a board with halls carries the calibrated hall
        // angle: the hall modes run on it, the rest are scored against it.
        let map = hall_map(&p);
        // Keep the speed and angle the halls have been tracking (they run in
        // every mode, from boot): a drive started on a turning rotor needs
        // them from its first tick.
        if let Some(h) = self.hall_angle.as_mut() {
            h.retune(map);
        }
        self.hall.widths = map.widths;
        if let Some(c) = self.cog.as_mut() {
            c.retune(&map, p(param::POLE_PAIRS).max(1.0) as u8);
        }
        self.ff_theta = None;
        // HFI: a shadow in the hall modes; in sensorless, the start from
        // standstill unless a flying start already caught the rotor.
        let sl_hfi = mode == mode::SENSORLESS
            && self
                .seq
                .as_ref()
                .is_some_and(|q| q.phase() != Phase::Closed);
        let hfi_on =
            p(param::HFI_V) > 0.0 && (matches!(mode, mode::HALL_FOC | mode::HALL_POS) || sl_hfi);
        self.hfi = hfi_on.then(|| {
            HfiTracker::new(
                p(param::HFI_V),
                p(param::HFI_XI),
                p(param::HFI_BW),
                self.hall_last.filter(|_| !sl_hfi).unwrap_or(0.0),
            )
            .with_spread(p(param::HFI_SPREAD) as u8)
        });
        self.hfi_start = (hfi_on && sl_hfi).then_some(HfiStart {
            phase: HfiPhase::Lock,
            ticks: 0,
            acc: [0.0; 2],
            n: [0; 2],
            pairs: 0,
        });
        if self.hfi_start.is_some() {
            // The speed reference ramps from rest, not from the I-f handoff.
            self.omega_ref_cur = 0.0;
            self.sl_preload = 0.0;
            // HFI's own speed gains (0 = the shared ones) until the handover
            // to the observer: a stiff loop is what beats stiction at a few
            // rad/s, and the shared gains also serve the observer and halls.
            if p(param::HFI_KP) > 0.0 {
                if let Some(s) = self.speed.as_mut() {
                    s.set_gains(PiGains {
                        kp: p(param::HFI_KP),
                        ki: p(param::HFI_KI),
                    });
                }
            }
        }
        #[cfg(feature = "fixq")]
        {
            self.fixq = self.hfi_start.is_some().then(|| {
                let hfi_gains = p(param::HFI_KP) > 0.0;
                let fp = FixParams {
                    i_base: 2.0 * spec.i_trip,
                    v_base: 32.0,
                    dt: sh.cfg.dt(),
                    r: p(param::R),
                    l: p(param::L),
                    flux: p(param::FLUX),
                    cur_bw: p(param::CUR_BW),
                    speed_kp: p(if hfi_gains {
                        param::HFI_KP
                    } else {
                        param::SPEED_KP
                    }),
                    speed_ki: p(if hfi_gains {
                        param::HFI_KI
                    } else {
                        param::SPEED_KI
                    }),
                    iq_limit: p(param::IQ_LIMIT),
                    omega_accel: p(param::OMEGA_ACCEL),
                    hfi_v: p(param::HFI_V),
                    hfi_xi: p(param::HFI_XI),
                    hfi_bw: p(param::HFI_BW),
                    hfi_xsat: p(param::HFI_XSAT),
                    hfi_spread: p(param::HFI_SPREAD) as u8,
                    id_inject: p(param::ID_INJECT),
                    pol_a: HFI_POL_A.min(0.5 * spec.i_trip),
                    pol_s: p(param::HFI_POL_S),
                    pol_n: p(param::HFI_POL_N) as u32,
                    lock_s: HFI_LOCK_S,
                    lock_ramp_s: HFI_LOCK_RAMP_S,
                    stuck_s: HFI_STUCK_S,
                    advance_periods: 0.5 + spec.pwm_latency,
                    omega_max: 2.0 * p(param::SL_HANDOFF),
                };
                (FixHfi::new(&fp), fp)
            });
            if self.fixq.is_some() {
                // The integer path injects and modulates itself.
                self.hfi = None;
            }
        }
        if mode == mode::HALL_FOC && spec.has_halls {
            // Hall FOC keeps the hall angle for its FOC; the tracker only
            // places the position-torque feed-forward.
            let pp = p(param::POLE_PAIRS).max(1.0);
            let g = 1.5 * pp * pp * p(param::FLUX) / p(param::INERTIA);
            self.tracker = Some(HallTracker::new(map, g));
        }
        self.hall_bad = 0;
        // Online R/ψ in the closed-loop FOC modes, starting from the
        // profile. Tuned per 50 ms block, as the host estimator is
        // (docs/CALIBRATION.md): R may drift ~1 %/s, ψ ~0.1 %/s.
        self.rpsi = matches!(mode, mode::HALL_FOC | mode::SENSORLESS).then(|| {
            let l = p(param::L);
            let cfg = RpsiCfg {
                ld: l,
                lq: l,
                q_r: 1e-5,
                q_psi: 2e-12,
                q_bias: 1e-6,
                noise: 1e-4,
                omega_min: ESTIM_OMEGA_MIN,
                i_min: 0.1,
                use_derivative: false,
                p0: 1.0,
            };
            RpsiAverager::new(RpsiEstimator::new(cfg, p(param::R), p(param::FLUX)), 0.05)
        });
        self.dither_t = 0.0;
        self.dither_hi = false;
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
        if p(param::SS_CONDUCTION) >= 150.0 {
            // 180°: all three legs driven, on the six active vectors. The
            // state 3 on is the complement, so reversing works as above.
            let mut state = sixstep::state_180(self.theta);
            if dir < 0.0 {
                state = (state + 3) % sixstep::SECTORS;
            }
            self.ss_sector = state;
            b.set_phase_enables(0b111);
            sh.sixstep_sector.store(state as u8, Ordering::Relaxed);
            return sixstep::duties_180(state, duty);
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

/// The position-torque series as the parameter table holds it.
fn cog_terms(p: &impl Fn(u8) -> f32) -> [Term; TERMS] {
    core::array::from_fn(|k| Term {
        order: p(param::COG_N0 + k as u8),
        amp: p(param::COG_A0 + k as u8),
        phase: p(param::COG_P0 + k as u8),
    })
}

/// The hall calibration as the parameter table holds it.
fn hall_map(p: &impl Fn(u8) -> f32) -> HallMap {
    let w: [f32; 6] = core::array::from_fn(|k| p(param::HALL_W0 + k as u8));
    HallMap {
        offset: p(param::HALL_OFFSET),
        dir: if p(param::HALL_DIR) < 0.0 { -1.0 } else { 1.0 },
        hyst: p(param::HALL_HYST),
        widths: HallMap::normalized_widths(w),
    }
}

impl Engine {
    /// Observer → HFI: the tracker starts on the observer's angle (less the
    /// cross-saturation offset the tracker will see at this q current) and
    /// speed, straight into the run phase; the speed loop keeps its
    /// integrator and takes the HFI gains.
    #[cold]
    #[inline(never)]
    fn hfi_handback(&mut self, p: &impl Fn(u8) -> f32, theta: f32, omega: f32, iq: f32) {
        let mut tr = HfiTracker::new(
            p(param::HFI_V),
            p(param::HFI_XI),
            p(param::HFI_BW),
            theta - p(param::HFI_XSAT) * iq,
        )
        .with_spread(p(param::HFI_SPREAD) as u8);
        tr.set_omega(omega);
        self.hfi = Some(tr);
        self.hfi_start = Some(HfiStart {
            phase: HfiPhase::Run,
            ticks: 0,
            acc: [0.0; 2],
            n: [0; 2],
            pairs: 0,
        });
        if p(param::HFI_KP) > 0.0 {
            if let Some(s) = self.speed.as_mut() {
                s.set_gains(PiGains {
                    kp: p(param::HFI_KP),
                    ki: p(param::HFI_KI),
                });
            }
        }
    }
}

/// One tick of the integer HFI drive, converted back to the engine's units.
struct FixTick {
    duties: [f32; 3],
    v_dq: Dq,
    i_dq: Dq,
    v_ab: AlphaBeta,
    iq_ref: f32,
    theta_tr: f32,
    omega: f32,
}

/// Worst and average (EMA, ×16) cycles of the integer step (feature
/// `fixq`); read them with a debugger.
#[cfg(feature = "fixq")]
pub static FIXQ_CYCLES_MAX: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "fixq")]
pub static FIXQ_CYCLES_AVG16: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

impl Engine {
    #[cfg(feature = "fixq")]
    fn fixq_active(&self) -> bool {
        self.fixq.is_some()
    }

    #[cfg(not(feature = "fixq"))]
    fn fixq_active(&self) -> bool {
        false
    }

    #[cfg(not(feature = "fixq"))]
    fn fixq_tick(&mut self, _: &impl MotorBoard, _: Abc, _: f32, _: f32) -> FixTick {
        unreachable!()
    }

    /// The board boundary converts to and from the integer units (an
    /// FPU-less board would hand over ADC counts and take timer compares).
    #[cfg(feature = "fixq")]
    fn fixq_tick(
        &mut self,
        b: &impl MotorBoard,
        i_abc: Abc,
        vbus: f32,
        omega_target: f32,
    ) -> FixTick {
        use mmc_core::fixq::ONE;
        let (fx, fp) = self.fixq.as_mut().unwrap();
        let qi = ONE as f32 / fp.i_base;
        let qv = ONE as f32 / fp.v_base;
        let i = [
            (i_abc.a * qi) as i32,
            (i_abc.b * qi) as i32,
            (i_abc.c * qi) as i32,
        ];
        let v = (vbus * qv) as i32;
        let target = FixHfi::target_units(fp, omega_target);
        let t0 = b.cycles();
        let o = fx.step(i, v, target);
        let dur = b.cycles().wrapping_sub(t0);
        FIXQ_CYCLES_MAX.fetch_max(dur, Ordering::Relaxed);
        let avg = FIXQ_CYCLES_AVG16.load(Ordering::Relaxed);
        FIXQ_CYCLES_AVG16.store(avg - (avg >> 6) + (dur << 4 >> 6), Ordering::Relaxed);
        if o.stuck {
            self.hfi_stuck = u32::MAX / 2;
        }
        let units_per_rad = 4_294_967_296.0 / (2.0 * core::f32::consts::PI);
        let ang = |t: u32| (t as i32) as f32 / units_per_rad;
        let (ia, va) = (1.0 / qi, 1.0 / qv);
        let omega = o.omega_u as f32 / (fp.dt * units_per_rad);
        self.theta = ang(o.theta);
        self.omega = omega;
        FixTick {
            duties: o.duties.map(|d| d as f32 / ONE as f32),
            v_dq: Dq {
                d: o.v_dq.0 as f32 * va,
                q: o.v_dq.1 as f32 * va,
            },
            i_dq: Dq {
                d: o.i_dq.0 as f32 * ia,
                q: o.i_dq.1 as f32 * ia,
            },
            v_ab: AlphaBeta {
                alpha: o.v_ab.0 as f32 * va,
                beta: o.v_ab.1 as f32 * va,
            },
            iq_ref: o.iq_cmd as f32 * ia,
            theta_tr: ang(fx.tracker.theta),
            omega,
        }
    }
}

/// Coulomb friction feed-forward [A] for the HFI speed loop: `sl_fric`
/// in the direction of the speed reference, faded in over ±[`FRIC_OMEGA`].
/// It supplies the breakaway current a stuck rotor would otherwise wait for
/// the integrator to wind up (the low-speed stick-slip, session 39).
fn fric_ff(i_fric: f32, omega_ref: f32) -> f32 {
    i_fric * (omega_ref / FRIC_OMEGA).clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn friction_feedforward_follows_the_reference_and_fades_through_zero() {
        assert_eq!(fric_ff(0.2, 100.0), 0.2);
        assert_eq!(fric_ff(0.2, -100.0), -0.2);
        assert_eq!(fric_ff(0.2, 0.0), 0.0);
        assert!((fric_ff(0.2, 0.5 * FRIC_OMEGA) - 0.1).abs() < 1e-6);
        assert_eq!(fric_ff(0.0, 50.0), 0.0);
    }
}
