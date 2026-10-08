//! The simulator behind the wire protocol: a TCP server that runs the virtual
//! motor + FOC loop in real time and speaks mmc-proto, exactly like firmware
//! will over UART. Host tooling cannot tell the difference — by design.

use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc, FocOutput};
use mmc_core::inverter::DeadtimeModel;
use mmc_core::math::{sin_cos, wrap_angle};
use mmc_core::observer::{FluxObserver, FluxObserverCfg};
use mmc_core::pi::PiGains;
use mmc_core::probe;
use mmc_core::sensorless::{Phase, Sequencer, SequencerCfg, SpeedLoop};
use mmc_core::svpwm::svpwm;
use mmc_core::transforms::{clarke, inverse_clarke, inverse_park, park, Abc, AlphaBeta, Dq};
use mmc_core::tuning::{current_pi_gains, speed_pi_gains};
use mmc_hal::{BusVoltageSense, CurrentSense, PwmOutput};
use mmc_proto::{
    channel, encode, param, test, BoardTraits, BurstChunk, Deframer, DeviceInfo, DeviceKind,
    DriveMode, Message, MAX_FRAME,
};
use mmc_sim::{PmsmParams, TruthAngle, VirtualMotor};

// Drive-mode ramps/limits mirrored from the firmware ISR, so the sim spins up
// the way the hardware does.
const OMEGA_SLEW: f32 = 500.0; // rad/s^2 electrical
const V_SLEW: f32 = 5.0; // V/s
const I_SLEW: f32 = 2.0; // A/s
const SL_OMEGA_HANDOFF: f32 = 150.0; // I-f -> observer handoff [rad/s el]
const SL_IQ_LIMIT: f32 = 0.8; // speed-loop i_q authority [A]
const SPEED_BW: f32 = 40.0; // speed-loop bandwidth [rad/s el]

// R/L probe schedule mirrored from the firmware (main.rs); the saliency
// schedule is shared through `mmc_core::probe` and cannot drift. The RL fit
// (`tools/profile.py`) finds edges from the recorded v column, so these only
// have to be *close*, but keep them identical anyway.
const PROBE_ALIGN_TICKS: u32 = 6000;
const PROBE_HALF_TICKS: u32 = 32;
const PROBE_PRE_PAIRS: usize = 256;
const BURST_PAIRS: usize = 4096;
const I_TRIP_A: f32 = 1.5;
const V_AMP_MAX: f32 = 3.0;
// Six-step param defaults, mirrored from the firmware so the table the host
// reads back is the same shape and scale on both targets.
const ONTIME_CCR5_DEFAULT: f32 = 100.0;
const SS_KP_DEFAULT: f32 = 2.0e-4;
const SS_KI_DEFAULT: f32 = 5.0e-4;

/// The slice of the runtime parameter table the sensorless drive reads at a
/// clean start. The firmware rebuilds its control blocks from `param_get` on
/// every `SetDrive`; mirroring that here is what makes `apply` → behaviour
/// testable without hardware — the coverage gap that let the FOC speed loop
/// run on six-step's gains from `0882ae7` until 2026-08-08.
#[derive(Copy, Clone, Debug)]
struct StartupParams {
    handoff: f32,
    omega_accel: f32,
    iq_limit: f32,
    speed_gains: PiGains,
    deadtime_comp: DeadtimeModel,
}

pub struct ServeCfg {
    pub ctrl_freq: f32,
    pub bandwidth: f32,
    pub vbus: f32,
    /// Exit after the first client disconnects (used by `suite` and tests).
    pub once: bool,
    /// Virtual motor on the bench.
    pub params: PmsmParams,
    /// Mechanically clamp the rotor (locked-rotor test bench).
    pub locked: bool,
    /// Dead-time voltage error to give the virtual bridge. Default ideal;
    /// set it to give `profile --only vdead` something known to recover.
    pub deadtime: DeadtimeModel,
}

impl Default for ServeCfg {
    fn default() -> Self {
        Self {
            ctrl_freq: 10_000.0,
            bandwidth: 2000.0,
            vbus: 24.0,
            once: false,
            params: PmsmParams::small_bldc(),
            locked: false,
            deadtime: DeadtimeModel::default(),
        }
    }
}

/// One control period's output, for telemetry.
struct StepOut {
    out: FocOutput,
    iq_ref: f32,
    theta_est: f32,
    omega_est: f32,
    theta_err: f32,
    /// `channel::STATE` code (1 run, 6 I-f ramp, 7 blend, 0 idle).
    state: f32,
}

/// Live control for the sim server. Mirrors the firmware ISR's drive-mode
/// dispatch against the one virtual rig: mode 0 is the legacy truth-angle torque
/// mode (`SetIqRef`, used by `capture --iq`); modes 1/2/3 are open-loop voltage,
/// I-f current, and sensorless — reached via `SetDrive`, exactly like hardware.
struct SimControl {
    params: PmsmParams,
    ctrl_dt: f32,
    current_bw: f32,
    mode: u8,
    // mode 0 (truth-angle torque)
    angle: TruthAngle,
    iq_ref_cmd: f32,
    // forced-frame (1/2) + shared blocks
    foc: Foc,
    obs: FluxObserver,
    /// Previous tick's voltage command (what the bridge applied).
    v_applied: AlphaBeta,
    theta: f32,
    omega: f32,
    amp: f32,
    omega_target: f32,
    amp_target: f32,
    // sensorless (3) — startup knobs mirror the firmware's runtime params,
    // refreshed from the session's param table on every SetDrive.
    seq: Option<Sequencer>,
    speed: Option<SpeedLoop>,
    omega_ref_cur: f32,
    sl_preload: f32,
    sl_handoff: f32,
    omega_accel: f32,
    iq_limit: f32,
    speed_gains: PiGains,
    /// The controller's belief about the bridge, from the param table —
    /// distinct from `rig.inverter_error`, the bridge's actual behaviour.
    deadtime_comp: DeadtimeModel,
    /// Consecutive low-observer-flux ticks in closed-loop sensorless.
    stall_strikes: u32,
    /// Latched fault code (ST_STALL) until the next SetDrive.
    fault: f32,
    // probes (4): RunTest capture into the burst buffer, firmware-identical
    probe_kind: u8,
    probe_ticks: u32,
    probe_v: (f32, f32),
    /// Saliency half-period [ticks], τ-picked at start like the firmware.
    probe_half: usize,
    burst: Vec<f32>,
    /// Mirrors the firmware's BURST_STATE: 0 idle, 1 recording, 2 done.
    burst_state: u8,
}

impl SimControl {
    fn new(params: PmsmParams, ctrl_dt: f32, current_bw: f32) -> Self {
        Self {
            params,
            ctrl_dt,
            current_bw,
            mode: 0,
            angle: TruthAngle::default(),
            iq_ref_cmd: 0.0,
            foc: Foc::with_feedforward(
                current_pi_gains(params.rs, params.lq, current_bw),
                Decoupling {
                    ld: params.ld,
                    lq: params.lq,
                    flux: params.flux,
                },
            ),
            obs: FluxObserver::new(FluxObserverCfg::new(params.rs, params.lq)),
            v_applied: AlphaBeta::default(),
            theta: 0.0,
            omega: 0.0,
            amp: 0.0,
            omega_target: 0.0,
            amp_target: 0.0,
            seq: None,
            speed: None,
            omega_ref_cur: 0.0,
            sl_preload: 0.0,
            sl_handoff: SL_OMEGA_HANDOFF,
            omega_accel: OMEGA_SLEW,
            iq_limit: SL_IQ_LIMIT,
            speed_gains: speed_pi_gains(
                params.inertia,
                params.torque_constant(),
                params.pole_pairs,
                SPEED_BW,
            ),
            deadtime_comp: DeadtimeModel::default(),
            stall_strikes: 0,
            fault: 0.0,
            probe_kind: 0,
            probe_ticks: 0,
            probe_v: (0.0, 0.0),
            probe_half: 8,
            burst: Vec::new(),
            burst_state: 0,
        }
    }

    /// `RunTest`: start a probe capture (mode 4). Mirrors the firmware's
    /// preconditions — quiet stage only — its L_THETA voltage clamp, and its
    /// τ-adaptive half-period pick. Returns false (→ NAK) when refused.
    fn start_probe(&mut self, kind: u8, a: f32, b: f32, r_param: f32, l_param: f32) -> bool {
        if self.mode != 0 || self.burst_state == 1 {
            return false;
        }
        let v_max = if kind == test::L_THETA {
            let tau_ticks = l_param / r_param / self.ctrl_dt;
            self.probe_half = probe::sal_half_ticks(tau_ticks);
            (0.75 * I_TRIP_A * r_param).min(V_AMP_MAX)
        } else {
            V_AMP_MAX
        };
        self.probe_kind = kind;
        self.probe_v = (a.clamp(0.05, v_max), b.clamp(0.05, v_max));
        self.probe_ticks = 0;
        self.burst.clear();
        self.burst_state = 1;
        self.mode = 4;
        true
    }

    /// Rebuild the current loop from the motor params — a clean start with no
    /// integrator carryover between mode switches.
    fn rebuild_foc(&mut self) {
        let p = self.params;
        self.foc = Foc::with_feedforward(
            current_pi_gains(p.rs, p.lq, self.current_bw),
            Decoupling {
                ld: p.ld,
                lq: p.lq,
                flux: p.flux,
            },
        );
        // What the *controller* believes the bridge takes, from the param
        // table — deliberately separate from `rig.inverter_error`, which is
        // what the bridge actually takes. Setting them independently is how
        // a mis-calibrated compensation gets tested.
        self.foc.deadtime = Some(self.deadtime_comp);
    }

    /// `SetIqRef`: live i_q for I-f, torque command otherwise (legacy path).
    fn set_iq(&mut self, iq: f32) {
        if self.mode == 2 {
            self.amp_target = iq;
        } else {
            self.iq_ref_cmd = iq;
        }
    }

    /// `SetDrive`: `mode` 0 off, 1 open-loop voltage, 2 I-f, 3 sensorless.
    /// A new running mode starts from rest with fresh control blocks; a repeat
    /// of the same mode just retargets amplitude/speed (smooth live changes).
    /// `sl` carries the sensorless slice of the session param table.
    fn set_drive(&mut self, mode: u8, amp: f32, omega: f32, sl: StartupParams) {
        self.sl_handoff = sl.handoff;
        self.omega_accel = sl.omega_accel;
        self.iq_limit = sl.iq_limit;
        self.speed_gains = sl.speed_gains;
        self.deadtime_comp = sl.deadtime_comp;
        self.fault = 0.0;
        self.stall_strikes = 0;
        if mode == 0 {
            self.mode = 0;
            self.omega = 0.0;
            self.amp = 0.0;
            self.iq_ref_cmd = 0.0;
            self.seq = None;
            self.speed = None;
            // burst_abort(): a killed probe hands back the partial buffer.
            if self.burst_state == 1 {
                self.burst_state = 2;
            }
            self.rebuild_foc();
            return;
        }
        let fresh = mode != self.mode;
        self.mode = mode;
        self.omega_target = omega;
        self.amp_target = amp;
        if !fresh {
            return;
        }
        // Leaving a probe mid-recording hands back the partial buffer, as
        // the firmware does.
        if self.burst_state == 1 {
            self.burst_state = 2;
        }
        self.theta = 0.0;
        self.omega = 0.0;
        self.amp = 0.0;
        self.rebuild_foc();
        self.obs = FluxObserver::new(FluxObserverCfg::new(self.params.rs, self.params.lq));
        self.v_applied = AlphaBeta::default();
        if mode == 3 {
            let dir = if omega < 0.0 { -1.0 } else { 1.0 };
            let i_start = amp.abs().clamp(0.1, self.iq_limit);
            self.seq = Some(Sequencer::new(SequencerCfg {
                i_start,
                accel: self.omega_accel,
                omega_handoff: self.sl_handoff * dir,
                ..SequencerCfg::default()
            }));
            self.speed = Some(SpeedLoop::new(self.speed_gains, self.iq_limit));
            self.omega_ref_cur = self.sl_handoff * dir;
            self.sl_preload = i_start * dir;
        } else {
            self.seq = None;
            self.speed = None;
        }
    }

    /// Ramp the forced electrical frequency and amplitude toward their targets.
    fn ramp_forced(&mut self, amp_slew: f32) {
        let dt = self.ctrl_dt;
        let d_omega =
            (self.omega_target - self.omega).clamp(-self.omega_accel * dt, self.omega_accel * dt);
        self.omega += d_omega;
        let d_amp = (self.amp_target - self.amp).clamp(-amp_slew * dt, amp_slew * dt);
        self.amp += d_amp;
        self.theta = wrap_angle(self.theta + self.omega * dt);
    }

    /// One control period against the rig: read currents, compute, apply,
    /// advance the motor, then update the observer from what was applied.
    fn step(&mut self, rig: &mut VirtualMotor) -> StepOut {
        let dt = self.ctrl_dt;
        let [ia, ib, ic] = rig.phase_currents();
        let i_abc = Abc {
            a: ia,
            b: ib,
            c: ic,
        };
        let vbus = rig.vbus().max(1.0);
        let i_ab = clarke(i_abc);

        let mut iq_ref = 0.0;
        let mut state = 1.0;
        let out = match self.mode {
            1 => {
                // Open-loop rotating voltage vector.
                self.ramp_forced(V_SLEW);
                let sc = sin_cos(self.theta);
                let v_dq = Dq {
                    d: self.amp,
                    q: 0.0,
                };
                let v_ab = inverse_park(v_dq, sc);
                let duties = svpwm(v_ab, vbus);
                FocOutput {
                    duties,
                    i_dq: park(i_ab, sc),
                    v_dq,
                    v_ab,
                    i_ab,
                }
            }
            2 => {
                // I-f: closed current loop on the forced angle.
                self.ramp_forced(I_SLEW);
                iq_ref = self.amp;
                self.foc.step(
                    i_abc,
                    self.theta,
                    self.omega,
                    Dq { d: 0.0, q: iq_ref },
                    vbus,
                    dt,
                )
            }
            4 => {
                // Probe capture (RunTest), firmware-identical: align on θ=0,
                // then either the RL square wave at θ=0 (i_d, v_d legacy
                // layout) or the shared saliency schedule (header + i_d, i_q
                // in the excitation frame).
                self.probe_ticks += 1;
                let (v_a, v_b) = self.probe_v;
                let saliency = self.probe_kind == test::L_THETA;
                let total = if saliency {
                    probe::SAL_HDR + probe::SAL_TICKS * 2
                } else {
                    BURST_PAIRS * 2
                };
                if self.burst.len() >= total {
                    // Recording complete: hand the buffer to the host.
                    self.mode = 0;
                    self.burst_state = 2;
                    state = 0.0;
                    FocOutput {
                        duties: [0.0; 3],
                        i_dq: Dq::default(),
                        v_dq: Dq::default(),
                        v_ab: Default::default(),
                        i_ab,
                    }
                } else {
                    let aligning = self.probe_ticks <= PROBE_ALIGN_TICKS;
                    let (theta_x, v) = if aligning {
                        (0.0, v_a)
                    } else {
                        let t = (self.probe_ticks - PROBE_ALIGN_TICKS - 1) as usize;
                        if saliency {
                            let half = self.probe_half;
                            if t == 0 {
                                self.burst.extend_from_slice(&probe::sal_header(
                                    test::L_THETA,
                                    half,
                                    v_a,
                                    v_b,
                                    1.0 / self.ctrl_dt,
                                ));
                            }
                            let v = if probe::sal_level_is_high(t, half) {
                                v_b
                            } else {
                                v_a
                            };
                            (probe::sal_angle(t, half), v)
                        } else {
                            let half = t as u32 / PROBE_HALF_TICKS;
                            (0.0, if half.is_multiple_of(2) { v_b } else { v_a })
                        }
                    };
                    let sc = sin_cos(theta_x);
                    let i_dq = park(i_ab, sc);
                    if saliency && !aligning {
                        self.burst.push(i_dq.d);
                        self.burst.push(i_dq.q);
                    } else if !saliency
                        && self.probe_ticks + PROBE_PRE_PAIRS as u32 > PROBE_ALIGN_TICKS
                    {
                        self.burst.push(i_dq.d);
                        self.burst.push(v);
                    }
                    let v_dq = Dq { d: v, q: 0.0 };
                    let v_ab = inverse_park(v_dq, sc);
                    let duties = svpwm(v_ab, vbus);
                    FocOutput {
                        duties,
                        i_dq,
                        v_dq,
                        v_ab,
                        i_ab,
                    }
                }
            }
            3 => {
                // Sensorless: sequencer owns the angle (I-f ramp -> blend ->
                // observer), speed loop owns i_q once closed.
                let seq_out = self.seq.as_mut().unwrap().update(&self.obs, dt);
                iq_ref = match seq_out.iq_open {
                    Some(iq) => iq,
                    None => {
                        let speed = self.speed.as_mut().unwrap();
                        if self.sl_preload != 0.0 {
                            let taper = self.seq.as_ref().map_or(1.0, |q| q.taper_end());
                            speed.preload(self.sl_preload * taper);
                            self.sl_preload = 0.0;
                        }
                        let d = (self.omega_target - self.omega_ref_cur)
                            .clamp(-self.omega_accel * dt, self.omega_accel * dt);
                        self.omega_ref_cur += d;
                        speed.update(self.omega_ref_cur, seq_out.omega, dt)
                    }
                };
                self.theta = seq_out.theta;
                self.omega = seq_out.omega;
                state = match seq_out.phase {
                    Phase::Ramp => 6.0,
                    Phase::Blend => 7.0,
                    Phase::Closed => 1.0,
                };
                self.foc.step(
                    i_abc,
                    self.theta,
                    self.omega,
                    Dq { d: 0.0, q: iq_ref },
                    vbus,
                    dt,
                )
            }
            _ => {
                // Mode 0: legacy torque on the truth angle (SetIqRef).
                self.angle.sync(&rig.motor);
                iq_ref = self.iq_ref_cmd;
                state = if iq_ref != 0.0 { 1.0 } else { 0.0 };
                self.foc.step(
                    i_abc,
                    self.angle.electrical_angle(),
                    self.angle.electrical_velocity(),
                    Dq { d: 0.0, q: iq_ref },
                    vbus,
                    dt,
                )
            }
        };
        rig.set_duties(out.duties);
        rig.advance(dt);
        // The current just measured was driven by the previous command (the
        // firmware's timing): integrating this tick's command would put the
        // stator-flux estimate a tick ahead of the L·i it is corrected with.
        let v_obs = core::mem::replace(&mut self.v_applied, out.v_ab);
        self.obs.update(out.i_ab, v_obs, dt);

        // Stall detector, firmware-identical: in closed-loop sensorless a
        // stalled rotor leaves the observer confidently locked onto the L·i
        // artifact with flux ≈ L·|i| ≪ ψ. 100 ms below 0.35·ψ trips.
        if self.mode == 3 && self.seq.as_ref().map(|q| q.phase()) == Some(Phase::Closed) {
            if self.obs.flux_mag() < 0.35 * self.params.flux {
                self.stall_strikes += 1;
                if self.stall_strikes as f32 * dt >= 0.1 {
                    self.mode = 0;
                    self.iq_ref_cmd = 0.0;
                    self.seq = None;
                    self.speed = None;
                    self.fault = 8.0; // ST_STALL
                }
            } else {
                self.stall_strikes = 0;
            }
        } else if self.mode != 3 {
            self.stall_strikes = 0;
        }
        if self.fault != 0.0 && self.mode == 0 {
            state = self.fault;
        }

        let theta_est = self.obs.electrical_angle();
        StepOut {
            iq_ref,
            theta_est,
            omega_est: self.obs.electrical_velocity(),
            theta_err: wrap_angle(theta_est - rig.motor.theta_e()),
            state,
            out,
        }
    }
}

pub fn serve(listener: TcpListener, cfg: &ServeCfg) -> std::io::Result<()> {
    println!("sim server listening on {}", listener.local_addr()?);
    // The parameter table belongs to the *device*, not to the connection.
    // On hardware it lives in the firmware's RAM (and optionally flash) and
    // outlives any host tool; rebuilding it per session made `apply` verify a
    // write that silently reverted the moment the tool disconnected, so a
    // profile → fit → apply → capture loop against the sim quietly ran on
    // defaults. Motor and controller state stay per-session — that part
    // *should* start fresh.
    let mut sim_params = default_params(cfg);
    loop {
        let (stream, peer) = listener.accept()?;
        println!("client {peer} connected");
        match session(stream, cfg, &mut sim_params) {
            Ok(()) => println!("client {peer} disconnected"),
            Err(e) if e.kind() == ErrorKind::ConnectionReset => {
                println!("client {peer} disconnected")
            }
            Err(e) => println!("client {peer}: {e}"),
        }
        if cfg.once {
            return Ok(());
        }
    }
}

/// The device's parameter table as it powers up: motor values from the
/// virtual motor, gains derived from it — the sim's equivalent of the
/// firmware's compiled-in bench fits. A session that never writes params
/// behaves as before; one that does actually retunes the loop (see
/// `set_drive`), which is the path `apply` exercises.
fn default_params(cfg: &ServeCfg) -> [f32; param::COUNT] {
    let params = cfg.params;
    let seed_speed = speed_pi_gains(
        params.inertia,
        params.torque_constant(),
        params.pole_pairs,
        SPEED_BW,
    );
    [
        params.rs,
        params.lq,
        params.flux,
        cfg.bandwidth,
        seed_speed.kp,
        seed_speed.ki,
        params.pole_pairs as f32,
        SL_OMEGA_HANDOFF,
        OMEGA_SLEW,
        SL_IQ_LIMIT,
        // Six-step knobs: stored and range-checked so `apply`/`panel` round-
        // trip the whole table against the sim, though the dq sim has no
        // ADC trigger to move and no six-step mode to tune.
        ONTIME_CCR5_DEFAULT,
        SS_KP_DEFAULT,
        SS_KI_DEFAULT,
        // Compensation off until the rig is measured; see param::V_DEAD.
        0.0,
        0.5,
        // Hall map and position loop: stored for round-trips; the TCP sim
        // has no halls.
        0.0,
        1.0,
        0.0,
        0.02,
        0.2,
        9e-4,
        200.0,
        params.inertia,
        0.0,
        120.0, // ss_conduction
        0.0,   // id_inject
        std::f32::consts::FRAC_PI_3,
        std::f32::consts::FRAC_PI_3,
        std::f32::consts::FRAC_PI_3,
        std::f32::consts::FRAC_PI_3,
        std::f32::consts::FRAC_PI_3,
        std::f32::consts::FRAC_PI_3,
        0.0,  // id_dither
        0.5,  // id_dither_period
        0.0,  // cog_ff
        -1.0, // cog_shift
        0.0,
        0.0,
        0.0,
        0.0, // cog_n0..3
        0.0,
        0.0,
        0.0,
        0.0, // cog_a0..3
        0.0,
        0.0,
        0.0,
        0.0,   // cog_p0..3
        0.0,   // hfi_v
        300.0, // hfi_bw
        0.05,  // hfi_xi
        0.0,   // hfi_xsat
        0.0,   // sl_fric
    ]
}

/// One client session: fresh motor, fresh controller, zero reference. The
/// parameter table is owned by the caller and persists across sessions.
fn session(
    mut stream: TcpStream,
    cfg: &ServeCfg,
    sim_params: &mut [f32; param::COUNT],
) -> std::io::Result<()> {
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(Duration::from_millis(1)))?;

    let params = cfg.params;
    let ctrl_dt = 1.0 / cfg.ctrl_freq;
    let mut rig = VirtualMotor::new(params, cfg.vbus);
    rig.motor.locked = cfg.locked;
    rig.inverter_error = cfg.deadtime;
    rig.enable();
    let mut control = SimControl::new(params, ctrl_dt, cfg.bandwidth);

    let mut deframer = Deframer::new();
    let mut mask = channel::ALL;
    let mut divider: u32 = 10;
    let mut streaming = false;
    let started = Instant::now();
    let mut steps_done: u64 = 0;
    let mut rx = [0u8; 256];

    loop {
        // Ingest whatever arrived (the 1 ms read timeout is also our pacing).
        match stream.read(&mut rx) {
            Ok(0) => return Ok(()),
            Ok(n) => {
                for &b in &rx[..n] {
                    if let Some(Ok(msg)) = deframer.push(b) {
                        handle(
                            &msg,
                            &mut stream,
                            &mut control,
                            &mut mask,
                            &mut divider,
                            &mut streaming,
                            sim_params,
                        )?;
                    }
                    // Frame errors over TCP mean a client bug; ignore & resync.
                }
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
        }

        // Catch the simulation up to wall time (capped: no death spiral).
        let target = (started.elapsed().as_secs_f64() * cfg.ctrl_freq as f64) as u64;
        let burst = (target - steps_done).min(cfg.ctrl_freq as u64 / 10);
        for _ in 0..burst {
            let step = control.step(&mut rig);
            steps_done += 1;

            if streaming && steps_done.is_multiple_of(divider as u64) {
                let frame = telemetry(&rig, &step, mask);
                send(&mut stream, &Message::Telemetry(frame))?;
            }
        }
    }
}

fn handle(
    msg: &Message,
    stream: &mut TcpStream,
    control: &mut SimControl,
    mask: &mut u32,
    divider: &mut u32,
    streaming: &mut bool,
    params: &mut [f32],
) -> std::io::Result<()> {
    match *msg {
        Message::Ping { nonce } => send(stream, &Message::Pong { nonce }),
        Message::GetInfo => send(
            stream,
            &Message::Info(DeviceInfo::new(DeviceKind::Sim, 1, "mmc-sim").with_board(
                BoardTraits {
                    ctrl_hz: (1.0 / control.ctrl_dt).round() as u32,
                    // Average-value inverter: no series resistance to subtract.
                    r_path: 0.0,
                    burst_cap: (BURST_PAIRS * 2).max(probe::SAL_HDR + probe::SAL_TICKS * 2) as u32,
                },
            )),
        ),
        Message::SetTelemetry {
            divider: d,
            mask: m,
        } => {
            *divider = d.max(1) as u32;
            *mask = m & channel::ALL;
            send(
                stream,
                &Message::Ack {
                    of: msg.wire_type(),
                },
            )
        }
        Message::Stream { enable } => {
            *streaming = enable;
            send(
                stream,
                &Message::Ack {
                    of: msg.wire_type(),
                },
            )
        }
        Message::SetIqRef { iq } => {
            control.set_iq(iq);
            send(
                stream,
                &Message::Ack {
                    of: msg.wire_type(),
                },
            )
        }
        Message::SetDrive(DriveMode::SixStepForced { .. })
        | Message::SetDrive(DriveMode::SixStepSensorless { .. })
        | Message::SetDrive(DriveMode::HallFoc { .. })
        | Message::SetDrive(DriveMode::SixStepHall { .. })
        | Message::SetDrive(DriveMode::HallPosition { .. }) => {
            // The virtual motor is a dq average-value model: it has no
            // floating terminal and no trapezoidal EMF, so it cannot honestly
            // run six-step. NAK until the phase-domain model lands, rather
            // than silently simulating something the hardware does not do.
            send(
                stream,
                &Message::Nak {
                    of: msg.wire_type(),
                    err: 4,
                },
            )
        }
        Message::SetDrive(mode) => {
            let (m, amp, omega) = match mode {
                DriveMode::Off => (0u8, 0.0f32, 0.0f32),
                DriveMode::OpenLoopVoltage { volts, omega_e } => (1, volts, omega_e),
                DriveMode::IfCurrent { amps, omega_e } => (2, amps, omega_e),
                DriveMode::Sensorless { amps, omega_e } => (3, amps, omega_e),
                DriveMode::SixStepForced { .. }
                | DriveMode::SixStepSensorless { .. }
                | DriveMode::HallFoc { .. }
                | DriveMode::SixStepHall { .. }
                | DriveMode::HallPosition { .. } => {
                    unreachable!("handled above")
                }
            };
            control.set_drive(
                m,
                amp,
                omega,
                StartupParams {
                    handoff: params[param::SL_HANDOFF as usize],
                    omega_accel: params[param::OMEGA_ACCEL as usize],
                    iq_limit: params[param::IQ_LIMIT as usize],
                    speed_gains: PiGains {
                        kp: params[param::SPEED_KP as usize],
                        ki: params[param::SPEED_KI as usize],
                    },
                    deadtime_comp: DeadtimeModel {
                        v_dead: params[param::V_DEAD as usize],
                        i_thresh: params[param::I_THRESH as usize],
                    },
                },
            );
            send(
                stream,
                &Message::Ack {
                    of: msg.wire_type(),
                },
            )
        }
        // The sim has no flash; params already live for the session, so
        // persist/erase are no-ops that ack (keeps `apply --persist` and the
        // panel's Save button exercisable without hardware).
        Message::SaveParams | Message::EraseParams => send(
            stream,
            &Message::Ack {
                of: msg.wire_type(),
            },
        ),
        Message::RunTest { kind, a, b } => {
            if kind != test::RL_STEP && kind != test::L_THETA {
                return send(
                    stream,
                    &Message::Nak {
                        of: msg.wire_type(),
                        err: 1,
                    },
                );
            }
            if control.start_probe(
                kind,
                a,
                b,
                params[param::R as usize],
                params[param::L as usize],
            ) {
                send(
                    stream,
                    &Message::Ack {
                        of: msg.wire_type(),
                    },
                )
            } else {
                send(
                    stream,
                    &Message::Nak {
                        of: msg.wire_type(),
                        err: 2,
                    },
                )
            }
        }
        Message::ReadBurst { offset } => {
            if control.burst_state != 2 {
                return send(
                    stream,
                    &Message::Nak {
                        of: msg.wire_type(),
                        err: 4, // no finished recording to read
                    },
                );
            }
            let len = control.burst.len();
            let off = (offset as usize).min(len);
            let n = (len - off).min(mmc_proto::BURST_CHUNK);
            match BurstChunk::new(off as u16, len as u16, &control.burst[off..off + n]) {
                Some(chunk) => send(stream, &Message::BurstData(chunk)),
                None => send(
                    stream,
                    &Message::Nak {
                        of: msg.wire_type(),
                        err: 1,
                    },
                ),
            }
        }
        Message::GetParam { id } if (id as usize) < params.len() => send(
            stream,
            &Message::ParamValue {
                id,
                value: params[id as usize],
            },
        ),
        Message::SetParam { id, value } => match sim_param_range(id) {
            Some((lo, hi)) if (lo..=hi).contains(&value) => {
                params[id as usize] = value;
                send(
                    stream,
                    &Message::Ack {
                        of: msg.wire_type(),
                    },
                )
            }
            _ => send(
                stream,
                &Message::Nak {
                    of: msg.wire_type(),
                    err: 1,
                },
            ),
        },
        // Device-to-host messages arriving at the device: refuse politely.
        _ => send(
            stream,
            &Message::Nak {
                of: msg.wire_type(),
                err: 1,
            },
        ),
    }
}

/// Same acceptance windows as the firmware's `param_range` — the sim rejects
/// what the device would, so `apply`/panel writes fail identically here.
fn sim_param_range(id: u8) -> Option<(f32, f32)> {
    Some(match id {
        param::R => (0.05, 20.0),
        param::L => (5e-6, 0.05),
        param::FLUX => (1e-5, 0.5),
        param::CUR_BW => (100.0, 4000.0),
        param::POLE_PAIRS => (1.0, 50.0),
        param::SL_HANDOFF => (30.0, 1000.0),
        param::OMEGA_ACCEL => (20.0, 5000.0),
        param::IQ_LIMIT => (0.05, 1.2),
        param::SPEED_KP => (0.0, 0.1),
        param::SPEED_KI => (0.0, 10.0),
        // The firmware's upper bound is PWM_ARR/2, which is a property of its
        // timer, not of the parameter; 1062 is that value at 40 kHz PWM.
        param::ONTIME_CCR5 => (20.0, 1062.0),
        param::SS_KP => (0.0, 0.01),
        param::SS_KI => (0.0, 0.1),
        param::V_DEAD => (0.0, 2.0),
        param::I_THRESH => (0.01, 5.0),
        param::HALL_OFFSET => (-core::f32::consts::PI, core::f32::consts::PI),
        param::HALL_DIR => (-1.0, 1.0),
        param::HALL_HYST => (0.0, 0.3),
        param::POS_KP => (0.0, 1.0),
        param::POS_KI => (0.0, 20.0),
        param::POS_KD => (0.0, 0.05),
        param::POS_VMAX => (0.1, 2000.0),
        param::INERTIA => (1e-8, 1e-2),
        param::I_FRIC => (0.0, 1.0),
        param::SS_CONDUCTION => (120.0, 180.0),
        param::ID_INJECT => (-1.0, 1.0),
        param::ID_DITHER => (-1.0, 1.0),
        param::ID_DITHER_PERIOD => (0.2, 10.0),
        param::COG_FF => (0.0, 2.0),
        param::COG_SHIFT => (-1.0, 15.0),
        id if (param::COG_N0..param::COG_N0 + 4).contains(&id) => (0.0, 255.0),
        id if (param::COG_A0..param::COG_A0 + 4).contains(&id) => (-0.1, 0.1),
        id if (param::COG_P0..param::COG_P0 + 4).contains(&id) => (-7.0, 7.0),
        param::HFI_V => (0.0, 3.0),
        param::HFI_BW => (10.0, 2000.0),
        param::HFI_XI => (0.005, 0.5),
        param::HFI_XSAT => (-2.0, 2.0),
        param::SL_FRIC => (0.0, 0.75),
        id if (param::HALL_W0..param::HALL_W0 + 6).contains(&id) => {
            (std::f32::consts::FRAC_PI_6, std::f32::consts::FRAC_PI_2)
        }
        _ => return None,
    })
}

fn telemetry(rig: &VirtualMotor, step: &StepOut, mask: u32) -> mmc_proto::TelemetryFrame {
    let out = &step.out;
    let i_abc = rig.motor.phase_currents();
    let mut values = [0f32; channel::COUNT];
    let mut n = 0;
    for id in 0..channel::COUNT as u8 {
        if mask & (1 << id) == 0 {
            continue;
        }
        values[n] = match id {
            channel::IQ_REF => step.iq_ref,
            channel::I_D => out.i_dq.d,
            channel::I_Q => out.i_dq.q,
            channel::V_D => out.v_dq.d,
            channel::V_Q => out.v_dq.q,
            channel::DUTY_A => out.duties[0],
            channel::DUTY_B => out.duties[1],
            channel::DUTY_C => out.duties[2],
            channel::OMEGA_M => rig.motor.omega_m,
            channel::THETA_E => rig.motor.theta_e(),
            channel::VBUS => rig.v_bus,
            channel::I_A => i_abc.a,
            channel::I_B => i_abc.b,
            channel::I_C => i_abc.c,
            channel::STATE => step.state,
            channel::THETA_EST => step.theta_est,
            channel::OMEGA_EST => step.omega_est,
            channel::THETA_ERR => step.theta_err,
            // The sim reports the model's per-phase EMF (≡ terminal voltage
            // when coasting, which is when the hardware channels matter).
            channel::VB_U | channel::VB_V | channel::VB_W => {
                let p = rig.motor.params;
                let we = rig.motor.omega_m * p.pole_pairs as f32;
                let e = inverse_clarke(inverse_park(
                    Dq {
                        d: 0.0,
                        q: p.flux * we,
                    },
                    mmc_core::math::sin_cos(rig.motor.theta_e()),
                ));
                match id {
                    channel::VB_U => e.a,
                    channel::VB_V => e.b,
                    _ => e.c,
                }
            }
            _ => 0.0,
        };
        n += 1;
    }
    let t_us = (rig.time() * 1e6) as u64 as u32;
    mmc_proto::TelemetryFrame::new(t_us, mask, &values[..n]).expect("mask/value count agree")
}

fn send(stream: &mut TcpStream, msg: &Message) -> std::io::Result<()> {
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(msg, &mut buf).expect("MAX_FRAME-sized buffer");
    stream.write_all(&buf[..n])
}
