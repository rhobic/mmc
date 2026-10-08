//! The motor-drive application, independent of any MCU.
//!
//! Everything the bench firmware *does* — the drive state machine (open-loop,
//! I-f, sensorless FOC, forced and sensorless six-step), the profiler probes,
//! protection trips, the deadman, runtime parameters and their flash blob,
//! host-command handling and the telemetry snapshot — lives here and talks
//! to hardware only through [`mmc_hal::MotorBoard`]. A board crate supplies
//! clocks, peripheral setup, the trait implementation, a control interrupt
//! that calls [`Engine::tick`], and the serial/flash glue; nothing else.
//!
//! Two halves, split by who runs them:
//!
//! - [`Shared`] — lock-free state crossing between the control interrupt and
//!   the host-link tasks (commands in, telemetry and burst recordings out).
//!   One `static` per firmware.
//! - [`Engine`] — state owned by the control interrupt alone.
//!
//! All time constants derive from [`BoardSpec::ctrl_hz`], so the same code
//! runs a 20 kHz loop on a 170 MHz MCU and a 10 kHz loop on a 72 MHz one.

#![no_std]

mod engine;
#[cfg(feature = "link")]
pub mod link;
pub mod nvparam;

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use mmc_core::probe;
use mmc_hal::BoardSpec;
use mmc_proto::{
    channel, param, test, BoardTraits, BurstChunk, DeviceInfo, DeviceKind, DriveMode, Message,
    TelemetryFrame,
};

pub use engine::Engine;
pub use nvparam::ParamStore;

// drive state values (channel::STATE)
pub const ST_OFF: u8 = 0;
pub const ST_RUN: u8 = 1;
pub const ST_FAULT_OC: u8 = 2;
pub const ST_FAULT_DRV: u8 = 3;
pub const ST_FAULT_VBUS: u8 = 4;
pub const ST_CAL: u8 = 5;
// Sensorless startup phases on the telemetry STATE channel (same codes the
// host sim scenario writes, so dashboards read identically): Closed = ST_RUN.
pub const ST_SL_RAMP: u8 = 6;
pub const ST_SL_BLEND: u8 = 7;
/// Stall fault: the observer's flux magnitude collapsed to the L·i artifact
/// while nominally closed-loop — the rotor is not actually following.
/// ≥ ST_FAULT_OC, so the fault gating and Off-to-re-arm flow apply.
pub const ST_STALL: u8 = 8;
/// Six-step commutating on measured crossings but with the detector not
/// confident — it is coasting on the timeout fallback, so the speed it reports
/// is a guess and the drive is one missed crossing from losing sync.
pub const ST_SS_UNLOCKED: u8 = 9;
/// Hall fault: a hall-sensored drive saw an invalid state (0b000/0b111 — a
/// lost sensor supply or a broken wire) for longer than a glitch.
pub const ST_FAULT_HALL: u8 = 10;

/// Drive-mode codes on `CMD_MODE` (mode 4 is the probe sequencer).
pub(crate) mod mode {
    pub const OFF: u8 = 0;
    pub const VOLT: u8 = 1;
    pub const IF: u8 = 2;
    pub const SENSORLESS: u8 = 3;
    pub const PROBE: u8 = 4;
    pub const SS_FORCED: u8 = 5;
    pub const SS_SENSORLESS: u8 = 6;
    pub const HALL_FOC: u8 = 7;
    pub const SS_HALL: u8 = 8;
    pub const HALL_POS: u8 = 9;
}

/// Open-loop voltage ceiling [V] (also the R/L probe's clamp).
pub const V_AMP_MAX: f32 = 3.0;
/// Commanded electrical-speed ceiling [rad/s].
pub const OMEGA_E_MAX: f32 = 2000.0;

/// Everything about a firmware build the drive needs to know: the board's
/// electrical facts plus its identity on the wire and its parameter
/// defaults (the motor fits that apply until the host writes others).
#[derive(Copy, Clone)]
pub struct DriveConfig {
    pub spec: BoardSpec,
    pub kind: DeviceKind,
    pub fw_version: u16,
    pub name: &'static str,
    pub defaults: [f32; param::COUNT],
}

impl DriveConfig {
    /// Control ticks in `secs` seconds.
    pub(crate) fn ticks(&self, secs: f32) -> u32 {
        (secs * self.spec.ctrl_hz as f32) as u32
    }

    pub(crate) fn dt(&self) -> f32 {
        1.0 / self.spec.ctrl_hz as f32
    }

    /// Sanity window per parameter — a NAK beats bricking the control loop.
    pub fn param_range(&self, id: u8) -> Option<(f32, f32)> {
        Some(match id {
            param::R => (0.05, 20.0),
            param::L => (5e-6, 0.05),
            param::FLUX => (1e-5, 0.5),
            param::CUR_BW => (100.0, 4000.0),
            param::SPEED_KP => (0.0, 0.1),
            param::SPEED_KI => (0.0, 10.0),
            param::POLE_PAIRS => (1.0, 50.0),
            // Handoff floor sits above the observer's ~4·leak trust floor at
            // the low end only nominally — going below ~80 is an experiment,
            // allowed but on the operator's head.
            param::SL_HANDOFF => (30.0, 1000.0),
            param::OMEGA_ACCEL => (20.0, 5000.0),
            // Current ceiling: 80% of the software trip.
            param::IQ_LIMIT => (0.05, 0.8 * self.spec.i_trip),
            // Terminal sample point, in the board's units (timer counts on
            // the boards so far); the board states its own ceiling.
            param::ONTIME_CCR5 => {
                let hi = self.spec.terminal_offset_max;
                (20.0f32.min(hi), hi)
            }
            // Duty per rad/s electrical: 1e-3 already commands full duty from
            // a 900 rad/s error, so the useful range is small.
            param::SS_KP => (0.0, 0.01),
            param::SS_KI => (0.0, 0.1),
            // 0 disables. The ceiling is generous — a slow driver on a high
            // bus can genuinely lose a volt — but it is still a ceiling: a
            // fitted value near it means the fit found something other than
            // dead time.
            param::V_DEAD => (0.0, 2.0),
            // Below ~10 mA the correction is a step at every zero crossing,
            // which is the chatter the band exists to avoid.
            param::I_THRESH => (0.01, 5.0),
            param::HALL_OFFSET => (-core::f32::consts::PI, core::f32::consts::PI),
            // Only the sign is used; a calibration writes ±1.
            param::HALL_DIR => (-1.0, 1.0),
            param::HALL_HYST => (0.0, 0.3),
            param::POS_KP => (0.0, 1.0),
            param::POS_KI => (0.0, 20.0),
            param::POS_KD => (0.0, 0.05),
            param::POS_VMAX => (0.1, OMEGA_E_MAX),
            param::INERTIA => (1e-8, 1e-2),
            param::I_FRIC => (0.0, 0.5 * self.spec.i_trip),
            param::SS_CONDUCTION => (120.0, 180.0),
            param::ID_INJECT => (-0.5 * self.spec.i_trip, 0.5 * self.spec.i_trip),
            param::ID_DITHER => (-0.5 * self.spec.i_trip, 0.5 * self.spec.i_trip),
            // At least two estimator blocks (50 ms) per level.
            param::ID_DITHER_PERIOD => (0.2, 10.0),
            param::COG_FF => (0.0, 2.0),
            param::COG_SHIFT => (-1.0, 15.0),
            id if (param::COG_N0..param::COG_N0 + 4).contains(&id) => (0.0, 255.0),
            id if (param::COG_A0..param::COG_A0 + 4).contains(&id) => (-0.1, 0.1),
            id if (param::COG_P0..param::COG_P0 + 4).contains(&id) => (-7.0, 7.0),
            param::HFI_V => (0.0, V_AMP_MAX),
            param::HFI_BW => (10.0, 2000.0),
            param::HFI_XI => (0.005, 0.5),
            // A sector between 30° and 90°: anything outside is a broken
            // sensor or a bad fit, not a placement tolerance.
            id if (param::HALL_W0..param::HALL_W0 + 6).contains(&id) => {
                (core::f32::consts::FRAC_PI_6, core::f32::consts::FRAC_PI_2)
            }
            _ => return None,
        })
    }
}

/// Burst-buffer handoff: 0 idle, 1 recording (control ISR owns the buffer),
/// 2 done (host may read).
pub(crate) const BURST_IDLE: u8 = 0;
pub(crate) const BURST_RECORDING: u8 = 1;
pub(crate) const BURST_DONE: u8 = 2;

/// The probe recording buffer. A separate `static` from [`Shared`] on
/// purpose: it is all zeros, so it lands in `.bss` and costs no flash, while
/// `Shared` carries non-zero parameter defaults and is stored in `.data` —
/// as one struct the whole buffer would be copied out of flash at boot.
pub struct BurstBuffer<const N: usize>(pub(crate) UnsafeCell<[f32; N]>);
// Safety: written only by the control ISR while BURST_STATE == RECORDING,
// read by the host task only while BURST_STATE == DONE.
unsafe impl<const N: usize> Sync for BurstBuffer<N> {}

impl<const N: usize> BurstBuffer<N> {
    pub const fn new() -> Self {
        Self(UnsafeCell::new([0.0; N]))
    }
}

impl<const N: usize> Default for BurstBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// State shared between the control interrupt and the host-link tasks.
/// `N` is the burst-buffer capacity in f32s — RAM is the constraint that
/// varies most between MCUs, so the firmware picks it.
pub struct Shared<const N: usize> {
    pub(crate) cfg: DriveConfig,

    // Host command → control ISR.
    pub(crate) cmd_mode: AtomicU8,
    pub(crate) cmd_amp: AtomicU32,
    pub(crate) cmd_omega: AtomicU32,
    pub(crate) cmd_epoch: AtomicU32,
    /// `control_ticks` value at the last host message (deadman).
    pub(crate) last_rx_tick: AtomicU32,

    // Control ISR → tasks.
    pub(crate) control_ticks: AtomicU32,
    /// Max ISR duration in cycles since boot — not on the wire, but readable
    /// live via the debug probe; this is how the libm f64-soft-float stall
    /// was found.
    pub isr_max_cycles: AtomicU32,
    pub(crate) state: AtomicU8,
    pub(crate) sixstep_sector: AtomicU8,
    pub(crate) telem_seq: AtomicU32,
    pub(crate) telem: [AtomicU32; channel::COUNT],

    // Telemetry config (rx task → tx task).
    pub(crate) mask: AtomicU32,
    pub(crate) divider: AtomicU32,
    pub(crate) streaming: AtomicBool,

    /// Runtime parameters (mmc_proto::param ids), profiler-writable over the
    /// protocol. A new value applies at the next clean drive start.
    pub(crate) params: [AtomicU32; param::COUNT],

    // Probe recordings at the full control rate — resolution the telemetry
    // stream can't deliver over the serial link. Layout is kind-keyed:
    // RL_STEP records (i_d, v_d) pairs from index 0; L_THETA writes a
    // `probe::SAL_HDR` self-describing header first, then (i_d, i_q) pairs in
    // the excitation frame.
    pub(crate) burst: &'static BurstBuffer<N>,
    pub(crate) burst_state: AtomicU8,
    /// f32s recorded so far.
    pub(crate) burst_len: AtomicU32,
    pub(crate) probe_v_align: AtomicU32,
    pub(crate) probe_v_step: AtomicU32,
    /// Which test sequence the probe mode is running (`mmc_proto::test`).
    pub(crate) probe_kind: AtomicU8,
    /// Saliency-sweep half-period [ticks], picked from the live R/L params at
    /// probe start (τ-adaptive; see `mmc_core::probe::sal_half_ticks`).
    pub(crate) probe_half: AtomicU32,
}

impl<const N: usize> Shared<N> {
    pub const fn new(cfg: DriveConfig, burst: &'static BurstBuffer<N>) -> Self {
        let mut params = [const { AtomicU32::new(0) }; param::COUNT];
        let mut i = 0;
        while i < param::COUNT {
            params[i] = AtomicU32::new(cfg.defaults[i].to_bits());
            i += 1;
        }
        Self {
            cfg,
            cmd_mode: AtomicU8::new(0),
            cmd_amp: AtomicU32::new(0),
            cmd_omega: AtomicU32::new(0),
            cmd_epoch: AtomicU32::new(0),
            last_rx_tick: AtomicU32::new(0),
            control_ticks: AtomicU32::new(0),
            isr_max_cycles: AtomicU32::new(0),
            state: AtomicU8::new(ST_CAL),
            sixstep_sector: AtomicU8::new(0),
            telem_seq: AtomicU32::new(0),
            telem: [const { AtomicU32::new(0) }; channel::COUNT],
            mask: AtomicU32::new(channel::ALL),
            divider: AtomicU32::new(20),
            streaming: AtomicBool::new(false),
            params,
            burst,
            burst_state: AtomicU8::new(BURST_IDLE),
            burst_len: AtomicU32::new(0),
            probe_v_align: AtomicU32::new(0),
            probe_v_step: AtomicU32::new(0),
            probe_kind: AtomicU8::new(0),
            probe_half: AtomicU32::new(8),
        }
    }

    pub fn config(&self) -> &DriveConfig {
        &self.cfg
    }

    pub fn param(&self, id: u8) -> f32 {
        f32::from_bits(self.params[id as usize].load(Ordering::Relaxed))
    }

    /// Drive state (`ST_*`).
    pub fn state(&self) -> u8 {
        self.state.load(Ordering::Relaxed)
    }

    /// Restore a persisted parameter table (call before the control loop
    /// starts). Each value is range-checked, so a stale blob from an older
    /// firmware can't push a value the control loop would choke on.
    pub fn restore(&self, store: &mut impl ParamStore) {
        if let Some(params) = nvparam::decode(store.read()) {
            for (id, &v) in params.iter().enumerate() {
                if let Some((lo, hi)) = self.cfg.param_range(id as u8) {
                    if (lo..=hi).contains(&v) {
                        self.params[id].store(v.to_bits(), Ordering::Relaxed);
                    }
                }
            }
        }
    }

    /// The host link saw traffic: holds off the deadman.
    pub fn host_activity(&self) {
        self.last_rx_tick.store(
            self.control_ticks.load(Ordering::Relaxed),
            Ordering::Relaxed,
        );
    }

    /// Handle one host message; persistence requests go to `store`.
    pub fn handle(&self, msg: &Message, store: &mut impl ParamStore) -> Message {
        match msg {
            Message::SaveParams | Message::EraseParams => self.persist(store, msg),
            other => self.handle_volatile(other),
        }
    }

    /// Persist / erase the parameter table, gated on a quiet stage (drive
    /// off, no probe recording) so a flash stall never hits a live drive.
    fn persist(&self, store: &mut impl ParamStore, msg: &Message) -> Message {
        let of = msg.wire_type();
        let state = self.state();
        if state == ST_CAL {
            return Message::Nak { of, err: 3 };
        }
        if state == ST_RUN
            || self.cmd_mode.load(Ordering::Relaxed) != mode::OFF
            || self.burst_state.load(Ordering::Relaxed) == BURST_RECORDING
        {
            return Message::Nak { of, err: 2 };
        }
        let ok = if matches!(msg, Message::EraseParams) {
            store.erase()
        } else {
            let mut params = [0.0f32; param::COUNT];
            for (id, p) in params.iter_mut().enumerate() {
                *p = self.param(id as u8);
            }
            store.write(&nvparam::encode(&params))
        };
        if ok {
            Message::Ack { of }
        } else {
            Message::Nak { of, err: 5 } // flash program error
        }
    }

    fn info(&self) -> DeviceInfo {
        DeviceInfo::new(self.cfg.kind, self.cfg.fw_version, self.cfg.name).with_board(BoardTraits {
            ctrl_hz: self.cfg.spec.ctrl_hz,
            r_path: self.cfg.spec.r_path,
            burst_cap: N as u32,
        })
    }

    fn amp_limit(&self, m: u8) -> f32 {
        match m {
            // six-step commands a PWM duty, not volts or amps
            mode::SS_FORCED | mode::SS_SENSORLESS | mode::SS_HALL => self.cfg.spec.max_duty,
            mode::VOLT => V_AMP_MAX,
            _ => self.param(param::IQ_LIMIT),
        }
    }

    fn handle_volatile(&self, msg: &Message) -> Message {
        let ack = Message::Ack {
            of: msg.wire_type(),
        };
        let nak = |err| Message::Nak {
            of: msg.wire_type(),
            err,
        };
        match *msg {
            Message::Ping { nonce } => Message::Pong { nonce },
            Message::GetInfo => Message::Info(self.info()),
            Message::SetTelemetry { divider, mask } => {
                self.divider.store(divider.max(1) as u32, Ordering::Relaxed);
                self.mask.store(mask & channel::ALL, Ordering::Relaxed);
                ack
            }
            Message::Stream { enable } => {
                self.streaming.store(enable, Ordering::Relaxed);
                ack
            }
            Message::SetDrive(drive) => {
                let state = self.state();
                let (m, amp, omega) = match drive {
                    DriveMode::Off => (mode::OFF, 0.0f32, 0.0f32),
                    DriveMode::OpenLoopVoltage { volts, omega_e } => (mode::VOLT, volts, omega_e),
                    DriveMode::IfCurrent { amps, omega_e } => (mode::IF, amps, omega_e),
                    DriveMode::Sensorless { amps, omega_e } => (mode::SENSORLESS, amps, omega_e),
                    DriveMode::SixStepForced { duty, omega_e } => (mode::SS_FORCED, duty, omega_e),
                    DriveMode::SixStepSensorless {
                        duty,
                        omega_handoff,
                    } => (mode::SS_SENSORLESS, duty, omega_handoff),
                    DriveMode::HallFoc { amps, omega_e } => (mode::HALL_FOC, amps, omega_e),
                    DriveMode::SixStepHall { duty, omega_e } => (mode::SS_HALL, duty, omega_e),
                    DriveMode::HallPosition { amps, theta_m } => (mode::HALL_POS, amps, theta_m),
                };
                // `clamp` passes NaN through; never let one reach the loop.
                if !amp.is_finite() || !omega.is_finite() {
                    return nak(1);
                }
                if matches!(m, mode::HALL_FOC | mode::SS_HALL | mode::HALL_POS)
                    && !self.cfg.spec.has_halls
                {
                    return nak(1); // this board has no hall inputs
                }
                if m != mode::OFF {
                    if state == ST_CAL {
                        return nak(3); // still calibrating
                    }
                    if state >= ST_FAULT_OC {
                        return nak(2); // faulted: requires Off first
                    }
                }
                let lim = self.amp_limit(m);
                self.cmd_amp
                    .store(amp.clamp(-lim, lim).to_bits(), Ordering::Relaxed);
                self.cmd_omega.store(
                    omega.clamp(-OMEGA_E_MAX, OMEGA_E_MAX).to_bits(),
                    Ordering::Relaxed,
                );
                self.cmd_mode.store(m, Ordering::Relaxed);
                self.cmd_epoch.fetch_add(1, Ordering::Release);
                ack
            }
            Message::RunTest { kind, a, b } => {
                let fits = match kind {
                    // Any length works; a short buffer just folds fewer edges.
                    test::RL_STEP => N >= 1024,
                    test::L_THETA => N >= probe::SAL_HDR + probe::SAL_TICKS * 2,
                    test::HFI_SWEEP => N >= mmc_core::hfi::LEN,
                    _ => false,
                };
                if !fits || !a.is_finite() || !b.is_finite() {
                    return nak(1);
                }
                let state = self.state();
                if state == ST_CAL {
                    return nak(3);
                }
                if state >= ST_FAULT_OC {
                    return nak(2);
                }
                // Only from a quiet stage, and not while a probe is recording.
                if self.cmd_mode.load(Ordering::Relaxed) != mode::OFF
                    || state == ST_RUN
                    || self.burst_state.load(Ordering::Relaxed) == BURST_RECORDING
                {
                    return nak(2);
                }
                // The saliency sweep bounds its steady-state plateau below the
                // software trip using the live R estimate, and picks its
                // square-wave half-period from the live τ = L/R so the
                // plateaus settle (profile + apply R/L before running it).
                // RL_STEP keeps its hardware-validated V_AMP_MAX clamp.
                let v_max = if kind == test::L_THETA {
                    let tau_ticks =
                        self.param(param::L) / self.param(param::R) * self.cfg.spec.ctrl_hz as f32;
                    self.probe_half
                        .store(probe::sal_half_ticks(tau_ticks) as u32, Ordering::Relaxed);
                    (0.75 * self.cfg.spec.i_trip * self.param(param::R)).min(V_AMP_MAX)
                } else {
                    V_AMP_MAX
                };
                if kind == test::HFI_SWEEP {
                    // a: carrier amplitude; b: align angle, as given.
                    self.probe_v_step
                        .store(a.clamp(0.05, v_max).to_bits(), Ordering::Relaxed);
                    self.probe_v_align.store(b.to_bits(), Ordering::Relaxed);
                } else {
                    self.probe_v_align
                        .store(a.clamp(0.05, v_max).to_bits(), Ordering::Relaxed);
                    self.probe_v_step
                        .store(b.clamp(0.05, v_max).to_bits(), Ordering::Relaxed);
                }
                self.probe_kind.store(kind, Ordering::Relaxed);
                self.burst_len.store(0, Ordering::Relaxed);
                self.burst_state.store(BURST_RECORDING, Ordering::Relaxed);
                self.cmd_mode.store(mode::PROBE, Ordering::Relaxed);
                self.cmd_epoch.fetch_add(1, Ordering::Release);
                ack
            }
            Message::ReadBurst { offset } => {
                if self.burst_state.load(Ordering::Acquire) != BURST_DONE {
                    return nak(4); // no finished recording to read
                }
                let len = self.burst_len.load(Ordering::Relaxed) as usize;
                let off = (offset as usize).min(len);
                // Safety: the ISR only writes while recording.
                let buf = unsafe { &*self.burst.0.get() };
                let n = (len - off).min(mmc_proto::BURST_CHUNK);
                match BurstChunk::new(off as u16, len as u16, &buf[off..off + n]) {
                    Some(chunk) => Message::BurstData(chunk),
                    None => nak(1),
                }
            }
            Message::SetParam { id, value } => match self.cfg.param_range(id) {
                Some((lo, hi)) if (lo..=hi).contains(&value) => {
                    self.params[id as usize].store(value.to_bits(), Ordering::Relaxed);
                    ack
                }
                _ => nak(1),
            },
            Message::GetParam { id } => {
                if (id as usize) < param::COUNT {
                    Message::ParamValue {
                        id,
                        value: self.param(id),
                    }
                } else {
                    nak(1)
                }
            }
            // Adjust the I-f current target on the fly; otherwise ignored.
            Message::SetIqRef { iq } => {
                if !iq.is_finite() {
                    return nak(1);
                }
                if self.cmd_mode.load(Ordering::Relaxed) == mode::IF {
                    let lim = self.param(param::IQ_LIMIT);
                    self.cmd_amp
                        .store(iq.clamp(-lim, lim).to_bits(), Ordering::Relaxed);
                    self.cmd_epoch.fetch_add(1, Ordering::Release);
                }
                ack
            }
            _ => nak(1),
        }
    }

    /// Interval between telemetry frames [µs]: the divider counts control
    /// periods (the protocol's definition), so it scales with the board.
    ///
    /// `⌊d·10⁶/hz⌋` in 32-bit steps, `d·⌊10⁶/hz⌋ + ⌊d·(10⁶ mod hz)/hz⌋`: a
    /// 64-bit division pulls ~900 bytes of `u64_div_rem` into the image. Exact
    /// while `d·(10⁶ mod hz)` fits, which a `u16` divider guarantees for any
    /// `ctrl_hz ≤ 65 536`; beyond that the remainder term saturates.
    pub fn telemetry_period_us(&self) -> u64 {
        let d = self.divider.load(Ordering::Relaxed).max(1);
        let hz = self.cfg.spec.ctrl_hz;
        let whole = d as u64 * (1_000_000 / hz) as u64;
        whole + (d.saturating_mul(1_000_000 % hz) / hz) as u64
    }

    /// A consistent snapshot of the selected channels, or `None` while not
    /// streaming. Seqlock read: retries while the ISR is mid-update.
    pub fn telemetry(&self) -> Option<Message> {
        if !self.streaming.load(Ordering::Relaxed) {
            return None;
        }
        let mask = self.mask.load(Ordering::Relaxed);
        let us_per_tick = 1_000_000 / self.cfg.spec.ctrl_hz;
        let mut values = [0f32; channel::COUNT];
        let (t_us, n) = loop {
            let seq = self.telem_seq.load(Ordering::Acquire);
            if seq & 1 != 0 {
                continue;
            }
            let ticks = self.control_ticks.load(Ordering::Relaxed);
            let mut n = 0;
            for id in 0..channel::COUNT as u8 {
                if mask & (1 << id) == 0 {
                    continue;
                }
                values[n] = f32::from_bits(self.telem[id as usize].load(Ordering::Relaxed));
                n += 1;
            }
            if self.telem_seq.load(Ordering::Acquire) == seq {
                break (ticks.wrapping_mul(us_per_tick), n);
            }
        };
        TelemetryFrame::new(t_us, mask, &values[..n]).map(Message::Telemetry)
    }

    /// A drive abort with a probe in flight still hands the (partial) buffer
    /// to the host — a short read beats a hung poll loop.
    pub(crate) fn burst_abort(&self) {
        if self.burst_state.load(Ordering::Relaxed) == BURST_RECORDING {
            self.burst_state.store(BURST_DONE, Ordering::Release);
        }
    }
}
