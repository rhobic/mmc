//! Message definitions and frame encode/parse.

use crate::{cobs, crc::crc16};

/// Largest payload: telemetry with a full channel mask
/// (`t_us` + mask + [`MAX_CHANNELS`] × f32).
pub const MAX_PAYLOAD: usize = 8 + MAX_CHANNELS * 4;

/// Cap on simultaneously streamed channels (mask bits 0..24).
pub const MAX_CHANNELS: usize = 24;

mod ty {
    pub const PING: u8 = 0x01;
    pub const GET_INFO: u8 = 0x02;
    pub const SET_TELEMETRY: u8 = 0x03;
    pub const STREAM: u8 = 0x04;
    pub const SET_IQ_REF: u8 = 0x05;
    pub const SET_DRIVE: u8 = 0x06;
    pub const RUN_TEST: u8 = 0x07;
    pub const READ_BURST: u8 = 0x08;
    pub const SET_PARAM: u8 = 0x09;
    pub const GET_PARAM: u8 = 0x0A;
    pub const SAVE_PARAMS: u8 = 0x0B;
    pub const ERASE_PARAMS: u8 = 0x0C;
    pub const PONG: u8 = 0x81;
    pub const INFO: u8 = 0x82;
    pub const TELEMETRY: u8 = 0x83;
    pub const ACK: u8 = 0x84;
    pub const NAK: u8 = 0x85;
    pub const BURST_DATA: u8 = 0x86;
    pub const PARAM_VALUE: u8 = 0x87;
}

/// Profiler test-sequence ids for [`Message::RunTest`] (MS6). The firmware
/// executes the sequence and records per-control-tick samples into its burst
/// buffer; the host reads them back with [`Message::ReadBurst`] and does the
/// fitting (`tools/profile.py`).
pub mod test {
    /// Locked-rotor R/L probe: align the rotor with `a` volts on the d-axis,
    /// then step (unslewed) to `b` volts, recording (i_d, v_d) per tick.
    /// The two levels make the fit differential — dead-time distortion and
    /// offsets cancel.
    pub const RL_STEP: u8 = 0;
    /// Saliency (Ld/Lq) sweep: align at `a` volts on θ=0, then square-wave
    /// between `a` and `b` volts along a firmware-owned schedule of ±paired
    /// electrical angles (`mmc_core::probe`), recording (i_d, i_q) in the
    /// excitation frame per tick behind a self-describing header. The i_q
    /// transient is a null channel: at DC it is zero at every angle unless
    /// Ld ≠ Lq. No rotor clamping required — the ± pairing cancels net
    /// torque impulse and the fit recovers the rotor angle as a parameter.
    pub const L_THETA: u8 = 1;
}

/// Runtime device parameter ids for [`Message::SetParam`] / `GetParam` —
/// the profiler's writeback target. Values apply at the next clean drive
/// start; they live in RAM (flash persistence is future work).
pub mod param {
    pub const R: u8 = 0; // stator resistance [Ω]
    pub const L: u8 = 1; // stator inductance [H]
    pub const FLUX: u8 = 2; // rotor flux linkage [Wb]
    pub const CUR_BW: u8 = 3; // current-loop bandwidth [rad/s]
    pub const SPEED_KP: u8 = 4; // speed PI kp [A/(rad/s el)]
    pub const SPEED_KI: u8 = 5; // speed PI ki
    /// Motor pole pairs (f32 on the wire like every param; integer-valued).
    /// Scales the mechanical-speed telemetry and the host's kt/J fits only —
    /// the electrical control loops never consume it.
    pub const POLE_PAIRS: u8 = 6;
    /// Sensorless I-f→observer handoff speed [rad/s electrical]. Must sit
    /// above the observer's trust floor (~4× its leak) and below what the
    /// motor can reach open-loop from rest.
    pub const SL_HANDOFF: u8 = 7;
    /// Electrical acceleration for every commanded ramp [rad/s²]: forced-mode
    /// (volt/I-f) frequency slew, the sensorless startup ramp, and speed
    /// retargets. A heavy or high-drag rotor needs this lowered to hold sync.
    pub const OMEGA_ACCEL: u8 = 8;
    /// Current-amplitude ceiling [A] for the I-f/sensorless drives: clamps
    /// the commanded amplitude, the startup current, and the speed loop's
    /// i_q authority. Keep under the 1.5 A trips with margin.
    pub const IQ_LIMIT: u8 = 9;
    /// Six-step idle-phase sample point, in timer counts before the PWM
    /// counter valley (TIM1 CCR5). The valley is the middle of the high-side
    /// on-time, so this places the ADC trigger inside the on-window; it must
    /// stay below the commanded duty's compare value or the sample lands
    /// while the bridge is freewheeling. Larger values sample earlier and
    /// leave the sense network less time to settle.
    pub const ONTIME_CCR5: u8 = 10;
    /// Six-step duty→speed loop gains [duty per rad/s electrical, and its
    /// integral]. Deliberately separate from [`SPEED_KP`]/[`SPEED_KI`], which
    /// belong to the FOC speed loop and are in amps: the two control schemes
    /// must not share a tuning knob.
    pub const SS_KP: u8 = 11;
    pub const SS_KI: u8 = 12;
    /// Inverter dead-time voltage error [V] and the half-width of its
    /// zero-current band [A] — the two numbers `profile --only vdead` fits.
    /// The FOC modulator adds `v_dead·clip(i/i_thresh)` back per phase, so
    /// what reaches the winding is what the controller asked for.
    ///
    /// `v_dead = 0` disables the compensation, and that is the default:
    /// over-compensating is worse than not compensating, because the
    /// correction then drives the current back across zero and the error
    /// reverses sign underneath it. Measure before enabling.
    pub const V_DEAD: u8 = 13;
    pub const I_THRESH: u8 = 14;
    /// Hall calibration (`mmc_core::hall::HallMap`): electrical angle [rad]
    /// of the center of the first state in the Gray-code sequence, and the
    /// direction (+1 / −1) that sequence runs in. `tools/hall_cal.py` fits
    /// both from two slow open-loop runs; the hall drive modes need them.
    pub const HALL_OFFSET: u8 = 15;
    pub const HALL_DIR: u8 = 16;
    /// Hall switching hysteresis [rad electrical]: how far past its
    /// midpoint each edge fires in the direction of travel. Measured against
    /// the flux observer at speed (`tools/hall_ref.py`); 0 until then.
    pub const HALL_HYST: u8 = 17;
    pub const COUNT: usize = 18;
    pub const NAMES: [&str; COUNT] = [
        "r",
        "l",
        "flux",
        "cur_bw",
        "speed_kp",
        "speed_ki",
        "pole_pairs",
        "sl_handoff",
        "omega_accel",
        "iq_limit",
        "ontime_ccr5",
        "ss_kp",
        "ss_ki",
        "v_dead",
        "i_thresh",
        "hall_offset",
        "hall_dir",
        "hall_hyst",
    ];
}

/// Max f32 values per [`BurstChunk`] frame (fits [`MAX_PAYLOAD`]).
pub const BURST_CHUNK: usize = 20;

/// One slice of the device's burst buffer: `values` are `total` samples long
/// overall, this frame carrying `values.len()` of them starting at `offset`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BurstChunk {
    pub offset: u16,
    pub total: u16,
    count: u8,
    values: [f32; BURST_CHUNK],
}

impl BurstChunk {
    pub fn new(offset: u16, total: u16, values: &[f32]) -> Option<Self> {
        if values.len() > BURST_CHUNK {
            return None;
        }
        let mut buf = [0f32; BURST_CHUNK];
        buf[..values.len()].copy_from_slice(values);
        Some(Self {
            offset,
            total,
            count: values.len() as u8,
            values: buf,
        })
    }

    pub fn values(&self) -> &[f32] {
        &self.values[..self.count as usize]
    }
}

/// Power-stage drive request. `Off` is always accepted; a faulted device NAKs
/// everything else until it sees `Off` (the fault re-arm).
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DriveMode {
    /// Power stage disabled (outputs high-impedance).
    Off,
    /// Open-loop rotating voltage vector: `amplitude` volts at `omega_e`
    /// rad/s electrical. The first spin of a bring-up.
    OpenLoopVoltage { volts: f32, omega_e: f32 },
    /// I-f drive: closed current loop on a forced rotating angle —
    /// `amplitude` amps q-axis at `omega_e` rad/s electrical.
    IfCurrent { amps: f32, omega_e: f32 },
    /// Closed-loop sensorless: I-f startup at `amps`, handoff to the flux
    /// observer, then the on-device speed loop regulates to `omega_e` rad/s
    /// electrical (sign sets direction; retargetable while running).
    Sensorless { amps: f32, omega_e: f32 },
    /// Forced six-step commutation: two phases conduct, the third is Hi-Z so
    /// it can be read as a back-EMF sense node. `duty` is the high-side PWM
    /// duty (0..1), not volts or amps — there is no current loop here. The
    /// sector advances with a forced `omega_e`, dragging the rotor like
    /// open-loop voltage does. Wire code 5: code 4 is taken by the on-device
    /// R/L probe, which arrives as `RunTest` rather than `SetDrive`.
    SixStepForced { duty: f32, omega_e: f32 },
    /// Sensorless six-step: forced ramp up to `omega_handoff`, then
    /// commutation timed from measured back-EMF zero-crossings on the idle
    /// phase. `duty` is the high-side PWM duty throughout. Wire code 6.
    SixStepSensorless { duty: f32, omega_handoff: f32 },
    /// Hall-sensored FOC: the rotor angle comes from the calibrated halls
    /// (`hall_offset`/`hall_dir`), so the speed loop closes from standstill
    /// with no I-f ramp. `amps` is the speed loop's i_q authority (clamped to
    /// `iq_limit`), `omega_e` the speed target (sign = direction,
    /// retargetable). NAKed by a board without halls. Wire code 7.
    HallFoc { amps: f32, omega_e: f32 },
    /// Hall-sensored six-step: commutation from the hall angle, both
    /// directions, a duty→speed loop (`ss_kp`/`ss_ki`) toward `omega_e`.
    /// `duty` is the duty ceiling. Wire code 8.
    SixStepHall { duty: f32, omega_e: f32 },
}

impl DriveMode {
    fn to_wire(self) -> (u8, f32, f32) {
        match self {
            DriveMode::Off => (0, 0.0, 0.0),
            DriveMode::OpenLoopVoltage { volts, omega_e } => (1, volts, omega_e),
            DriveMode::IfCurrent { amps, omega_e } => (2, amps, omega_e),
            DriveMode::Sensorless { amps, omega_e } => (3, amps, omega_e),
            DriveMode::SixStepForced { duty, omega_e } => (5, duty, omega_e),
            DriveMode::SixStepSensorless {
                duty,
                omega_handoff,
            } => (6, duty, omega_handoff),
            DriveMode::HallFoc { amps, omega_e } => (7, amps, omega_e),
            DriveMode::SixStepHall { duty, omega_e } => (8, duty, omega_e),
        }
    }

    fn from_wire(mode: u8, amp: f32, omega_e: f32) -> Result<Self, FrameError> {
        match mode {
            0 => Ok(DriveMode::Off),
            1 => Ok(DriveMode::OpenLoopVoltage {
                volts: amp,
                omega_e,
            }),
            2 => Ok(DriveMode::IfCurrent { amps: amp, omega_e }),
            3 => Ok(DriveMode::Sensorless { amps: amp, omega_e }),
            5 => Ok(DriveMode::SixStepForced { duty: amp, omega_e }),
            6 => Ok(DriveMode::SixStepSensorless {
                duty: amp,
                omega_handoff: omega_e,
            }),
            7 => Ok(DriveMode::HallFoc { amps: amp, omega_e }),
            8 => Ok(DriveMode::SixStepHall { duty: amp, omega_e }),
            _ => Err(FrameError::Malformed),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DeviceKind {
    Sim,
    BoardG0b1,
    BoardG474,
    /// STM32F302R8 dev board + L6230 three-shunt inverter shield.
    BoardF302,
    Unknown(u8),
}

impl DeviceKind {
    fn to_wire(self) -> u8 {
        match self {
            DeviceKind::Sim => 0,
            DeviceKind::BoardG0b1 => 1,
            DeviceKind::BoardG474 => 2,
            DeviceKind::BoardF302 => 3,
            DeviceKind::Unknown(v) => v,
        }
    }

    fn from_wire(v: u8) -> Self {
        match v {
            0 => DeviceKind::Sim,
            1 => DeviceKind::BoardG0b1,
            2 => DeviceKind::BoardG474,
            3 => DeviceKind::BoardF302,
            v => DeviceKind::Unknown(v),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    pub proto_version: u8,
    pub kind: DeviceKind,
    /// Firmware/build version, device-defined.
    pub fw_version: u16,
    /// Zero-padded ASCII device name.
    pub name: [u8; 12],
    /// What a motor board knows about itself that the host would otherwise
    /// have to hard-code per board. Optional on the wire (a trailing block),
    /// so firmware that predates it still decodes — as `None`.
    pub board: Option<BoardTraits>,
}

/// Board facts the host needs to interpret captures from a motor board.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BoardTraits {
    /// Control-loop rate [Hz]: the time base of every burst recording.
    pub ctrl_hz: u32,
    /// Series resistance of the drive path (switches + shunt) [Ω], which the
    /// R/L probe measures on top of the winding.
    pub r_path: f32,
    /// Burst-buffer capacity [f32s]. A probe whose schedule does not fit is
    /// NAKed by the device; the host uses this to skip it up front.
    pub burst_cap: u32,
}

impl DeviceInfo {
    pub fn new(kind: DeviceKind, fw_version: u16, name: &str) -> Self {
        let mut buf = [0u8; 12];
        let n = name.len().min(12);
        buf[..n].copy_from_slice(&name.as_bytes()[..n]);
        Self {
            proto_version: crate::PROTO_VERSION,
            kind,
            fw_version,
            name: buf,
            board: None,
        }
    }

    pub fn with_board(mut self, board: BoardTraits) -> Self {
        self.board = Some(board);
        self
    }

    /// Control rate, falling back to 20 kHz for firmware that predates
    /// [`BoardTraits`] (every such motor board ran the loop at 20 kHz).
    pub fn ctrl_hz(&self) -> f32 {
        self.board.map_or(20_000.0, |b| b.ctrl_hz as f32)
    }

    pub fn name_str(&self) -> &str {
        let end = self.name.iter().position(|&b| b == 0).unwrap_or(12);
        core::str::from_utf8(&self.name[..end]).unwrap_or("<non-utf8>")
    }
}

/// One telemetry sample: values for the channels set in `mask`, packed in
/// ascending channel-id order.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TelemetryFrame {
    /// Device-local timestamp, wrapping microseconds.
    pub t_us: u32,
    pub mask: u32,
    values: [f32; MAX_CHANNELS],
}

impl TelemetryFrame {
    /// `values` must hold exactly one value per set mask bit, in ascending
    /// channel order. Returns `None` on count mismatch or oversized mask.
    pub fn new(t_us: u32, mask: u32, values: &[f32]) -> Option<Self> {
        if mask >= 1 << MAX_CHANNELS as u32 || values.len() != mask.count_ones() as usize {
            return None;
        }
        let mut buf = [0f32; MAX_CHANNELS];
        buf[..values.len()].copy_from_slice(values);
        Some(Self {
            t_us,
            mask,
            values: buf,
        })
    }

    pub fn values(&self) -> &[f32] {
        &self.values[..self.mask.count_ones() as usize]
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Message {
    /// Liveness check; the peer echoes the nonce in [`Message::Pong`].
    Ping {
        nonce: u32,
    },
    Pong {
        nonce: u32,
    },
    GetInfo,
    Info(DeviceInfo),
    /// Select streamed channels and the sample-rate divider
    /// (`0`/`1` = every control period, `n` = every n-th).
    SetTelemetry {
        divider: u16,
        mask: u32,
    },
    Stream {
        enable: bool,
    },
    /// Torque-mode q-axis current reference [A].
    SetIqRef {
        iq: f32,
    },
    /// Power-stage drive request (hardware bring-up and open-loop testing).
    SetDrive(DriveMode),
    /// Run a profiler test sequence (see [`test`]); `a`/`b` are
    /// sequence-defined. Device must be calibrated, off and unfaulted.
    RunTest {
        kind: u8,
        a: f32,
        b: f32,
    },
    /// Read `BURST_CHUNK` samples of the burst buffer starting at `offset`.
    /// NAKed until the armed test has finished recording.
    ReadBurst {
        offset: u16,
    },
    /// Write a runtime parameter (see [`param`]); applied at next drive start.
    SetParam {
        id: u8,
        value: f32,
    },
    GetParam {
        id: u8,
    },
    /// Persist the current runtime parameter table to flash so it survives a
    /// power cycle (device restores it at boot). Acked when written.
    SaveParams,
    /// Erase the persisted parameters; the device reverts to firmware
    /// defaults on the next boot.
    EraseParams,
    ParamValue {
        id: u8,
        value: f32,
    },
    BurstData(BurstChunk),
    Ack {
        of: u8,
    },
    Nak {
        of: u8,
        err: u8,
    },
    Telemetry(TelemetryFrame),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// Frame exceeded the receive buffer; stream resynchronized.
    TooLong,
    /// COBS structure invalid.
    Cobs,
    /// Frame shorter than type + CRC.
    TooShort,
    Crc,
    UnknownType(u8),
    /// Payload length/content inconsistent with the message type.
    Malformed,
}

/// Encode `msg` as a complete wire frame (COBS + `0x00` delimiter) into `out`.
/// Returns the frame length. `out` of [`crate::MAX_FRAME`] bytes never fails.
pub fn encode(msg: &Message, out: &mut [u8]) -> Option<usize> {
    let mut raw = [0u8; crate::MAX_RAW];
    let n = serialize(msg, &mut raw)?;
    let crc = crc16(&raw[..n]);
    raw[n] = crc as u8;
    raw[n + 1] = (crc >> 8) as u8;
    let enc = cobs::encode(&raw[..n + 2], out)?;
    if enc >= out.len() {
        return None;
    }
    out[enc] = 0;
    Some(enc + 1)
}

/// Parse the raw (COBS-decoded) contents of one frame: type + payload + CRC.
pub fn parse(raw: &[u8]) -> Result<Message, FrameError> {
    if raw.len() < 3 {
        return Err(FrameError::TooShort);
    }
    let (body, crc_bytes) = raw.split_at(raw.len() - 2);
    let expect = u16::from_le_bytes([crc_bytes[0], crc_bytes[1]]);
    if crc16(body) != expect {
        return Err(FrameError::Crc);
    }
    let (&t, payload) = body.split_first().ok_or(FrameError::TooShort)?;
    let mut r = Reader(payload);
    let msg = match t {
        ty::PING => Message::Ping { nonce: r.u32()? },
        ty::PONG => Message::Pong { nonce: r.u32()? },
        ty::GET_INFO => Message::GetInfo,
        ty::INFO => {
            let proto_version = r.u8()?;
            let kind = DeviceKind::from_wire(r.u8()?);
            let fw_version = r.u16()?;
            let mut name = [0u8; 12];
            name.copy_from_slice(r.bytes(12)?);
            let board = if r.0.is_empty() {
                None
            } else {
                Some(BoardTraits {
                    ctrl_hz: r.u32()?,
                    r_path: r.f32()?,
                    burst_cap: r.u32()?,
                })
            };
            Message::Info(DeviceInfo {
                proto_version,
                kind,
                fw_version,
                name,
                board,
            })
        }
        ty::SET_TELEMETRY => Message::SetTelemetry {
            divider: r.u16()?,
            mask: r.u32()?,
        },
        ty::STREAM => Message::Stream {
            enable: r.u8()? != 0,
        },
        ty::SET_IQ_REF => Message::SetIqRef { iq: r.f32()? },
        ty::RUN_TEST => Message::RunTest {
            kind: r.u8()?,
            a: r.f32()?,
            b: r.f32()?,
        },
        ty::READ_BURST => Message::ReadBurst { offset: r.u16()? },
        ty::SET_PARAM => Message::SetParam {
            id: r.u8()?,
            value: r.f32()?,
        },
        ty::GET_PARAM => Message::GetParam { id: r.u8()? },
        ty::SAVE_PARAMS => Message::SaveParams,
        ty::ERASE_PARAMS => Message::EraseParams,
        ty::PARAM_VALUE => Message::ParamValue {
            id: r.u8()?,
            value: r.f32()?,
        },
        ty::BURST_DATA => {
            let offset = r.u16()?;
            let total = r.u16()?;
            let count = r.u8()?;
            if count as usize > BURST_CHUNK {
                return Err(FrameError::Malformed);
            }
            let mut values = [0f32; BURST_CHUNK];
            for v in values.iter_mut().take(count as usize) {
                *v = r.f32()?;
            }
            Message::BurstData(BurstChunk {
                offset,
                total,
                count,
                values,
            })
        }
        ty::SET_DRIVE => {
            let mode = r.u8()?;
            let amp = r.f32()?;
            let omega_e = r.f32()?;
            Message::SetDrive(DriveMode::from_wire(mode, amp, omega_e)?)
        }
        ty::ACK => Message::Ack { of: r.u8()? },
        ty::NAK => Message::Nak {
            of: r.u8()?,
            err: r.u8()?,
        },
        ty::TELEMETRY => {
            let t_us = r.u32()?;
            let mask = r.u32()?;
            if mask >= 1 << MAX_CHANNELS as u32 {
                return Err(FrameError::Malformed);
            }
            let n = mask.count_ones() as usize;
            let mut values = [0f32; MAX_CHANNELS];
            for v in values.iter_mut().take(n) {
                *v = r.f32()?;
            }
            Message::Telemetry(TelemetryFrame { t_us, mask, values })
        }
        other => return Err(FrameError::UnknownType(other)),
    };
    if !r.0.is_empty() {
        return Err(FrameError::Malformed);
    }
    Ok(msg)
}

fn serialize(msg: &Message, raw: &mut [u8]) -> Option<usize> {
    let mut w = Writer { buf: raw, pos: 0 };
    match msg {
        Message::Ping { nonce } => {
            w.u8(ty::PING)?;
            w.u32(*nonce)?;
        }
        Message::Pong { nonce } => {
            w.u8(ty::PONG)?;
            w.u32(*nonce)?;
        }
        Message::GetInfo => w.u8(ty::GET_INFO)?,
        Message::Info(info) => {
            w.u8(ty::INFO)?;
            w.u8(info.proto_version)?;
            w.u8(info.kind.to_wire())?;
            w.u16(info.fw_version)?;
            w.bytes(&info.name)?;
            if let Some(b) = info.board {
                w.u32(b.ctrl_hz)?;
                w.f32(b.r_path)?;
                w.u32(b.burst_cap)?;
            }
        }
        Message::SetTelemetry { divider, mask } => {
            w.u8(ty::SET_TELEMETRY)?;
            w.u16(*divider)?;
            w.u32(*mask)?;
        }
        Message::Stream { enable } => {
            w.u8(ty::STREAM)?;
            w.u8(*enable as u8)?;
        }
        Message::SetIqRef { iq } => {
            w.u8(ty::SET_IQ_REF)?;
            w.f32(*iq)?;
        }
        Message::SetDrive(mode) => {
            let (m, amp, omega_e) = mode.to_wire();
            w.u8(ty::SET_DRIVE)?;
            w.u8(m)?;
            w.f32(amp)?;
            w.f32(omega_e)?;
        }
        Message::RunTest { kind, a, b } => {
            w.u8(ty::RUN_TEST)?;
            w.u8(*kind)?;
            w.f32(*a)?;
            w.f32(*b)?;
        }
        Message::ReadBurst { offset } => {
            w.u8(ty::READ_BURST)?;
            w.u16(*offset)?;
        }
        Message::SetParam { id, value } => {
            w.u8(ty::SET_PARAM)?;
            w.u8(*id)?;
            w.f32(*value)?;
        }
        Message::GetParam { id } => {
            w.u8(ty::GET_PARAM)?;
            w.u8(*id)?;
        }
        Message::SaveParams => w.u8(ty::SAVE_PARAMS)?,
        Message::EraseParams => w.u8(ty::ERASE_PARAMS)?,
        Message::ParamValue { id, value } => {
            w.u8(ty::PARAM_VALUE)?;
            w.u8(*id)?;
            w.f32(*value)?;
        }
        Message::BurstData(c) => {
            w.u8(ty::BURST_DATA)?;
            w.u16(c.offset)?;
            w.u16(c.total)?;
            w.u8(c.count)?;
            for &v in c.values() {
                w.f32(v)?;
            }
        }
        Message::Ack { of } => {
            w.u8(ty::ACK)?;
            w.u8(*of)?;
        }
        Message::Nak { of, err } => {
            w.u8(ty::NAK)?;
            w.u8(*of)?;
            w.u8(*err)?;
        }
        Message::Telemetry(f) => {
            w.u8(ty::TELEMETRY)?;
            w.u32(f.t_us)?;
            w.u32(f.mask)?;
            for &v in f.values() {
                w.f32(v)?;
            }
        }
    }
    Some(w.pos)
}

impl Message {
    /// The wire type id, e.g. for `Ack { of }`.
    pub fn wire_type(&self) -> u8 {
        match self {
            Message::Ping { .. } => ty::PING,
            Message::Pong { .. } => ty::PONG,
            Message::GetInfo => ty::GET_INFO,
            Message::Info(_) => ty::INFO,
            Message::SetTelemetry { .. } => ty::SET_TELEMETRY,
            Message::Stream { .. } => ty::STREAM,
            Message::SetIqRef { .. } => ty::SET_IQ_REF,
            Message::SetDrive(_) => ty::SET_DRIVE,
            Message::RunTest { .. } => ty::RUN_TEST,
            Message::ReadBurst { .. } => ty::READ_BURST,
            Message::SetParam { .. } => ty::SET_PARAM,
            Message::GetParam { .. } => ty::GET_PARAM,
            Message::SaveParams => ty::SAVE_PARAMS,
            Message::EraseParams => ty::ERASE_PARAMS,
            Message::ParamValue { .. } => ty::PARAM_VALUE,
            Message::BurstData(_) => ty::BURST_DATA,
            Message::Ack { .. } => ty::ACK,
            Message::Nak { .. } => ty::NAK,
            Message::Telemetry(_) => ty::TELEMETRY,
        }
    }
}

struct Reader<'a>(&'a [u8]);

impl<'a> Reader<'a> {
    fn bytes(&mut self, n: usize) -> Result<&'a [u8], FrameError> {
        if self.0.len() < n {
            return Err(FrameError::Malformed);
        }
        let (head, tail) = self.0.split_at(n);
        self.0 = tail;
        Ok(head)
    }
    fn u8(&mut self) -> Result<u8, FrameError> {
        Ok(self.bytes(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, FrameError> {
        Ok(u16::from_le_bytes(self.bytes(2)?.try_into().unwrap()))
    }
    fn u32(&mut self) -> Result<u32, FrameError> {
        Ok(u32::from_le_bytes(self.bytes(4)?.try_into().unwrap()))
    }
    fn f32(&mut self) -> Result<f32, FrameError> {
        Ok(f32::from_le_bytes(self.bytes(4)?.try_into().unwrap()))
    }
}

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Writer<'_> {
    fn bytes(&mut self, data: &[u8]) -> Option<()> {
        let end = self.pos + data.len();
        if end > self.buf.len() {
            return None;
        }
        self.buf[self.pos..end].copy_from_slice(data);
        self.pos = end;
        Some(())
    }
    fn u8(&mut self, v: u8) -> Option<()> {
        self.bytes(&[v])
    }
    fn u16(&mut self, v: u16) -> Option<()> {
        self.bytes(&v.to_le_bytes())
    }
    fn u32(&mut self, v: u32) -> Option<()> {
        self.bytes(&v.to_le_bytes())
    }
    fn f32(&mut self, v: f32) -> Option<()> {
        self.bytes(&v.to_le_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Deframer;

    fn round_trip(msg: Message) {
        let mut wire = [0u8; crate::MAX_FRAME];
        let n = encode(&msg, &mut wire).unwrap();
        assert_eq!(wire[n - 1], 0, "delimiter terminates the frame");
        assert!(!wire[..n - 1].contains(&0), "zero-free before delimiter");

        let mut d = Deframer::new();
        let mut got = None;
        for &b in &wire[..n] {
            if let Some(res) = d.push(b) {
                assert!(got.is_none(), "one frame in, one message out");
                got = Some(res.unwrap());
            }
        }
        assert_eq!(got.unwrap(), msg);
    }

    #[test]
    fn all_messages_round_trip() {
        round_trip(Message::Ping { nonce: 0xDEAD_BEEF });
        round_trip(Message::Pong { nonce: 0 });
        round_trip(Message::GetInfo);
        round_trip(Message::Info(DeviceInfo::new(
            DeviceKind::BoardG0b1,
            0x0102,
            "mmc-g0b1",
        )));
        round_trip(Message::Info(
            DeviceInfo::new(DeviceKind::BoardF302, 1, "mmc-f302").with_board(BoardTraits {
                ctrl_hz: 10_000,
                r_path: 1.8,
                burst_cap: 2048,
            }),
        ));
        round_trip(Message::SetTelemetry {
            divider: 10,
            mask: crate::channel::ALL,
        });
        round_trip(Message::Stream { enable: true });
        round_trip(Message::SetIqRef { iq: -1.25 });
        round_trip(Message::SetDrive(DriveMode::Off));
        round_trip(Message::SetDrive(DriveMode::OpenLoopVoltage {
            volts: 1.5,
            omega_e: 125.6,
        }));
        round_trip(Message::SetDrive(DriveMode::IfCurrent {
            amps: 0.4,
            omega_e: -62.8,
        }));
        round_trip(Message::SetDrive(DriveMode::SixStepForced {
            duty: 0.25,
            omega_e: 40.0,
        }));
        round_trip(Message::SetDrive(DriveMode::Sensorless {
            amps: 0.5,
            omega_e: 600.0,
        }));
        round_trip(Message::SetDrive(DriveMode::HallFoc {
            amps: 1.0,
            omega_e: -300.0,
        }));
        round_trip(Message::SetDrive(DriveMode::SixStepHall {
            duty: 0.3,
            omega_e: 250.0,
        }));
        round_trip(Message::RunTest {
            kind: test::RL_STEP,
            a: 0.5,
            b: 1.0,
        });
        round_trip(Message::ReadBurst { offset: 4090 });
        round_trip(Message::SetParam {
            id: param::FLUX,
            value: 0.894e-3,
        });
        round_trip(Message::GetParam { id: param::L });
        round_trip(Message::ParamValue {
            id: param::R,
            value: 1.0,
        });
        round_trip(Message::BurstData(
            BurstChunk::new(40, 8192, &[0.25; BURST_CHUNK]).unwrap(),
        ));
        round_trip(Message::BurstData(
            BurstChunk::new(8190, 8192, &[1.0, 2.0]).unwrap(),
        ));
        round_trip(Message::Ack { of: 0x03 });
        round_trip(Message::Nak { of: 0x05, err: 2 });
        round_trip(Message::Telemetry(
            TelemetryFrame::new(
                123_456,
                crate::channel::mask_of(&[crate::channel::I_Q, crate::channel::V_Q]),
                &[1.5, -3.25],
            )
            .unwrap(),
        ));
        round_trip(Message::Telemetry(
            TelemetryFrame::new(u32::MAX, (1 << MAX_CHANNELS) - 1, &[0.5; MAX_CHANNELS]).unwrap(),
        ));
    }

    #[test]
    fn corruption_is_rejected_and_stream_recovers() {
        let msg = Message::Ping { nonce: 42 };
        let mut wire = [0u8; crate::MAX_FRAME];
        let n = encode(&msg, &mut wire).unwrap();

        let mut d = Deframer::new();
        // Corrupt a body byte: CRC must catch it.
        let mut bad = wire;
        bad[2] ^= 0x5A;
        let mut results = std::vec::Vec::new();
        for &b in &bad[..n] {
            if let Some(r) = d.push(b) {
                results.push(r);
            }
        }
        assert!(matches!(
            results.as_slice(),
            [Err(FrameError::Crc)] | [Err(FrameError::Cobs)]
        ));

        // The very next clean frame parses fine.
        let mut ok = None;
        for &b in &wire[..n] {
            if let Some(r) = d.push(b) {
                ok = Some(r.unwrap());
            }
        }
        assert_eq!(ok.unwrap(), msg);
    }

    #[test]
    fn mid_stream_attach_resyncs() {
        // Join half-way through one frame, then receive a full one.
        let msg = Message::SetIqRef { iq: 2.0 };
        let mut wire = [0u8; crate::MAX_FRAME];
        let n = encode(&msg, &mut wire).unwrap();

        let mut d = Deframer::new();
        let mut seen = std::vec::Vec::new();
        for &b in wire[n / 2..n].iter().chain(&wire[..n]) {
            if let Some(r) = d.push(b) {
                seen.push(r);
            }
        }
        // The torn frame errors (or parses as garbage type); the second is clean.
        assert_eq!(*seen.last().unwrap(), Ok(msg));
    }

    #[test]
    fn telemetry_frame_validates_mask() {
        assert!(TelemetryFrame::new(0, 0b11, &[1.0]).is_none());
        assert!(TelemetryFrame::new(0, 1 << 16, &[]).is_none());
    }
}
