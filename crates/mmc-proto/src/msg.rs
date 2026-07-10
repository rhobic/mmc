//! Message definitions and frame encode/parse.

use crate::{cobs, crc::crc16};

/// Largest payload: telemetry with a full channel mask
/// (`t_us` + mask + [`MAX_CHANNELS`] × f32).
pub const MAX_PAYLOAD: usize = 8 + MAX_CHANNELS * 4;

/// Cap on simultaneously streamed channels (mask bits 0..16).
pub const MAX_CHANNELS: usize = 16;

mod ty {
    pub const PING: u8 = 0x01;
    pub const GET_INFO: u8 = 0x02;
    pub const SET_TELEMETRY: u8 = 0x03;
    pub const STREAM: u8 = 0x04;
    pub const SET_IQ_REF: u8 = 0x05;
    pub const PONG: u8 = 0x81;
    pub const INFO: u8 = 0x82;
    pub const TELEMETRY: u8 = 0x83;
    pub const ACK: u8 = 0x84;
    pub const NAK: u8 = 0x85;
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DeviceKind {
    Sim,
    NucleoG0b1,
    NucleoG474,
    Unknown(u8),
}

impl DeviceKind {
    fn to_wire(self) -> u8 {
        match self {
            DeviceKind::Sim => 0,
            DeviceKind::NucleoG0b1 => 1,
            DeviceKind::NucleoG474 => 2,
            DeviceKind::Unknown(v) => v,
        }
    }

    fn from_wire(v: u8) -> Self {
        match v {
            0 => DeviceKind::Sim,
            1 => DeviceKind::NucleoG0b1,
            2 => DeviceKind::NucleoG474,
            v => DeviceKind::Unknown(v),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct DeviceInfo {
    pub proto_version: u8,
    pub kind: DeviceKind,
    /// Firmware/build version, device-defined.
    pub fw_version: u16,
    /// Zero-padded ASCII device name.
    pub name: [u8; 12],
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
        }
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
            Message::Info(DeviceInfo {
                proto_version,
                kind,
                fw_version,
                name,
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
            DeviceKind::NucleoG0b1,
            0x0102,
            "mmc-g0b1",
        )));
        round_trip(Message::SetTelemetry {
            divider: 10,
            mask: crate::channel::ALL,
        });
        round_trip(Message::Stream { enable: true });
        round_trip(Message::SetIqRef { iq: -1.25 });
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
