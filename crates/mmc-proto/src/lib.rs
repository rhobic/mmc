//! Telemetry and command wire protocol shared by firmware and host.
//!
//! One frame format on every transport: the simulator serves these bytes over
//! TCP, firmware over UART / USB-CDC, so the same host tooling talks to both.
//!
//! Wire format, outermost first:
//!
//! ```text
//! COBS( [type: u8] [payload …] [crc16: u16 LE] ) 0x00
//! ```
//!
//! - Frames are delimited by `0x00`; COBS guarantees no zero bytes inside, so
//!   a receiver can resynchronize on any byte stream by waiting for the next
//!   delimiter (UART hot-plug, dropped bytes, mid-stream attach).
//! - CRC16/CCITT-FALSE over `type + payload` rejects corruption.
//! - All multi-byte integers and `f32` are little-endian.
//!
//! Everything here is `no_std`, allocation-free, and panic-free by
//! construction: encoding takes caller-provided buffers, decoding is
//! incremental via [`Deframer::push`].

#![no_std]

#[cfg(test)]
extern crate std;

pub mod channel;
pub mod cobs;
pub mod crc;

mod deframe;
mod msg;

pub use deframe::Deframer;
pub use msg::{encode, DeviceInfo, DeviceKind, FrameError, Message, TelemetryFrame};

/// Protocol version reported in [`DeviceInfo`]; bump on breaking wire changes.
pub const PROTO_VERSION: u8 = 1;

/// Maximum `type + payload + crc` size before COBS.
pub const MAX_RAW: usize = 3 + msg::MAX_PAYLOAD;
/// Maximum encoded frame size on the wire, including the `0x00` delimiter.
/// Buffers of this size are always large enough for [`encode`].
pub const MAX_FRAME: usize = MAX_RAW + MAX_RAW / 254 + 2;
