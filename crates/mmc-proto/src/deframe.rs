//! Incremental frame extraction from a byte stream.

use crate::msg::{parse, FrameError, Message};

/// Feed bytes in as they arrive (UART ISR, TCP read, …); complete frames come
/// out as parsed [`Message`]s. Corrupt or oversized frames surface as errors
/// and the stream resynchronizes on the next delimiter automatically.
pub struct Deframer {
    buf: [u8; crate::MAX_FRAME],
    len: usize,
    overflow: bool,
}

impl Default for Deframer {
    fn default() -> Self {
        Self::new()
    }
}

impl Deframer {
    pub const fn new() -> Self {
        Self {
            buf: [0; crate::MAX_FRAME],
            len: 0,
            overflow: false,
        }
    }

    /// Push one received byte. Returns a parse result when `byte` completes a
    /// frame, `None` while a frame is still accumulating.
    pub fn push(&mut self, byte: u8) -> Option<Result<Message, FrameError>> {
        if byte != 0 {
            if self.len < self.buf.len() {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }
        // Delimiter: empty frames (idle keep-alive zeros) are skipped silently.
        let len = core::mem::take(&mut self.len);
        let overflow = core::mem::take(&mut self.overflow);
        if overflow {
            return Some(Err(FrameError::TooLong));
        }
        if len == 0 {
            return None;
        }
        let Some(raw_len) = crate::cobs::decode_in_place(&mut self.buf[..len]) else {
            return Some(Err(FrameError::Cobs));
        };
        Some(parse(&self.buf[..raw_len]))
    }
}
