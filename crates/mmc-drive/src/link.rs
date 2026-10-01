//! The host link: framed protocol in, replies and telemetry out, over any
//! byte stream implementing `embedded-io-async`. Board crates wrap these in
//! their executor's tasks (tasks cannot be generic) and are done.
//!
//! The receive side must not drop bytes between reads, so give it a
//! ring-buffered reader (e.g. embassy's `RingBufferedUartRx`): a plain DMA
//! read re-armed per call loses whatever arrives while a frame is being
//! handled, which on the F302 bench showed up as a lost ping and a profiler
//! timeout (session 30).

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use embedded_io_async::{Read, Write};
use mmc_proto::{encode, Deframer, Message};

use crate::{ParamStore, Shared};

/// Replies queued by the receive loop for the transmit loop.
pub type Replies = Channel<CriticalSectionRawMutex, Message, 8>;

/// Deframe and handle host commands forever. Owns the parameter store so
/// persistence requests can erase/program it.
pub async fn rx_loop<const N: usize>(
    sh: &Shared<N>,
    replies: &Replies,
    mut rx: impl Read,
    store: &mut impl ParamStore,
) -> ! {
    let mut deframer = Deframer::new();
    let mut buf = [0u8; 64];
    loop {
        // An overrun error resyncs the ring; the deframer drops the partial
        // frame on its own.
        let Ok(n) = rx.read(&mut buf).await else {
            continue;
        };
        sh.host_activity();
        for &b in &buf[..n] {
            if let Some(Ok(msg)) = deframer.push(b) {
                let _ = replies.try_send(sh.handle(&msg, store));
            }
        }
    }
}

/// Send replies as they come, and telemetry snapshots at the configured
/// divider, forever.
pub async fn tx_loop<const N: usize>(sh: &Shared<N>, replies: &Replies, mut tx: impl Write) -> ! {
    let mut period = sh.telemetry_period_us();
    let mut ticker = Ticker::every(Duration::from_micros(period));
    loop {
        match select(replies.receive(), ticker.next()).await {
            Either::First(reply) => send(&mut tx, &reply).await,
            Either::Second(()) => {
                let p = sh.telemetry_period_us();
                if p != period {
                    period = p;
                    ticker = Ticker::every(Duration::from_micros(period));
                }
                if let Some(frame) = sh.telemetry() {
                    send(&mut tx, &frame).await;
                }
            }
        }
    }
}

async fn send(tx: &mut impl Write, msg: &Message) {
    let mut buf = [0u8; mmc_proto::MAX_FRAME];
    if let Some(n) = encode(msg, &mut buf) {
        let _ = tx.write_all(&buf[..n]).await;
    }
}
