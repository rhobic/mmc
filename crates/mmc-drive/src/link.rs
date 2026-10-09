//! The host link: framed protocol in, replies and telemetry out, over any
//! byte stream implementing `embedded-io-async`. Board crates wrap these in
//! their executor's tasks (tasks cannot be generic) and are done.
//!
//! The receive side must not drop bytes between reads, so give it a
//! ring-buffered reader (e.g. embassy's `RingBufferedUartRx`): a plain DMA
//! read re-armed per call loses whatever arrives while a frame is being
//! handled, which on the F302 bench showed up as a lost ping and a profiler
//! timeout (session 30).

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::Ordering;
use core::task::{Context, Poll};

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use embedded_io_async::{Read, Write};
use mmc_proto::{encode, Deframer, Message};

use crate::{ParamStore, Shared};

/// Replies queued by the receive loop for the transmit loop.
pub type Replies = Channel<CriticalSectionRawMutex, Message, 8>;

/// A link loop whose CPU time is booked to [`Shared::link_cycles`]: each
/// poll is timed with the board's cycle counter, less any control ticks
/// that ran inside it. Wrap the loops in the board's tasks:
/// `Metered::new(&SHARED, cycles, link::rx_loop(..)).await`.
pub struct Metered<'a, F, C, const N: usize> {
    sh: &'a Shared<N>,
    cycles: C,
    inner: F,
}

impl<'a, F: Future, C: Fn() -> u32, const N: usize> Metered<'a, F, C, N> {
    pub fn new(sh: &'a Shared<N>, cycles: C, inner: F) -> Self {
        Self { sh, cycles, inner }
    }
}

impl<F: Future, C: Fn() -> u32, const N: usize> Future for Metered<'_, F, C, N> {
    type Output = F::Output;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<F::Output> {
        // Safety: `inner` is never moved out of the pinned wrapper.
        let this = unsafe { self.get_unchecked_mut() };
        let (t0, i0) = (
            (this.cycles)(),
            this.sh.isr_sum_cycles.load(Ordering::Relaxed),
        );
        let r = unsafe { Pin::new_unchecked(&mut this.inner) }.poll(cx);
        let dt = (this.cycles)().wrapping_sub(t0);
        let di = this
            .sh
            .isr_sum_cycles
            .load(Ordering::Relaxed)
            .wrapping_sub(i0);
        this.sh
            .link_cycles
            .fetch_add(dt.wrapping_sub(di), Ordering::Relaxed);
        r
    }
}

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
pub async fn tx_loop<const N: usize>(
    sh: &Shared<N>,
    replies: &Replies,
    mut tx: impl Write,
    cycles: impl Fn() -> u32,
) -> ! {
    let mut period = sh.telemetry_period_us();
    let mut ticker = Ticker::every(Duration::from_micros(period));
    loop {
        if !sh.streaming() {
            // Not streaming (or `telemetry` not built): replies only, and a
            // look at the stream switch every 100 ms rather than a wake per
            // telemetry period (0.8 % of the F302's CPU doing nothing).
            if let Either::First(reply) =
                select(replies.receive(), Timer::after(Duration::from_millis(100))).await
            {
                send(&mut tx, &reply).await;
            }
            ticker.reset();
            continue;
        }
        match select(replies.receive(), ticker.next()).await {
            Either::First(reply) => send(&mut tx, &reply).await,
            Either::Second(()) => {
                let p = sh.telemetry_period_us();
                if p != period {
                    period = p;
                    ticker = Ticker::every(Duration::from_micros(period));
                }
                let (t0, i0) = (cycles(), sh.isr_sum_cycles.load(Ordering::Relaxed));
                let mut buf = [0u8; mmc_proto::MAX_FRAME];
                let n = sh.telemetry().and_then(|frame| encode(&frame, &mut buf));
                let di = sh.isr_sum_cycles.load(Ordering::Relaxed).wrapping_sub(i0);
                sh.encode_cycles.fetch_add(
                    cycles().wrapping_sub(t0).wrapping_sub(di),
                    Ordering::Relaxed,
                );
                if let Some(n) = n {
                    let _ = tx.write_all(&buf[..n]).await;
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
