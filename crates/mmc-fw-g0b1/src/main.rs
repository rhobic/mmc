//! Protocol bring-up firmware for an STM32G0B1RE dev board (Cortex-M0+).
//!
//! This board is *not* a motor target — it exists to prove the modularity
//! story early: `mmc-proto` + `mmc-core` running on a Cortex-M0+ (thumbv6m,
//! no FPU), streaming telemetry over the debug USB serial port to the exact same
//! host tooling that talks to the simulator over TCP.
//!
//! Since there is no inverter attached, telemetry is a synthetic plant: `i_q`
//! tracks the commanded reference as a first-order lag (τ = 20 ms), and the
//! other channels are derived from it — so a host-commanded current step
//! produces a measurable "step response" end to end through the wire.
//!
//! UART: USART2 on PA2/PA3 (the debug USB serial port), 1 Mbaud — divides both the
//! G0's 16 MHz HSI (BRR = 16) and the probe's UART clock exactly.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};
use panic_halt as _;
use portable_atomic::{AtomicBool, AtomicU32};

use mmc_core::math::{sin_cos, wrap_angle};
use mmc_proto::{channel, encode, Deframer, DeviceInfo, DeviceKind, Message, TelemetryFrame};

bind_interrupts!(struct Irqs {
    USART2_LPUART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2_3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>;
});

/// Commanded q-axis reference [A], as f32 bits.
static IQ_REF: AtomicU32 = AtomicU32::new(0);
static MASK: AtomicU32 = AtomicU32::new(channel::ALL);
/// Telemetry divider in 100 µs control periods.
static DIVIDER: AtomicU32 = AtomicU32::new(10);
static STREAMING: AtomicBool = AtomicBool::new(false);

/// Command responses from the RX task for the TX task to transmit.
static RESPONSES: Channel<CriticalSectionRawMutex, Message, 8> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 64 MHz from HSI16 via PLL (VCO 128 MHz / R 2): frame encode + soft-float
    // math is CPU-bound at the reset default of 16 MHz.
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::{Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL8,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
    }
    let p = embassy_stm32::init(config);

    let mut cfg = usart::Config::default();
    cfg.baudrate = 1_000_000;
    let uart = Uart::new(p.USART2, p.PA3, p.PA2, p.DMA1_CH1, p.DMA1_CH2, Irqs, cfg)
        .expect("USART2 config");
    let (tx, rx) = uart.split();

    spawner.spawn(rx_task(rx).unwrap());
    spawner.spawn(tx_task(tx).unwrap());
}

/// Always listening: frames may arrive while the TX task is mid-transmission,
/// which is exactly why RX gets its own task.
#[embassy_executor::task]
async fn rx_task(mut rx: UartRx<'static, Async>) {
    let mut deframer = Deframer::new();
    let mut buf = [0u8; 128];
    loop {
        let Ok(n) = rx.read_until_idle(&mut buf).await else {
            continue; // noise/overrun: the deframer resyncs on the next 0x00
        };
        for &b in &buf[..n] {
            let Some(Ok(msg)) = deframer.push(b) else {
                continue;
            };
            let reply = match msg {
                Message::Ping { nonce } => Message::Pong { nonce },
                Message::GetInfo => {
                    Message::Info(DeviceInfo::new(DeviceKind::BoardG0b1, 1, "mmc-g0b1"))
                }
                Message::SetTelemetry { divider, mask } => {
                    DIVIDER.store(divider.max(1) as u32, Ordering::Relaxed);
                    MASK.store(mask & channel::ALL, Ordering::Relaxed);
                    Message::Ack {
                        of: msg.wire_type(),
                    }
                }
                Message::Stream { enable } => {
                    STREAMING.store(enable, Ordering::Relaxed);
                    Message::Ack {
                        of: msg.wire_type(),
                    }
                }
                Message::SetIqRef { iq } => {
                    IQ_REF.store(iq.to_bits(), Ordering::Relaxed);
                    Message::Ack {
                        of: msg.wire_type(),
                    }
                }
                other => Message::Nak {
                    of: other.wire_type(),
                    err: 1,
                },
            };
            // Drop replies rather than stall RX if TX is saturated.
            let _ = RESPONSES.try_send(reply);
        }
    }
}

#[embassy_executor::task]
async fn tx_task(mut tx: UartTx<'static, Async>) {
    let mut divider = DIVIDER.load(Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_micros(100 * divider as u64));

    // Synthetic plant state.
    const TAU: f32 = 0.02; // i_q first-order time constant [s]
    const POLE_PAIRS: f32 = 7.0;
    let mut i_q = 0.0f32;
    let mut theta_e = 0.0f32;
    let mut was_streaming = false;

    loop {
        match select(RESPONSES.receive(), ticker.next()).await {
            Either::First(reply) => send(&mut tx, &reply).await,
            Either::Second(()) => {
                let d = DIVIDER.load(Ordering::Relaxed);
                if d != divider {
                    divider = d;
                    ticker = Ticker::every(Duration::from_micros(100 * divider as u64));
                }
                let streaming = STREAMING.load(Ordering::Relaxed);
                if streaming && !was_streaming {
                    // Fresh trace per capture.
                    i_q = 0.0;
                    theta_e = 0.0;
                }
                was_streaming = streaming;
                if !streaming {
                    continue;
                }

                let dt = divider as f32 * 100e-6;
                let iq_ref = f32::from_bits(IQ_REF.load(Ordering::Relaxed));
                i_q += (iq_ref - i_q) * (dt / TAU).min(1.0);
                let omega_m = 40.0 * i_q; // synthetic "speed" [rad/s per A]
                theta_e = wrap_angle(theta_e + omega_m * POLE_PAIRS * dt);

                // One transcendental call; the other phases via sin/cos(θ∓2π/3)
                // angle-addition identities — sinf is expensive on M0+.
                const C: f32 = -0.5; // cos(2π/3)
                const S: f32 = 0.866_025_4; // sin(2π/3)
                let (sin_a, cos_a) = sin_cos(theta_e);
                let sin_b = sin_a * C - cos_a * S;
                let cos_b = cos_a * C + sin_a * S;
                let sin_c = sin_a * C + cos_a * S;

                let mask = MASK.load(Ordering::Relaxed);
                let mut values = [0f32; channel::COUNT];
                let mut n = 0;
                for id in 0..channel::COUNT as u8 {
                    if mask & (1 << id) == 0 {
                        continue;
                    }
                    values[n] = match id {
                        channel::IQ_REF => iq_ref,
                        channel::I_D => 0.02 * sin_a * cos_b,
                        channel::I_Q => i_q,
                        channel::V_D => -0.02 * omega_m * i_q,
                        channel::V_Q => 0.5 * i_q + 0.056 * omega_m,
                        channel::DUTY_A => 0.5 + 0.45 * sin_a,
                        channel::DUTY_B => 0.5 + 0.45 * sin_b,
                        channel::DUTY_C => 0.5 + 0.45 * sin_c,
                        channel::OMEGA_M => omega_m,
                        channel::THETA_E => theta_e,
                        channel::VBUS => 3.3, // bare dev board, no inverter: VDD
                        _ => 0.0,
                    };
                    n += 1;
                }
                let t_us = Instant::now().as_micros() as u32;
                if let Some(frame) = TelemetryFrame::new(t_us, mask, &values[..n]) {
                    send(&mut tx, &Message::Telemetry(frame)).await;
                }
            }
        }
    }
}

async fn send(tx: &mut UartTx<'static, Async>, msg: &Message) {
    let mut buf = [0u8; mmc_proto::MAX_FRAME];
    if let Some(n) = encode(msg, &mut buf) {
        let _ = tx.write(&buf[..n]).await;
    }
}
