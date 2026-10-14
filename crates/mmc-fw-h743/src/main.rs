//! STM32H743ZI (Nucleo-H743ZI: Cortex-M7 at 480 MHz, 2 MB flash, 1 MB RAM)
//! skeleton: the full `mmc-drive` application, every subsystem and the host
//! link on the ST-Link's virtual COM port (USART3, PD8/PD9), on an H7.
//!
//! **Not a motor port yet.** The power stage is a stub: [`H743Board`] reads
//! zero currents and drops its duties, the control tick runs from an
//! executor ticker instead of an ADC interrupt, and parameters live in RAM.
//! It exists to show what the application costs on this part (flash, RAM)
//! and to be the starting point of a real port: wire TIM1 (or HRTIM)
//! centre-aligned PWM with an injected-ADC trigger, as the F302 and G474
//! boards do, and move `Engine::tick` into that interrupt.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::{Duration, Ticker};
use panic_halt as _;

use mmc_drive::{link, nvparam, BurstBuffer, DriveConfig, Engine, ParamStore, Shared};
use mmc_hal::{BoardSpec, MotorBoard, Sample};
use mmc_proto::{param, DeviceKind};

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
    DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
});

const CTRL_FREQ_HZ: u32 = 20_000;

/// Placeholder power-stage spec (an IHM07M1-class shield) until a board is
/// chosen.
const SPEC: BoardSpec = BoardSpec {
    ctrl_hz: CTRL_FREQ_HZ,
    cur_volts_per_amp: 1.53 * 0.33,
    i_trip: 1.5,
    vbus_max: 30.0,
    vbus_min_run: 5.0,
    max_duty: 0.9,
    r_path: 1.0,
    terminal_offset_max: 100.0,
    has_halls: true,
    pwm_latency: 0.5,
    terminal_full_scale: 3.3 * 12.2 / 2.2,
};

/// Firmware defaults: the F302's, which suit an unknown small motor.
const DEFAULTS: [f32; param::COUNT] = {
    let mut d = [0.0f32; param::COUNT];
    d[param::R as usize] = 1.0;
    d[param::L as usize] = 0.5e-3;
    d[param::FLUX as usize] = 5.0e-3;
    d[param::CUR_BW as usize] = 1000.0;
    d[param::SPEED_KP as usize] = 1.0e-4;
    d[param::SPEED_KI as usize] = 1.0e-3;
    d[param::POLE_PAIRS as usize] = 7.0;
    d[param::SL_HANDOFF as usize] = 150.0;
    d[param::OMEGA_ACCEL as usize] = 300.0;
    d[param::IQ_LIMIT as usize] = 0.8;
    d[param::I_THRESH as usize] = 0.05;
    d[param::HALL_DIR as usize] = 1.0;
    d[param::HALL_HYST as usize] = 0.06;
    d[param::SS_CONDUCTION as usize] = 120.0;
    d[param::ID_DITHER_PERIOD as usize] = 0.5;
    d[param::COG_SHIFT as usize] = -1.0;
    d[param::HFI_BW as usize] = 300.0;
    d[param::HFI_XI as usize] = 0.05;
    d[param::HFI_POL_S as usize] = 0.006;
    d[param::HFI_POL_N as usize] = 8.0;
    d[param::HFI_ID as usize] = 0.5;
    let mut k = 0;
    while k < 6 {
        d[(param::HALL_W0 + k) as usize] = core::f32::consts::FRAC_PI_3;
        k += 1;
    }
    d
};

/// The saliency sweep's full schedule fits easily (1 MB of RAM).
const BURST: usize = if cfg!(feature = "burst") {
    mmc_core::probe::SAL_HDR + mmc_core::probe::SAL_TICKS * 2
} else {
    0
};

static BURST_BUF: BurstBuffer<BURST> = BurstBuffer::new();
static SHARED: Shared<BURST> = Shared::new(
    DriveConfig {
        spec: SPEC,
        kind: DeviceKind::Unknown(4),
        fw_version: 2,
        name: "mmc-h743",
        defaults: DEFAULTS,
    },
    &BURST_BUF,
);

/// Power stage stub (see the module docs).
struct H743Board;

impl MotorBoard for H743Board {
    fn sample(&mut self) -> Sample {
        Sample {
            phase_volts: [1.65; 3],
            vbus: 0.0,
        }
    }
    fn terminal_volts(&mut self) -> [f32; 3] {
        [0.0; 3]
    }
    fn set_duties(&mut self, _duties: [f32; 3]) {}
    fn set_phase_enables(&mut self, _mask: u8) {}
    fn driver_fault(&mut self) -> bool {
        false
    }
    fn hall_state(&mut self) -> Option<u8> {
        Some(0b001)
    }
    fn cycles(&self) -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }
}

/// Parameters in RAM until the H7's 128 KB flash sectors get a store.
struct RamStore([u8; nvparam::BYTES]);

impl ParamStore for RamStore {
    fn read(&mut self) -> &[u8] {
        &self.0
    }
    fn write(&mut self, blob: &[u8; nvparam::BYTES]) -> bool {
        self.0 = *blob;
        true
    }
    fn erase(&mut self) -> bool {
        self.0 = [0xFF; nvparam::BYTES];
        true
    }
}

static REPLIES: link::Replies = link::Replies::new();

/// The executor's own loop (embassy's thread executor: poll, then sleep on
/// WFE until an interrupt or a wake), with the sleep booked as idle so the
/// CPU can be accounted (`Shared::meter_idle`, `tools/cpu_profile.sh`).
#[cortex_m_rt::entry]
fn main() -> ! {
    // The thread executor's context marker: its pender answers with SEV.
    let exec = cortex_m::singleton!(: embassy_executor::raw::Executor =
        embassy_executor::raw::Executor::new(usize::MAX as *mut ()))
    .unwrap();
    let spawner = exec.spawner();
    spawner.spawn(init(spawner).unwrap());
    loop {
        // Safety: polled from this one thread only, as embassy's own loop.
        unsafe { exec.poll() };
        SHARED.meter_idle(cortex_m::peripheral::DWT::cycle_count, cortex_m::asm::wfe);
    }
}

#[embassy_executor::task]
async fn init(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        // HSI 64 MHz / 4 x 60 / 2 = 480 MHz on VOS0.
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL60,
            divp: Some(PllDiv::DIV2),
            divq: None,
            divr: None,
            fracn: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV2;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.apb3_pre = APBPrescaler::DIV2;
        config.rcc.apb4_pre = APBPrescaler::DIV2;
        config.rcc.voltage_scale = VoltageScale::Scale0;
    }
    let p = embassy_stm32::init(config);

    let mut store = RamStore([0xFF; nvparam::BYTES]);
    SHARED.restore(&mut store);

    unsafe {
        let mut cp = cortex_m::Peripherals::steal();
        cp.SCB.enable_icache();
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
    }

    let mut cfg = usart::Config::default();
    cfg.baudrate = 1_000_000;
    let uart = Uart::new(p.USART3, p.PD9, p.PD8, p.DMA1_CH0, p.DMA1_CH1, Irqs, cfg)
        .expect("USART3 config");
    let (tx, rx) = uart.split();
    let rx_ring = cortex_m::singleton!(: [u8; 256] = [0; 256]).unwrap();
    let rx = rx.into_ring_buffered(rx_ring);

    spawner.spawn(control_task().unwrap());
    spawner.spawn(rx_task(rx, store).unwrap());
    spawner.spawn(tx_task(tx).unwrap());
}

/// Stand-in for the ADC interrupt a real port drives the tick from.
#[embassy_executor::task]
async fn control_task() {
    let mut engine = Engine::new();
    let mut board = H743Board;
    let mut ticker = Ticker::every(Duration::from_hz(CTRL_FREQ_HZ as u64));
    loop {
        engine.tick(&SHARED, &mut board);
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn rx_task(rx: RingBufferedUartRx<'static>, mut store: RamStore) {
    let link = link::rx_loop(&SHARED, &REPLIES, rx, &mut store);
    link::Metered::new(&SHARED, cortex_m::peripheral::DWT::cycle_count, link).await
}

#[embassy_executor::task]
async fn tx_task(tx: UartTx<'static, Async>) {
    let link = link::tx_loop(
        &SHARED,
        &REPLIES,
        tx,
        cortex_m::peripheral::DWT::cycle_count,
    );
    link::Metered::new(&SHARED, cortex_m::peripheral::DWT::cycle_count, link).await
}
