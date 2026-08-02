//! STM32G474RE + X-NUCLEO-IHM16M1 (STSPIN830) motor bring-up firmware.
//!
//! MS5 hardware layer: center-aligned TIM1 PWM, PWM-synchronized
//! injected-ADC shunt current sensing, zero-current calibration, software
//! protection trips, and open-loop / I-f / **closed-loop sensorless** drive
//! modes — all instrumented over `mmc-proto` on the ST-Link VCP so every
//! test is a dashboard capture. Sensorless (Stage F) runs MS4's stack in
//! the control ISR: I-f startup, blend handoff to the flux observer, then
//! the speed loop commands i_q on the estimated angle.
//!
//! ## Pin map (from hw/x-nucleo-ihm16m1_schematic.pdf + mb1367 Nucleo)
//!
//! | Function            | Pin  | Notes                                     |
//! |---------------------|------|-------------------------------------------|
//! | VCP UART            | PA2/PA3 | LPUART1 (SB17/SB23), 1 Mbaud           |
//! | PWM U/V/W (STSPIN IN)| PA8/PA9/PA10 | TIM1 CH1/2/3, AF6, 20 kHz center |
//! | Phase enables (EN)  | PB13/PB14/PB15 | GPIO; low = phase Hi-Z          |
//! | STSPIN830 STBY      | PB5  | high = run                                |
//! | EN_FAULT (in)       | PA11 + PB12 | open-drain, low = fault. The shield routes it to PB12 (R37) by default and to PA11 (R35) on F302/F303-style builds — ST's example .ioc uses PA11. Both are read with internal pull-ups, so whichever is unconnected floats high and stays silent. (TIM1_BKIN2 hardware break on PA11 is a follow-up.) |
//! | Current ref (VREF)  | PB4  | GPIO high → VREF ≈ 0.50 V (max via 22k/3.9k divider). This is the *weakest* hardware current limit (≈1.5 A on 0.33 Ω); floating PB4 would pull VREF toward 0 V and trip continuously (STSPIN830: VSNS > VREF disables outputs for tOFF) |
//! | i_U / i_V / i_W     | PA1/PB1/PB0 | ADC1 IN2/IN12/IN15, TSV994 ×2 amp |
//! | VBUS                | PA0  | ADC1 IN1, 180k/12k divider (×16)          |
//!
//! ## Current-sense scaling (sheet 3)
//!
//! 0.33 Ω shunt → 680R/2.2k bias to 3.3 V → TSV994 non-inverting ×2:
//! `v_adc = 1.558 V − 1.528·0.33·i_phase` (positive current into the motor
//! discharges the node). Offsets are measured at boot with the stage disabled;
//! the slope is 0.5042 V/A.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::mode::Async;
use embassy_stm32::pac::{self, ADC1, ADC2, GPIOA, GPIOB, GPIOC, RCC, TIM1};
use embassy_stm32::usart::{self, Uart, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use panic_halt as _;

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc};
use mmc_core::math::{sin_cos, wrap_angle};
use mmc_core::observer::{FluxObserver, FluxObserverCfg};
use mmc_core::pi::PiGains;
use mmc_core::probe;
use mmc_core::sensorless::{Phase, Sequencer, SequencerCfg, SpeedLoop};
use mmc_core::svpwm::svpwm;
use mmc_core::transforms::{clarke, inverse_park, park, Abc, AlphaBeta, Dq};
use mmc_core::tuning::current_pi_gains;
use mmc_proto::{
    channel, encode, param, test, BurstChunk, Deframer, DeviceInfo, DeviceKind, DriveMode, Message,
};

bind_interrupts!(struct Irqs {
    LPUART1 => usart::InterruptHandler<peripherals::LPUART1>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>;
});

// ---------------------------------------------------------------- constants

const PWM_ARR: u16 = 4250; // 170 MHz / (2·4250) = 20 kHz center-aligned
const CTRL_FREQ: f32 = 20_000.0;
const CTRL_DT: f32 = 1.0 / CTRL_FREQ;

const ADC_VOLTS_PER_LSB: f32 = 3.3 / 4096.0;
const CUR_VOLTS_PER_AMP: f32 = 1.528 * 0.33; // amp gain × shunt
const VBUS_GAIN: f32 = 16.0; // 180k/12k divider
/// BEMF divider: OUTx → 10K → 2.2K → IO_BEMF (PC9 low), ratio 2.2/12.2.
const BEMF_GAIN: f32 = 12.2 / 2.2;

const I_TRIP_A: f32 = 1.5; // software overcurrent, 2 consecutive samples
const VBUS_MAX: f32 = 30.0;
const VBUS_MIN_RUN: f32 = 5.0;
const MAX_DUTY: f32 = 0.85; // keeps the low-side sampling window open
const V_AMP_MAX: f32 = 3.0;
const OMEGA_E_MAX: f32 = 2000.0; // rad/s electrical
const OMEGA_SLEW: f32 = 500.0; // rad/s²
const V_SLEW: f32 = 5.0; // V/s
const I_SLEW: f32 = 2.0; // A/s
/// Assumed pole pairs for the mechanical-speed telemetry channel only.
const POLE_PAIRS: f32 = 7.0;
/// Drive shuts off if the host goes silent this long (capture keep-alive pings).
const DEADMAN_TICKS: u32 = 2 * 20_000;
const CAL_TICKS: u32 = 8192;

// Motor parameters measured on this bench (Stage F0 rotating I-f sweep,
// `tools/fit_params.py`): flux 0.894 ± 0.04 mWb, apparent R 0.97 Ω
// (locked-rotor R = 1.0), friction ≈ 0.8 mN·m, J ≈ 0.31 µN·m·s² (rough).
// L was ill-conditioned in that test (0.05 ± 0.10 mH; hang angle near π/2
// hides it) — 0.1 mH is a robust design center, and it only sets the
// current-PI zero and a small observer flux correction.
const CUR_BANDWIDTH: f32 = 1000.0;
const MOTOR_RS: f32 = 1.0;
const MOTOR_LS: f32 = 0.1e-3;
const MOTOR_FLUX: f32 = 0.894e-3;
// Speed loop at ~40 rad/s (electrical) from the fitted J and kt; J is the
// least-trusted number, so these are deliberately conservative.
const SPEED_KP: f32 = 2.0e-4;
const SPEED_KI: f32 = 2.0e-3;
/// Speed-loop i_q authority [A].
const SL_IQ_LIMIT: f32 = 0.8;
/// I-f speed at which sensorless startup hands off to the observer
/// [rad/s electrical] — comfortably above the observer's ~4·leak floor.
const SL_OMEGA_HANDOFF: f32 = 150.0;

// Locked-rotor R/L probe (RunTest RL_STEP): rotor-align time at the first
// voltage level, then square-wave excitation between the two levels — τ=L/R
// sits near the 50 µs sample period, so a single edge has ~2 usable samples;
// folding ~60 edges recovers it. Half-period 32 ticks = 1.6 ms ≫ τ, so the
// plateaus settle and R comes out differentially (dead-time cancels).
const PROBE_ALIGN_TICKS: u32 = 6000; // 300 ms
const PROBE_HALF_TICKS: u32 = 32;
const PROBE_PRE_PAIRS: usize = 256;
const BURST_PAIRS: usize = 4096;

// drive state values (channel::STATE)
const ST_OFF: u8 = 0;
const ST_RUN: u8 = 1;
const ST_FAULT_OC: u8 = 2;
const ST_FAULT_DRV: u8 = 3;
const ST_FAULT_VBUS: u8 = 4;
const ST_CAL: u8 = 5;
// Sensorless startup phases on the telemetry STATE channel (same codes the
// host sim scenario writes, so dashboards read identically): Closed = ST_RUN.
const ST_SL_RAMP: u8 = 6;
const ST_SL_BLEND: u8 = 7;
/// Stall fault: the observer's flux magnitude collapsed to the L·i artifact
/// while nominally closed-loop — the rotor is not actually following.
/// ≥ ST_FAULT_OC, so the fault gating and Off-to-re-arm flow apply.
const ST_STALL: u8 = 8;
/// Consecutive low-flux ticks before the stall fault trips (100 ms).
const STALL_TICKS: u32 = 2000;

// ------------------------------------------------------------ shared state

// Host command → control ISR.
static CMD_MODE: AtomicU8 = AtomicU8::new(0); // 0 off, 1 volt, 2 i-f, 3 sensorless, 4 rl-probe
static CMD_AMP: AtomicU32 = AtomicU32::new(0);
static CMD_OMEGA: AtomicU32 = AtomicU32::new(0);
static CMD_EPOCH: AtomicU32 = AtomicU32::new(0);
/// CONTROL_TICKS value at the last host message (deadman).
static LAST_RX_TICK: AtomicU32 = AtomicU32::new(0);

// Control ISR → tasks.
static CONTROL_TICKS: AtomicU32 = AtomicU32::new(0);
/// Max ISR duration in cycles since boot — not on the wire, but readable live
/// via the debug probe (`probe-rs read b32 <addr> 1`); this is how the libm
/// f64-soft-float stall was found.
static ISR_MAX_CYCLES: AtomicU32 = AtomicU32::new(0);
static STATE: AtomicU8 = AtomicU8::new(ST_CAL);
static TELEM_SEQ: AtomicU32 = AtomicU32::new(0);
static TELEM: [AtomicU32; channel::COUNT] = [const { AtomicU32::new(0) }; channel::COUNT];

// Telemetry config (rx task → tx task).
static MASK: AtomicU32 = AtomicU32::new(channel::ALL);
static DIVIDER: AtomicU32 = AtomicU32::new(20); // ×50 µs = 1 kHz default
static STREAMING: AtomicBool = AtomicBool::new(false);

static RESPONSES: Channel<CriticalSectionRawMutex, Message, 8> = Channel::new();

// Runtime parameters (mmc_proto::param ids), profiler-writable over the
// protocol. Defaults are the Stage F0 bench fits; a new value applies at the
// next clean drive start. RAM only — flash persistence is future work.
const fn p32(v: f32) -> AtomicU32 {
    AtomicU32::new(v.to_bits())
}
static PARAMS: [AtomicU32; param::COUNT] = [
    p32(MOTOR_RS),
    p32(MOTOR_LS),
    p32(MOTOR_FLUX),
    p32(CUR_BANDWIDTH),
    p32(SPEED_KP),
    p32(SPEED_KI),
    p32(POLE_PAIRS),
    p32(SL_OMEGA_HANDOFF),
    p32(OMEGA_SLEW),
    p32(SL_IQ_LIMIT),
];

fn param_get(id: u8) -> f32 {
    f32::from_bits(PARAMS[id as usize].load(Ordering::Relaxed))
}

/// Sanity window per parameter — a NAK beats bricking the control loop.
fn param_range(id: u8) -> Option<(f32, f32)> {
    Some(match id {
        param::R => (0.05, 20.0),
        param::L => (5e-6, 0.05),
        param::FLUX => (1e-5, 0.5),
        param::CUR_BW => (100.0, 4000.0),
        param::SPEED_KP => (0.0, 0.1),
        param::SPEED_KI => (0.0, 10.0),
        param::POLE_PAIRS => (1.0, 50.0),
        // Handoff floor sits above the observer's ~4·leak trust floor at the
        // low end only nominally — going below ~80 is an experiment, allowed
        // but on the operator's head.
        param::SL_HANDOFF => (30.0, 1000.0),
        param::OMEGA_ACCEL => (20.0, 5000.0),
        // Current ceiling: 1.2 A leaves 20% margin to the 1.5 A trips.
        param::IQ_LIMIT => (0.05, 1.2),
        _ => return None,
    })
}

/// Flash-persisted parameter table. Stored in the last 2 KB page of bank 2
/// (0x0807_F800). Code executes from bank 1, so erasing/programming this page
/// is read-while-write — the control ISR keeps running from bank 1 untouched.
/// Layout (little-endian u32 words): magic, version, [f32; param::COUNT],
/// crc32(preceding), padded to a write-granularity multiple.
mod nvparam {
    use super::{param, Flash};
    use embassy_stm32::flash::Blocking;

    const NV_OFFSET: u32 = 0x7_F800; // last bank-2 page, from FLASH_BASE
    const NV_ADDR: usize = 0x0807_F800;
    const MAGIC: u32 = 0x4D4D_4350; // "MMCP"
    const VERSION: u32 = 1;
    const HDR: usize = 2; // magic + version
    const CRC_IDX: usize = HDR + param::COUNT;
    const WORDS: usize = (CRC_IDX + 1 + 1) & !1; // +crc, then round up to even (8-byte write)

    fn crc32(words: &[u32]) -> u32 {
        let mut crc = 0xFFFF_FFFFu32;
        for &word in words {
            crc ^= word;
            for _ in 0..32 {
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ 0xEDB8_8320
                } else {
                    crc >> 1
                };
            }
        }
        !crc
    }

    /// Read + validate the stored table (a plain memory read — flash is
    /// mapped). `None` when blank, wrong version, or CRC-mismatched.
    pub fn load() -> Option<[f32; param::COUNT]> {
        let base = NV_ADDR as *const u32;
        let read = |i: usize| unsafe { core::ptr::read_volatile(base.add(i)) };
        if read(0) != MAGIC || read(1) != VERSION {
            return None;
        }
        let mut words = [0u32; CRC_IDX];
        for (i, w) in words.iter_mut().enumerate() {
            *w = read(i);
        }
        if crc32(&words) != read(CRC_IDX) {
            return None;
        }
        let mut params = [0f32; param::COUNT];
        for (i, p) in params.iter_mut().enumerate() {
            *p = f32::from_bits(words[HDR + i]);
        }
        Some(params)
    }

    /// Erase the page, then program the current parameter table. ~22 ms
    /// (erase-dominated); caller gates on a quiet stage.
    pub fn save(flash: &mut Flash<'static, Blocking>, params: &[f32; param::COUNT]) -> bool {
        let mut words = [0u32; WORDS];
        words[0] = MAGIC;
        words[1] = VERSION;
        for (i, v) in params.iter().enumerate() {
            words[HDR + i] = v.to_bits();
        }
        words[CRC_IDX] = crc32(&words[..CRC_IDX]);
        let mut bytes = [0u8; WORDS * 4];
        for (i, w) in words.iter().enumerate() {
            bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        flash.blocking_erase(NV_OFFSET, NV_OFFSET + 2048).is_ok()
            && flash.blocking_write(NV_OFFSET, &bytes).is_ok()
    }

    /// Erase the page so the device boots on firmware defaults.
    pub fn erase(flash: &mut Flash<'static, Blocking>) -> bool {
        flash.blocking_erase(NV_OFFSET, NV_OFFSET + 2048).is_ok()
    }
}

// Burst buffer: per-tick sample pairs recorded by the probes at the full
// 20 kHz — resolution the telemetry stream can't deliver at 1 Mbaud. Layout
// is kind-keyed: RL_STEP records legacy (i_d, v_d) pairs from index 0;
// L_THETA writes a `probe::SAL_HDR` self-describing header first, then
// (i_d, i_q) pairs in the excitation frame (hence the +SAL_HDR size).
const BURST_F32S: usize = BURST_PAIRS * 2 + probe::SAL_HDR;
struct BurstCell(UnsafeCell<[f32; BURST_F32S]>);
// Safety: written only by the ISR while BURST_STATE == 1, read by the rx
// task only while BURST_STATE == 2.
unsafe impl Sync for BurstCell {}
static BURST: BurstCell = BurstCell(UnsafeCell::new([0.0; BURST_F32S]));
/// 0 idle, 1 recording (ISR owns), 2 done (host may read).
static BURST_STATE: AtomicU8 = AtomicU8::new(0);
/// f32s recorded so far.
static BURST_LEN: AtomicU32 = AtomicU32::new(0);
static PROBE_V_ALIGN: AtomicU32 = AtomicU32::new(0);
static PROBE_V_STEP: AtomicU32 = AtomicU32::new(0);
/// Which test sequence mode 4 is running (`mmc_proto::test` kind).
static PROBE_KIND: AtomicU8 = AtomicU8::new(0);
/// Saliency-sweep half-period [ticks], picked from the live R/L params at
/// probe start (τ-adaptive; see `mmc_core::probe::sal_half_ticks`).
static PROBE_HALF: AtomicU32 = AtomicU32::new(8);

/// A drive abort with a probe in flight still hands the (partial) buffer to
/// the host — a short read beats a hung poll loop.
fn burst_abort() {
    if BURST_STATE.load(Ordering::Relaxed) == 1 {
        BURST_STATE.store(2, Ordering::Release);
    }
}

// ---------------------------------------------------------------- ISR state

struct IsrState {
    // calibration
    cal_count: u32,
    cal_sum: [u32; 3],
    offset_v: [f32; 3], // amp output at zero current
    // drive
    epoch_seen: u32,
    mode: u8,
    theta: f32,
    omega: f32,
    amp: f32,
    foc: Option<Foc>,
    /// Flux observer: shadow-instrumented during the forced-angle drives,
    /// the angle source in sensorless mode.
    obs: Option<FluxObserver>,
    // Sensorless mode (MS4 stack): startup sequencer + speed loop.
    seq: Option<Sequencer>,
    speed: Option<SpeedLoop>,
    /// Slewed speed-loop reference [rad/s electrical].
    omega_ref_cur: f32,
    /// Signed startup current, consumed as the speed-PI preload on the
    /// first closed-loop tick (0.0 = already consumed).
    sl_preload: f32,
    /// R/L probe tick counter (mode 4).
    probe_ticks: u32,
    oc_strikes: u8,
    /// Consecutive low-observer-flux ticks in closed-loop sensorless.
    stall_strikes: u32,
    vbus_filt: f32,
}

struct IsrCell(UnsafeCell<IsrState>);
// Safety: written only from the ADC1_2 ISR after init.
unsafe impl Sync for IsrCell {}

static ISR_STATE: IsrCell = IsrCell(UnsafeCell::new(IsrState {
    cal_count: 0,
    cal_sum: [0; 3],
    offset_v: [1.558; 3],
    epoch_seen: 0,
    mode: 0,
    theta: 0.0,
    omega: 0.0,
    amp: 0.0,
    foc: None,
    obs: None,
    seq: None,
    speed: None,
    omega_ref_cur: 0.0,
    sl_preload: 0.0,
    probe_ticks: 0,
    oc_strikes: 0,
    stall_strikes: 0,
    vbus_filt: 0.0,
}));

// ------------------------------------------------------------------- tasks

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::{Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};
        // HSI16 / 4 × 85 / 2 = 170 MHz.
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.boost = true;
    }
    let p = embassy_stm32::init(config);

    // Restore persisted parameters before anything reads them (a plain flash
    // read; no peripheral needed). Range-check each so a stale blob from an
    // older firmware can't push a value the control loop would choke on.
    if let Some(params) = nvparam::load() {
        for (id, &v) in params.iter().enumerate() {
            if let Some((lo, hi)) = param_range(id as u8) {
                if (lo..=hi).contains(&v) {
                    PARAMS[id].store(v.to_bits(), Ordering::Relaxed);
                }
            }
        }
    }
    let flash = Flash::new_blocking(p.FLASH);

    // Cycle counter feeds the ISR_MAX_CYCLES diagnostic.
    unsafe {
        let mut cp = cortex_m::Peripherals::steal();
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
    }

    let mut cfg = usart::Config::default();
    cfg.baudrate = 1_000_000;
    let uart = Uart::new(p.LPUART1, p.PA3, p.PA2, p.DMA1_CH1, p.DMA1_CH2, Irqs, cfg)
        .expect("LPUART1 config");
    let (tx, rx) = uart.split();

    init_motor_peripherals();

    spawner.spawn(rx_task(rx, flash).unwrap());
    spawner.spawn(tx_task(tx).unwrap());
}

/// GPIO + TIM1 + ADC1 register-level setup, then the control interrupt.
fn init_motor_peripherals() {
    use pac::gpio::vals::Moder;

    RCC.ahb2enr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
        w.set_adc12en(true);
    });
    RCC.apb2enr().modify(|w| w.set_tim1en(true));

    // Analog inputs: PA0 (VBUS), PA1 (iU), PB0 (iW), PB1 (iV).
    GPIOA.moder().modify(|w| {
        w.set_moder(0, Moder::ANALOG);
        w.set_moder(1, Moder::ANALOG);
    });
    GPIOB.moder().modify(|w| {
        w.set_moder(0, Moder::ANALOG);
        w.set_moder(1, Moder::ANALOG);
    });
    // BEMF divider network (shield sheet 4): OUTx → 10K → 2.2K → IO_BEMF,
    // sensed on PC0..PC3 (BEMF2's exact pin is resolved empirically — the
    // shield routes it to PC2 or PC3 depending on build), returned through
    // PC9: drive it LOW to enable the dividers. PC0..3 are analog at reset.
    GPIOC.bsrr().write(|w| w.set_br(9, true));
    GPIOC.moder().modify(|w| w.set_moder(9, Moder::OUTPUT));

    // TIM1 CH1/2/3 on PA8/PA9/PA10, AF6.
    GPIOA.moder().modify(|w| {
        w.set_moder(8, Moder::ALTERNATE);
        w.set_moder(9, Moder::ALTERNATE);
        w.set_moder(10, Moder::ALTERNATE);
    });
    GPIOA.afr(1).modify(|w| {
        w.set_afr(0, 6);
        w.set_afr(1, 6);
        w.set_afr(2, 6);
    });

    // EN pins low (phases Hi-Z), STBY high (run), VREF (PB4) high ≈ 0.5 V.
    GPIOB.bsrr().write(|w| {
        w.set_br(13, true);
        w.set_br(14, true);
        w.set_br(15, true);
        w.set_bs(5, true);
        w.set_bs(4, true);
    });
    GPIOB.moder().modify(|w| {
        w.set_moder(13, Moder::OUTPUT);
        w.set_moder(14, Moder::OUTPUT);
        w.set_moder(15, Moder::OUTPUT);
        w.set_moder(5, Moder::OUTPUT);
        w.set_moder(4, Moder::OUTPUT);
    });
    // EN_FAULT candidates (see pin-map note): inputs with internal pull-ups
    // so the unpopulated route reads high instead of floating.
    {
        use pac::gpio::vals::Pupdr;
        GPIOA.moder().modify(|w| w.set_moder(11, Moder::INPUT));
        GPIOA.pupdr().modify(|w| w.set_pupdr(11, Pupdr::PULL_UP));
        GPIOB.moder().modify(|w| w.set_moder(12, Moder::INPUT));
        GPIOB.pupdr().modify(|w| w.set_pupdr(12, Pupdr::PULL_UP));
    }

    // --- TIM1: 20 kHz center-aligned PWM, CH4 as the ADC trigger point.
    {
        use pac::timer::vals;
        TIM1.arr().write(|w| w.set_arr(PWM_ARR));
        TIM1.psc().write_value(0);
        TIM1.rcr().write(|w| w.set_rep(0));
        // CH1..3 PWM mode 1 with preload; CH4 the same, compared near the
        // counter peak — that's where all low sides conduct and the shunts
        // carry the phase currents.
        TIM1.ccmr_output(0).modify(|w| {
            w.set_ocm(0, vals::Ocm::PWM_MODE1);
            w.set_ocpe(0, true);
            w.set_ocm(1, vals::Ocm::PWM_MODE1);
            w.set_ocpe(1, true);
        });
        TIM1.ccmr_output(1).modify(|w| {
            w.set_ocm(0, vals::Ocm::PWM_MODE1);
            w.set_ocpe(0, true);
            w.set_ocm(1, vals::Ocm::PWM_MODE1);
            w.set_ocpe(1, true);
        });
        for ch in 0..3 {
            TIM1.ccr(ch).write(|w| w.set_ccr(0));
        }
        TIM1.ccr(3).write(|w| w.set_ccr(PWM_ARR - 20));
        TIM1.ccer().modify(|w| {
            for ch in 0..4 {
                w.set_cce(ch, true);
            }
        });
        TIM1.cr1().modify(|w| {
            w.set_cms(vals::Cms::CENTER_ALIGNED1);
            w.set_arpe(true);
        });
        TIM1.bdtr().modify(|w| w.set_moe(true));
        TIM1.egr().write(|w| w.set_ug(true));
        TIM1.cr1().modify(|w| w.set_cen(true));
    }

    // --- ADC1: injected sequence iU, iV, iW, VBUS triggered by TIM1 CC4.
    {
        use pac::adccommon::vals::Ckmode;
        pac::ADC12_COMMON
            .ccr()
            .modify(|w| w.set_ckmode(Ckmode::SYNC_DIV4)); // 42.5 MHz

        ADC1.cr().modify(|w| w.set_deeppwd(false));
        ADC1.cr().modify(|w| w.set_advregen(true));
        cortex_m::asm::delay(170 * 25); // t_ADCVREG_STUP ≥ 20 µs

        ADC1.cr().modify(|w| w.set_adcal(true));
        while ADC1.cr().read().adcal() {}
        cortex_m::asm::delay(170);

        ADC1.isr().write(|w| w.set_adrdy(true));
        ADC1.cr().modify(|w| w.set_aden(true));
        while !ADC1.isr().read().adrdy() {}

        use pac::adc::vals::SampleTime;
        ADC1.smpr().modify(|w| {
            w.set_smp(1, SampleTime::CYCLES47_5); // IN1 VBUS (divider + 220 nF)
            w.set_smp(2, SampleTime::CYCLES12_5); // IN2 iU (op-amp driven)
        });
        ADC1.smpr2().modify(|w| {
            w.set_smp(12 - 10, SampleTime::CYCLES12_5); // IN12 iV
            w.set_smp(15 - 10, SampleTime::CYCLES12_5); // IN15 iW
        });

        // Disable the injected queue (G4 default is enabled): with the queue
        // on, JSQR is consumed per sequence and triggering silently stops.
        ADC1.cfgr().modify(|w| w.set_jqdis(true));

        use pac::adc::vals::Exten;
        ADC1.jsqr().write(|w| {
            w.set_jl(3); // 4 conversions
            w.set_jextsel(1); // tim1_cc4
            w.set_jexten(Exten::RISING_EDGE);
            w.set_jsq(0, 2); // iU  PA1
            w.set_jsq(1, 12); // iV  PB1
            w.set_jsq(2, 15); // iW  PB0
            w.set_jsq(3, 1); // VBUS PA0
        });

        ADC1.ier().modify(|w| w.set_jeosie(true));
        ADC1.cr().modify(|w| w.set_jadstart(true)); // arm the hardware trigger

        // ADC2: BEMF terminal voltages on PC0..PC3 (IN6..IN9), same TIM1_CC4
        // trigger, running in parallel with ADC1 — zero cost to the current-
        // sense timing budget. No interrupt: the ADC1-paced ISR reads the
        // latest JDRs (late channels may be one 50 µs period stale, harmless
        // for <100 Hz BEMF). 47.5-cycle sampling for the 10K divider source.
        ADC2.cr().modify(|w| w.set_deeppwd(false));
        ADC2.cr().modify(|w| w.set_advregen(true));
        cortex_m::asm::delay(170 * 25);

        ADC2.cr().modify(|w| w.set_adcal(true));
        while ADC2.cr().read().adcal() {}
        cortex_m::asm::delay(170);

        ADC2.isr().write(|w| w.set_adrdy(true));
        ADC2.cr().modify(|w| w.set_aden(true));
        while !ADC2.isr().read().adrdy() {}

        ADC2.smpr().modify(|w| {
            w.set_smp(6, SampleTime::CYCLES47_5);
            w.set_smp(7, SampleTime::CYCLES47_5);
            w.set_smp(8, SampleTime::CYCLES47_5);
            w.set_smp(9, SampleTime::CYCLES47_5);
        });
        ADC2.cfgr().modify(|w| w.set_jqdis(true));
        ADC2.jsqr().write(|w| {
            w.set_jl(3); // 4 conversions
            w.set_jextsel(1); // tim1_cc4
            w.set_jexten(Exten::RISING_EDGE);
            w.set_jsq(0, 6); // BEMF1 PC0
            w.set_jsq(1, 7); // BEMF3 PC1
            w.set_jsq(2, 8); // BEMF2 candidate PC2
            w.set_jsq(3, 9); // BEMF2 candidate PC3
        });
        ADC2.cr().modify(|w| w.set_jadstart(true));
    }

    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ADC1_2);
    }
}

/// Deframe + handle host commands; replies go through the TX task. Owns the
/// flash so the persist commands can erase/program without a global.
#[embassy_executor::task]
async fn rx_task(mut rx: UartRx<'static, Async>, mut flash: Flash<'static, Blocking>) {
    let mut deframer = Deframer::new();
    let mut buf = [0u8; 128];
    loop {
        let Ok(n) = rx.read_until_idle(&mut buf).await else {
            continue;
        };
        LAST_RX_TICK.store(CONTROL_TICKS.load(Ordering::Relaxed), Ordering::Relaxed);
        for &b in &buf[..n] {
            let Some(Ok(msg)) = deframer.push(b) else {
                continue;
            };
            // Flash writes stall ~22 ms and need the drive quiet; keep them in
            // the rx task (which owns the flash) rather than the pure `handle`.
            let reply = match msg {
                Message::SaveParams | Message::EraseParams => persist(&mut flash, &msg),
                other => handle(&other),
            };
            let _ = RESPONSES.try_send(reply);
        }
    }
}

/// Persist / erase the parameter table, gated on a quiet stage (drive off, no
/// probe recording) so the ~22 ms flash stall never hits a live control loop.
fn persist(flash: &mut Flash<'static, Blocking>, msg: &Message) -> Message {
    let of = msg.wire_type();
    let state = STATE.load(Ordering::Relaxed);
    if state == ST_CAL {
        return Message::Nak { of, err: 3 };
    }
    if state == ST_RUN
        || CMD_MODE.load(Ordering::Relaxed) != 0
        || BURST_STATE.load(Ordering::Relaxed) == 1
    {
        return Message::Nak { of, err: 2 };
    }
    let ok = if matches!(msg, Message::EraseParams) {
        nvparam::erase(flash)
    } else {
        let mut params = [0.0f32; param::COUNT];
        for (id, p) in params.iter_mut().enumerate() {
            *p = param_get(id as u8);
        }
        nvparam::save(flash, &params)
    };
    if ok {
        Message::Ack { of }
    } else {
        Message::Nak { of, err: 5 } // flash program error
    }
}

fn handle(msg: &Message) -> Message {
    let ack = Message::Ack {
        of: msg.wire_type(),
    };
    let nak = |err| Message::Nak {
        of: msg.wire_type(),
        err,
    };
    match *msg {
        Message::Ping { nonce } => Message::Pong { nonce },
        Message::GetInfo => Message::Info(DeviceInfo::new(DeviceKind::NucleoG474, 7, "mmc-g474")),
        Message::SetTelemetry { divider, mask } => {
            DIVIDER.store(divider.max(1) as u32, Ordering::Relaxed);
            MASK.store(mask & channel::ALL, Ordering::Relaxed);
            ack
        }
        Message::Stream { enable } => {
            STREAMING.store(enable, Ordering::Relaxed);
            ack
        }
        Message::SetDrive(mode) => {
            let state = STATE.load(Ordering::Relaxed);
            let (m, amp, omega) = match mode {
                DriveMode::Off => (0u8, 0.0f32, 0.0f32),
                DriveMode::OpenLoopVoltage { volts, omega_e } => (1, volts, omega_e),
                DriveMode::IfCurrent { amps, omega_e } => (2, amps, omega_e),
                DriveMode::Sensorless { amps, omega_e } => (3, amps, omega_e),
            };
            if m != 0 {
                if state == ST_CAL {
                    return nak(3); // still calibrating
                }
                if state >= ST_FAULT_OC {
                    return nak(2); // faulted: requires Off first
                }
            }
            CMD_AMP.store(
                amp.clamp(-amp_limit(m), amp_limit(m)).to_bits(),
                Ordering::Relaxed,
            );
            CMD_OMEGA.store(
                omega.clamp(-OMEGA_E_MAX, OMEGA_E_MAX).to_bits(),
                Ordering::Relaxed,
            );
            CMD_MODE.store(m, Ordering::Relaxed);
            CMD_EPOCH.fetch_add(1, Ordering::Release);
            ack
        }
        Message::RunTest { kind, a, b } => {
            if kind != test::RL_STEP && kind != test::L_THETA {
                return nak(1);
            }
            let state = STATE.load(Ordering::Relaxed);
            if state == ST_CAL {
                return nak(3);
            }
            if state >= ST_FAULT_OC {
                return nak(2);
            }
            // Only from a quiet stage, and not while a probe is recording.
            if CMD_MODE.load(Ordering::Relaxed) != 0
                || state == ST_RUN
                || BURST_STATE.load(Ordering::Relaxed) == 1
            {
                return nak(2);
            }
            // The saliency sweep bounds its steady-state plateau below the
            // software trip using the live R estimate, and picks its square-
            // wave half-period from the live τ = L/R so the plateaus settle
            // (profile + apply R/L before running it). RL_STEP keeps its
            // hardware-validated V_AMP_MAX clamp unchanged.
            let v_max = if kind == test::L_THETA {
                let tau_ticks = param_get(param::L) / param_get(param::R) * CTRL_FREQ;
                PROBE_HALF.store(probe::sal_half_ticks(tau_ticks) as u32, Ordering::Relaxed);
                (0.75 * I_TRIP_A * param_get(param::R)).min(V_AMP_MAX)
            } else {
                V_AMP_MAX
            };
            PROBE_V_ALIGN.store(a.clamp(0.05, v_max).to_bits(), Ordering::Relaxed);
            PROBE_V_STEP.store(b.clamp(0.05, v_max).to_bits(), Ordering::Relaxed);
            PROBE_KIND.store(kind, Ordering::Relaxed);
            BURST_LEN.store(0, Ordering::Relaxed);
            BURST_STATE.store(1, Ordering::Relaxed);
            CMD_MODE.store(4, Ordering::Relaxed);
            CMD_EPOCH.fetch_add(1, Ordering::Release);
            ack
        }
        Message::ReadBurst { offset } => {
            if BURST_STATE.load(Ordering::Acquire) != 2 {
                return nak(4); // no finished recording to read
            }
            let len = BURST_LEN.load(Ordering::Relaxed) as usize;
            let off = (offset as usize).min(len);
            // Safety: ISR only writes while BURST_STATE == 1.
            let buf = unsafe { &*BURST.0.get() };
            let n = (len - off).min(mmc_proto::BURST_CHUNK);
            match BurstChunk::new(off as u16, len as u16, &buf[off..off + n]) {
                Some(chunk) => Message::BurstData(chunk),
                None => nak(1),
            }
        }
        Message::SetParam { id, value } => match param_range(id) {
            Some((lo, hi)) if (lo..=hi).contains(&value) => {
                PARAMS[id as usize].store(value.to_bits(), Ordering::Relaxed);
                ack
            }
            _ => nak(1),
        },
        Message::GetParam { id } => {
            if (id as usize) < param::COUNT {
                Message::ParamValue {
                    id,
                    value: param_get(id),
                }
            } else {
                nak(1)
            }
        }
        // Adjust the I-f current target on the fly; otherwise ignored.
        Message::SetIqRef { iq } => {
            if CMD_MODE.load(Ordering::Relaxed) == 2 {
                let lim = param_get(param::IQ_LIMIT);
                CMD_AMP.store(iq.clamp(-lim, lim).to_bits(), Ordering::Relaxed);
                CMD_EPOCH.fetch_add(1, Ordering::Release);
            }
            ack
        }
        _ => nak(1),
    }
}

fn amp_limit(mode: u8) -> f32 {
    if mode >= 2 {
        param_get(param::IQ_LIMIT)
    } else {
        V_AMP_MAX
    }
}

/// Responses + decimated telemetry snapshots.
#[embassy_executor::task]
async fn tx_task(mut tx: UartTx<'static, Async>) {
    let mut divider = DIVIDER.load(Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_micros(50 * divider as u64));
    loop {
        match select(RESPONSES.receive(), ticker.next()).await {
            Either::First(reply) => send(&mut tx, &reply).await,
            Either::Second(()) => {
                let d = DIVIDER.load(Ordering::Relaxed);
                if d != divider {
                    divider = d;
                    ticker = Ticker::every(Duration::from_micros(50 * divider as u64));
                }
                if !STREAMING.load(Ordering::Relaxed) {
                    continue;
                }
                let mask = MASK.load(Ordering::Relaxed);
                let mut values = [0f32; channel::COUNT];
                // Seqlock read: retry while the ISR is mid-update.
                let (t_us, n) = loop {
                    let seq = TELEM_SEQ.load(Ordering::Acquire);
                    if seq & 1 != 0 {
                        continue;
                    }
                    let ticks = CONTROL_TICKS.load(Ordering::Relaxed);
                    let mut n = 0;
                    for id in 0..channel::COUNT as u8 {
                        if mask & (1 << id) == 0 {
                            continue;
                        }
                        values[n] = f32::from_bits(TELEM[id as usize].load(Ordering::Relaxed));
                        n += 1;
                    }
                    if TELEM_SEQ.load(Ordering::Acquire) == seq {
                        break (ticks.wrapping_mul(50), n);
                    }
                };
                if let Some(frame) = mmc_proto::TelemetryFrame::new(t_us, mask, &values[..n]) {
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

// ------------------------------------------------------------- control ISR

fn stage_off() {
    GPIOB.bsrr().write(|w| {
        w.set_br(13, true);
        w.set_br(14, true);
        w.set_br(15, true);
    });
    for ch in 0..3 {
        TIM1.ccr(ch).write(|w| w.set_ccr(0));
    }
}

fn stage_on() {
    GPIOB.bsrr().write(|w| {
        w.set_bs(13, true);
        w.set_bs(14, true);
        w.set_bs(15, true);
    });
}

fn set_duties(d: [f32; 3]) {
    for (ch, duty) in d.iter().enumerate() {
        let duty = duty.clamp(0.0, MAX_DUTY);
        TIM1.ccr(ch)
            .write(|w| w.set_ccr((duty * PWM_ARR as f32) as u16));
    }
}

/// The 20 kHz control loop, clocked by the injected-conversion ADC interrupt
/// (which TIM1 CH4 fires at the counter peak — mid low-side conduction).
#[no_mangle]
unsafe extern "C" fn ADC1_2() {
    if !ADC1.isr().read().jeos() {
        return;
    }
    ADC1.isr().write(|w| w.set_jeos(true));
    let isr_t0 = cortex_m::peripheral::DWT::cycle_count();

    let s = &mut *ISR_STATE.0.get();
    let ticks = CONTROL_TICKS.fetch_add(1, Ordering::Relaxed) + 1;

    let raw = [
        ADC1.jdr(0).read().jdata(),
        ADC1.jdr(1).read().jdata(),
        ADC1.jdr(2).read().jdata(),
        ADC1.jdr(3).read().jdata(),
    ];
    let volts = raw.map(|r| r as f32 * ADC_VOLTS_PER_LSB);
    let vbus = volts[3] * VBUS_GAIN;
    s.vbus_filt += 0.05 * (vbus - s.vbus_filt);

    // --- zero-current calibration (stage is off; measure amp offsets).
    if s.cal_count < CAL_TICKS {
        for (sum, &r) in s.cal_sum.iter_mut().zip(&raw[..3]) {
            *sum += r as u32;
        }
        s.cal_count += 1;
        if s.cal_count == CAL_TICKS {
            for (offset, &sum) in s.offset_v.iter_mut().zip(&s.cal_sum) {
                *offset = sum as f32 / CAL_TICKS as f32 * ADC_VOLTS_PER_LSB;
            }
            STATE.store(ST_OFF, Ordering::Relaxed);
        }
        return;
    }

    // Positive phase current (into the motor) pulls the amp output below its
    // zero-current offset.
    let i_abc = Abc {
        a: (s.offset_v[0] - volts[0]) / CUR_VOLTS_PER_AMP,
        b: (s.offset_v[1] - volts[1]) / CUR_VOLTS_PER_AMP,
        c: (s.offset_v[2] - volts[2]) / CUR_VOLTS_PER_AMP,
    };

    // --- pick up new host commands.
    let epoch = CMD_EPOCH.load(Ordering::Acquire);
    if epoch != s.epoch_seen {
        s.epoch_seen = epoch;
        let mode = CMD_MODE.load(Ordering::Relaxed);
        let state = STATE.load(Ordering::Relaxed);
        if mode == 0 {
            s.mode = 0;
            s.omega = 0.0;
            s.amp = 0.0;
            stage_off();
            burst_abort();
            if state >= ST_FAULT_OC {
                STATE.store(ST_OFF, Ordering::Relaxed); // fault re-arm
            } else if state == ST_RUN {
                STATE.store(ST_OFF, Ordering::Relaxed);
            }
        } else if state == ST_OFF || state == ST_RUN {
            if s.vbus_filt < VBUS_MIN_RUN || s.vbus_filt > VBUS_MAX {
                s.mode = 0;
                stage_off();
                burst_abort();
                STATE.store(ST_FAULT_VBUS, Ordering::Relaxed);
            } else {
                if s.mode == 0 {
                    // clean start — control blocks built from the runtime
                    // parameter table (profiler-writable).
                    s.theta = 0.0;
                    s.omega = 0.0;
                    s.amp = 0.0;
                    s.oc_strikes = 0;
                    s.stall_strikes = 0;
                    s.probe_ticks = 0;
                    let (rs, ls) = (param_get(param::R), param_get(param::L));
                    let gains = current_pi_gains(rs, ls, param_get(param::CUR_BW));
                    s.obs = Some(FluxObserver::new(FluxObserverCfg::new(rs, ls)));
                    if mode == 3 {
                        // Sensorless: feedforward FOC (flux is measured now),
                        // I-f sequencer toward the commanded direction, speed
                        // loop preloaded with the (blend-tapered) startup
                        // current at handoff. Handoff speed, ramp accel, and
                        // the current ceiling are runtime params so a new
                        // motor tunes without a reflash.
                        s.foc = Some(Foc::with_feedforward(
                            gains,
                            Decoupling {
                                ld: ls,
                                lq: ls,
                                flux: param_get(param::FLUX),
                            },
                        ));
                        let omega_t = f32::from_bits(CMD_OMEGA.load(Ordering::Relaxed));
                        let dir = if omega_t < 0.0 { -1.0 } else { 1.0 };
                        let iq_lim = param_get(param::IQ_LIMIT);
                        let handoff = param_get(param::SL_HANDOFF);
                        let i_start = f32::from_bits(CMD_AMP.load(Ordering::Relaxed))
                            .abs()
                            .clamp(0.1, iq_lim);
                        s.seq = Some(Sequencer::new(SequencerCfg {
                            i_start,
                            accel: param_get(param::OMEGA_ACCEL),
                            omega_handoff: handoff * dir,
                            ..SequencerCfg::default()
                        }));
                        s.speed = Some(SpeedLoop::new(
                            PiGains {
                                kp: param_get(param::SPEED_KP),
                                ki: param_get(param::SPEED_KI),
                            },
                            iq_lim,
                        ));
                        s.omega_ref_cur = handoff * dir;
                        s.sl_preload = i_start * dir;
                    } else {
                        s.foc = Some(Foc::new(gains));
                        s.seq = None;
                        s.speed = None;
                        s.sl_preload = 0.0;
                    }
                    stage_on();
                }
                s.mode = mode;
                STATE.store(ST_RUN, Ordering::Relaxed);
            }
        }
    }

    // --- protection trips (only meaningful once running).
    if s.mode != 0 {
        let drv_fault = GPIOA.idr().read().idr(11) == pac::gpio::vals::Idr::LOW
            || GPIOB.idr().read().idr(12) == pac::gpio::vals::Idr::LOW;
        let fault = if drv_fault {
            Some(ST_FAULT_DRV)
        } else if s.vbus_filt > VBUS_MAX {
            Some(ST_FAULT_VBUS)
        } else {
            let oc =
                i_abc.a.abs() > I_TRIP_A || i_abc.b.abs() > I_TRIP_A || i_abc.c.abs() > I_TRIP_A;
            s.oc_strikes = if oc { s.oc_strikes + 1 } else { 0 };
            (s.oc_strikes >= 2).then_some(ST_FAULT_OC)
        };
        if let Some(f) = fault {
            s.mode = 0;
            s.omega = 0.0;
            s.amp = 0.0;
            stage_off();
            burst_abort();
            STATE.store(f, Ordering::Relaxed);
            CMD_MODE.store(0, Ordering::Relaxed);
        }
        // Deadman: host silent too long with the stage live.
        let last = LAST_RX_TICK.load(Ordering::Relaxed);
        if ticks.wrapping_sub(last) > DEADMAN_TICKS {
            s.mode = 0;
            s.omega = 0.0;
            s.amp = 0.0;
            stage_off();
            burst_abort();
            STATE.store(ST_OFF, Ordering::Relaxed);
            CMD_MODE.store(0, Ordering::Relaxed);
        }
    }

    // --- drive.
    let mut duties = [0.0f32; 3];
    let mut v_dq = Dq::default();
    let mut i_dq = Dq::default();
    let mut iq_ref = 0.0f32;
    let mut theta_est = 0.0f32;
    let mut omega_est = 0.0f32;
    let mut theta_err = 0.0f32;

    if s.mode != 0 {
        let omega_target = f32::from_bits(CMD_OMEGA.load(Ordering::Relaxed));
        let amp_target = f32::from_bits(CMD_AMP.load(Ordering::Relaxed));
        let i_ab = clarke(i_abc);

        let v_ab = if s.mode == 4 && PROBE_KIND.load(Ordering::Relaxed) == test::L_THETA {
            // Saliency sweep: align at v_low on θ = 0 (parks a free rotor;
            // a clamped one just stays put and the fit recovers its angle),
            // then run the shared `mmc_core::probe` schedule — square-wave
            // v_d along ±paired electrical angles, recording (i_d, i_q) in
            // the excitation frame behind a self-describing header.
            s.probe_ticks += 1;
            let n = BURST_LEN.load(Ordering::Relaxed) as usize;
            if n >= probe::SAL_HDR + probe::SAL_TICKS * 2 {
                // Recording complete: stage off, hand the buffer to the host.
                s.mode = 0;
                stage_off();
                STATE.store(ST_OFF, Ordering::Relaxed);
                CMD_MODE.store(0, Ordering::Relaxed);
                BURST_STATE.store(2, Ordering::Release);
                AlphaBeta::default()
            } else {
                let v_low = f32::from_bits(PROBE_V_ALIGN.load(Ordering::Relaxed));
                let v_high = f32::from_bits(PROBE_V_STEP.load(Ordering::Relaxed));
                let half = PROBE_HALF.load(Ordering::Relaxed) as usize;
                let (theta_x, v) = if s.probe_ticks <= PROBE_ALIGN_TICKS {
                    (0.0, v_low)
                } else {
                    let t = (s.probe_ticks - PROBE_ALIGN_TICKS - 1) as usize;
                    // Safety: rx task only reads while BURST_STATE == 2.
                    let buf = unsafe { &mut *BURST.0.get() };
                    if t == 0 {
                        let hdr = probe::sal_header(test::L_THETA, half, v_low, v_high, CTRL_FREQ);
                        buf[..probe::SAL_HDR].copy_from_slice(&hdr);
                    }
                    (
                        probe::sal_angle(t, half),
                        if probe::sal_level_is_high(t, half) {
                            v_high
                        } else {
                            v_low
                        },
                    )
                };
                let sc = sin_cos(theta_x);
                i_dq = park(i_ab, sc);
                if s.probe_ticks > PROBE_ALIGN_TICKS {
                    let t = (s.probe_ticks - PROBE_ALIGN_TICKS - 1) as usize;
                    let idx = probe::SAL_HDR + 2 * t;
                    // Safety: rx task only reads while BURST_STATE == 2.
                    let buf = unsafe { &mut *BURST.0.get() };
                    buf[idx] = i_dq.d;
                    buf[idx + 1] = i_dq.q;
                    BURST_LEN.store((idx + 2) as u32, Ordering::Relaxed);
                }
                v_dq = Dq { d: v, q: 0.0 };
                let v_ab = inverse_park(v_dq, sc);
                duties = svpwm(v_ab, s.vbus_filt.max(1.0));
                v_ab
            }
        } else if s.mode == 4 {
            // Locked-rotor R/L probe: θ held at 0 (rotor aligned during the
            // first phase), then unslewed square-wave v_d between the two
            // levels, recording (i_d, v_d) per tick into the burst buffer at
            // the full 20 kHz.
            s.probe_ticks += 1;
            let n = BURST_LEN.load(Ordering::Relaxed) as usize;
            if n >= BURST_PAIRS * 2 {
                // Recording complete: stage off, hand the buffer to the host.
                s.mode = 0;
                stage_off();
                STATE.store(ST_OFF, Ordering::Relaxed);
                CMD_MODE.store(0, Ordering::Relaxed);
                BURST_STATE.store(2, Ordering::Release);
                AlphaBeta::default()
            } else {
                let v = f32::from_bits(if s.probe_ticks <= PROBE_ALIGN_TICKS {
                    PROBE_V_ALIGN.load(Ordering::Relaxed)
                } else {
                    let half = (s.probe_ticks - PROBE_ALIGN_TICKS - 1) / PROBE_HALF_TICKS;
                    if half.is_multiple_of(2) {
                        PROBE_V_STEP.load(Ordering::Relaxed)
                    } else {
                        PROBE_V_ALIGN.load(Ordering::Relaxed)
                    }
                });
                let sc = sin_cos(0.0);
                i_dq = park(i_ab, sc);
                if s.probe_ticks + PROBE_PRE_PAIRS as u32 > PROBE_ALIGN_TICKS {
                    // Safety: rx task only reads while BURST_STATE == 2.
                    let buf = unsafe { &mut *BURST.0.get() };
                    buf[n] = i_dq.d;
                    buf[n + 1] = v;
                    BURST_LEN.store((n + 2) as u32, Ordering::Relaxed);
                }
                v_dq = Dq { d: v, q: 0.0 };
                let v_ab = inverse_park(v_dq, sc);
                duties = svpwm(v_ab, s.vbus_filt.max(1.0));
                v_ab
            }
        } else if s.mode == 3 {
            // Sensorless: the sequencer owns the angle (I-f ramp → blend →
            // observer), the speed loop owns i_q once closed. The observer
            // state is one tick old here; it is fed below, same as the rig.
            let seq_out = s
                .seq
                .as_mut()
                .unwrap()
                .update(s.obs.as_ref().unwrap(), CTRL_DT);
            iq_ref = match seq_out.iq_open {
                Some(iq) => iq,
                None => {
                    let speed = s.speed.as_mut().unwrap();
                    if s.sl_preload != 0.0 {
                        // Bumpless takeover from the (blend-tapered) startup
                        // current the sequencer actually ended on.
                        let taper = s.seq.as_ref().map_or(1.0, |q| q.taper_end());
                        speed.preload(s.sl_preload * taper);
                        s.sl_preload = 0.0;
                    }
                    // Slew the reference from the handoff speed toward the
                    // (live-retargetable) command.
                    let accel = param_get(param::OMEGA_ACCEL);
                    let d = (omega_target - s.omega_ref_cur)
                        .clamp(-accel * CTRL_DT, accel * CTRL_DT);
                    s.omega_ref_cur += d;
                    speed.update(s.omega_ref_cur, seq_out.omega, CTRL_DT)
                }
            };
            s.theta = seq_out.theta;
            s.omega = seq_out.omega;
            let out = s.foc.as_mut().unwrap().step(
                i_abc,
                s.theta,
                s.omega,
                Dq { d: 0.0, q: iq_ref },
                s.vbus_filt.max(1.0),
                CTRL_DT,
            );
            duties = out.duties;
            v_dq = out.v_dq;
            i_dq = out.i_dq;
            out.v_ab
        } else {
            // Forced-frame modes: ramp the electrical frequency + amplitude.
            let accel = param_get(param::OMEGA_ACCEL);
            let d_omega = (omega_target - s.omega).clamp(-accel * CTRL_DT, accel * CTRL_DT);
            s.omega += d_omega;
            let slew = if s.mode == 2 { I_SLEW } else { V_SLEW };
            let d_amp = (amp_target - s.amp).clamp(-slew * CTRL_DT, slew * CTRL_DT);
            s.amp += d_amp;
            s.theta = wrap_angle(s.theta + s.omega * CTRL_DT);

            let sc = sin_cos(s.theta);
            i_dq = park(i_ab, sc);

            if s.mode == 1 {
                // Open-loop rotating voltage vector.
                v_dq = Dq { d: s.amp, q: 0.0 };
                let v_ab = inverse_park(v_dq, sc);
                duties = svpwm(v_ab, s.vbus_filt.max(1.0));
                v_ab
            } else {
                // I-f: closed current loop on the forced angle.
                iq_ref = s.amp;
                let out = s.foc.as_mut().unwrap().step(
                    i_abc,
                    s.theta,
                    s.omega,
                    Dq { d: 0.0, q: iq_ref },
                    s.vbus_filt.max(1.0),
                    CTRL_DT,
                );
                duties = out.duties;
                v_dq = out.v_dq;
                i_dq = out.i_dq;
                out.v_ab
            }
        };
        set_duties(duties);

        // Observer update from what was applied and measured. In the forced
        // modes it runs in shadow and theta_err is the (wrong-frame) hang
        // angle; in sensorless mode theta_err is the one-tick innovation.
        if let Some(obs) = s.obs.as_mut() {
            obs.update(i_ab, v_ab, CTRL_DT);
            theta_est = obs.electrical_angle();
            omega_est = obs.electrical_velocity();
            theta_err = wrap_angle(theta_est - s.theta);

            // Stall detector (closed-loop sensorless only): a stalled rotor
            // makes no back-EMF, but the observer still "locks" onto the
            // rotating L·i artifact and reports a confident, wrong speed —
            // with flux magnitude ≈ L·|i| instead of ψ (an order of
            // magnitude low; the MS4/MS5 fake-lock lesson). 100 ms below
            // 0.35·ψ while nominally closed-loop trips a stall fault.
            if s.mode == 3 && s.seq.as_ref().is_some_and(|q| q.phase() == Phase::Closed) {
                if obs.flux_mag() < 0.35 * param_get(param::FLUX) {
                    s.stall_strikes += 1;
                    if s.stall_strikes >= STALL_TICKS {
                        s.mode = 0;
                        s.omega = 0.0;
                        s.amp = 0.0;
                        stage_off();
                        burst_abort();
                        STATE.store(ST_STALL, Ordering::Relaxed);
                        CMD_MODE.store(0, Ordering::Relaxed);
                    }
                } else {
                    s.stall_strikes = 0;
                }
            } else {
                s.stall_strikes = 0;
            }
        }
    }

    // --- telemetry snapshot (seqlock).
    let seq = TELEM_SEQ.load(Ordering::Relaxed);
    TELEM_SEQ.store(seq.wrapping_add(1), Ordering::Release);
    let put = |id: u8, v: f32| TELEM[id as usize].store(v.to_bits(), Ordering::Relaxed);
    put(channel::IQ_REF, iq_ref);
    put(channel::I_D, i_dq.d);
    put(channel::I_Q, i_dq.q);
    put(channel::V_D, v_dq.d);
    put(channel::V_Q, v_dq.q);
    put(channel::DUTY_A, duties[0]);
    put(channel::DUTY_B, duties[1]);
    put(channel::DUTY_C, duties[2]);
    put(channel::OMEGA_M, s.omega / param_get(param::POLE_PAIRS).max(1.0));
    put(channel::THETA_E, s.theta);
    put(channel::VBUS, s.vbus_filt);
    put(channel::I_A, i_abc.a);
    put(channel::I_B, i_abc.b);
    put(channel::I_C, i_abc.c);
    // While sensorless runs, the state channel reports the startup phase.
    let state_telem = match (s.mode, s.seq.as_ref().map(|q| q.phase())) {
        (3, Some(Phase::Ramp)) => ST_SL_RAMP,
        (3, Some(Phase::Blend)) => ST_SL_BLEND,
        _ => STATE.load(Ordering::Relaxed),
    };
    put(channel::STATE, state_telem as f32);
    put(channel::THETA_EST, theta_est);
    put(channel::OMEGA_EST, omega_est);
    put(channel::THETA_ERR, theta_err);
    // Terminal voltages via the BEMF dividers (ADC2, parallel). Mapping
    // confirmed on hardware 2026-07-20: BEMF1=U on PC0/jdr0, BEMF3=W on
    // PC1/jdr1, BEMF2=V on PC3/jdr3 (PC2/jdr2 is the SPEED pot — railed).
    // The BAT30 clamps rectify these, so they read 0..peak (a zero-cross /
    // coast-down instrument, not a live terminal-voltage sense under PWM).
    let vb = |i: usize| ADC2.jdr(i).read().jdata() as f32 * ADC_VOLTS_PER_LSB * BEMF_GAIN;
    put(channel::VB_U, vb(0));
    put(channel::VB_V, vb(3));
    put(channel::VB_W, vb(1));
    TELEM_SEQ.store(seq.wrapping_add(2), Ordering::Release);

    let dur = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(isr_t0);
    let max = ISR_MAX_CYCLES.load(Ordering::Relaxed).max(dur);
    ISR_MAX_CYCLES.store(max, Ordering::Relaxed);
}
