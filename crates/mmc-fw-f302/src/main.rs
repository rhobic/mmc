//! Motor firmware for the STM32F302R8 dev board + L6230 inverter shield.
//! Bench hardware and schematics: hw/README.md.
//!
//! The second board under `mmc-drive`, and the first written against the
//! MCU's HAL from the start: clocks, GPIO, TIM1, the serial link and the
//! flash go through embassy's drivers. The one register-level piece is
//! [`init_adc`]: embassy has no timer-triggered conversions for this ADC
//! (the F3 "v1" ADC only gets blocking single reads), and PWM-synchronized
//! sampling is the whole point of the current sense. It is confined to that
//! function and the two reads in [`MotorBoard`].
//!
//! The MCU runs at 72 MHz, under half the first board's clock, so the control
//! loop runs at 10 kHz (the shared drive derives every time constant from
//! [`SPEC`]'s `ctrl_hz`).
//!
//! ## Pin map (shield schematic, F302 column; see hw/README.md)
//!
//! | Function | Pin | Notes |
//! |---|---|---|
//! | VCP UART | PA2/PA3 | USART2, 1 Mbaud |
//! | PWM U/V/W (L6230 IN1-3) | PA8/PA9/PA10 | TIM1 CH1-3, 20 kHz center-aligned |
//! | Phase enables (EN1-3) | PC10/PC11/PC12 | low = phase Hi-Z |
//! | DIAG/EN | PA6 | L6230 open-drain fault, 10k pull-up; low = fault |
//! | Current ref | PB4 | driven high: the L6230 comparator reference |
//! | i_U / i_V / i_W | PA0/PC1/PC0 | ADC1 IN1/IN7/IN6, 0.33 Ω, AV 1.53 |
//! | VBUS | PA1 | ADC1 IN2, 169k/9.31k |
//! | BEMF U / V / W | PC3/PB0/PA7 | ADC1 IN9/IN11/IN15, 10k/2.2k |
//! | BEMF divider enable | PC9 | low = dividers referenced to ground |
//! | Halls H1/H2/H3 | PA15/PB3/PB10 | TIM2 CH1-3 pins, read as GPIO |
//!
//! ## Sampling
//!
//! TIM1 counts up/down (center-aligned mode 1). Currents + VBUS are an
//! injected sequence on TIM1_CC4, compared just past the counter peak — the
//! middle of the low-side conduction, where the shunts carry phase current.
//! The BEMF terminals are a regular sequence on TIM1_TRGO2 (OC5REF), placed
//! before the valley inside the high-side on-time, landing in a 3-word DMA
//! ring the control tick reads.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::pac;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{CountingMode, OutputCompareMode, Timer};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Channel as TimCh};
use embassy_stm32::usart::{self, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use panic_halt as _;

use mmc_drive::{link, nvparam, BurstBuffer, DriveConfig, Engine, ParamStore, Shared};
use mmc_hal::{BoardSpec, MotorBoard, Sample};
use mmc_proto::{param, DeviceKind};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH7>;
});

// ---------------------------------------------------------------- the board

const SYSCLK_HZ: u32 = 72_000_000;
const CTRL_FREQ_HZ: u32 = 10_000;
/// PWM periods per control tick: the bridge switches at 20 kHz (inaudible,
/// and half the ripple of 10 kHz) while the loop runs at 10 kHz, which a
/// 72 MHz M4 can afford with margin.
const PWM_DIV: u32 = 2;
/// 72 MHz / (2·1800) = 20 kHz center-aligned.
const PWM_ARR: u16 = (SYSCLK_HZ / (2 * CTRL_FREQ_HZ * PWM_DIV)) as u16;

const ADC_VOLTS_PER_LSB: f32 = 3.3 / 4096.0;
/// VBUS divider 169k / 9.31k.
const VBUS_GAIN: f32 = (169.0 + 9.31) / 9.31;
/// BEMF divider: OUTx → 10k → 2.2k → GPIO_BEMF (PC9 low).
const BEMF_GAIN: f32 = 12.2 / 2.2;
const MAX_DUTY: f32 = 0.85; // keeps the low-side sampling window open

/// TIM1 CCR5 default, in counts before the valley: where the BEMF sequence
/// starts. Its three conversions take 3·(19.5+12.5)/72 MHz = 1.33 µs, so
/// they finish inside the on-time from duty ≈ (96 + 100)/1800 ≈ 0.11 up.
const ONTIME_CCR5_DEFAULT: u16 = 100;

const SPEC: BoardSpec = BoardSpec {
    ctrl_hz: CTRL_FREQ_HZ,
    // 0.33 Ω shunt → 680R/2.2k bias → ×1.53 amplifier (same front end as the
    // G474 shield's).
    cur_volts_per_amp: 1.53 * 0.33,
    // The L6230 is rated 2.8 A peak; the trip stays where the first bench
    // proved it until the new motor is characterised.
    i_trip: 1.5,
    vbus_max: 30.0,
    vbus_min_run: 5.0,
    max_duty: MAX_DUTY,
    // L6230 R_DS(on) HS + LS ≈ 1.35 Ω typ → ~0.68 Ω conducting per leg, plus
    // the 0.33 Ω shunt duty-weighted ≈ 0.32 Ω. Datasheet-typical, not yet
    // closed against a known motor.
    r_path: 1.0,
    terminal_offset_max: (PWM_ARR / 2) as f32,
    has_halls: true,
    // The ISR (~30 µs to the duty write) ends after the PWM valley that
    // follows its trigger, so new compares load at the next peak: half a
    // 100 µs tick late.
    pwm_latency: 0.5,
    // 3.3 V ADC full scale through the BEMF divider.
    terminal_full_scale: 3.3 * BEMF_GAIN,
};

/// Parameter defaults: gentle placeholders for an unknown motor, except
/// where motor 3 (maxon EC-i 40) has been measured on this bench — pole
/// pairs from its datasheet, the hall map from `tools/hall_cal.py`
/// (session 30). The profiler replaces R/L/flux/speed gains via `apply`.
const DEFAULTS: [f32; param::COUNT] = [
    1.0,                        // R
    0.5e-3,                     // L
    5.0e-3,                     // FLUX
    1000.0,                     // CUR_BW
    1.0e-4,                     // SPEED_KP
    1.0e-3,                     // SPEED_KI
    7.0,                        // POLE_PAIRS (maxon EC-i 40 datasheet)
    150.0,                      // SL_HANDOFF
    300.0,                      // OMEGA_ACCEL
    0.8,                        // IQ_LIMIT
    ONTIME_CCR5_DEFAULT as f32, // ONTIME_CCR5
    2.0e-4,                     // SS_KP
    5.0e-4,                     // SS_KI
    0.0,                        // V_DEAD
    0.3,                        // I_THRESH
    -1.0775,                    // HALL_OFFSET (motor 3, hall_cal.py)
    1.0,                        // HALL_DIR
    0.06,                       // HALL_HYST (motor 3, hall_ref.py vs observer)
    // Position loop for motor 3: p·kt/J ≈ 1.1e5 rad/s² per A (electrical),
    // so ~50 rad/s bandwidth is kp = 50²/1.1e5 and kd = 2·0.9·50/1.1e5.
    0.022,  // POS_KP [A/rad el]
    0.2,    // POS_KI
    8e-4,   // POS_KD [A/(rad/s el)]
    200.0,  // POS_VMAX [rad/s el]
    4.4e-6, // INERTIA [kg·m²] (maxon EC-i 40 datasheet)
    0.11,   // I_FRIC [A] (motor 3: steady i_q in hall FOC)
    120.0,  // SS_CONDUCTION [deg el]
    0.0,    // ID_INJECT [A]
    // HALL_W0..5 [rad el]: even until tools/hall_widths.py has run.
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    0.0, // ID_DITHER [A] (off)
    0.5, // ID_DITHER_PERIOD [s]
    0.0, // COG_FF (off)
    -1.0, // COG_SHIFT (identify)
    0.0, 0.0, 0.0, 0.0, // COG_N0..3 (no series until measured)
    0.0, 0.0, 0.0, 0.0, // COG_A0..3 [N·m]
    0.0, 0.0, 0.0, 0.0, // COG_P0..3 [rad]
    0.0,   // HFI_V [V] (off)
    300.0, // HFI_BW [rad/s]
    0.05,  // HFI_XI
    0.0,   // HFI_XSAT [rad/A] (off)
    0.0,   // SL_FRIC [A] (off)
    0.0,   // HFI_KP (0 = speed_kp/speed_ki)
    0.0,   // HFI_KI
    0.006, // HFI_POL_S [s]
    8.0,   // HFI_POL_N
    0.0,   // HFI_SPREAD (fixed ++--)
];

/// Probe burst capacity [f32s]: 8 KB of the 16 KB RAM. Enough for the R/L
/// probe (1024 pairs); the 32 KB saliency sweep does not fit this MCU and is
/// NAKed (the host knows from `BoardTraits::burst_cap`).
const BURST: usize = 2048;

static BURST_BUF: BurstBuffer<BURST> = BurstBuffer::new();
static SHARED: Shared<BURST> = Shared::new(
    DriveConfig {
        spec: SPEC,
        kind: DeviceKind::BoardF302,
        fw_version: 28,
        name: "mmc-f302",
        defaults: DEFAULTS,
    },
    &BURST_BUF,
);

/// DMA target of the BEMF regular sequence: [U (IN9), V (IN11), W (IN15)].
struct BemfRing(UnsafeCell<[u16; 3]>);
// Safety: written by DMA, read (volatile) by the control ISR only.
unsafe impl Sync for BemfRing {}
static BEMF_RING: BemfRing = BemfRing(UnsafeCell::new([0; 3]));

struct F302Board {
    pwm: Timer<'static, TIM1>,
    en: [Output<'static>; 3],
    diag: Input<'static>,
    halls: [Input<'static>; 3],
    // Held so their pin configuration outlives `main` (embassy pins reset
    // to analog when dropped).
    _pwm_pins: (
        PwmPin<'static, TIM1, Ch1>,
        PwmPin<'static, TIM1, Ch2>,
        PwmPin<'static, TIM1, Ch3>,
    ),
    _bemf_enable: Output<'static>,
    _current_ref: Output<'static>,
}

impl MotorBoard for F302Board {
    fn sample(&mut self) -> Sample {
        let adc = pac::ADC1;
        let v = |i| adc.jdr(i).read().jdata() as f32 * ADC_VOLTS_PER_LSB;
        Sample {
            phase_volts: [v(0), v(1), v(2)],
            vbus: v(3) * VBUS_GAIN,
        }
    }

    fn terminal_volts(&mut self) -> [f32; 3] {
        let ring = BEMF_RING.0.get() as *const u16;
        // Safety: aligned static the DMA writes halfwords into.
        let r = |i| unsafe { core::ptr::read_volatile(ring.add(i)) };
        [0, 1, 2].map(|i| r(i) as f32 * ADC_VOLTS_PER_LSB * BEMF_GAIN)
    }

    fn set_terminal_sample_offset(&mut self, offset: f32) {
        // CH5 is outside embassy's 4-channel API; same typed register block.
        self.pwm
            .regs_advanced()
            .ccr5()
            .modify(|w| w.set_ccr(offset as u16));
    }

    fn set_duties(&mut self, d: [f32; 3]) {
        for (ch, duty) in [TimCh::Ch1, TimCh::Ch2, TimCh::Ch3].into_iter().zip(d) {
            let duty = duty.clamp(0.0, MAX_DUTY);
            self.pwm
                .set_compare_value(ch, (duty * PWM_ARR as f32) as u16);
        }
    }

    fn set_phase_enables(&mut self, mask: u8) {
        for (bit, pin) in self.en.iter_mut().enumerate() {
            pin.set_level(Level::from(mask & (1 << bit) != 0));
        }
    }

    fn driver_fault(&mut self) -> bool {
        self.diag.is_low()
    }

    fn hall_state(&mut self) -> Option<u8> {
        let mut s = 0;
        for (bit, pin) in self.halls.iter().enumerate() {
            s |= (pin.is_high() as u8) << bit;
        }
        Some(s)
    }

    fn cycles(&self) -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }
}

/// Parameter blob in the last 2 KB flash page (0x0800_F800), which memory.x
/// keeps out of the image. This part has one flash bank, so an erase stalls
/// the CPU (and the control ISR) for tens of ms — the drive only persists
/// with the stage off, so that stall drives nothing.
struct FlashStore(Flash<'static, Blocking>);

const NV_OFFSET: u32 = 0xF800; // from FLASH_BASE
const NV_ADDR: usize = 0x0800_F800;

impl ParamStore for FlashStore {
    fn read(&mut self) -> &[u8] {
        // Safety: flash is memory-mapped and this page is only written here.
        unsafe { core::slice::from_raw_parts(NV_ADDR as *const u8, nvparam::BYTES) }
    }
    fn write(&mut self, blob: &[u8; nvparam::BYTES]) -> bool {
        self.erase() && self.0.blocking_write(NV_OFFSET, blob).is_ok()
    }
    fn erase(&mut self) -> bool {
        self.0.blocking_erase(NV_OFFSET, NV_OFFSET + 2048).is_ok()
    }
}

/// Everything the control interrupt owns: the drive engine, the board, and
/// the PWM-period phase counter (see [`PWM_DIV`]). Written once by `main`
/// before the interrupt is unmasked.
struct IsrCell(UnsafeCell<MaybeUninit<(Engine, F302Board, u32)>>);
// Safety: initialised before ADC1 is unmasked; only the ISR touches it after.
unsafe impl Sync for IsrCell {}
static ISR: IsrCell = IsrCell(UnsafeCell::new(MaybeUninit::uninit()));

static REPLIES: link::Replies = link::Replies::new();

// ------------------------------------------------------------------- tasks

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        // 8 MHz from the debugger's MCO (HSE bypass) × 9 = 72 MHz. The ADC
        // runs synchronously from HCLK/1.
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.adc = AdcClockSource::Hclk(AdcHclkPrescaler::Div1);
    }
    let p = embassy_stm32::init(config);

    // Restore persisted parameters before the control loop reads them.
    let mut store = FlashStore(Flash::new_blocking(p.FLASH));
    SHARED.restore(&mut store);

    // Cycle counter feeds the isr_max_cycles diagnostic.
    unsafe {
        let mut cp = cortex_m::Peripherals::steal();
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
    }

    let mut cfg = usart::Config::default();
    cfg.baudrate = 1_000_000;
    let uart = Uart::new(p.USART2, p.PA3, p.PA2, p.DMA1_CH7, p.DMA1_CH6, Irqs, cfg)
        .expect("USART2 config");
    let (tx, rx) = uart.split();
    let rx_ring = cortex_m::singleton!(: [u8; 256] = [0; 256]).unwrap();
    let rx = rx.into_ring_buffered(rx_ring);

    // --- power stage: everything off before the timer starts.
    let en = [
        Output::new(p.PC10, Level::Low, Speed::Low),
        Output::new(p.PC11, Level::Low, Speed::Low),
        Output::new(p.PC12, Level::Low, Speed::Low),
    ];
    let diag = Input::new(p.PA6, Pull::Up);
    // The L6230 compares the shunt sense against this node; left floating it
    // would sit at ground and hold the comparator tripped.
    let current_ref = Output::new(p.PB4, Level::High, Speed::Low);
    let bemf_enable = Output::new(p.PC9, Level::Low, Speed::Low);
    let halls = [
        Input::new(p.PA15, Pull::Up),
        Input::new(p.PB3, Pull::Up),
        Input::new(p.PB10, Pull::Up),
    ];

    // --- TIM1: 20 kHz center-aligned PWM; CH4 places the current sample,
    // CH5 (internal) the BEMF sample.
    // Kept for the life of the firmware: dropping a `PwmPin` returns its pin
    // to analog mode — the L6230's IN pins then float with EN high, which on
    // the first bring-up drove the bridge into overcurrent (session 30).
    let pwm_pins = (
        PwmPin::<TIM1, Ch1>::new(p.PA8, OutputType::PushPull),
        PwmPin::<TIM1, Ch2>::new(p.PA9, OutputType::PushPull),
        PwmPin::<TIM1, Ch3>::new(p.PA10, OutputType::PushPull),
    );
    let pwm = Timer::new(p.TIM1);
    pwm.set_counting_mode(CountingMode::CenterAlignedDownInterrupts);
    pwm.set_max_compare_value(PWM_ARR);
    pwm.set_autoreload_preload(true);
    for ch in [TimCh::Ch1, TimCh::Ch2, TimCh::Ch3, TimCh::Ch4] {
        pwm.set_output_compare_mode(ch, OutputCompareMode::PwmMode1);
        pwm.set_output_compare_preload(ch, true);
        pwm.set_compare_value(ch, 0);
        pwm.enable_channel(ch, true);
    }
    // CC4 fires on the down-count just past the peak (CMS mode 1 sets the
    // flag only when counting down): mid low-side conduction.
    pwm.set_compare_value(TimCh::Ch4, PWM_ARR - 20);
    {
        use pac::timer::vals;
        let r = pwm.regs_advanced();
        // OC5REF is high while CNT < CCR5, so its rising edge lands just
        // before the valley — inside the high-side on-time — and TRGO2
        // follows it to start the BEMF sequence.
        r.ccmr3().modify(|w| {
            w.set_ocm(0, vals::Ocm::PWM_MODE1);
            w.set_ocpe(0, true);
        });
        r.ccr5().modify(|w| w.set_ccr(ONTIME_CCR5_DEFAULT));
        r.ccer().modify(|w| w.set_cce(4, true));
    }
    pwm.set_mms2_selection(pac::timer::vals::Mms2::COMPARE_OC5);
    pwm.set_moe(true);
    pwm.generate_update_event();
    pwm.start();

    init_adc();

    unsafe {
        (*ISR.0.get()).write((
            Engine::new(),
            F302Board {
                pwm,
                en,
                diag,
                halls,
                _pwm_pins: pwm_pins,
                _bemf_enable: bemf_enable,
                _current_ref: current_ref,
            },
            0,
        ));
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ADC1);
    }

    spawner.spawn(rx_task(rx, store).unwrap());
    spawner.spawn(tx_task(tx).unwrap());
}

/// ADC1 + its DMA channel, register-level (see the module docs for why).
/// Injected: iU, iV, iW, VBUS on TIM1_CC4 with an end-of-sequence interrupt
/// (the control clock). Regular: BEMF U, V, W on TIM1_TRGO2 into
/// [`BEMF_RING`] by circular DMA.
fn init_adc() {
    use pac::adc::vals::{Advregen, Dmacfg, Exten, SampleTime};
    let adc = pac::ADC1;

    pac::RCC.ahbenr().modify(|w| {
        w.set_adc12en(true);
        w.set_dma1en(true);
    });
    // Synchronous clock, HCLK/1 = 72 MHz (requires AHB prescaler 1).
    pac::ADC1_COMMON
        .ccr()
        .modify(|w| w.set_ckmode(pac::adccommon::vals::Ckmode::SYNC_DIV1));

    // Regulator: intermediate → enabled, then ≥ 10 µs startup.
    adc.cr().modify(|w| w.set_advregen(Advregen::INTERMEDIATE));
    adc.cr().modify(|w| w.set_advregen(Advregen::ENABLED));
    cortex_m::asm::delay(72 * 20);

    adc.cr().modify(|w| {
        w.set_adcaldif(false);
        w.set_adcal(true);
    });
    while adc.cr().read().adcal() {}
    cortex_m::asm::delay(72);

    adc.isr().write(|w| w.set_adrdy(true));
    adc.cr().modify(|w| w.set_aden(true));
    while !adc.isr().read().adrdy() {}

    // Sampling times. SMPR1 holds channels 1..9 at index ch−1, SMPR2
    // channels 10..18 at ch−10. The current amps are op-amp driven (7.5);
    // VBUS sits behind ~8.8 kΩ with a 4.7 nF hold cap (19.5); the BEMF
    // dividers' Thevenin source is ~1.8 kΩ (19.5).
    adc.smpr1().modify(|w| {
        w.set_smp(0, SampleTime::CYCLES7_5); // IN1  iU   PA0
        w.set_smp(1, SampleTime::CYCLES19_5); // IN2  VBUS PA1
        w.set_smp(5, SampleTime::CYCLES7_5); // IN6  iW   PC0
        w.set_smp(6, SampleTime::CYCLES7_5); // IN7  iV   PC1
        w.set_smp(8, SampleTime::CYCLES19_5); // IN9  BEMF U PC3
    });
    adc.smpr2().modify(|w| {
        w.set_smp(1, SampleTime::CYCLES19_5); // IN11 BEMF V PB0
        w.set_smp(5, SampleTime::CYCLES19_5); // IN15 BEMF W PA7
    });

    // Injected: 4 conversions on TIM1_CC4 (JEXTSEL 1).
    adc.jsqr().write(|w| {
        w.set_jl(3);
        w.set_jextsel(1);
        w.set_jexten(Exten::RISING_EDGE);
        w.set_jsq(0, 1); // iU
        w.set_jsq(1, 7); // iV
        w.set_jsq(2, 6); // iW
        w.set_jsq(3, 2); // VBUS
    });

    // Regular: 3 conversions on TIM1_TRGO2 (EXTSEL 10), circular DMA.
    adc.sqr1().write(|w| {
        w.set_l(2);
        w.set_sq(0, 9); // BEMF U
        w.set_sq(1, 11); // BEMF V
        w.set_sq(2, 15); // BEMF W
    });
    adc.cfgr().modify(|w| {
        w.set_extsel(10);
        w.set_exten(Exten::RISING_EDGE);
        w.set_ovrmod(true); // a missed read overwrites rather than stalls
        w.set_dmaen(true);
        w.set_dmacfg(Dmacfg::CIRCULAR);
    });
    // ADC1 is hard-wired to DMA1 channel 1 on this family.
    let ch = pac::DMA1.ch(0);
    ch.par().write_value(adc.dr().as_ptr() as u32);
    ch.mar().write_value(BEMF_RING.0.get() as u32);
    ch.ndtr().write(|w| w.set_ndt(3));
    ch.cr().write(|w| {
        use pac::bdma::vals::{Dir, Size};
        w.set_dir(Dir::FROM_PERIPHERAL);
        w.set_psize(Size::BITS16);
        w.set_msize(Size::BITS16);
        w.set_minc(true);
        w.set_circ(true);
        w.set_en(true);
    });

    adc.ier().modify(|w| w.set_jeosie(true));
    adc.cr().modify(|w| {
        w.set_jadstart(true); // arm both hardware triggers
        w.set_adstart(true);
    });
}

/// The host link: `mmc_drive::link`'s loops on this board's USART halves.
/// The receive side is ring-buffered (circular DMA) so no byte is lost while
/// a frame is being handled.
#[embassy_executor::task]
async fn rx_task(rx: RingBufferedUartRx<'static>, mut store: FlashStore) {
    link::rx_loop(&SHARED, &REPLIES, rx, &mut store).await
}

#[embassy_executor::task]
async fn tx_task(tx: UartTx<'static, Async>) {
    link::tx_loop(&SHARED, &REPLIES, tx).await
}

// ------------------------------------------------------------- control ISR

/// The control loop, clocked by the injected end-of-sequence interrupt
/// (TIM1 CC4, just past the counter peak). It arrives at the PWM rate; every
/// [`PWM_DIV`]-th one runs a tick.
#[no_mangle]
unsafe extern "C" fn ADC1() {
    let adc = pac::ADC1;
    if !adc.isr().read().jeos() {
        return;
    }
    adc.isr().write(|w| w.set_jeos(true));

    let (engine, board, pwm_phase) = (*ISR.0.get()).assume_init_mut();
    *pwm_phase += 1;
    if *pwm_phase < PWM_DIV {
        return;
    }
    *pwm_phase = 0;
    engine.tick(&SHARED, board);
}
