//! Motor firmware for the STM32G474RE dev board + STSPIN830 inverter shield.
//! Bench hardware and schematics: hw/README.md.
//!
//! This crate is the *board*: clocks, the center-aligned TIM1 PWM, the
//! PWM-synchronized injected-ADC shunt sensing, the BEMF ADC, GPIO, the
//! serial link and the parameter flash page — wired up as an
//! [`mmc_hal::MotorBoard`]. Everything the drive *does* (modes, probes,
//! trips, telemetry, protocol) is the board-agnostic `mmc-drive` crate, run
//! from the ADC interrupt.
//!
//! The peripheral setup is still register-level: it is the hardware-proven
//! configuration from MS5–MS8, moved here unchanged when the drive was split
//! out (session 30) because no G474 was on the bench to re-validate a
//! rewrite. Moving it onto embassy's timer/ADC drivers is a follow-up for the
//! next session that has this board connected.
//!
//! ## Pin map (from the shield + dev-board schematics; see hw/README.md)
//!
//! | Function            | Pin  | Notes                                     |
//! |---------------------|------|-------------------------------------------|
//! | VCP UART            | PA2/PA3 | LPUART1 (SB17/SB23), 1 Mbaud           |
//! | PWM U/V/W (driver IN)| PA8/PA9/PA10 | TIM1 CH1/2/3, AF6, 40 kHz center |
//! | Phase enables (EN)  | PB13/PB14/PB15 | GPIO; low = phase Hi-Z          |
//! | Gate-driver STBY    | PB5  | high = run                                |
//! | EN_FAULT (in)       | PA11 + PB12 | open-drain, low = fault. The shield routes it to PB12 (R37) by default and to PA11 (R35) on other board variants — the vendor's example config uses PA11. Both are read with internal pull-ups, so whichever is unconnected floats high and stays silent. (TIM1_BKIN2 hardware break on PA11 is a follow-up.) |
//! | Current ref (VREF)  | PB4  | GPIO high → VREF ≈ 0.50 V (max via 22k/3.9k divider). This is the *weakest* hardware current limit (≈1.5 A on 0.33 Ω); floating PB4 would pull VREF toward 0 V and trip continuously (the driver disables outputs for tOFF whenever VSNS > VREF) |
//! | i_U / i_V / i_W     | PA1/PB1/PB0 | ADC1 IN2/IN12/IN15, ×2 shunt amp  |
//! | VBUS                | PA0  | ADC1 IN1, 180k/12k divider (×16)          |
//! | BEMF U / W / V      | PC0/PC1/PC3 | ADC2 IN6/IN7/IN9, 10k/2.2k, enabled by PC9 low |
//!
//! ## Current-sense scaling
//!
//! 0.33 Ω shunt → 680R/2.2k bias to 3.3 V → non-inverting ×2 amplifier:
//! `v_adc = 1.558 V − 1.528·0.33·i_phase` (positive current into the motor
//! discharges the node). Offsets are measured at boot with the stage disabled;
//! the slope is 0.5042 V/A.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;

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

use mmc_drive::{nvparam, BurstBuffer, DriveConfig, Engine, ParamStore, Shared};
use mmc_hal::{BoardSpec, MotorBoard, Sample};
use mmc_proto::{encode, param, Deframer, DeviceKind, Message};

bind_interrupts!(struct Irqs {
    LPUART1 => usart::InterruptHandler<peripherals::LPUART1>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>;
});

// ---------------------------------------------------------------- the board

const CTRL_FREQ_HZ: u32 = 20_000;
/// PWM periods per control tick. The bridge switches faster than the loop
/// runs because ripple current is `V_bus·d·(1−d)/(2L·f_sw)`, and on a
/// low-inductance motor (30 µH here) 20 kHz put the six-step *peak* past the
/// driver's limit while the average still looked modest — the current sample
/// sits at the counter peak, which is the middle of the freewheel and so the
/// ripple *minimum*, so the peak never appeared in any capture. Doubling the
/// switching frequency halves the ripple. Control cannot simply follow it: the
/// FOC tick is ~4060 cycles against the 4250 a 40 kHz tick would allow, which
/// is no margin, so every second conversion is dropped instead.
const PWM_DIV: u32 = 2;
/// 170 MHz / (2·2125) = 40 kHz center-aligned; control ticks every 2nd period.
const PWM_ARR: u16 = (170_000_000 / (2 * CTRL_FREQ_HZ * PWM_DIV)) as u16;

const ADC_VOLTS_PER_LSB: f32 = 3.3 / 4096.0;
const VBUS_GAIN: f32 = 16.0; // 180k/12k divider
/// BEMF divider: OUTx → 10K → 2.2K → IO_BEMF (PC9 low), ratio 2.2/12.2.
const BEMF_GAIN: f32 = 12.2 / 2.2;
const MAX_DUTY: f32 = 0.85; // keeps the low-side sampling window open

/// TIM1 CCR5, in counts before the counter valley, where OC5REF rises and
/// triggers ADC2. The valley is the middle of the high-side on-time (PWM mode
/// 1, centre-aligned), so this samples the terminals while the bridge is
/// driving — which is the only point where the idle phase carries back-EMF
/// referenced to V_bus/2 rather than to ground. 100 counts = 0.59 us.
///
/// It must sit inside the on-window, i.e. below the commanded duty's compare
/// value, with room left for ADC2's four conversions (1.79 us). The window
/// scales with the PWM period, so raising the switching frequency raises the
/// minimum usable duty: at 40 kHz (`PWM_ARR` 2125) the trigger leaves
/// `(100 + d·2125)/170 MHz`, which clears 1.79 us only from **duty ≈ 0.10**
/// upward. Below that the sample lands while the bridge is freewheeling and
/// the idle phase reads nothing useful. At 20 kHz the same default cleared it
/// from duty 0.07.
const ONTIME_CCR5_DEFAULT: u16 = 100;

const SPEC: BoardSpec = BoardSpec {
    ctrl_hz: CTRL_FREQ_HZ,
    // 0.33 Ω shunt → 680R/2.2k bias → ×1.528 amplifier (see module docs).
    cur_volts_per_amp: 1.528 * 0.33,
    i_trip: 1.5,
    vbus_max: 30.0,
    vbus_min_run: 5.0,
    max_duty: MAX_DUTY,
    // STSPIN830 R_DSon HS+LS ≈ 1 Ω typ → 0.5 Ω conducting, plus the 0.33 Ω
    // shunt duty-weighted ≈ 0.32 Ω (testresults/motor2-4pole/
    // datasheet-comparison.md).
    r_path: 0.85,
    // Sample point inside a 50% duty on-window, so it tracks the PWM period.
    terminal_offset_max: (PWM_ARR / 2) as f32,
    // The shield has a hall connector, but no motor on this bench has used
    // it; the inputs are not wired up in this firmware.
    has_halls: false,
    // Same timing argument as the F302, unmeasured here: the ~24 µs ISR
    // writes after the 40 kHz PWM's next update point (12.5 µs past the
    // trigger), so compares load a 25 µs PWM period later = half a tick.
    pwm_latency: 0.5,
    // 3.3 V ADC full scale through the BEMF divider.
    terminal_full_scale: 3.3 * BEMF_GAIN,
};

/// Parameter defaults: motor 1, measured on this bench (Stage F0 rotating
/// I-f sweep, `tools/fit_params.py`): flux 0.894 ± 0.04 mWb, apparent R
/// 0.97 Ω (locked-rotor R = 1.0), friction ≈ 0.8 mN·m. L was ill-conditioned
/// in that test — 0.1 mH is a robust design center, and it only sets the
/// current-PI zero and a small observer flux correction. Speed gains are
/// conservative (~40 rad/s el from the fitted J and kt). Dead-time
/// compensation stays 0 until `profile --only vdead` has run on the rig: an
/// over-compensated bridge is worse than an uncompensated one.
const DEFAULTS: [f32; param::COUNT] = [
    1.0,                        // R
    0.1e-3,                     // L
    0.894e-3,                   // FLUX
    1000.0,                     // CUR_BW
    2.0e-4,                     // SPEED_KP
    2.0e-3,                     // SPEED_KI
    7.0,                        // POLE_PAIRS
    150.0,                      // SL_HANDOFF
    500.0,                      // OMEGA_ACCEL
    0.8,                        // IQ_LIMIT
    ONTIME_CCR5_DEFAULT as f32, // ONTIME_CCR5
    2.0e-4,                     // SS_KP
    5.0e-4,                     // SS_KI
    0.0,                        // V_DEAD
    0.5,                        // I_THRESH (FOC ripple amplitude, 30 µH at 40 kHz)
    0.0,                        // HALL_OFFSET (unused: no halls)
    1.0,                        // HALL_DIR
    0.0,                        // HALL_HYST
    0.0,                        // POS_KP (no halls: position mode unavailable)
    0.0,                        // POS_KI
    0.0,                        // POS_KD
    200.0,                      // POS_VMAX
    1.75e-6,                    // INERTIA (motor 1 fit)
    0.0,                        // I_FRIC
    120.0,                      // SS_CONDUCTION [deg el]
    0.0,                        // ID_INJECT [A]
    core::f32::consts::FRAC_PI_3, // HALL_W0..5 [rad el] (no halls: unused)
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    core::f32::consts::FRAC_PI_3,
    0.0,                        // ID_DITHER [A] (off)
    0.5,                        // ID_DITHER_PERIOD [s]
    0.0, // COG_FF (off)
    -1.0, // COG_SHIFT (identify)
    0.0, 0.0, 0.0, 0.0, // COG_N0..3 (no series until measured)
    0.0, 0.0, 0.0, 0.0, // COG_A0..3 [N·m]
    0.0, 0.0, 0.0, 0.0, // COG_P0..3 [rad]
];

/// f32 capacity of the probe burst buffer: the saliency sweep's full
/// schedule (the R/L probe uses the same buffer).
const BURST: usize = mmc_core::probe::SAL_HDR + mmc_core::probe::SAL_TICKS * 2;

static BURST_BUF: BurstBuffer<BURST> = BurstBuffer::new();
static SHARED: Shared<BURST> = Shared::new(
    DriveConfig {
        spec: SPEC,
        kind: DeviceKind::BoardG474,
        fw_version: 24,
        name: "mmc-g474",
        defaults: DEFAULTS,
    },
    &BURST_BUF,
);

/// The power stage and sensing, as the drive sees them. Zero-sized: every
/// method addresses the (already configured) peripherals directly.
struct G474Board;

impl MotorBoard for G474Board {
    fn sample(&mut self) -> Sample {
        let v = |i| ADC1.jdr(i).read().jdata() as f32 * ADC_VOLTS_PER_LSB;
        Sample {
            phase_volts: [v(0), v(1), v(2)],
            vbus: v(3) * VBUS_GAIN,
        }
    }

    /// Mapping confirmed on hardware 2026-07-20: BEMF1=U on PC0/jdr0, BEMF3=W
    /// on PC1/jdr1, BEMF2=V on PC3/jdr3 (PC2/jdr2 is the SPEED pot — railed).
    fn terminal_volts(&mut self) -> [f32; 3] {
        let v = |i| ADC2.jdr(i).read().jdata() as f32 * ADC_VOLTS_PER_LSB * BEMF_GAIN;
        [v(0), v(3), v(1)]
    }

    fn set_terminal_sample_offset(&mut self, offset: f32) {
        TIM1.ccr5().modify(|w| w.set_ccr(offset as u16));
    }

    fn set_duties(&mut self, d: [f32; 3]) {
        for (ch, duty) in d.iter().enumerate() {
            let duty = duty.clamp(0.0, MAX_DUTY);
            TIM1.ccr(ch)
                .write(|w| w.set_ccr((duty * PWM_ARR as f32) as u16));
        }
    }

    /// Phase enables on PB13/PB14/PB15. A disabled phase is Hi-Z — both its
    /// switches open.
    fn set_phase_enables(&mut self, mask: u8) {
        GPIOB.bsrr().write(|w| {
            for (bit, pin) in [(0, 13), (1, 14), (2, 15)] {
                if mask & (1 << bit) != 0 {
                    w.set_bs(pin, true);
                } else {
                    w.set_br(pin, true);
                }
            }
        });
    }

    fn driver_fault(&mut self) -> bool {
        GPIOA.idr().read().idr(11) == pac::gpio::vals::Idr::LOW
            || GPIOB.idr().read().idr(12) == pac::gpio::vals::Idr::LOW
    }

    fn cycles(&self) -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }
}

/// Parameter blob in the last 2 KB page of bank 2 (0x0807_F800). Code
/// executes from bank 1, so erasing/programming this page is
/// read-while-write — the control ISR keeps running from bank 1 untouched.
struct FlashStore(Flash<'static, Blocking>);

const NV_OFFSET: u32 = 0x7_F800; // from FLASH_BASE
const NV_ADDR: usize = 0x0807_F800;

impl ParamStore for FlashStore {
    fn read(&mut self) -> &[u8] {
        // Safety: flash is memory-mapped and this page is only written by us.
        unsafe { core::slice::from_raw_parts(NV_ADDR as *const u8, nvparam::BYTES) }
    }
    fn write(&mut self, blob: &[u8; nvparam::BYTES]) -> bool {
        self.erase() && self.0.blocking_write(NV_OFFSET, blob).is_ok()
    }
    fn erase(&mut self) -> bool {
        self.0.blocking_erase(NV_OFFSET, NV_OFFSET + 2048).is_ok()
    }
}

struct IsrCell(UnsafeCell<(Engine, u32)>);
// Safety: only the ADC1_2 interrupt touches it after init.
unsafe impl Sync for IsrCell {}
/// The drive engine plus the PWM-period phase counter (see [`PWM_DIV`]).
static ISR: IsrCell = IsrCell(UnsafeCell::new((Engine::new(), 0)));

static RESPONSES: Channel<CriticalSectionRawMutex, Message, 8> = Channel::new();

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
    // DMA arguments are tx first: the RX channel is DMA1 CH2 (session 29).
    let uart = Uart::new(p.LPUART1, p.PA3, p.PA2, p.DMA1_CH1, p.DMA1_CH2, Irqs, cfg)
        .expect("LPUART1 config");
    let (tx, rx) = uart.split();

    init_motor_peripherals();

    spawner.spawn(rx_task(rx, store).unwrap());
    spawner.spawn(tx_task(tx).unwrap());
}

/// GPIO + TIM1 + ADC1/ADC2 register-level setup, then the control interrupt.
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
        // CH5 is not routed to a pin; it exists to place a second ADC trigger.
        // OC5REF is high while CNT < CCR5, so its rising edge lands just
        // before the counter valley — inside the high-side on-time.
        TIM1.ccmr3().modify(|w| {
            w.set_ocm(0, vals::Ocm::PWM_MODE1);
            w.set_ocpe(0, true);
        });
        TIM1.ccr5().modify(|w| w.set_ccr(ONTIME_CCR5_DEFAULT));
        TIM1.ccer().modify(|w| {
            for ch in 0..5 {
                w.set_cce(ch, true);
            }
        });
        // TRGO2 follows OC5REF; ADC2 triggers from it while ADC1 keeps CC4 at
        // the counter peak, where the low-side shunts carry phase current.
        TIM1.cr2().modify(|w| w.set_mms2(vals::Mms2::COMPARE_OC5));
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
        // ADC1 stays on CC4 at the counter peak: the low-side shunts only carry
        // phase current while the low side conducts. ADC2's move into the PWM
        // on-time (below) is for the BEMF nodes and must not be copied here —
        // sampling the shunts inside the high-side on-time reads zero current
        // no matter what the bridge is actually doing.
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

        // ADC2: BEMF terminal voltages on PC0..PC3 (IN6..IN9), triggered in
        // the PWM on-time via TIM1_TRGO2 rather than the old TIM1_CC4
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

        // 6.5 cycles, not 47.5: four conversions must fit inside the
        // on-window. 4 x (6.5 + 12.5)/42.5 MHz = 1.79 us. The divider's
        // Thevenin source is ~1.8 kOhm, so the sample capacitor still settles
        // in about eleven time constants.
        ADC2.smpr().modify(|w| {
            w.set_smp(6, SampleTime::CYCLES6_5);
            w.set_smp(7, SampleTime::CYCLES6_5);
            w.set_smp(8, SampleTime::CYCLES6_5);
            w.set_smp(9, SampleTime::CYCLES6_5);
        });
        ADC2.cfgr().modify(|w| w.set_jqdis(true));
        ADC2.jsqr().write(|w| {
            w.set_jl(3); // 4 conversions
            w.set_jextsel(8); // tim1_trgo2 (= OC5REF), inside the on-time
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
/// parameter flash so persistence requests can erase/program it.
#[embassy_executor::task]
async fn rx_task(mut rx: UartRx<'static, Async>, mut store: FlashStore) {
    let mut deframer = Deframer::new();
    let mut buf = [0u8; 128];
    loop {
        let Ok(n) = rx.read_until_idle(&mut buf).await else {
            continue;
        };
        SHARED.host_activity();
        for &b in &buf[..n] {
            if let Some(Ok(msg)) = deframer.push(b) {
                let _ = RESPONSES.try_send(SHARED.handle(&msg, &mut store));
            }
        }
    }
}

/// Responses + decimated telemetry snapshots.
#[embassy_executor::task]
async fn tx_task(mut tx: UartTx<'static, Async>) {
    let mut period = SHARED.telemetry_period_us();
    let mut ticker = Ticker::every(Duration::from_micros(period));
    loop {
        match select(RESPONSES.receive(), ticker.next()).await {
            Either::First(reply) => send(&mut tx, &reply).await,
            Either::Second(()) => {
                let p = SHARED.telemetry_period_us();
                if p != period {
                    period = p;
                    ticker = Ticker::every(Duration::from_micros(period));
                }
                if let Some(frame) = SHARED.telemetry() {
                    send(&mut tx, &frame).await;
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

/// The control loop, clocked by the injected-conversion ADC interrupt (which
/// TIM1 CH4 fires at the counter peak — mid low-side conduction). The
/// interrupt arrives at the PWM rate; every [`PWM_DIV`]-th one runs a tick.
#[no_mangle]
unsafe extern "C" fn ADC1_2() {
    if !ADC1.isr().read().jeos() {
        return;
    }
    ADC1.isr().write(|w| w.set_jeos(true));

    let (engine, pwm_phase) = &mut *ISR.0.get();
    // Drop the conversions that fall between control ticks. Sampling still
    // happens every PWM period — the discarded ones cost only this test — but
    // the loop runs at CTRL_FREQ_HZ, so every gain, slew rate and tick counter
    // downstream keeps the timebase it was tuned for.
    *pwm_phase += 1;
    if *pwm_phase < PWM_DIV {
        return;
    }
    *pwm_phase = 0;
    engine.tick(&SHARED, &mut G474Board);
}
