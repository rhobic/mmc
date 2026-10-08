//! The firmware's drive application, run against a simulated board.
//!
//! This is the same `Engine::tick` and `Shared::handle` the MCUs execute —
//! only the `MotorBoard` underneath is the virtual motor instead of timer and
//! ADC registers. Each scenario runs at two control rates, because the rate is
//! a board choice now and nothing in the drive may assume one.

use mmc_core::math::sin_cos;
use mmc_core::transforms::{clarke, inverse_clarke, inverse_park, Abc};
use mmc_core::tuning::speed_pi_gains;
use mmc_drive::{nvparam, BurstBuffer, DriveConfig, Engine, ParamStore, Shared, ST_OFF, ST_RUN};
use mmc_hal::{BoardSpec, MotorBoard, Sample};
use mmc_proto::{channel, param, test, DeviceKind, DriveMode, Message};
use mmc_sim::motor::{PmsmModel, PmsmParams};

const VBUS: f32 = 24.0;
const AMP_ZERO: f32 = 1.65;
const VOLTS_PER_AMP: f32 = 0.5;
/// Plant sub-steps per control tick.
const SUBSTEPS: usize = 10;

struct SimBoard {
    motor: PmsmModel,
    duties: [f32; 3],
    enables: u8,
    dt_plant: f32,
    /// Hall sensors unplugged: every read is the invalid all-high state.
    halls_dead: bool,
    /// Mechanical angle [rad], unwrapped (the plant's own wraps at ±π).
    theta_m_total: f32,
}

impl SimBoard {
    fn new(params: PmsmParams, ctrl_hz: u32) -> Self {
        Self {
            motor: PmsmModel::new(params),
            duties: [0.0; 3],
            enables: 0,
            dt_plant: 1.0 / (ctrl_hz as f32 * SUBSTEPS as f32),
            halls_dead: false,
            theta_m_total: 0.0,
        }
    }

    /// Advance the plant one control period under the latched duties. A
    /// stage that is off applies no voltage (freewheel diodes ignored). With
    /// one phase floating (six-step) the dq plant has no open terminal, so
    /// the floating phase is held at the driven pair's midpoint — close to
    /// the zero-current voltage it would float to.
    fn advance(&mut self) {
        let mut d = self.duties;
        let v_ab = match self.enables {
            0b111 => clarke(Abc {
                a: d[0] * VBUS,
                b: d[1] * VBUS,
                c: d[2] * VBUS,
            }),
            0b110 | 0b101 | 0b011 => {
                let fl = (!self.enables & 0b111).trailing_zeros() as usize;
                let (x, y) = ((fl + 1) % 3, (fl + 2) % 3);
                d[fl] = 0.5 * (d[x] + d[y]);
                clarke(Abc {
                    a: d[0] * VBUS,
                    b: d[1] * VBUS,
                    c: d[2] * VBUS,
                })
            }
            _ => Default::default(),
        };
        for _ in 0..SUBSTEPS {
            self.motor.step(v_ab, 0.0, self.dt_plant);
            self.theta_m_total += self.motor.omega_m * self.dt_plant;
        }
    }
}

impl MotorBoard for SimBoard {
    fn sample(&mut self) -> Sample {
        let i = self.motor.phase_currents();
        Sample {
            phase_volts: [i.a, i.b, i.c].map(|i| AMP_ZERO - VOLTS_PER_AMP * i),
            vbus: VBUS,
        }
    }
    fn terminal_volts(&mut self) -> [f32; 3] {
        [0.0; 3]
    }
    fn set_duties(&mut self, duties: [f32; 3]) {
        self.duties = duties.map(|d| d.clamp(0.0, 0.95));
    }
    fn set_phase_enables(&mut self, mask: u8) {
        self.enables = mask;
    }
    fn driver_fault(&mut self) -> bool {
        false
    }
    fn hall_state(&mut self) -> Option<u8> {
        if self.halls_dead {
            return Some(0b111);
        }
        // Ideal 120° halls: state 0b001 is centered on θ = 0 and the
        // sequence runs forward, i.e. HallMap { offset: 0, dir: +1 }.
        let sc = sin_cos(self.motor.theta_e());
        let i = inverse_clarke(inverse_park(
            mmc_core::transforms::Dq { d: 1.0, q: 0.0 },
            sc,
        ));
        Some((i.a > 0.0) as u8 | ((i.b > 0.0) as u8) << 1 | ((i.c > 0.0) as u8) << 2)
    }
}

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

fn config(ctrl_hz: u32, motor: &PmsmParams) -> DriveConfig {
    let speed = speed_pi_gains(
        motor.inertia,
        motor.torque_constant(),
        motor.pole_pairs,
        40.0,
    );
    let mut defaults = [0.0f32; param::COUNT];
    defaults[param::R as usize] = motor.rs;
    defaults[param::L as usize] = motor.ld;
    defaults[param::FLUX as usize] = motor.flux;
    defaults[param::CUR_BW as usize] = 2000.0;
    defaults[param::SPEED_KP as usize] = speed.kp;
    defaults[param::SPEED_KI as usize] = speed.ki;
    defaults[param::POLE_PAIRS as usize] = motor.pole_pairs as f32;
    defaults[param::SL_HANDOFF as usize] = 150.0;
    defaults[param::OMEGA_ACCEL as usize] = 400.0;
    defaults[param::IQ_LIMIT as usize] = 1.5;
    defaults[param::ONTIME_CCR5 as usize] = 0.0;
    defaults[param::SS_KP as usize] = 2e-4;
    defaults[param::SS_KI as usize] = 5e-4;
    defaults[param::I_THRESH as usize] = 0.5;
    defaults[param::HALL_OFFSET as usize] = 0.0;
    defaults[param::HALL_DIR as usize] = 1.0;
    // small_bldc: p·kt/J = 7·0.084/1e-5 ≈ 5.9e4 rad/s² per A → ~35 rad/s.
    defaults[param::POS_KP as usize] = 0.02;
    defaults[param::POS_KI as usize] = 0.1;
    defaults[param::POS_KD as usize] = 6e-4;
    defaults[param::POS_VMAX as usize] = 200.0;
    defaults[param::INERTIA as usize] = motor.inertia;
    defaults[param::SS_CONDUCTION as usize] = 120.0;
    for k in 0..6 {
        defaults[(param::HALL_W0 + k) as usize] = std::f32::consts::FRAC_PI_3;
    }
    DriveConfig {
        spec: BoardSpec {
            ctrl_hz,
            cur_volts_per_amp: VOLTS_PER_AMP,
            i_trip: 2.5,
            vbus_max: 30.0,
            vbus_min_run: 5.0,
            max_duty: 0.95,
            r_path: 0.0,
            terminal_offset_max: 0.0,
            has_halls: true,
            // The sim board applies a duty for the whole next period.
            pwm_latency: 0.0,
            terminal_full_scale: f32::INFINITY,
        },
        kind: DeviceKind::Sim,
        fw_version: 1,
        name: "drive-test",
        defaults,
    }
}

/// A firmware image: the shared state, the ISR's engine, the board, and a
/// clock. Shared state is boxed rather than static so tests run in parallel.
struct Rig {
    sh: Box<Shared<8200>>,
    eng: Engine,
    board: SimBoard,
    store: RamStore,
    hz: u32,
}

impl Rig {
    fn new(hz: u32) -> Self {
        Self::with_motor(hz, PmsmParams::small_bldc())
    }

    fn with_motor(hz: u32, motor: PmsmParams) -> Self {
        Self::build(hz, motor, true)
    }

    /// A board that does not report halls (the drive never reads them).
    fn without_halls(hz: u32) -> Self {
        Self::build(hz, PmsmParams::small_bldc(), false)
    }

    fn build(hz: u32, motor: PmsmParams, halls: bool) -> Self {
        let mut cfg = config(hz, &motor);
        cfg.spec.has_halls = halls;
        let mut rig = Self {
            sh: Box::new(Shared::new(cfg, Box::leak(Box::default()))),
            eng: Engine::new(),
            board: SimBoard::new(motor, hz),
            store: RamStore([0xFF; nvparam::BYTES]),
            hz,
        };
        rig.send(&Message::Stream { enable: true });
        rig.run(1.0); // boot calibration
        assert_eq!(rig.sh.state(), ST_OFF, "calibrated");
        rig
    }

    fn send(&mut self, msg: &Message) -> Message {
        self.sh.host_activity();
        self.sh.handle(msg, &mut self.store)
    }

    /// Run `secs` of control ticks, with the host pinging (deadman) unless
    /// `silent`.
    fn run_with(&mut self, secs: f32, silent: bool) {
        let n = (secs * self.hz as f32) as u32;
        for k in 0..n {
            if !silent && k % (self.hz / 10) == 0 {
                self.sh.host_activity();
            }
            self.eng.tick(&self.sh, &mut self.board);
            self.board.advance();
        }
    }

    fn run(&mut self, secs: f32) {
        self.run_with(secs, false);
    }

    /// Run `secs` and return the slowest and fastest true rotor speed
    /// [rad/s el] seen, signed.
    fn run_speed_range(&mut self, secs: f32) -> (f32, f32) {
        let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
        let n = (secs * self.hz as f32) as u32;
        for k in 0..n {
            if k % (self.hz / 10) == 0 {
                self.sh.host_activity();
            }
            self.eng.tick(&self.sh, &mut self.board);
            self.board.advance();
            let w = self.board.motor.omega_e();
            lo = lo.min(w);
            hi = hi.max(w);
        }
        (lo, hi)
    }

    fn telem(&self, ch: u8) -> f32 {
        match self.sh.telemetry() {
            Some(Message::Telemetry(f)) => f.values()[ch as usize],
            other => panic!("no telemetry: {other:?}"),
        }
    }
}

const RATES: [u32; 2] = [10_000, 20_000];

#[test]
fn reports_its_board_traits() {
    for hz in RATES {
        let mut rig = Rig::new(hz);
        let Message::Info(info) = rig.send(&Message::GetInfo) else {
            panic!()
        };
        let b = info.board.expect("board traits");
        assert_eq!(b.ctrl_hz, hz);
        assert_eq!(b.burst_cap, 8200);
        // telemetry time base follows the rate
        rig.send(&Message::SetTelemetry {
            divider: 20,
            mask: channel::ALL,
        });
        assert_eq!(rig.sh.telemetry_period_us(), 20 * 1_000_000 / hz as u64);
    }
}

#[test]
fn if_drive_tracks_current_and_spins() {
    for hz in RATES {
        let mut rig = Rig::new(hz);
        let ack = rig.send(&Message::SetDrive(DriveMode::IfCurrent {
            amps: 0.5,
            omega_e: 100.0,
        }));
        assert!(matches!(ack, Message::Ack { .. }), "{hz}: {ack:?}");
        rig.run(1.5);
        assert_eq!(rig.sh.state(), ST_RUN);
        let i = rig.telem(channel::I_D).hypot(rig.telem(channel::I_Q));
        assert!((i - 0.5).abs() < 0.05, "{hz} Hz: |i| = {i}");
        let w = rig.board.motor.omega_e();
        assert!((w - 100.0).abs() < 10.0, "{hz} Hz: rotor at {w} rad/s el");
        // The sim's ideal halls run in SEQUENCE order for positive rotation.
        let wh = rig.telem(channel::OMEGA_HALL);
        assert!((wh - w).abs() < 10.0, "{hz} Hz: halls say {wh}, rotor {w}");
    }
}

#[test]
fn sensorless_closes_the_loop_on_the_observer() {
    for hz in RATES {
        let mut rig = Rig::new(hz);
        rig.send(&Message::SetDrive(DriveMode::Sensorless {
            amps: 0.5,
            omega_e: 400.0,
        }));
        rig.run(3.0);
        assert_eq!(rig.sh.state(), ST_RUN, "{hz} Hz: no stall/fault");
        let w = rig.board.motor.omega_e();
        assert!((w - 400.0).abs() < 20.0, "{hz} Hz: rotor at {w} rad/s el");
        let err = rig.telem(channel::THETA_ERR).abs();
        assert!(err < 0.1, "{hz} Hz: observer innovation {err} rad");
    }
}

#[test]
fn deadman_stops_a_silent_host() {
    for hz in RATES {
        let mut rig = Rig::new(hz);
        rig.send(&Message::SetDrive(DriveMode::IfCurrent {
            amps: 0.3,
            omega_e: 50.0,
        }));
        rig.run_with(1.5, true);
        assert_eq!(rig.sh.state(), ST_RUN, "{hz} Hz: still inside 2 s");
        rig.run_with(1.0, true);
        assert_eq!(rig.sh.state(), ST_OFF, "{hz} Hz: deadman fired");
        assert_eq!(rig.board.enables, 0, "{hz} Hz: stage off");
    }
}

#[test]
fn rl_probe_records_a_settled_resistance() {
    // The probe's plateau half-period is a fixed 32 ticks, so it assumes
    // τ = L/R well under that: a short-τ motor (0.2 ms) here.
    let motor = PmsmParams {
        ld: 0.1e-3,
        lq: 0.1e-3,
        ..PmsmParams::small_bldc()
    };
    for hz in RATES {
        let mut rig = Rig::with_motor(hz, motor);
        rig.board.motor.locked = true;
        let ack = rig.send(&Message::RunTest {
            kind: test::RL_STEP,
            a: 0.25,
            b: 0.5,
        });
        assert!(matches!(ack, Message::Ack { .. }));
        rig.run(1.5);
        let mut data = Vec::new();
        loop {
            match rig.send(&Message::ReadBurst {
                offset: data.len() as u16,
            }) {
                Message::BurstData(c) => {
                    data.extend_from_slice(c.values());
                    if data.len() >= c.total as usize {
                        break;
                    }
                }
                other => panic!("{hz} Hz: {other:?}"),
            }
        }
        assert!(
            data.len() >= 8000,
            "{hz} Hz: full recording, got {}",
            data.len()
        );
        // Settled plateaus: the last tick before each level change (the
        // recording itself ends mid-half-period, wherever the buffer filled).
        let pairs: Vec<(f32, f32)> = data.chunks(2).map(|c| (c[0], c[1])).collect();
        let settled = |level: f32| {
            pairs
                .windows(2)
                .rev()
                .find(|w| w[0].1 == level && w[1].1 != level)
                .unwrap()[0]
                .0
        };
        let (hi, lo) = (settled(0.5), settled(0.25));
        let r = (0.5 - 0.25) / (hi - lo);
        assert!((r - 0.5).abs() < 0.05, "{hz} Hz: R = {r}");
    }
}

#[test]
fn params_persist_and_restore() {
    let mut rig = Rig::new(10_000);
    rig.send(&Message::SetParam {
        id: param::FLUX,
        value: 0.0123,
    });
    assert!(matches!(
        rig.send(&Message::SaveParams),
        Message::Ack { .. }
    ));
    let mut fresh = Rig::new(10_000);
    fresh.store = RamStore(rig.store.0);
    fresh.sh.restore(&mut fresh.store);
    assert_eq!(fresh.sh.param(param::FLUX), 0.0123);
}

#[test]
fn a_small_burst_buffer_refuses_the_saliency_sweep() {
    let motor = PmsmParams::small_bldc();
    let burst: &'static BurstBuffer<2048> = Box::leak(Box::default());
    let sh = Box::new(Shared::new(config(10_000, &motor), burst));
    let mut store = RamStore([0xFF; nvparam::BYTES]);
    let reply = sh.handle(
        &Message::RunTest {
            kind: test::L_THETA,
            a: 0.3,
            b: 0.9,
        },
        &mut store,
    );
    assert!(matches!(reply, Message::Nak { err: 1, .. }), "{reply:?}");
}

#[test]
fn hall_foc_closes_the_speed_loop_from_rest_both_ways() {
    for hz in RATES {
        for target in [300.0f32, -300.0] {
            let mut rig = Rig::new(hz);
            let ack = rig.send(&Message::SetDrive(DriveMode::HallFoc {
                amps: 1.0,
                omega_e: target,
            }));
            assert!(matches!(ack, Message::Ack { .. }), "{ack:?}");
            rig.run(2.0);
            assert_eq!(rig.sh.state(), ST_RUN, "{hz} Hz {target}: running");
            let w = rig.board.motor.omega_e();
            assert!(
                (w - target).abs() < 15.0,
                "{hz} Hz: rotor at {w}, target {target}"
            );
            // The shadow observer agrees with the halls once spinning.
            let err = rig.telem(channel::THETA_ERR).abs();
            assert!(err < 0.35, "{hz} Hz {target}: observer vs halls {err} rad");
        }
    }
}

#[test]
fn hall_six_step_spins_both_ways() {
    for hz in RATES {
        for target in [250.0f32, -250.0] {
            let mut rig = Rig::new(hz);
            rig.send(&Message::SetDrive(DriveMode::SixStepHall {
                duty: 0.6,
                omega_e: target,
            }));
            // The duty loop runs on the conservative default ss gains.
            rig.run(4.0);
            assert_eq!(rig.sh.state(), ST_RUN, "{hz} Hz {target}: running");
            let w = rig.board.motor.omega_e();
            assert!(
                (w - target).abs() < 0.15 * target.abs(),
                "{hz} Hz: rotor at {w}, target {target}"
            );
        }
    }
}

#[test]
fn hall_six_step_at_180_degrees_spins_both_ways() {
    for hz in RATES {
        for target in [250.0f32, -250.0] {
            let mut rig = Rig::new(hz);
            rig.send(&Message::SetParam {
                id: param::SS_CONDUCTION,
                value: 180.0,
            });
            rig.send(&Message::SetDrive(DriveMode::SixStepHall {
                duty: 0.6,
                omega_e: target,
            }));
            rig.run(4.0);
            assert_eq!(rig.sh.state(), ST_RUN, "{hz} Hz {target}: running");
            assert_eq!(rig.board.enables, 0b111, "no phase floats at 180°");
            let w = rig.board.motor.omega_e();
            assert!(
                (w - target).abs() < 0.15 * target.abs(),
                "{hz} Hz: rotor at {w}, target {target}"
            );
        }
    }
}

#[test]
fn a_dead_hall_sensor_trips_the_hall_drive() {
    let mut rig = Rig::new(10_000);
    rig.send(&Message::SetDrive(DriveMode::HallFoc {
        amps: 1.0,
        omega_e: 200.0,
    }));
    rig.run(0.5);
    rig.board.halls_dead = true;
    rig.run(0.05);
    assert_eq!(rig.sh.state(), mmc_drive::ST_FAULT_HALL);
    assert_eq!(rig.board.enables, 0, "stage off");
}

#[test]
fn a_board_without_halls_refuses_the_hall_modes() {
    let motor = PmsmParams::small_bldc();
    let mut cfg = config(10_000, &motor);
    cfg.spec.has_halls = false;
    let burst: &'static BurstBuffer<2048> = Box::leak(Box::default());
    let sh = Box::new(Shared::new(cfg, burst));
    let mut store = RamStore([0xFF; nvparam::BYTES]);
    let reply = sh.handle(
        &Message::SetDrive(DriveMode::HallFoc {
            amps: 1.0,
            omega_e: 100.0,
        }),
        &mut store,
    );
    assert!(matches!(reply, Message::Nak { err: 1, .. }), "{reply:?}");
}

#[test]
fn switching_modes_while_running_rebuilds_the_drive() {
    // Straight from one running mode to another, no Off in between: each
    // mode's control blocks must be built for it (a stale set panicked the
    // ISR before origin 40d502c / session 30b).
    let mut rig = Rig::new(10_000);
    let modes = [
        DriveMode::OpenLoopVoltage {
            volts: 1.0,
            omega_e: 50.0,
        },
        DriveMode::Sensorless {
            amps: 0.5,
            omega_e: 300.0,
        },
        DriveMode::HallFoc {
            amps: 1.0,
            omega_e: 200.0,
        },
        DriveMode::SixStepHall {
            duty: 0.5,
            omega_e: 150.0,
        },
        DriveMode::IfCurrent {
            amps: 0.5,
            omega_e: 100.0,
        },
    ];
    for m in modes {
        assert!(matches!(
            rig.send(&Message::SetDrive(m)),
            Message::Ack { .. }
        ));
        rig.run(0.5);
        assert_eq!(rig.sh.state(), ST_RUN, "after switching to {m:?}");
    }
}

#[test]
fn hall_position_moves_and_holds_both_ways() {
    for hz in RATES {
        for target_deg in [90.0f32, -360.0] {
            let mut rig = Rig::new(hz);
            let start = rig.board.theta_m_total;
            let ack = rig.send(&Message::SetDrive(DriveMode::HallPosition {
                amps: 1.0,
                theta_m: target_deg.to_radians(),
            }));
            assert!(matches!(ack, Message::Ack { .. }), "{ack:?}");
            rig.run(3.0);
            assert_eq!(rig.sh.state(), ST_RUN);
            let moved = (rig.board.theta_m_total - start).to_degrees();
            // Hall resolution on 7 pole pairs is 360/42 = 8.6° mechanical.
            assert!(
                (moved - target_deg).abs() < 8.6,
                "{hz} Hz: moved {moved}° for {target_deg}°"
            );
            let reported = rig.telem(channel::POS_M).to_degrees();
            assert!(
                (reported - moved).abs() < 8.6,
                "reported {reported}° vs {moved}°"
            );
            // Holding: nothing inside a hall sector is measurable, and the
            // commutation angle at rest is the sector centre (≤30° el off),
            // so this frictionless rotor dithers about one sector (8.6°
            // mechanical on 7 pole pairs). The real motor's friction holds it
            // tighter (session 30c: holds within ±4.3°).
            let mut worst = 0.0f32;
            for _ in 0..20 {
                rig.run(0.05);
                worst =
                    worst.max(((rig.board.theta_m_total - start).to_degrees() - target_deg).abs());
            }
            assert!(
                worst < 12.0,
                "{hz} Hz {target_deg}°: hold wandered {worst}°"
            );
        }
    }
}

#[test]
fn hall_position_tracks_a_slow_move() {
    // A far target at a slow cruise speed is the low-speed mode: here 5 rad/s
    // el, which the hall speed loop cannot hold.
    let mut rig = Rig::new(10_000);
    rig.send(&Message::SetParam {
        id: param::POS_VMAX,
        value: 5.0,
    });
    rig.send(&Message::SetDrive(DriveMode::HallPosition {
        amps: 1.0,
        theta_m: 100.0,
    }));
    rig.run(2.0);
    let a = rig.board.theta_m_total * 7.0;
    rig.run(4.0);
    let b = rig.board.theta_m_total * 7.0;
    let speed = (b - a) / 4.0;
    assert!((speed - 5.0).abs() < 0.5, "average {speed} rad/s el for 5");
}

/// `f32::clamp` passes NaN through, so a non-finite command must be refused
/// at the door rather than reach a control loop (review follow-up, origin
/// `5bc16cd`).
#[test]
fn non_finite_commands_are_refused() {
    let mut rig = Rig::new(RATES[0]);
    rig.run(0.5); // past the zero-current calibration
    for msg in [
        Message::SetDrive(DriveMode::IfCurrent {
            amps: f32::NAN,
            omega_e: 100.0,
        }),
        Message::SetDrive(DriveMode::Sensorless {
            amps: 0.5,
            omega_e: f32::INFINITY,
        }),
        Message::SetIqRef { iq: f32::NAN },
        Message::RunTest {
            kind: test::RL_STEP,
            a: f32::NAN,
            b: 1.0,
        },
    ] {
        let reply = rig.send(&msg);
        assert!(
            matches!(reply, Message::Nak { err: 1, .. }),
            "{msg:?}: {reply:?}"
        );
    }
    assert_eq!(rig.telem(channel::STATE) as u8, ST_OFF, "nothing started");
}

/// Flying start: switching modes on a turning rotor starts the new mode at
/// the rotor's angle and speed (from the halls, or from the locked flux
/// observer of the outgoing mode) instead of from rest. Before, I-f →
/// sensorless restarted the I-f ramp from 0 rad/s under a rotor at speed.
#[test]
fn a_live_switch_catches_the_turning_rotor() {
    let cases: [(&str, DriveMode, DriveMode); 3] = [
        (
            "I-f -> sensorless",
            DriveMode::IfCurrent {
                amps: 0.5,
                omega_e: 400.0,
            },
            DriveMode::Sensorless {
                amps: 0.5,
                omega_e: 400.0,
            },
        ),
        (
            "open loop -> I-f",
            // Open loop pulls out on this motor much above 1 V / 50 rad/s el.
            DriveMode::OpenLoopVoltage {
                volts: 1.0,
                omega_e: 50.0,
            },
            DriveMode::IfCurrent {
                amps: 0.5,
                omega_e: 50.0,
            },
        ),
        (
            "sensorless -> I-f",
            DriveMode::Sensorless {
                amps: 0.5,
                omega_e: 400.0,
            },
            DriveMode::IfCurrent {
                amps: 0.5,
                omega_e: 400.0,
            },
        ),
    ];
    for halls in [false, true] {
        for (name, from, to) in cases {
            // Without halls the catch comes from the flux observer, which is
            // not trusted below half the handoff speed; open loop only holds
            // this motor at 50 rad/s el.
            if !halls && name.starts_with("open loop") {
                continue;
            }
            let mut rig = if halls {
                Rig::new(10_000)
            } else {
                Rig::without_halls(10_000)
            };
            assert!(matches!(
                rig.send(&Message::SetDrive(from)),
                Message::Ack { .. }
            ));
            rig.run(2.0);
            let w0 = rig.board.motor.omega_e();
            assert!(
                w0 > 40.0,
                "{name} (halls {halls}): only {w0} rad/s before the switch"
            );
            assert!(matches!(
                rig.send(&Message::SetDrive(to)),
                Message::Ack { .. }
            ));
            let (lo, hi) = rig.run_speed_range(0.5);
            println!("{name} (halls {halls}): {w0:.0} rad/s el, then {lo:.0}..{hi:.0}");
            assert_eq!(rig.sh.state(), ST_RUN, "{name} (halls {halls}): faulted");
            assert!(
                lo > 0.6 * w0 && hi < 1.6 * w0,
                "{name} (halls {halls}): speed {lo:.0}..{hi:.0} after the switch from {w0:.0}"
            );
        }
    }
}

/// Hall FOC re-entered on a turning rotor ramps from the rotor's speed
/// instead of braking it to rest first.
#[test]
fn hall_foc_resumes_at_speed() {
    let mut rig = Rig::new(10_000);
    let hall = DriveMode::HallFoc {
        amps: 1.0,
        omega_e: 400.0,
    };
    rig.send(&Message::SetDrive(hall));
    rig.run(2.0);
    rig.send(&Message::SetDrive(DriveMode::IfCurrent {
        amps: 0.5,
        omega_e: 400.0,
    }));
    rig.run(0.5);
    let w0 = rig.board.motor.omega_e();
    rig.send(&Message::SetDrive(hall));
    let (lo, _) = rig.run_speed_range(0.5);
    println!("hall FOC re-entry at {w0:.0} rad/s el: slowest {lo:.0}");
    assert_eq!(rig.sh.state(), ST_RUN);
    assert!(lo > 0.7 * w0, "dipped to {lo:.0} from {w0:.0}");
}
