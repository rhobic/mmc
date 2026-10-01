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
}

impl SimBoard {
    fn new(params: PmsmParams, ctrl_hz: u32) -> Self {
        Self {
            motor: PmsmModel::new(params),
            duties: [0.0; 3],
            enables: 0,
            dt_plant: 1.0 / (ctrl_hz as f32 * SUBSTEPS as f32),
        }
    }

    /// Advance the plant one control period under the latched duties. A
    /// stage that is off applies no voltage (freewheel diodes ignored).
    fn advance(&mut self) {
        let v_ab = if self.enables == 0b111 {
            clarke(Abc {
                a: self.duties[0] * VBUS,
                b: self.duties[1] * VBUS,
                c: self.duties[2] * VBUS,
            })
        } else {
            Default::default()
        };
        for _ in 0..SUBSTEPS {
            self.motor.step(v_ab, 0.0, self.dt_plant);
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
        // Ideal 120° halls, for completeness of the trait surface.
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
        let mut rig = Self {
            sh: Box::new(Shared::new(config(hz, &motor), Box::leak(Box::default()))),
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
