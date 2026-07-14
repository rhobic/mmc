//! The simulator behind the wire protocol: a TCP server that runs the virtual
//! motor + FOC loop in real time and speaks mmc-proto, exactly like firmware
//! will over UART. Host tooling cannot tell the difference — by design.

use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc, FocOutput};
use mmc_core::transforms::{Abc, Dq};
use mmc_core::tuning::current_pi_gains;
use mmc_hal::{BusVoltageSense, CurrentSense, PwmOutput};
use mmc_proto::{channel, encode, param, Deframer, DeviceInfo, DeviceKind, Message, MAX_FRAME};
use mmc_sim::{PmsmParams, TruthAngle, VirtualMotor};

pub struct ServeCfg {
    pub ctrl_freq: f32,
    pub bandwidth: f32,
    pub vbus: f32,
    /// Exit after the first client disconnects (used by `suite` and tests).
    pub once: bool,
}

impl Default for ServeCfg {
    fn default() -> Self {
        Self {
            ctrl_freq: 10_000.0,
            bandwidth: 2000.0,
            vbus: 24.0,
            once: false,
        }
    }
}

pub fn serve(listener: TcpListener, cfg: &ServeCfg) -> std::io::Result<()> {
    println!("sim server listening on {}", listener.local_addr()?);
    loop {
        let (stream, peer) = listener.accept()?;
        println!("client {peer} connected");
        match session(stream, cfg) {
            Ok(()) => println!("client {peer} disconnected"),
            Err(e) if e.kind() == ErrorKind::ConnectionReset => {
                println!("client {peer} disconnected")
            }
            Err(e) => println!("client {peer}: {e}"),
        }
        if cfg.once {
            return Ok(());
        }
    }
}

/// One client session: fresh motor, fresh controller, zero reference.
fn session(mut stream: TcpStream, cfg: &ServeCfg) -> std::io::Result<()> {
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(Duration::from_millis(1)))?;

    let params = PmsmParams::small_bldc();
    let ctrl_dt = 1.0 / cfg.ctrl_freq;
    let mut rig = VirtualMotor::new(params, cfg.vbus);
    rig.enable();
    let mut foc = Foc::with_feedforward(
        current_pi_gains(params.rs, params.lq, cfg.bandwidth),
        Decoupling {
            ld: params.ld,
            lq: params.lq,
            flux: params.flux,
        },
    );
    let mut angle = TruthAngle::default();
    let mut i_ref = Dq::default();

    let mut deframer = Deframer::new();
    let mut mask = channel::ALL;
    let mut divider: u32 = 10;
    let mut streaming = false;
    // Runtime parameter table (profiler read/apply target), seeded from this
    // motor and mirroring the firmware's ids. The sim stores and range-validates
    // them but does not yet re-tune its control loop from a write.
    let mut sim_params = [params.rs, params.lq, params.flux, cfg.bandwidth, 2.0e-4, 2.0e-3];

    let started = Instant::now();
    let mut steps_done: u64 = 0;
    let mut rx = [0u8; 256];

    loop {
        // Ingest whatever arrived (the 1 ms read timeout is also our pacing).
        match stream.read(&mut rx) {
            Ok(0) => return Ok(()),
            Ok(n) => {
                for &b in &rx[..n] {
                    if let Some(Ok(msg)) = deframer.push(b) {
                        handle(
                            &msg,
                            &mut stream,
                            &mut i_ref,
                            &mut mask,
                            &mut divider,
                            &mut streaming,
                            &mut sim_params,
                        )?;
                    }
                    // Frame errors over TCP mean a client bug; ignore & resync.
                }
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
        }

        // Catch the simulation up to wall time (capped: no death spiral).
        let target = (started.elapsed().as_secs_f64() * cfg.ctrl_freq as f64) as u64;
        let burst = (target - steps_done).min(cfg.ctrl_freq as u64 / 10);
        for _ in 0..burst {
            angle.sync(&rig.motor);
            let [ia, ib, ic] = rig.phase_currents();
            let vbus = rig.vbus();
            let out = foc.step(
                Abc {
                    a: ia,
                    b: ib,
                    c: ic,
                },
                angle.electrical_angle(),
                angle.electrical_velocity(),
                i_ref,
                vbus,
                ctrl_dt,
            );
            rig.set_duties(out.duties);
            rig.advance(ctrl_dt);
            steps_done += 1;

            if streaming && steps_done.is_multiple_of(divider as u64) {
                let frame = telemetry(&rig, &out, i_ref, mask);
                send(&mut stream, &Message::Telemetry(frame))?;
            }
        }
    }
}

fn handle(
    msg: &Message,
    stream: &mut TcpStream,
    i_ref: &mut Dq,
    mask: &mut u32,
    divider: &mut u32,
    streaming: &mut bool,
    params: &mut [f32],
) -> std::io::Result<()> {
    match *msg {
        Message::Ping { nonce } => send(stream, &Message::Pong { nonce }),
        Message::GetInfo => send(
            stream,
            &Message::Info(DeviceInfo::new(DeviceKind::Sim, 1, "mmc-sim")),
        ),
        Message::SetTelemetry {
            divider: d,
            mask: m,
        } => {
            *divider = d.max(1) as u32;
            *mask = m & channel::ALL;
            send(
                stream,
                &Message::Ack {
                    of: msg.wire_type(),
                },
            )
        }
        Message::Stream { enable } => {
            *streaming = enable;
            send(
                stream,
                &Message::Ack {
                    of: msg.wire_type(),
                },
            )
        }
        Message::SetIqRef { iq } => {
            i_ref.q = iq;
            send(
                stream,
                &Message::Ack {
                    of: msg.wire_type(),
                },
            )
        }
        Message::GetParam { id } if (id as usize) < params.len() => send(
            stream,
            &Message::ParamValue {
                id,
                value: params[id as usize],
            },
        ),
        Message::SetParam { id, value } => match sim_param_range(id) {
            Some((lo, hi)) if (lo..=hi).contains(&value) => {
                params[id as usize] = value;
                send(
                    stream,
                    &Message::Ack {
                        of: msg.wire_type(),
                    },
                )
            }
            _ => send(
                stream,
                &Message::Nak {
                    of: msg.wire_type(),
                    err: 1,
                },
            ),
        },
        // Device-to-host messages arriving at the device: refuse politely.
        _ => send(
            stream,
            &Message::Nak {
                of: msg.wire_type(),
                err: 1,
            },
        ),
    }
}

/// Same acceptance windows as the firmware's `param_range` — the sim rejects
/// what the device would, so `apply`/panel writes fail identically here.
fn sim_param_range(id: u8) -> Option<(f32, f32)> {
    Some(match id {
        param::R => (0.05, 20.0),
        param::L => (5e-6, 0.05),
        param::FLUX => (1e-5, 0.5),
        param::CUR_BW => (100.0, 4000.0),
        param::SPEED_KP => (0.0, 0.1),
        param::SPEED_KI => (0.0, 10.0),
        _ => return None,
    })
}

fn telemetry(
    rig: &VirtualMotor,
    out: &FocOutput,
    i_ref: Dq,
    mask: u32,
) -> mmc_proto::TelemetryFrame {
    let i_abc = rig.motor.phase_currents();
    let mut values = [0f32; channel::COUNT];
    let mut n = 0;
    for id in 0..channel::COUNT as u8 {
        if mask & (1 << id) == 0 {
            continue;
        }
        values[n] = match id {
            channel::IQ_REF => i_ref.q,
            channel::I_D => out.i_dq.d,
            channel::I_Q => out.i_dq.q,
            channel::V_D => out.v_dq.d,
            channel::V_Q => out.v_dq.q,
            channel::DUTY_A => out.duties[0],
            channel::DUTY_B => out.duties[1],
            channel::DUTY_C => out.duties[2],
            channel::OMEGA_M => rig.motor.omega_m,
            channel::THETA_E => rig.motor.theta_e(),
            channel::VBUS => rig.v_bus,
            channel::I_A => i_abc.a,
            channel::I_B => i_abc.b,
            channel::I_C => i_abc.c,
            channel::STATE => 1.0, // the sim's stage is always "running"
            _ => 0.0,
        };
        n += 1;
    }
    let t_us = (rig.time() * 1e6) as u64 as u32;
    mmc_proto::TelemetryFrame::new(t_us, mask, &values[..n]).expect("mask/value count agree")
}

fn send(stream: &mut TcpStream, msg: &Message) -> std::io::Result<()> {
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(msg, &mut buf).expect("MAX_FRAME-sized buffer");
    stream.write_all(&buf[..n])
}
