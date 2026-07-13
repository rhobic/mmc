//! MS6 profiler orchestration. `profile` runs the firmware test sequences
//! and captures whose traces `tools/profile.py` fits into a motor profile
//! (R, L, flux, friction, inertia + suggested gains); `apply` writes a
//! fitted profile back to the device over the protocol and verifies it.
//!
//! The division of labor is the plan's: firmware executes dumb sequences and
//! records raw samples; all fitting happens on the host.

use std::io::Write as _;
use std::path::Path;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mmc_proto::{param, test, DriveMode, Message};

use crate::capture::{self, CaptureCfg};
use crate::link::Link;

const CTRL_FREQ: f32 = 20_000.0;

/// One I-f operating point of the flux sweep.
const SWEEP: [(f32, f32); 5] = [
    // (amps, omega_e rad/s)
    (0.3, 150.0),
    (0.3, 300.0),
    (0.3, 450.0),
    (0.45, 150.0),
    (0.45, 450.0),
];

pub fn run(link: &mut Link, dir: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let t = Duration::from_secs(2);

    // --- 1. Rotating I-f flux sweep (host-side captures, hang-angle fit).
    //
    // ORDER MATTERS: the R/L probe parks the rotor aligned to θ = 0, and an
    // I-f start then puts its current vector exactly 90° away — the rotor is
    // released on the separatrix of the torque well (undamped pendulum at
    // marginal capture energy) and the ramp ejects it: every probe-first
    // sweep stalled on the bench, every standalone sweep caught. So the
    // spinning tests run first, the rotor-aligning probe last.
    for (i, &(amps, omega)) in SWEEP.iter().enumerate() {
        let name = format!("sweep_i{:03}_w{}", (amps * 100.0) as u32, omega as u32);
        let out = dir.join(format!("{name}.csv"));
        println!("profile: I-f sweep {amps} A @ {omega} rad/s el");
        capture::run(
            link,
            &CaptureCfg {
                divider: 20,
                mask: mmc_proto::channel::ALL,
                duration: 3.5 + omega / 500.0, // ramp time grows with speed
                iq: None,
                drive: Some(DriveMode::IfCurrent {
                    amps,
                    omega_e: omega,
                }),
                drive_step: None,
                title: &format!("Flux sweep: I-f {amps} A @ {omega} rad/s"),
                description: "Profiler flux-sweep input (hang-angle-aware fit).",
                order: 1 + i as u32,
                command: "mmc-host profile".into(),
            },
            &out,
        )?;
    }

    // --- 2. Sensorless accel run: J from i_q during the reference slew,
    //        friction from the steady i_q at two speeds.
    println!("profile: sensorless accel 300 -> 900 rad/s el");
    capture::run(
        link,
        &CaptureCfg {
            divider: 20,
            mask: mmc_proto::channel::ALL,
            duration: 8.0,
            iq: None,
            drive: Some(DriveMode::Sensorless {
                amps: 0.5,
                omega_e: 300.0,
            }),
            drive_step: Some(DriveMode::Sensorless {
                amps: 0.5,
                omega_e: 900.0,
            }),
            title: "Accel run: sensorless 300 -> 900 rad/s el",
            description: "Profiler inertia/friction input: i_q during the \
                          500 rad/s^2 reference slew vs the steady levels.",
            order: 10,
            command: "mmc-host profile".into(),
        },
        dir.join("accel.csv").as_path(),
    )?;

    // --- 3. Locked-rotor R/L step probe (on-device 20 kHz burst). Last!
    println!("profile: R/L probe (0.5 V align -> 1.0 V step, locked rotor)");
    let req = Message::RunTest {
        kind: test::RL_STEP,
        a: 0.5,
        b: 1.0,
    };
    match link.request(
        &req,
        |m| matches!(m, Message::Ack { of: 0x07 } | Message::Nak { of: 0x07, .. }),
        t,
    )? {
        Message::Ack { .. } => {}
        other => {
            return Err(std::io::Error::other(format!(
                "device refused RL probe: {other:?}"
            )))
        }
    }

    // The probe takes ~0.5 s; keep pinging (deadman) and poll for the data.
    let samples = loop {
        std::thread::sleep(Duration::from_millis(150));
        match link.request(
            &Message::ReadBurst { offset: 0 },
            |m| matches!(m, Message::BurstData(_) | Message::Nak { of: 0x08, .. }),
            t,
        )? {
            Message::BurstData(first) => break read_burst(link, &first)?,
            _ => continue, // still recording
        }
    };
    let pairs = samples.len() / 2;
    let out = dir.join("rl_step.csv");
    let mut w = std::io::BufWriter::new(std::fs::File::create(&out)?);
    writeln!(w, "t,i_d,v_d")?;
    for (k, p) in samples.chunks_exact(2).enumerate() {
        writeln!(w, "{},{},{}", k as f32 / CTRL_FREQ, p[0], p[1])?;
    }
    w.flush()?;
    write_meta(
        dir,
        "rl_step",
        "R/L step probe",
        "Locked-rotor d-axis voltage step (0.5 V align -> 1.0 V, unslewed), \
         (i_d, v_d) recorded on-device at 20 kHz and read back over the \
         protocol. tools/profile.py fits R from the level change and L from \
         the exponential.",
        0,
    )?;
    println!("profile: {pairs} pairs -> {}", out.display());

    println!();
    println!(
        "profile: done. Fit it:   python tools/profile.py {}",
        dir.display()
    );
    println!("         then apply:     mmc-host apply --serial auto --baud 1000000 --profile {}/profile.json", dir.display());
    Ok(())
}

/// Read the remaining burst chunks after the first.
fn read_burst(link: &mut Link, first: &mmc_proto::BurstChunk) -> std::io::Result<Vec<f32>> {
    let total = first.total as usize;
    let mut data = Vec::with_capacity(total);
    data.extend_from_slice(first.values());
    while data.len() < total {
        let off = data.len() as u16;
        match link.request(
            &Message::ReadBurst { offset: off },
            |m| matches!(m, Message::BurstData(c) if c.offset == off),
            Duration::from_secs(2),
        )? {
            Message::BurstData(c) => {
                if c.values().is_empty() {
                    break; // device says shorter than advertised
                }
                data.extend_from_slice(c.values());
            }
            _ => unreachable!(),
        }
    }
    Ok(data)
}

/// Apply a fitted profile (tools/profile.py JSON) and verify by read-back.
pub fn apply(link: &mut Link, profile: &Path) -> std::io::Result<()> {
    let text = std::fs::read_to_string(profile)?;
    let json: serde_json::Value = serde_json::from_str(&text)?;
    let t = Duration::from_secs(2);

    let mut applied = 0;
    for (id, name) in param::NAMES.iter().enumerate() {
        let Some(value) = json.get(*name).and_then(|v| v.as_f64()).map(|v| v as f32) else {
            println!("apply: {name:9} (absent, skipped)");
            continue;
        };
        let set = Message::SetParam {
            id: id as u8,
            value,
        };
        match link.request(
            &set,
            |m| matches!(m, Message::Ack { of: 0x09 } | Message::Nak { of: 0x09, .. }),
            t,
        )? {
            Message::Ack { .. } => {}
            other => {
                return Err(std::io::Error::other(format!(
                    "device refused {name} = {value}: {other:?}"
                )))
            }
        }
        // Read back to confirm.
        let got = match link.request(
            &Message::GetParam { id: id as u8 },
            |m| matches!(m, Message::ParamValue { id: i, .. } if *i == id as u8),
            t,
        )? {
            Message::ParamValue { value, .. } => value,
            _ => unreachable!(),
        };
        if got != value {
            return Err(std::io::Error::other(format!(
                "verify failed: {name} wrote {value}, read {got}"
            )));
        }
        println!("apply: {name:9} = {value:.6}  (verified)");
        applied += 1;
    }
    println!(
        "apply: {applied} parameters set — they take effect at the next drive start (RAM only)."
    );
    Ok(())
}

fn write_meta(
    dir: &Path,
    stem: &str,
    title: &str,
    description: &str,
    order: u32,
) -> std::io::Result<()> {
    let meta = serde_json::json!({
        "title": title,
        "description": description,
        "order": order,
        "command": "mmc-host profile",
        "unix_time": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
        "notes": [],
    });
    std::fs::write(
        dir.join(format!("{stem}.meta.json")),
        serde_json::to_string_pretty(&meta)?,
    )
}
