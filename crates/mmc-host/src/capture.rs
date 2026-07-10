//! Telemetry capture over any [`Link`]: handshake, configure channels, stream
//! to CSV (+ meta sidecar) in the same shape the sim scenarios write — so the
//! dashboard treats live captures and offline runs identically.

use std::io::Write as _;
use std::path::Path;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use mmc_proto::{channel, DeviceInfo, Message};

use crate::link::Link;

pub struct CaptureCfg<'a> {
    /// Telemetry rate divider (device control periods per sample).
    pub divider: u16,
    /// Channel selection mask.
    pub mask: u32,
    /// Capture length [s] of wall time.
    pub duration: f32,
    /// If set, a q-axis current step to this value is sent at 10% of the
    /// capture, so the trace records a live step response.
    pub iq: Option<f32>,
    pub title: &'a str,
    pub description: &'a str,
    /// Dashboard sort key within the group.
    pub order: u32,
    /// How this capture was invoked, recorded in the meta sidecar.
    pub command: String,
}

pub struct CaptureSummary {
    pub frames: usize,
    pub frame_errors: usize,
}

pub fn run(link: &mut Link, cfg: &CaptureCfg, out: &Path) -> std::io::Result<CaptureSummary> {
    let t = Duration::from_secs(2);

    // Liveness + identity before anything else.
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(1);
    link.request(
        &Message::Ping { nonce },
        |m| matches!(m, Message::Pong { nonce: n } if *n == nonce),
        t,
    )?;
    let device = match link.request(&Message::GetInfo, |m| matches!(m, Message::Info(_)), t)? {
        Message::Info(info) => info,
        _ => unreachable!(),
    };
    println!(
        "device: {} ({:?}, proto v{}, fw {})",
        device.name_str(),
        device.kind,
        device.proto_version,
        device.fw_version
    );

    let ack = |req: &Message| {
        let ty = req.wire_type();
        move |m: &Message| matches!(m, Message::Ack { of } if *of == ty)
    };
    let set = Message::SetTelemetry {
        divider: cfg.divider,
        mask: cfg.mask & channel::ALL,
    };
    link.request(&set, ack(&set), t)?;
    let start_stream = Message::Stream { enable: true };
    link.request(&start_stream, ack(&start_stream), t)?;

    // Collect. The step reference (if any) goes out 10% into the capture.
    let started = Instant::now();
    let capture_len = Duration::from_secs_f32(cfg.duration);
    let step_at = capture_len.mul_f32(0.1);
    let mut step_sent = cfg.iq.is_none();
    let mut frames: Vec<(u32, u32, Vec<f32>)> = Vec::new();
    while started.elapsed() < capture_len {
        if !step_sent && started.elapsed() >= step_at {
            link.send(&Message::SetIqRef {
                iq: cfg.iq.unwrap(),
            })?;
            step_sent = true;
        }
        if let Some(Message::Telemetry(f)) = link.recv(Duration::from_millis(50))? {
            frames.push((f.t_us, f.mask, f.values().to_vec()));
        }
        // Acks and anything else mid-stream are simply skipped.
    }
    link.send(&Message::Stream { enable: false })?;
    if cfg.iq.is_some() {
        link.send(&Message::SetIqRef { iq: 0.0 })?;
    }
    // Drain in-flight frames so the device-side stop is clean.
    let drain_until = Instant::now() + Duration::from_millis(300);
    while Instant::now() < drain_until {
        if let Some(Message::Telemetry(f)) = link.recv(Duration::from_millis(50))? {
            frames.push((f.t_us, f.mask, f.values().to_vec()));
        }
    }

    if frames.is_empty() {
        return Err(std::io::Error::other("no telemetry received"));
    }
    write_csv(out, &frames)?;
    write_meta(cfg, out, &device, frames.len(), link.frame_errors)?;

    Ok(CaptureSummary {
        frames: frames.len(),
        frame_errors: link.frame_errors,
    })
}

fn write_csv(out: &Path, frames: &[(u32, u32, Vec<f32>)]) -> std::io::Result<()> {
    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mask = frames[0].1;
    let mut w = std::io::BufWriter::new(std::fs::File::create(out)?);
    write!(w, "t")?;
    for id in 0..channel::COUNT as u8 {
        if mask & (1 << id) != 0 {
            write!(w, ",{}", channel::NAMES[id as usize])?;
        }
    }
    writeln!(w)?;

    // Unwrap the device's wrapping-µs clock into seconds from first sample.
    let t0 = frames[0].0;
    let mut last = t0;
    let mut acc: u64 = 0;
    for (t_us, m, values) in frames {
        if *m != mask {
            continue; // mask changed mid-capture: not expected, skip frame
        }
        acc += t_us.wrapping_sub(last) as u64;
        last = *t_us;
        write!(w, "{}", acc as f64 / 1e6)?;
        for v in values {
            write!(w, ",{v}")?;
        }
        writeln!(w)?;
    }
    w.flush()
}

fn write_meta(
    cfg: &CaptureCfg,
    out: &Path,
    device: &DeviceInfo,
    frames: usize,
    errors: usize,
) -> std::io::Result<()> {
    let mut notes = vec![format!(
        "Captured live from {} ({:?}) over mmc-proto; {} frames.",
        device.name_str(),
        device.kind,
        frames
    )];
    if errors > 0 {
        notes.push(format!(
            "{errors} frames rejected (CRC/framing) during capture."
        ));
    }
    let meta = serde_json::json!({
        "title": cfg.title,
        "description": cfg.description,
        "order": cfg.order,
        "command": cfg.command,
        "unix_time": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
        "params": {
            "divider": cfg.divider,
            "duration_s": cfg.duration,
            "iq_A": cfg.iq,
        },
        "notes": notes,
    });
    std::fs::write(
        out.with_extension("meta.json"),
        serde_json::to_string_pretty(&meta)?,
    )
}
