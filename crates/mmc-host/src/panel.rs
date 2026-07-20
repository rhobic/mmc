//! Manual control panel: a small local web UI for exercising a device live —
//! drive mode buttons, amplitude/frequency, live telemetry charts. One
//! `Link`, so it drives the simulator over TCP or hardware over serial
//! identically.
//!
//! Safety posture: the firmware deadman is fed by this process's keep-alive
//! pings; additionally, if no browser has polled `/data` for 5 s while a
//! drive is active, the panel commands `DriveMode::Off` itself.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mmc_proto::{channel, param, DriveMode, Message};

use crate::link::Link;

const RING_CAP: usize = 20_000;
const PANEL_HTML: &str = include_str!("../templates/panel.html");

pub struct PanelCfg {
    /// HTTP bind address, e.g. `127.0.0.1:8484`.
    pub http: String,
    pub divider: u16,
    /// Where the profiler card's captures and fits land.
    pub profile_dir: std::path::PathBuf,
}

/// Command from the HTTP thread to the link pump. Drive/i_q are fire-and-forget;
/// parameter ops need a synchronous request/response, so the pump runs them
/// inline (it owns the single `Link`). Profiler runs block the pump for their
/// duration — telemetry freezes, the browser shows the log instead.
enum Cmd {
    Send(Message),
    RefreshParams,
    SetParam { id: u8, value: f32 },
    RunProfile {
        stages: Vec<String>,
        tuning: crate::profile::StageTuning,
    },
    ApplyProfile,
}

struct Shared {
    columns: Vec<String>,
    ring: VecDeque<Vec<f32>>,
    device: String,
    frames: u64,
    errors: usize,
    last_poll: Instant,
    drive_on: bool,
    /// Latest known device parameter values, indexed by `param` id (NaN = unread).
    params: Vec<f32>,
    /// Human-readable result of the last parameter read/write, for the UI.
    param_status: String,
    /// Profiler card: run in progress (telemetry frozen while true).
    busy: bool,
    /// Profiler card log (stage progress + fit output).
    plog: Vec<String>,
    /// Drive-path series resistance for at-the-motor displays (0 on the sim).
    r_path: f32,
}

pub fn run(mut link: Link, cfg: &PanelCfg) -> std::io::Result<()> {
    // Handshake mirrors `capture`.
    let t = Duration::from_secs(2);
    link.request(
        &Message::Ping { nonce: 7 },
        |m| matches!(m, Message::Pong { nonce: 7 }),
        t,
    )?;
    let (device, r_path) = match link.request(&Message::GetInfo, |m| matches!(m, Message::Info(_)), t)? {
        Message::Info(info) => (
            format!("{} ({:?})", info.name_str(), info.kind),
            crate::profile::r_drive_path(info.kind),
        ),
        _ => unreachable!(),
    };
    let set = Message::SetTelemetry {
        divider: cfg.divider,
        mask: channel::ALL,
    };
    link.request(&set, |m| matches!(m, Message::Ack { .. }), t)?;

    // Read the device's runtime parameters while the line is still quiet —
    // `request` discards non-matching frames, so do this before streaming. A
    // device without a parameter table (the sim, the G0B1) NAKs GetParam;
    // bail on the first NAK so startup stays instant.
    let mut params = vec![f32::NAN; param::COUNT];
    let mut has_params = true;
    for id in 0..param::COUNT as u8 {
        match link.request(
            &Message::GetParam { id },
            |m| {
                matches!(m, Message::ParamValue { id: i, .. } if *i == id)
                    || matches!(m, Message::Nak { of: 0x0A, .. })
            },
            t,
        ) {
            Ok(Message::ParamValue { value, .. }) => params[id as usize] = value,
            _ => {
                has_params = false;
                break;
            }
        }
    }
    let param_status = if has_params {
        "parameters read from device".to_string()
    } else {
        "device has no runtime parameters".to_string()
    };

    let stream = Message::Stream { enable: true };
    link.request(&stream, |m| matches!(m, Message::Ack { .. }), t)?;

    let mut columns = vec!["t".to_string()];
    for id in 0..channel::COUNT as u8 {
        if channel::ALL & (1 << id) != 0 {
            columns.push(channel::NAMES[id as usize].to_string());
        }
    }
    let shared = Arc::new(Mutex::new(Shared {
        columns,
        ring: VecDeque::new(),
        device: device.clone(),
        frames: 0,
        errors: 0,
        last_poll: Instant::now(),
        drive_on: false,
        params,
        param_status,
        busy: false,
        plog: Vec::new(),
        r_path,
    }));
    let (cmd_tx, cmd_rx) = mpsc::channel::<Cmd>();

    let listener = TcpListener::bind(&cfg.http)?;
    println!(
        "panel: http://{}  (device: {device})",
        listener.local_addr()?
    );
    println!("panel: STOP button / closing the browser turns the drive off.");
    {
        let shared = Arc::clone(&shared);
        std::thread::spawn(move || http_loop(listener, shared, cmd_tx));
    }

    // Telemetry + command pump.
    let mut last_us: Option<u32> = None;
    let mut acc_us = 0u64;
    let mut last_send = Instant::now();
    let mut last_frame = Instant::now();
    loop {
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                Cmd::Send(msg) => {
                    {
                        let mut s = shared.lock().unwrap();
                        match msg {
                            Message::SetDrive(DriveMode::Off) => s.drive_on = false,
                            Message::SetDrive(_) => s.drive_on = true,
                            _ => {}
                        }
                    }
                    link.send(&msg)?;
                }
                Cmd::RefreshParams => read_params(&mut link, &shared),
                Cmd::SetParam { id, value } => set_param(&mut link, &shared, id, value),
                Cmd::RunProfile { stages, tuning } => {
                    run_profile(&mut link, cfg, &shared, &stages, &tuning);
                    // Drop any drive/param commands queued while blocked —
                    // executing stale controls after ~30 s would surprise.
                    while cmd_rx.try_recv().is_ok() {}
                    last_frame = Instant::now(); // give the resumed stream a grace period
                }
                Cmd::ApplyProfile => {
                    let path = cfg.profile_dir.join("profile.json");
                    let msg = match crate::profile::apply(&mut link, &path) {
                        Ok(()) => "profile applied and verified — takes effect at next drive start".to_string(),
                        Err(e) => format!("apply failed: {e}"),
                    };
                    shared.lock().unwrap().plog.push(msg);
                    read_params(&mut link, &shared); // refresh the params card
                }
            }
            last_send = Instant::now();
        }
        {
            let mut s = shared.lock().unwrap();
            if s.drive_on && s.last_poll.elapsed() > Duration::from_secs(5) {
                s.drive_on = false;
                drop(s);
                println!("panel: no browser for 5 s — drive off");
                link.send(&Message::SetDrive(DriveMode::Off))?;
                last_send = Instant::now();
            }
        }
        if last_send.elapsed() >= Duration::from_millis(500) {
            link.send(&Message::Ping { nonce: 0 })?;
            last_send = Instant::now();
        }
        // Telemetry watchdog: the device streams continuously while enabled, so
        // a multi-second gap means streaming silently dropped — a serial/device
        // hiccup, observed once on a ~13 h session. Re-arm it (fire-and-forget,
        // so a wedged device can't block the pump) instead of sitting dead.
        if last_frame.elapsed() > Duration::from_secs(3) {
            let _ = link.send(&Message::SetTelemetry {
                divider: cfg.divider,
                mask: channel::ALL,
            });
            let _ = link.send(&Message::Stream { enable: true });
            last_frame = Instant::now(); // back off ~3 s before retrying
            println!("panel: telemetry stalled — re-enabling stream");
        }
        if let Some(Message::Telemetry(f)) = link.recv(Duration::from_millis(20))? {
            last_frame = Instant::now();
            // Unwrap the device's wrapping-µs clock (same as `capture`).
            if let Some(prev) = last_us {
                acc_us += f.t_us.wrapping_sub(prev) as u64;
            }
            last_us = Some(f.t_us);
            let t_rel = acc_us as f64 / 1e6;
            let mut row = Vec::with_capacity(1 + f.values().len());
            row.push(t_rel as f32);
            row.extend_from_slice(f.values());
            let mut s = shared.lock().unwrap();
            s.ring.push_back(row);
            if s.ring.len() > RING_CAP {
                s.ring.pop_front();
            }
            s.frames += 1;
            s.errors = link.frame_errors;
        }
    }
}

fn http_loop(listener: TcpListener, shared: Arc<Mutex<Shared>>, cmd_tx: mpsc::Sender<Cmd>) {
    for conn in listener.incoming() {
        let Ok(conn) = conn else { continue };
        let _ = conn.set_nodelay(true);
        let _ = handle_conn(conn, &shared, &cmd_tx);
    }
}

fn handle_conn(
    conn: TcpStream,
    shared: &Arc<Mutex<Shared>>,
    cmd_tx: &mpsc::Sender<Cmd>,
) -> std::io::Result<()> {
    conn.set_read_timeout(Some(Duration::from_secs(2)))?;
    let mut reader = BufReader::new(conn);

    let mut request_line = String::new();
    reader.read_line(&mut request_line)?;
    let mut parts = request_line.split_whitespace();
    let method = parts.next().unwrap_or("").to_string();
    let target = parts.next().unwrap_or("").to_string();

    let mut content_len = 0usize;
    loop {
        let mut line = String::new();
        reader.read_line(&mut line)?;
        let line = line.trim();
        if line.is_empty() {
            break;
        }
        if let Some(v) = line
            .to_ascii_lowercase()
            .strip_prefix("content-length:")
            .map(str::trim)
        {
            content_len = v.parse().unwrap_or(0);
        }
    }
    let mut body = vec![0u8; content_len.min(64 * 1024)];
    if !body.is_empty() {
        reader.read_exact(&mut body)?;
    }
    let mut conn = reader.into_inner();

    match (method.as_str(), target.split('?').next().unwrap_or("")) {
        ("GET", "/") => respond(
            &mut conn,
            200,
            "text/html; charset=utf-8",
            PANEL_HTML.as_bytes(),
        ),
        ("GET", "/data") => {
            let since: f64 = target
                .split_once("since=")
                .and_then(|(_, v)| v.parse().ok())
                .unwrap_or(-1.0);
            let mut s = shared.lock().unwrap();
            s.last_poll = Instant::now();
            let start = s
                .ring
                .partition_point(|row| (row[0] as f64) <= since)
                .max(s.ring.len().saturating_sub(3000));
            let rows: Vec<&Vec<f32>> = s.ring.iter().skip(start).collect();
            let plog_tail: Vec<&String> = s.plog.iter().rev().take(120).rev().collect();
            let json = serde_json::json!({
                "columns": s.columns,
                "rows": rows,
                "device": s.device,
                "frames": s.frames,
                "errors": s.errors,
                "params": s.params,
                "param_names": param::NAMES,
                "param_status": s.param_status,
                "busy": s.busy,
                "plog": plog_tail,
                "r_path": s.r_path,
            });
            let body = serde_json::to_vec(&json)?;
            respond(&mut conn, 200, "application/json", &body)
        }
        ("POST", "/cmd") => {
            shared.lock().unwrap().last_poll = Instant::now(); // commands count as presence
            let v: serde_json::Value = serde_json::from_slice(&body).unwrap_or_default();
            let msg = parse_cmd(&v);
            let (status, reply) = match msg {
                Some(m) => {
                    let _ = cmd_tx.send(m);
                    (200, r#"{"ok":true}"#)
                }
                None => (400, r#"{"ok":false,"error":"bad command"}"#),
            };
            respond(&mut conn, status, "application/json", reply.as_bytes())
        }
        _ => respond(&mut conn, 404, "text/plain", b"not found"),
    }
}

fn parse_cmd(v: &serde_json::Value) -> Option<Cmd> {
    let f = |k: &str| v.get(k).and_then(|x| x.as_f64()).map(|x| x as f32);
    match v.get("cmd")?.as_str()? {
        "drive" => {
            let mode = match v.get("mode")?.as_str()? {
                "off" => DriveMode::Off,
                "volt" => DriveMode::OpenLoopVoltage {
                    volts: f("amp")?,
                    omega_e: f("hz")? * core::f32::consts::TAU,
                },
                "if" => DriveMode::IfCurrent {
                    amps: f("amp")?,
                    omega_e: f("hz")? * core::f32::consts::TAU,
                },
                "sl" => DriveMode::Sensorless {
                    amps: f("amp")?,
                    omega_e: f("hz")? * core::f32::consts::TAU,
                },
                _ => return None,
            };
            Some(Cmd::Send(Message::SetDrive(mode)))
        }
        "iq" => Some(Cmd::Send(Message::SetIqRef { iq: f("iq")? })),
        "getparams" => Some(Cmd::RefreshParams),
        "setparam" => Some(Cmd::SetParam {
            id: v.get("id")?.as_u64()? as u8,
            value: f("value")?,
        }),
        "profile" => {
            let stages = v
                .get("stages")?
                .as_array()?
                .iter()
                .filter_map(|x| x.as_str().map(String::from))
                .collect::<Vec<_>>();
            // Optional per-motor excitation overrides:
            //   "rl_volts": [a, b], "sweep": [[amps, omega_e], ...],
            //   "accel": [lo, hi]
            let mut tuning = crate::profile::StageTuning::default();
            let pair = |x: &serde_json::Value| -> Option<(f32, f32)> {
                let a = x.as_array()?;
                Some((a.first()?.as_f64()? as f32, a.get(1)?.as_f64()? as f32))
            };
            if let Some(rv) = v.get("rl_volts").and_then(&pair) {
                tuning.rl_volts = rv;
            }
            if let Some(points) = v.get("sweep").and_then(|x| x.as_array()) {
                let parsed: Vec<_> = points.iter().filter_map(&pair).collect();
                if !parsed.is_empty() {
                    tuning.sweep = parsed;
                }
            }
            if let Some(t) = v.get("accel").and_then(&pair) {
                tuning.accel = t;
            }
            if let Some(a) = v.get("accel_amps").and_then(|x| x.as_f64()) {
                tuning.accel_amps = a as f32;
            }
            (!stages.is_empty()).then_some(Cmd::RunProfile { stages, tuning })
        }
        "applyprofile" => Some(Cmd::ApplyProfile),
        _ => None,
    }
}

/// Profiler card backend: quiet the telemetry stream, run the requested
/// stages through the shared `profile::run_stages` engine, shell out to the
/// Python fits, then restore the panel's telemetry config. Blocks the pump
/// for the duration — the UI shows the log and a busy flag instead of
/// charts. The stage code feeds the firmware deadman itself.
fn run_profile(
    link: &mut Link,
    cfg: &PanelCfg,
    shared: &Arc<Mutex<Shared>>,
    stages: &[String],
    tuning: &crate::profile::StageTuning,
) {
    let t = Duration::from_secs(2);
    {
        let mut s = shared.lock().unwrap();
        if s.busy {
            return;
        }
        s.busy = true;
        s.plog.clear();
        s.drive_on = false; // stages command their own drives and end Off
    }
    let _ = link.request(
        &Message::Stream { enable: false },
        |m| matches!(m, Message::Ack { .. }),
        t,
    );

    let ids: Vec<&str> = stages.iter().map(String::as_str).collect();
    let log_shared = Arc::clone(shared);
    let mut log = move |line: &str| {
        println!("panel-profile: {line}");
        log_shared.lock().unwrap().plog.push(line.to_string());
    };
    match crate::profile::run_stages(link, &cfg.profile_dir, &ids, tuning, &mut log) {
        Ok(()) => {
            run_fit("tools/profile.py", &cfg.profile_dir, &mut log);
            if ids.contains(&"saliency") {
                run_fit("tools/saliency.py", &cfg.profile_dir, &mut log);
            }
            log("done — review the fit above, then Apply to push it to the device");
        }
        Err(e) => log(&format!("profiler stopped: {e}")),
    }

    // Restore the panel's own telemetry configuration.
    let _ = link.request(
        &Message::SetTelemetry {
            divider: cfg.divider,
            mask: channel::ALL,
        },
        |m| matches!(m, Message::Ack { .. }),
        t,
    );
    let _ = link.request(
        &Message::Stream { enable: true },
        |m| matches!(m, Message::Ack { .. }),
        t,
    );
    shared.lock().unwrap().busy = false;
}

/// Run a Python fit script, folding its output into the profiler log. The
/// panel is a repo tool: the scripts are addressed relative to the working
/// directory, exactly like the CLI usage they wrap.
fn run_fit(script: &str, dir: &std::path::Path, log: &mut dyn FnMut(&str)) {
    log(&format!("$ python {script} {}", dir.display()));
    match std::process::Command::new("python").arg(script).arg(dir).output() {
        Ok(out) => {
            for l in String::from_utf8_lossy(&out.stdout).lines() {
                log(l);
            }
            for l in String::from_utf8_lossy(&out.stderr).lines() {
                log(l);
            }
        }
        Err(e) => log(&format!(
            "could not run python ({e}) — run the fit manually from the repo root"
        )),
    }
}

/// Read every runtime parameter and stash it for the UI. Briefly discards a
/// telemetry sample or two while waiting for each reply — negligible.
fn read_params(link: &mut Link, shared: &Arc<Mutex<Shared>>) {
    let t = Duration::from_millis(500);
    for id in 0..param::COUNT as u8 {
        match link.request(
            &Message::GetParam { id },
            |m| {
                matches!(m, Message::ParamValue { id: i, .. } if *i == id)
                    || matches!(m, Message::Nak { of: 0x0A, .. })
            },
            t,
        ) {
            Ok(Message::ParamValue { value, .. }) => shared.lock().unwrap().params[id as usize] = value,
            Ok(Message::Nak { .. }) => {
                shared.lock().unwrap().param_status = "device has no runtime parameters".into();
                return;
            }
            _ => {
                shared.lock().unwrap().param_status =
                    format!("read of {} timed out", param::NAMES[id as usize]);
                return;
            }
        }
    }
    shared.lock().unwrap().param_status = "parameters read from device".into();
}

/// Set one parameter and verify by read-back. The firmware NAKs out-of-range
/// values; surface that verbatim rather than pretending the write took.
fn set_param(link: &mut Link, shared: &Arc<Mutex<Shared>>, id: u8, value: f32) {
    if id as usize >= param::COUNT {
        shared.lock().unwrap().param_status = format!("no such parameter #{id}");
        return;
    }
    let name = param::NAMES[id as usize];
    let t = Duration::from_millis(500);
    let ack = link.request(
        &Message::SetParam { id, value },
        |m| matches!(m, Message::Ack { of: 0x09 } | Message::Nak { of: 0x09, .. }),
        t,
    );
    let status = match ack {
        Ok(Message::Ack { .. }) => match link.request(
            &Message::GetParam { id },
            |m| matches!(m, Message::ParamValue { id: i, .. } if *i == id),
            t,
        ) {
            Ok(Message::ParamValue { value: got, .. }) => {
                shared.lock().unwrap().params[id as usize] = got;
                format!("{name} = {got} — applies at next drive start")
            }
            _ => format!("{name} set, but read-back timed out"),
        },
        Ok(Message::Nak { .. }) => format!("{name}: {value} rejected by device (check range)"),
        _ => format!("{name}: no response from device"),
    };
    shared.lock().unwrap().param_status = status;
}

fn respond(
    conn: &mut TcpStream,
    status: u16,
    content_type: &str,
    body: &[u8],
) -> std::io::Result<()> {
    let reason = match status {
        200 => "OK",
        400 => "Bad Request",
        _ => "Not Found",
    };
    write!(
        conn,
        "HTTP/1.1 {status} {reason}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    conn.write_all(body)?;
    conn.flush()
}
