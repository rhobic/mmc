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

use mmc_proto::{channel, DriveMode, Message};

use crate::link::Link;

const RING_CAP: usize = 20_000;
const PANEL_HTML: &str = include_str!("../templates/panel.html");

pub struct PanelCfg {
    /// HTTP bind address, e.g. `127.0.0.1:8484`.
    pub http: String,
    pub divider: u16,
}

struct Shared {
    columns: Vec<String>,
    ring: VecDeque<Vec<f32>>,
    device: String,
    frames: u64,
    errors: usize,
    last_poll: Instant,
    drive_on: bool,
}

pub fn run(mut link: Link, cfg: &PanelCfg) -> std::io::Result<()> {
    // Handshake mirrors `capture`.
    let t = Duration::from_secs(2);
    link.request(
        &Message::Ping { nonce: 7 },
        |m| matches!(m, Message::Pong { nonce: 7 }),
        t,
    )?;
    let device = match link.request(&Message::GetInfo, |m| matches!(m, Message::Info(_)), t)? {
        Message::Info(info) => format!("{} ({:?})", info.name_str(), info.kind),
        _ => unreachable!(),
    };
    let set = Message::SetTelemetry {
        divider: cfg.divider,
        mask: channel::ALL,
    };
    link.request(&set, |m| matches!(m, Message::Ack { .. }), t)?;
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
    }));
    let (cmd_tx, cmd_rx) = mpsc::channel::<Message>();

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
    loop {
        while let Ok(msg) = cmd_rx.try_recv() {
            let mut s = shared.lock().unwrap();
            match msg {
                Message::SetDrive(DriveMode::Off) => s.drive_on = false,
                Message::SetDrive(_) => s.drive_on = true,
                _ => {}
            }
            drop(s);
            link.send(&msg)?;
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
        if let Some(Message::Telemetry(f)) = link.recv(Duration::from_millis(20))? {
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

fn http_loop(listener: TcpListener, shared: Arc<Mutex<Shared>>, cmd_tx: mpsc::Sender<Message>) {
    for conn in listener.incoming() {
        let Ok(conn) = conn else { continue };
        let _ = conn.set_nodelay(true);
        let _ = handle_conn(conn, &shared, &cmd_tx);
    }
}

fn handle_conn(
    conn: TcpStream,
    shared: &Arc<Mutex<Shared>>,
    cmd_tx: &mpsc::Sender<Message>,
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
            let json = serde_json::json!({
                "columns": s.columns,
                "rows": rows,
                "device": s.device,
                "frames": s.frames,
                "errors": s.errors,
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

fn parse_cmd(v: &serde_json::Value) -> Option<Message> {
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
                _ => return None,
            };
            Some(Message::SetDrive(mode))
        }
        "iq" => Some(Message::SetIqRef { iq: f("iq")? }),
        _ => None,
    }
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
