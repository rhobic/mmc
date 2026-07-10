//! Byte-stream link carrying mmc-proto frames: TCP to the simulator, serial
//! to hardware — the same code path either way, which is the point.

use std::io::{ErrorKind, Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use mmc_proto::{encode, Deframer, FrameError, Message, MAX_FRAME};

pub trait Io: Read + Write + Send {}
impl<T: Read + Write + Send> Io for T {}

pub struct Link {
    io: Box<dyn Io>,
    deframer: Deframer,
    /// Read-side chunk buffer: one syscall pulls in as much as is available,
    /// bytes are consumed across `recv` calls (byte-per-syscall reads cap a
    /// serial link at a few hundred frames/s).
    pending: [u8; 4096],
    pending_len: usize,
    pending_pos: usize,
    /// Frames rejected on this link (CRC, COBS, oversize) — protocol health.
    pub frame_errors: usize,
}

impl Link {
    /// Connect over TCP (`host:port`).
    pub fn tcp(addr: &str) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(Duration::from_millis(50)))?;
        Ok(Self::new(Box::new(stream)))
    }

    /// Open a serial port. `port` may be `auto` to pick the first ST-Link VCP.
    pub fn serial(port: &str, baud: u32) -> std::io::Result<Self> {
        let name = if port.eq_ignore_ascii_case("auto") {
            find_stlink_port()?
        } else {
            port.to_string()
        };
        let sp = serialport::new(&name, baud)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| std::io::Error::other(format!("open {name}: {e}")))?;
        println!("serial: {name} @ {baud} baud");
        Ok(Self::new(Box::new(sp)))
    }

    pub fn new(io: Box<dyn Io>) -> Self {
        Self {
            io,
            deframer: Deframer::new(),
            pending: [0; 4096],
            pending_len: 0,
            pending_pos: 0,
            frame_errors: 0,
        }
    }

    pub fn send(&mut self, msg: &Message) -> std::io::Result<()> {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode(msg, &mut buf).expect("MAX_FRAME-sized buffer");
        self.io.write_all(&buf[..n])?;
        self.io.flush()
    }

    /// Receive the next good frame, waiting up to `timeout`. `Ok(None)` on
    /// timeout; frame-level corruption is counted and skipped.
    pub fn recv(&mut self, timeout: Duration) -> std::io::Result<Option<Message>> {
        let deadline = Instant::now() + timeout;
        loop {
            while self.pending_pos < self.pending_len {
                let byte = self.pending[self.pending_pos];
                self.pending_pos += 1;
                match self.deframer.push(byte) {
                    Some(Ok(msg)) => return Ok(Some(msg)),
                    Some(Err(e)) => {
                        self.frame_errors += 1;
                        let _: FrameError = e;
                    }
                    None => {}
                }
            }
            match self.io.read(&mut self.pending) {
                Ok(0) => return Err(std::io::Error::new(ErrorKind::UnexpectedEof, "link closed")),
                Ok(n) => {
                    self.pending_len = n;
                    self.pending_pos = 0;
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {
                    if Instant::now() >= deadline {
                        return Ok(None);
                    }
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Send `req` and wait for a response matching `want` (other frames are
    /// discarded — fine during setup, before streaming starts).
    pub fn request(
        &mut self,
        req: &Message,
        want: impl Fn(&Message) -> bool,
        timeout: Duration,
    ) -> std::io::Result<Message> {
        self.send(req)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Some(msg) = self.recv(deadline - Instant::now())? {
                if want(&msg) {
                    return Ok(msg);
                }
            }
        }
        Err(std::io::Error::new(
            ErrorKind::TimedOut,
            format!("no response to {:#04x}", req.wire_type()),
        ))
    }
}

/// First serial port that looks like an ST-Link virtual COM port.
fn find_stlink_port() -> std::io::Result<String> {
    let ports = serialport::available_ports()
        .map_err(|e| std::io::Error::other(format!("enumerate ports: {e}")))?;
    for p in &ports {
        if let serialport::SerialPortType::UsbPort(usb) = &p.port_type {
            // ST-Link VID; covers V2-1/V3 VCP PIDs.
            if usb.vid == 0x0483 {
                return Ok(p.port_name.clone());
            }
        }
    }
    let names: Vec<_> = ports.iter().map(|p| p.port_name.clone()).collect();
    Err(std::io::Error::other(format!(
        "no ST-Link VCP found (ports: {names:?}) — pass --serial COMx"
    )))
}
