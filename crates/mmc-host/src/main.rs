//! Host-side CLI for the modular motor controller.
//!
//! - `sim` — run one simulation scenario ad hoc and write a CSV trace.
//! - `suite` — run the canonical scenario set into the results directory and
//!   rebuild the dashboard (`testresults/index.html`).
//! - `report` — rebuild the dashboard from whatever CSVs are in the results
//!   directory (sim runs and, later, hardware captures alike).
//!
//! Later (MS3+): live telemetry capture from sim or hardware over the shared
//! protocol, and the motor profiler.

mod capture;
mod link;
mod panel;
mod profile;
mod report;
mod scenario;
mod server;

use std::path::PathBuf;

use clap::{Parser, Subcommand, ValueEnum};

use scenario::{
    print_summary, run_current_step, run_sensorless, run_sixstep, RunSpec, SensorlessConfig,
    SensorlessSpec, SixStepConfig, SixStepSpec, StepConfig,
};

#[derive(Parser)]
#[command(name = "mmc-host", about = "modular motor controller host tools")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run a simulation scenario and write a CSV trace.
    Sim(SimArgs),
    /// Run the canonical scenario suite and rebuild the dashboard.
    Suite(SuiteArgs),
    /// Rebuild the HTML dashboard from the results directory.
    Report(ReportArgs),
    /// Serve the simulator over TCP, speaking the mmc-proto wire protocol.
    Serve(ServeArgs),
    /// Capture telemetry from a device (sim over TCP, hardware over serial).
    Capture(CaptureArgs),
    /// Serve a local web control panel for live testing (drive modes, charts).
    Panel(PanelArgs),
    /// Run the MS6 profiling sequence on hardware (R/L probe, flux sweep,
    /// accel run); fit the results with tools/profile.py afterwards.
    Profile(ProfileArgs),
    /// Apply a fitted motor profile (tools/profile.py JSON) to the device.
    Apply(ApplyArgs),
}

#[derive(clap::Args)]
struct ProfileArgs {
    /// Serial port of the hardware device (COMx, or `auto` for the first debug probe).
    #[arg(long, default_value = "auto")]
    serial: String,
    #[arg(long, default_value_t = 1_000_000)]
    baud: u32,
    /// TCP address of a sim server instead of hardware (e.g. 127.0.0.1:7770).
    #[arg(long)]
    addr: Option<String>,
    /// Output directory for the profiling captures.
    #[arg(long, default_value = "testresults/ms6-profile")]
    dir: PathBuf,
    /// Comma-separated stage subset (see --list): sweep,accel,rl,saliency.
    #[arg(long, value_delimiter = ',')]
    only: Option<Vec<String>>,
    /// Rerun stages profile_state.json already marks completed.
    #[arg(long)]
    redo: bool,
    /// Skip the interactive bench-setup confirmation.
    #[arg(long)]
    yes: bool,
    /// List the stages, what each measures, and what it expects on the bench.
    #[arg(long)]
    list: bool,
    /// R/L probe align,step voltages — lower for a low-resistance motor
    /// (default 0.5,1.0).
    #[arg(long, value_delimiter = ',', num_args = 2, default_values_t = [0.5, 1.0])]
    rl_volts: Vec<f32>,
    /// Flux-sweep operating points as amps@omega_e (rad/s el), e.g.
    /// "0.6@60,0.6@120,0.9@180". Size for the motor: it must hold I-f sync,
    /// and omega·flux must stay under the bus-voltage ceiling.
    #[arg(long, value_delimiter = ',')]
    sweep_points: Option<Vec<String>>,
    /// Accel-run sensorless speed targets start,step [rad/s el]
    /// (default 300,900).
    #[arg(long, value_delimiter = ',', num_args = 2)]
    accel_targets: Option<Vec<f32>>,
    /// Accel-run startup/authority current [A] (default 0.5) — raise for a
    /// high-drag or heavy motor.
    #[arg(long)]
    accel_amps: Option<f32>,
}

fn parse_sweep_points(specs: &[String]) -> std::io::Result<Vec<(f32, f32)>> {
    specs
        .iter()
        .map(|s| {
            s.split_once('@')
                .and_then(|(a, w)| Some((a.parse().ok()?, w.parse().ok()?)))
                .ok_or_else(|| {
                    std::io::Error::other(format!("bad sweep point `{s}` (want amps@omega)"))
                })
        })
        .collect()
}

#[derive(clap::Args)]
struct ApplyArgs {
    /// Serial port of the hardware device (COMx, or `auto` for the first debug probe).
    #[arg(long, default_value = "auto")]
    serial: String,
    #[arg(long, default_value_t = 1_000_000)]
    baud: u32,
    /// Profile JSON written by tools/profile.py.
    #[arg(long)]
    profile: PathBuf,
    /// Also write the applied table to flash so it survives a power cycle.
    #[arg(long)]
    persist: bool,
}

#[derive(clap::Args)]
struct PanelArgs {
    /// TCP address of a sim server (e.g. 127.0.0.1:7770).
    #[arg(long, conflicts_with = "serial")]
    addr: Option<String>,
    /// Serial port of a hardware device (COMx, or `auto` for the first debug probe).
    #[arg(long)]
    serial: Option<String>,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    /// Telemetry divider (device control periods per sample).
    #[arg(long, default_value_t = 40)]
    divider: u16,
    /// HTTP bind address for the panel UI.
    #[arg(long, default_value = "127.0.0.1:8484")]
    http: String,
    /// Output directory for profiler runs started from the panel.
    #[arg(long, default_value = "testresults/panel-profile")]
    profile_dir: PathBuf,
}

#[derive(clap::Args)]
struct ServeArgs {
    #[arg(long, default_value_t = 7770)]
    port: u16,
    /// Control-loop rate [Hz].
    #[arg(long, default_value_t = 10_000.0)]
    ctrl_freq: f32,
    /// Exit after the first client disconnects.
    #[arg(long)]
    once: bool,
    /// Virtual motor: `small` (0.6 mH hobby BLDC) or `bench` (the profiled
    /// G474 bench motor, 28 µH — the probes' τ < Ts regime).
    #[arg(long, value_parser = ["small", "bench"], default_value = "small")]
    motor: String,
    /// Saliency ratio Lq/Ld to give the virtual motor (probe positive
    /// control; 1.0 = non-salient).
    #[arg(long, default_value_t = 1.0)]
    saliency: f32,
    /// Mechanically clamp the virtual rotor (locked-rotor bench).
    #[arg(long)]
    locked: bool,
}

#[derive(clap::Args)]
struct CaptureArgs {
    /// TCP address of a sim server (e.g. 127.0.0.1:7770).
    #[arg(long, conflicts_with = "serial")]
    addr: Option<String>,
    /// Serial port of a hardware device (COMx, or `auto` for the first debug probe).
    #[arg(long)]
    serial: Option<String>,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    /// Telemetry divider (device control periods per sample).
    #[arg(long, default_value_t = 10)]
    divider: u16,
    /// Channel selection mask (default: all channels).
    #[arg(long, default_value_t = mmc_proto::channel::ALL)]
    mask: u32,
    /// Capture length [s].
    #[arg(long, default_value_t = 1.0)]
    duration: f32,
    /// Send a q-axis current step of this amplitude [A] at 10% of the capture.
    #[arg(long)]
    iq: Option<f32>,
    /// Command a drive mode at 10% of the capture: `volt` (open-loop voltage),
    /// `if` (I-f current), `sl` (closed-loop sensorless) or `six` (forced
    /// six-step commutation). Requires --amp and --hz; Off is sent at the end.
    #[arg(long, value_parser = ["volt", "if", "sl", "six"], conflicts_with = "iq")]
    drive: Option<String>,
    /// Drive amplitude: volts (volt), amps (if / sl startup), duty 0..1 (six).
    #[arg(long, requires = "drive")]
    amp: Option<f32>,
    /// Drive electrical frequency [Hz] (sl: speed target).
    #[arg(long, requires = "drive")]
    hz: Option<f32>,
    /// Retarget the drive to this electrical frequency [Hz] at 60% of the
    /// capture — records a live speed-step response.
    #[arg(long, requires = "drive")]
    step_hz: Option<f32>,
    /// Output CSV path.
    #[arg(long, default_value = "capture.csv")]
    out: PathBuf,
    /// Run title on the dashboard.
    #[arg(long)]
    title: Option<String>,
}

#[derive(clap::Args)]
struct SimArgs {
    #[arg(long, value_enum, default_value_t = Scenario::CurrentStep)]
    scenario: Scenario,
    /// Output CSV path.
    #[arg(long, default_value = "sim_out.csv")]
    out: PathBuf,
    /// Simulated duration [s].
    #[arg(long, default_value_t = 0.05)]
    duration: f32,
    /// Control-loop rate [Hz].
    #[arg(long, default_value_t = 10_000.0)]
    ctrl_freq: f32,
    /// Current-loop design bandwidth [rad/s].
    #[arg(long, default_value_t = 2000.0)]
    bandwidth: f32,
    /// q-axis current step amplitude [A].
    #[arg(long, default_value_t = 1.0)]
    iq: f32,
    /// DC bus voltage [V].
    #[arg(long, default_value_t = 24.0)]
    vbus: f32,
    /// External load torque [N·m].
    #[arg(long, default_value_t = 0.0)]
    load: f32,
    /// Hold the rotor at standstill (locked-rotor bench).
    #[arg(long)]
    locked: bool,
    /// Sensorless speed target [rad/s electrical].
    #[arg(long, default_value_t = 800.0)]
    omega_e: f32,
    /// Additional load stepped in at 60% of a sensorless run [N·m].
    #[arg(long, default_value_t = 0.0)]
    load_step: f32,
    /// Six-step: model a sinusoidal machine instead of a trapezoidal one.
    #[arg(long)]
    sinusoidal: bool,
    /// Six-step: sample the idle phase in the freewheel instead of the PWM
    /// on-time — reproduces the sample point MS8 step 1 measured on hardware.
    #[arg(long)]
    freewheel_sample: bool,
    /// Six-step: model a sense network that cannot read below ground.
    #[arg(long)]
    clamp_sense: bool,
}

#[derive(Copy, Clone, ValueEnum)]
enum Scenario {
    /// q-axis current reference step in torque mode.
    CurrentStep,
    /// I-f startup → observer handoff → sensorless speed loop (MS4).
    SensorlessSpeed,
    /// Forced commutation → back-EMF zero-cross sensing → duty speed loop (MS8).
    SixStep,
}

#[derive(clap::Args)]
struct SuiteArgs {
    /// Results directory.
    #[arg(long, default_value = "testresults")]
    dir: PathBuf,
}

#[derive(clap::Args)]
struct ReportArgs {
    /// Results directory to scan for CSV traces.
    #[arg(long, default_value = "testresults")]
    dir: PathBuf,
    /// Output HTML path (default: <dir>/index.html).
    #[arg(long)]
    out: Option<PathBuf>,
}

fn main() -> std::io::Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Sim(args) => match args.scenario {
            Scenario::CurrentStep => {
                let spec = RunSpec {
                    title: args
                        .out
                        .file_stem()
                        .map(|s| s.to_string_lossy().replace('_', " "))
                        .unwrap_or_else(|| "current step".into())
                        .leak(),
                    description: "Ad-hoc current-step run.",
                    order: 100,
                    cfg: StepConfig {
                        duration: args.duration,
                        ctrl_freq: args.ctrl_freq,
                        bandwidth: args.bandwidth,
                        iq: args.iq,
                        vbus: args.vbus,
                        load: args.load,
                        locked: args.locked,
                    },
                };
                let result = run_current_step(&spec, &args.out)?;
                print_summary(&spec, &args.out, &result, true);
                Ok(())
            }
            Scenario::SensorlessSpeed => {
                let spec = SensorlessSpec {
                    title: "sensorless speed",
                    description: "Ad-hoc sensorless run.",
                    order: 100,
                    cfg: SensorlessConfig {
                        duration: args.duration.max(1.0),
                        omega_e: args.omega_e,
                        load: args.load,
                        load_step: args.load_step,
                    },
                };
                let result = run_sensorless(&spec, &args.out)?;
                println!("wrote {} samples to {}", result.samples, args.out.display());
                for note in &result.notes {
                    println!("{note}");
                }
                Ok(())
            }
            Scenario::SixStep => {
                let spec = SixStepSpec {
                    title: "six-step",
                    description: "Ad-hoc six-step run.",
                    order: 100,
                    cfg: SixStepConfig {
                        duration: args.duration.max(1.0),
                        omega_e: args.omega_e,
                        load: args.load,
                        load_step: args.load_step,
                        trapezoidal: !args.sinusoidal,
                        on_time: !args.freewheel_sample,
                        clamp: args.clamp_sense,
                    },
                };
                let result = run_sixstep(&spec, &args.out)?;
                println!("wrote {} samples to {}", result.samples, args.out.display());
                for note in &result.notes {
                    println!("{note}");
                }
                Ok(())
            }
        },
        Command::Suite(args) => run_suite(&args.dir),
        Command::Report(args) => {
            let out = args.out.unwrap_or_else(|| args.dir.join("index.html"));
            let n = report::generate(&args.dir, &out)?;
            println!("dashboard: {n} runs -> {}", out.display());
            Ok(())
        }
        Command::Serve(args) => {
            let listener = std::net::TcpListener::bind(("0.0.0.0", args.port))?;
            let mut params = match args.motor.as_str() {
                "bench" => mmc_sim::PmsmParams::bench_g474(),
                _ => mmc_sim::PmsmParams::small_bldc(),
            };
            params.lq = params.ld * args.saliency;
            server::serve(
                listener,
                &server::ServeCfg {
                    ctrl_freq: args.ctrl_freq,
                    once: args.once,
                    params,
                    locked: args.locked,
                    ..Default::default()
                },
            )
        }
        Command::Capture(args) => {
            let mut link = match (&args.addr, &args.serial) {
                (Some(addr), _) => link::Link::tcp(addr)?,
                (None, Some(port)) => link::Link::serial(port, args.baud)?,
                (None, None) => {
                    eprintln!("pass --addr host:port (sim) or --serial COMx|auto (hardware)");
                    std::process::exit(2);
                }
            };
            let title = args.title.clone().unwrap_or_else(|| {
                args.out
                    .file_stem()
                    .map(|s| s.to_string_lossy().replace('_', " "))
                    .unwrap_or_else(|| "capture".into())
            });
            let mode_for = |kind: &str, amp: f32, hz: f32| {
                let omega_e = 2.0 * std::f32::consts::PI * hz;
                match kind {
                    "volt" => mmc_proto::DriveMode::OpenLoopVoltage {
                        volts: amp,
                        omega_e,
                    },
                    "sl" => mmc_proto::DriveMode::Sensorless { amps: amp, omega_e },
                    "six" => mmc_proto::DriveMode::SixStepForced { duty: amp, omega_e },
                    _ => mmc_proto::DriveMode::IfCurrent { amps: amp, omega_e },
                }
            };
            let (drive, drive_step) = match args.drive.as_deref() {
                None => (None, None),
                Some(kind) => {
                    let (Some(amp), Some(hz)) = (args.amp, args.hz) else {
                        eprintln!("--drive requires --amp and --hz");
                        std::process::exit(2);
                    };
                    (
                        Some(mode_for(kind, amp, hz)),
                        args.step_hz.map(|hz2| mode_for(kind, amp, hz2)),
                    )
                }
            };
            let summary = capture::run(
                &mut link,
                &capture::CaptureCfg {
                    divider: args.divider,
                    mask: args.mask,
                    duration: args.duration,
                    iq: args.iq,
                    drive,
                    drive_step,
                    title: &title,
                    description: "Ad-hoc telemetry capture.",
                    order: 100,
                    command: capture_cmdline(&args),
                },
                &args.out,
            )?;
            println!(
                "captured {} frames ({} rejected) -> {}",
                summary.frames,
                summary.frame_errors,
                args.out.display()
            );
            Ok(())
        }
        Command::Panel(args) => {
            let link = match (&args.addr, &args.serial) {
                (Some(addr), _) => link::Link::tcp(addr)?,
                (None, Some(port)) => link::Link::serial(port, args.baud)?,
                (None, None) => {
                    eprintln!("pass --addr host:port (sim) or --serial COMx|auto (hardware)");
                    std::process::exit(2);
                }
            };
            panel::run(
                link,
                &panel::PanelCfg {
                    http: args.http.clone(),
                    divider: args.divider,
                    profile_dir: args.profile_dir.clone(),
                },
            )
        }
        Command::Profile(args) => {
            if args.list {
                profile::list_stages();
                return Ok(());
            }
            let mut link = match &args.addr {
                Some(addr) => link::Link::tcp(addr)?,
                None => link::Link::serial(&args.serial, args.baud)?,
            };
            let mut tuning = profile::StageTuning {
                rl_volts: (args.rl_volts[0], args.rl_volts[1]),
                ..Default::default()
            };
            if let Some(specs) = &args.sweep_points {
                tuning.sweep = parse_sweep_points(specs)?;
            }
            if let Some(t) = &args.accel_targets {
                tuning.accel = (t[0], t[1]);
            }
            if let Some(a) = args.accel_amps {
                tuning.accel_amps = a;
            }
            profile::run(
                &mut link,
                &args.dir,
                &profile::ProfileOpts {
                    only: args.only.clone(),
                    redo: args.redo,
                    yes: args.yes,
                    tuning,
                },
            )
        }
        Command::Apply(args) => {
            let mut link = link::Link::serial(&args.serial, args.baud)?;
            profile::apply(&mut link, &args.profile, args.persist)
        }
    }
}

fn capture_cmdline(args: &CaptureArgs) -> String {
    let target = match (&args.addr, &args.serial) {
        (Some(a), _) => format!("--addr {a}"),
        (_, Some(s)) => format!("--serial {s} --baud {}", args.baud),
        _ => String::new(),
    };
    format!(
        "mmc-host capture {target} --divider {} --duration {}{}",
        args.divider,
        args.duration,
        args.iq.map(|i| format!(" --iq {i}")).unwrap_or_default(),
    )
}

/// The canonical regression set. Add new scenarios here as milestones land;
/// the dashboard picks them up from the directory automatically.
fn run_suite(dir: &std::path::Path) -> std::io::Result<()> {
    let ms2 = dir.join("ms2-current-loop");
    let runs = [
        (
            "step_locked",
            RunSpec {
                title: "Locked rotor",
                description: "1 A q-axis step at standstill — isolates the electrical \
                              dynamics from back-EMF; the regression baseline.",
                order: 0,
                cfg: StepConfig {
                    locked: true,
                    ..StepConfig::default()
                },
            },
        ),
        (
            "step_free",
            RunSpec {
                title: "Free rotor",
                description: "1 A q-axis step, unloaded — the rotor accelerates until \
                              back-EMF approaches the bus and the drive runs out of volts.",
                order: 1,
                cfg: StepConfig {
                    duration: 0.12,
                    ..StepConfig::default()
                },
            },
        ),
        (
            "step_loaded",
            RunSpec {
                title: "Loaded rotor",
                description: "1 A q-axis step against a 0.04 N·m load (torque constant \
                              is 0.084 N·m/A) — accelerates more slowly under load.",
                order: 2,
                cfg: StepConfig {
                    duration: 0.12,
                    load: 0.04,
                    ..StepConfig::default()
                },
            },
        ),
    ];

    println!("suite: {} runs -> {}", runs.len(), ms2.display());
    for (name, spec) in &runs {
        let out = ms2.join(format!("{name}.csv"));
        let result = run_current_step(spec, &out)?;
        print_summary(spec, &out, &result, false);
    }

    // MS4: canonical sensorless startup + load step.
    let ms4 = dir.join("ms4-sensorless");
    let spec = SensorlessSpec {
        title: "Sensorless startup + load step",
        description: "I-f startup, blend to the flux observer at 150 rad/s elec, speed loop \
                      to 800 rad/s elec, +0.03 N·m load step at 60% — angle estimate vs sim \
                      truth is the regression that MS4 lives by.",
        order: 0,
        cfg: SensorlessConfig {
            load_step: 0.03,
            ..SensorlessConfig::default()
        },
    };
    let out = ms4.join("sensorless_speed.csv");
    let result = run_sensorless(&spec, &out)?;
    println!(
        "  {:24} {} -> {}",
        "Sensorless speed",
        result.notes.first().map(String::as_str).unwrap_or(""),
        out.display()
    );

    suite_tcp_capture(dir)?;

    let out = dir.join("index.html");
    let n = report::generate(dir, &out)?;
    println!("dashboard: {n} runs -> {}", out.display());
    Ok(())
}

/// MS3 leg of the suite: spin the sim server up in-process and capture a live
/// current step through the actual TCP wire protocol.
fn suite_tcp_capture(dir: &std::path::Path) -> std::io::Result<()> {
    let listener = std::net::TcpListener::bind("127.0.0.1:0")?;
    let addr = listener.local_addr()?;
    let handle = std::thread::spawn(move || {
        server::serve(
            listener,
            &server::ServeCfg {
                once: true,
                ..Default::default()
            },
        )
    });

    let out = dir.join("ms3-telemetry").join("tcp_step.csv");
    let mut link = link::Link::tcp(&addr.to_string())?;
    let summary = capture::run(
        &mut link,
        &capture::CaptureCfg {
            divider: 10,
            mask: mmc_proto::channel::ALL,
            duration: 1.0,
            iq: Some(0.5),
            drive: None,
            drive_step: None,
            title: "TCP live capture — 0.5 A step",
            description: "End-to-end protocol regression: the sim runs behind the mmc-proto \
                          TCP server, the host connects like it would to hardware, streams \
                          telemetry at 1 kHz, and commands a 0.5 A q-axis step over the wire.",
            order: 0,
            command: "mmc-host suite (in-process sim server)".into(),
        },
        &out,
    )?;
    drop(link);
    let _ = handle.join();
    println!(
        "  {:24} {} frames, {} rejected -> {}",
        "TCP live capture",
        summary.frames,
        summary.frame_errors,
        out.display()
    );
    Ok(())
}
