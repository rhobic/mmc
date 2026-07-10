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

mod report;
mod scenario;

use std::path::PathBuf;

use clap::{Parser, Subcommand, ValueEnum};

use scenario::{run_current_step, print_summary, RunSpec, StepConfig};

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
}

#[derive(Copy, Clone, ValueEnum)]
enum Scenario {
    /// q-axis current reference step in torque mode.
    CurrentStep,
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
        },
        Command::Suite(args) => run_suite(&args.dir),
        Command::Report(args) => {
            let out = args.out.unwrap_or_else(|| args.dir.join("index.html"));
            let n = report::generate(&args.dir, &out)?;
            println!("dashboard: {n} runs -> {}", out.display());
            Ok(())
        }
    }
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

    let out = dir.join("index.html");
    let n = report::generate(dir, &out)?;
    println!("dashboard: {n} runs -> {}", out.display());
    Ok(())
}
