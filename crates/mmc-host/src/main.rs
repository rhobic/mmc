//! Host-side CLI for the modular motor controller. Today: run simulation
//! scenarios and dump CSV traces. Later (MS3+): live telemetry capture from
//! sim or hardware over the shared protocol, and the motor profiler.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use clap::{Parser, Subcommand, ValueEnum};

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc};
use mmc_core::transforms::{Abc, Dq};
use mmc_core::tuning::current_pi_gains;
use mmc_hal::{BusVoltageSense, CurrentSense, PwmOutput};
use mmc_sim::analysis::step_metrics;
use mmc_sim::{PmsmParams, TruthAngle, VirtualMotor};

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

fn main() -> std::io::Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Sim(args) => match args.scenario {
            Scenario::CurrentStep => run_current_step(&args),
        },
    }
}

fn run_current_step(args: &SimArgs) -> std::io::Result<()> {
    let params = PmsmParams::small_bldc();
    let ctrl_dt = 1.0 / args.ctrl_freq;

    let mut rig = VirtualMotor::new(params, args.vbus);
    rig.motor.locked = args.locked;
    rig.load_torque = args.load;
    rig.enable();

    // Params come from the sim's ground truth here; on hardware the profiler
    // (MS6) supplies them.
    let mut foc = Foc::with_feedforward(
        current_pi_gains(params.rs, params.lq, args.bandwidth),
        Decoupling {
            ld: params.ld,
            lq: params.lq,
            flux: params.flux,
        },
    );
    let mut angle = TruthAngle::default();

    // Step after 10% of the trace so the plot shows the quiescent state.
    let step_time = args.duration * 0.1;
    let steps = (args.duration / ctrl_dt) as usize;

    let mut writer = BufWriter::new(File::create(&args.out)?);
    writeln!(
        writer,
        "t,iq_ref,i_d,i_q,v_d,v_q,duty_a,duty_b,duty_c,omega_m,theta_e"
    )?;

    let mut t_trace = Vec::new();
    let mut iq_trace = Vec::new();
    let mut v_mag_end = 0.0f32;

    for _ in 0..steps {
        let t = rig.time() as f32;
        let i_ref = if t >= step_time {
            Dq { d: 0.0, q: args.iq }
        } else {
            Dq::default()
        };

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

        writeln!(
            writer,
            "{t},{},{},{},{},{},{},{},{},{},{}",
            i_ref.q,
            out.i_dq.d,
            out.i_dq.q,
            out.v_dq.d,
            out.v_dq.q,
            out.duties[0],
            out.duties[1],
            out.duties[2],
            rig.motor.omega_m,
            rig.motor.theta_e(),
        )?;

        if t >= step_time {
            t_trace.push(t - step_time);
            iq_trace.push(out.i_dq.q);
        }
        v_mag_end = (out.v_dq.d * out.v_dq.d + out.v_dq.q * out.v_dq.q).sqrt();
    }
    writer.flush()?;

    let voltage_limited = v_mag_end >= 0.95 * args.vbus / 3.0f32.sqrt();

    println!("wrote {} samples to {}", steps, args.out.display());
    match step_metrics(&t_trace, &iq_trace, args.iq) {
        Some(m) => {
            println!(
                "i_q step response ({} rad/s design bandwidth):",
                args.bandwidth
            );
            println!(
                "  rise time (10-90%):  {:.3} ms  (ideal {:.3} ms)",
                m.rise_time * 1e3,
                (9.0f32).ln() / args.bandwidth * 1e3
            );
            println!("  overshoot:           {:.1} %", m.overshoot * 100.0);
            println!(
                "  steady-state error:  {:.2} %",
                m.steady_state_error * 100.0
            );
        }
        None => println!("i_q never reached the step thresholds — check gains/limits"),
    }
    if voltage_limited {
        println!(
            "note: drive ended voltage-limited (back-EMF ≈ bus voltage). The motor \
             out-ran the bus, so the current reference is unreachable there — expected \
             without field weakening. Add --load or shorten --duration."
        );
    }
    if !args.locked {
        println!(
            "final speed: {:.0} rad/s mech ({:.0} rpm)",
            rig.motor.omega_m,
            rig.motor.omega_m * 60.0 / (2.0 * std::f32::consts::PI)
        );
    }
    Ok(())
}
