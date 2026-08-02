//! Simulation scenarios, reusable by the `sim` (ad-hoc) and `suite`
//! (canonical set) subcommands. Each run writes a CSV trace plus a
//! `<name>.meta.json` sidecar the report generator uses for provenance.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc};
use mmc_core::transforms::{Abc, Dq};
use mmc_core::tuning::current_pi_gains;
use mmc_hal::{BusVoltageSense, CurrentSense, PwmOutput};
use mmc_sim::analysis::{step_metrics, StepMetrics};
use mmc_sim::{
    BemfShape, Mode, PmsmParams, SamplePoint, SensorlessRunCfg, SensorlessSim, SixStepCfg,
    SixStepSim, TruthAngle, VirtualMotor,
};

/// Parameters of a q-axis current-step run.
#[derive(Copy, Clone, Debug)]
pub struct StepConfig {
    /// Simulated duration [s].
    pub duration: f32,
    /// Control-loop rate [Hz].
    pub ctrl_freq: f32,
    /// Current-loop design bandwidth [rad/s].
    pub bandwidth: f32,
    /// q-axis current step amplitude [A].
    pub iq: f32,
    /// DC bus voltage [V].
    pub vbus: f32,
    /// External load torque [N·m].
    pub load: f32,
    /// Hold the rotor at standstill (locked-rotor bench).
    pub locked: bool,
}

impl Default for StepConfig {
    fn default() -> Self {
        Self {
            duration: 0.05,
            ctrl_freq: 10_000.0,
            bandwidth: 2000.0,
            iq: 1.0,
            vbus: 24.0,
            load: 0.0,
            locked: false,
        }
    }
}

/// Identity attached to a run for the dashboard.
pub struct RunSpec<'a> {
    pub title: &'a str,
    pub description: &'a str,
    /// Dashboard sort key within a group (suite runs count up from 0).
    pub order: u32,
    pub cfg: StepConfig,
}

/// What a run produced, for the caller to print.
pub struct RunResult {
    pub samples: usize,
    pub metrics: Option<StepMetrics>,
    /// Final mechanical speed [rad/s]; `None` for locked-rotor runs.
    pub final_speed: Option<f32>,
    pub notes: Vec<String>,
}

/// Run a q-axis current step and write `out` (CSV) + `out`'s meta sidecar.
pub fn run_current_step(spec: &RunSpec, out: &Path) -> std::io::Result<RunResult> {
    let cfg = &spec.cfg;
    let params = PmsmParams::small_bldc();
    let ctrl_dt = 1.0 / cfg.ctrl_freq;

    let mut rig = VirtualMotor::new(params, cfg.vbus);
    rig.motor.locked = cfg.locked;
    rig.load_torque = cfg.load;
    rig.enable();

    // Params come from the sim's ground truth here; on hardware the profiler
    // (MS6) supplies them.
    let mut foc = Foc::with_feedforward(
        current_pi_gains(params.rs, params.lq, cfg.bandwidth),
        Decoupling {
            ld: params.ld,
            lq: params.lq,
            flux: params.flux,
        },
    );
    let mut angle = TruthAngle::default();

    // Step after 10% of the trace so the plot shows the quiescent state.
    let step_time = cfg.duration * 0.1;
    let steps = (cfg.duration / ctrl_dt) as usize;

    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut writer = BufWriter::new(File::create(out)?);
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
            Dq { d: 0.0, q: cfg.iq }
        } else {
            Dq::default()
        };

        angle.sync(&rig.motor);
        let [ia, ib, ic] = rig.phase_currents();
        let vbus = rig.vbus();
        let out_step = foc.step(
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
        rig.set_duties(out_step.duties);
        rig.advance(ctrl_dt);

        writeln!(
            writer,
            "{t},{},{},{},{},{},{},{},{},{},{}",
            i_ref.q,
            out_step.i_dq.d,
            out_step.i_dq.q,
            out_step.v_dq.d,
            out_step.v_dq.q,
            out_step.duties[0],
            out_step.duties[1],
            out_step.duties[2],
            rig.motor.omega_m,
            rig.motor.theta_e(),
        )?;

        if t >= step_time {
            t_trace.push(t - step_time);
            iq_trace.push(out_step.i_dq.q);
        }
        v_mag_end = (out_step.v_dq.d * out_step.v_dq.d + out_step.v_dq.q * out_step.v_dq.q).sqrt();
    }
    writer.flush()?;

    let voltage_limited = v_mag_end >= 0.95 * cfg.vbus / 3.0f32.sqrt();
    let mut notes = Vec::new();
    if voltage_limited {
        notes.push(
            "Drive ended voltage-limited (back-EMF ≈ bus voltage): the current reference \
             is unreachable there — expected without field weakening."
                .to_string(),
        );
    }

    let result = RunResult {
        samples: steps,
        metrics: step_metrics(&t_trace, &iq_trace, cfg.iq),
        final_speed: (!cfg.locked).then_some(rig.motor.omega_m),
        notes,
    };
    write_meta(spec, out, &result)?;
    Ok(result)
}

/// Parameters of a sensorless startup + speed run (MS4).
#[derive(Copy, Clone, Debug)]
pub struct SensorlessConfig {
    pub duration: f32,
    /// Speed target [rad/s electrical].
    pub omega_e: f32,
    /// Steady load torque [N·m].
    pub load: f32,
    /// Additional load stepped in at 60% of the run [N·m].
    pub load_step: f32,
}

impl Default for SensorlessConfig {
    fn default() -> Self {
        Self {
            duration: 2.0,
            omega_e: 800.0,
            load: 0.005,
            load_step: 0.0,
        }
    }
}

/// Identity + config for a sensorless run.
pub struct SensorlessSpec<'a> {
    pub title: &'a str,
    pub description: &'a str,
    pub order: u32,
    pub cfg: SensorlessConfig,
}

/// Run the full sensorless stack (I-f startup → observer handoff → speed
/// loop) against the virtual motor and record every channel.
pub fn run_sensorless(spec: &SensorlessSpec, out: &Path) -> std::io::Result<RunResult> {
    let cfg = &spec.cfg;
    let run_cfg = SensorlessRunCfg::small_bldc(cfg.omega_e);
    let mut sim = SensorlessSim::new(run_cfg);
    sim.set_load(cfg.load);
    let steps = (cfg.duration * run_cfg.ctrl_freq) as usize;
    let vbus = run_cfg.vbus;

    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut writer = BufWriter::new(File::create(out)?);
    writeln!(
        writer,
        "t,iq_ref,i_d,i_q,v_d,v_q,omega_m,theta_e,vbus,state,theta_est,omega_est,theta_err"
    )?;

    let mut handoff_t = f32::NAN;
    let mut err_max = 0.0f32;
    let mut err_sq = 0.0f64;
    let mut err_n = 0u32;
    let mut last_omega_est = 0.0;
    for _ in 0..steps {
        let s = sim.step();
        if cfg.load_step != 0.0 && s.t >= 0.6 * cfg.duration {
            sim.set_load(cfg.load + cfg.load_step);
        }
        let phase_code = match s.phase {
            mmc_core::sensorless::Phase::Ramp => 6.0,
            mmc_core::sensorless::Phase::Blend => 7.0,
            mmc_core::sensorless::Phase::Closed => 1.0,
        };
        if handoff_t.is_nan() && phase_code == 1.0 {
            handoff_t = s.t;
        }
        if !handoff_t.is_nan() && s.t > handoff_t + 0.1 {
            let e = s.theta_err.abs();
            err_max = err_max.max(e);
            err_sq += (e as f64) * (e as f64);
            err_n += 1;
        }
        last_omega_est = s.omega_est;
        writeln!(
            writer,
            "{},{},{},{},{},{},{},{},{vbus},{phase_code},{},{},{}",
            s.t,
            s.iq_ref,
            s.out.i_dq.d,
            s.out.i_dq.q,
            s.out.v_dq.d,
            s.out.v_dq.q,
            s.omega_m_true,
            s.theta_e_true,
            s.theta_est,
            s.omega_est,
            s.theta_err,
        )?;
    }
    writer.flush()?;

    let err_rms = (err_sq / err_n.max(1) as f64).sqrt();
    let notes = vec![format!(
        "Sensorless: handoff at {handoff_t:.3} s; post-handoff angle error \
         max {err_max:.3} rad / rms {err_rms:.3} rad; final speed estimate \
         {last_omega_est:.0} rad/s elec (target {}).",
        cfg.omega_e
    )];

    let result = RunResult {
        samples: steps,
        metrics: None,
        final_speed: Some(sim.rig.motor.omega_m),
        notes,
    };
    write_sensorless_meta(spec, out, &result)?;
    Ok(result)
}

fn write_sensorless_meta(
    spec: &SensorlessSpec,
    csv_path: &Path,
    result: &RunResult,
) -> std::io::Result<()> {
    let cfg = &spec.cfg;
    let meta = serde_json::json!({
        "title": spec.title,
        "description": spec.description,
        "order": spec.order,
        "command": format!(
            "mmc-host sim --scenario sensorless-speed --duration {} --omega-e {} --load {}{}",
            cfg.duration, cfg.omega_e, cfg.load,
            if cfg.load_step != 0.0 { format!(" --load-step {}", cfg.load_step) } else { String::new() },
        ),
        "unix_time": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
        "params": {
            "duration_s": cfg.duration,
            "omega_e_rad_s": cfg.omega_e,
            "load_Nm": cfg.load,
            "load_step_Nm": cfg.load_step,
        },
        "notes": result.notes,
    });
    std::fs::write(
        csv_path.with_extension("meta.json"),
        serde_json::to_string_pretty(&meta)?,
    )
}

/// Sidecar with provenance and context for the dashboard. Metrics are *not*
/// stored — the report recomputes them from the CSV so hand-captured traces
/// get them too.
fn write_meta(spec: &RunSpec, csv_path: &Path, result: &RunResult) -> std::io::Result<()> {
    let cfg = &spec.cfg;
    let meta = serde_json::json!({
        "title": spec.title,
        "description": spec.description,
        "order": spec.order,
        "command": format!(
            "mmc-host sim --scenario current-step --duration {} --bandwidth {} --iq {} --vbus {}{}{}",
            cfg.duration, cfg.bandwidth, cfg.iq, cfg.vbus,
            if cfg.load != 0.0 { format!(" --load {}", cfg.load) } else { String::new() },
            if cfg.locked { " --locked" } else { "" },
        ),
        "unix_time": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
        "params": {
            "duration_s": cfg.duration,
            "ctrl_freq_hz": cfg.ctrl_freq,
            "bandwidth_rad_s": cfg.bandwidth,
            "iq_A": cfg.iq,
            "vbus_V": cfg.vbus,
            "load_Nm": cfg.load,
            "locked": cfg.locked,
        },
        "notes": result.notes,
    });
    std::fs::write(
        csv_path.with_extension("meta.json"),
        serde_json::to_string_pretty(&meta)?,
    )
}

/// Print the human-readable summary of a run (used by `sim`, condensed by `suite`).
pub fn print_summary(spec: &RunSpec, out: &Path, r: &RunResult, verbose: bool) {
    if verbose {
        println!("wrote {} samples to {}", r.samples, out.display());
    }
    match &r.metrics {
        Some(m) => {
            if verbose {
                println!(
                    "i_q step response ({} rad/s design bandwidth):",
                    spec.cfg.bandwidth
                );
                println!(
                    "  rise time (10-90%):  {:.3} ms  (ideal {:.3} ms)",
                    m.rise_time * 1e3,
                    (9.0f32).ln() / spec.cfg.bandwidth * 1e3
                );
                println!("  overshoot:           {:.1} %", m.overshoot * 100.0);
                println!(
                    "  steady-state error:  {:.2} %",
                    m.steady_state_error * 100.0
                );
            } else {
                println!(
                    "  {:24} rise {:.3} ms, overshoot {:.1} %, sse {:.2} % -> {}",
                    spec.title,
                    m.rise_time * 1e3,
                    m.overshoot * 100.0,
                    m.steady_state_error * 100.0,
                    out.display()
                );
            }
        }
        None => println!(
            "  {:24} i_q never reached the step thresholds — check gains/limits",
            spec.title
        ),
    }
    if verbose {
        for note in &r.notes {
            println!("note: {note}");
        }
        if let Some(w) = r.final_speed {
            println!(
                "final speed: {:.0} rad/s mech ({:.0} rpm)",
                w,
                w * 60.0 / (2.0 * std::f32::consts::PI)
            );
        }
    }
}

// ---------------------------------------------------------------- six-step

/// Six-step run configuration.
pub struct SixStepConfig {
    pub duration: f32,
    /// Speed target [rad/s electrical].
    pub omega_e: f32,
    /// Steady load torque [N·m].
    pub load: f32,
    /// Additional load stepped in at 60% of the run [N·m].
    pub load_step: f32,
    /// Trapezoidal (the machine six-step is designed for) or sinusoidal
    /// (what the bench motors actually are).
    pub trapezoidal: bool,
    /// Sample the idle phase during the PWM on-time. False reproduces the
    /// freewheel sample point the bench measured in MS8 step 1.
    pub on_time: bool,
    /// Model a sense network that cannot read below ground.
    pub clamp: bool,
}

impl Default for SixStepConfig {
    fn default() -> Self {
        Self {
            duration: 2.5,
            omega_e: 600.0,
            load: 0.0,
            load_step: 0.0,
            trapezoidal: true,
            on_time: true,
            clamp: false,
        }
    }
}

pub struct SixStepSpec<'a> {
    pub title: &'a str,
    pub description: &'a str,
    pub order: u32,
    pub cfg: SixStepConfig,
}

/// Run six-step commutation (forced ramp → back-EMF zero-cross sensing →
/// duty speed loop) against the phase-domain motor and record every channel.
pub fn run_sixstep(spec: &SixStepSpec, out: &Path) -> std::io::Result<RunResult> {
    let cfg = &spec.cfg;
    let run_cfg = SixStepCfg {
        shape: if cfg.trapezoidal {
            BemfShape::Trapezoidal
        } else {
            BemfShape::Sinusoidal
        },
        sample_at: if cfg.on_time {
            SamplePoint::OnTime
        } else {
            SamplePoint::Freewheel
        },
        clamp_negative: cfg.clamp,
        ..SixStepCfg::bench(cfg.omega_e)
    };
    let mut sim = SixStepSim::new(run_cfg);
    sim.set_load(cfg.load);
    let steps = (cfg.duration * run_cfg.ctrl_freq) as usize;
    let vbus = run_cfg.vbus;

    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut writer = BufWriter::new(File::create(out)?);
    writeln!(
        writer,
        "t,sector,sector_true,sector_err,duty,i_line,v_float,v_ref,omega_e_meas,omega_e_true,theta_e,vbus,state"
    )?;

    let mut lock_t = f32::NAN;
    let mut worst_err = 0i32;
    let mut speed_err_sum = 0.0f64;
    let mut speed_err_n = 0u32;
    let mut tracked = 0u32;
    let mut counted = 0u32;
    let mut last = None;
    for _ in 0..steps {
        let s = sim.step();
        if cfg.load_step != 0.0 && s.t >= 0.6 * cfg.duration {
            sim.set_load(cfg.load + cfg.load_step);
        }
        if lock_t.is_nan() && s.locked {
            lock_t = s.t;
        }
        // Judge only the settled second half, as the tests do.
        if s.t > cfg.duration * 0.5 {
            worst_err = worst_err.max(s.sector_err.abs());
            counted += 1;
            if s.sector_err.abs() <= 1 {
                tracked += 1;
            }
            if s.omega_e_true.abs() > 1.0 {
                speed_err_sum += ((s.omega_zc - s.omega_e_true) / s.omega_e_true).abs() as f64;
                speed_err_n += 1;
            }
        }
        let state = match s.mode {
            Mode::Ramp => 6.0,
            Mode::Sensing => 1.0,
            Mode::Lost => 8.0,
        };
        writeln!(
            writer,
            "{},{},{},{},{},{},{},{},{},{},{},{vbus},{state}",
            s.t,
            s.sector,
            s.sector_true,
            s.sector_err,
            s.duty,
            s.i_line,
            s.v_float,
            s.v_ref,
            s.omega_zc,
            s.omega_e_true,
            s.theta_e_true,
        )?;
        last = Some(s);
    }
    writer.flush()?;

    let last = last.expect("at least one step");
    let tracked_pct = 100.0 * tracked as f32 / counted.max(1) as f32;
    let speed_err_pct = 100.0 * (speed_err_sum / speed_err_n.max(1) as f64);
    let notes = vec![
        format!(
            "Six-step: lock at {lock_t:.3} s; commutation within {worst_err} sector(s) of truth over the settled half, tracking {tracked_pct:.1}% of ticks."
        ),
        format!(
            "Speed measured from the zero-cross interval differs from sim truth by {speed_err_pct:.2}% on average; final {:.0} rad/s elec (target {}).",
            last.omega_e_true, cfg.omega_e
        ),
    ];

    let result = RunResult {
        samples: steps,
        metrics: None,
        final_speed: Some(sim.motor.omega_m),
        notes,
    };
    write_sixstep_meta(spec, out, &result)?;
    Ok(result)
}

fn write_sixstep_meta(
    spec: &SixStepSpec,
    csv_path: &Path,
    result: &RunResult,
) -> std::io::Result<()> {
    let cfg = &spec.cfg;
    let meta = serde_json::json!({
        "title": spec.title,
        "description": spec.description,
        "order": spec.order,
        "command": format!(
            "mmc-host sim --scenario six-step --duration {} --omega-e {} --load {}{}{}{}",
            cfg.duration,
            cfg.omega_e,
            cfg.load,
            if cfg.load_step != 0.0 { format!(" --load-step {}", cfg.load_step) } else { String::new() },
            if cfg.trapezoidal { "" } else { " --sinusoidal" },
            if cfg.on_time { "" } else { " --freewheel-sample" },
        ),
        "unix_time": std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
        "samples": result.samples,
        "final_speed_rpm": result.final_speed.map(|w| w * 60.0 / core::f32::consts::TAU),
        "notes": result.notes,
    });
    let path = csv_path.with_extension("meta.json");
    std::fs::write(path, serde_json::to_string_pretty(&meta)?)?;
    Ok(())
}
