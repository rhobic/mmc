//! Profiler orchestration (MS6 + saliency). `profile` runs the firmware test
//! sequences as named *stages* and captures whose traces `tools/profile.py`
//! (parameters) and `tools/saliency.py` (Ld/Lq verdict) fit; `apply` writes a
//! fitted profile back to the device over the protocol and verifies it.
//!
//! The division of labor is the plan's: firmware executes dumb sequences and
//! records raw samples; all fitting happens on the host.
//!
//! Each stage declares what it does and what it expects from the bench (all
//! of them work without mechanically locking the rotor), completed stages are
//! tracked in `<dir>/profile_state.json`, and `--only`/`--redo` rerun any
//! subset. Stage order is enforced: the spinning stages run before the
//! parking probes, because a parked rotor released into an I-f start sits on
//! the separatrix of the torque well and gets ejected (the MS6 lesson).

use std::io::Write as _;
use std::path::Path;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mmc_core::probe;
use mmc_proto::{param, test, DeviceKind, DriveMode, Message};

use crate::capture::{self, CaptureCfg};
use crate::link::Link;

const CTRL_FREQ: f32 = 20_000.0;

/// Series resistance of the drive path: gate-driver conducting switch
/// ≈ 0.5 Ω typ (R_DSon HS+LS = 1 Ω, see hw/README.md) + the 0.33 Ω
/// low-side shunt duty-weighted ≈ 0.32 Ω. The probes measure winding + this
/// path (which is what the control loop must use); subtracting it gives the
/// at-the-motor value a multimeter or datasheet quotes. A rig property, not a
/// motor one — see testresults/motor2-4pole/datasheet-comparison.md for the
/// derivation, closed against a known motor to ~±0.05 Ω (FET tolerance
/// dominates the residual).
pub const R_DRIVE_PATH: f32 = 0.85;

/// The drive-path resistance for a device kind: the sim's average-value
/// inverter has no series resistance, so its probes already read the winding.
pub fn r_drive_path(kind: DeviceKind) -> f32 {
    match kind {
        DeviceKind::Sim => 0.0,
        _ => R_DRIVE_PATH,
    }
}

/// Saliency-sweep square-wave levels [V]. The firmware additionally clamps
/// them below (0.75 · I_trip · R̂) using its live R parameter, so the plateau
/// current stays under the 1.5 A trips with margin.
const SAL_V_LOW: f32 = 0.3;
const SAL_V_HIGH: f32 = 0.9;

/// Hardware overcurrent trip [A], mirrored from the firmware. The dead-time
/// ladder sizes its top voltage against this — unlike the saliency sweep,
/// nothing on the device clamps a plain open-loop voltage command to a
/// current, so the ceiling has to be chosen here.
const I_TRIP_A: f32 = 1.5;

/// Fraction of the trip the dead-time ladder refuses to exceed. The rungs are
/// sized at 0.7·I_trip, so hitting this means the device's `R` parameter is
/// wrong for the connected motor, not that the ladder was aimed too high.
const VDEAD_I_CEILING: f32 = 0.85;

/// Safety margin on the next rung's predicted current. Linear extrapolation
/// from the previous rung under-predicts by the bridge error's share of the
/// command — ~17% at the top of a bench ladder — so round it up.
const VDEAD_PREDICT_MARGIN: f32 = 1.25;

/// Per-stage excitation, tunable per motor. The defaults are sized for the
/// original bench BLDC; a motor with more flux hits the voltage ceiling
/// (ω_max ≈ 0.7·(VBUS/√3)/ψ) far lower, and a heavier rotor needs more I-f
/// current to hold sync through the ramp — the sweep fit refuses slipped
/// points, so wrong values fail loudly, not silently.
#[derive(Clone)]
pub struct StageTuning {
    /// R/L probe (align, step) voltages.
    pub rl_volts: (f32, f32),
    /// Flux-sweep I-f operating points (amps, omega_e rad/s el).
    pub sweep: Vec<(f32, f32)>,
    /// Accel-run sensorless speed targets (start, step) [rad/s el].
    pub accel: (f32, f32),
    /// Accel-run startup/authority current [A] — raise for a high-drag or
    /// heavy motor that can't reach handoff on the default.
    pub accel_amps: f32,
}

impl Default for StageTuning {
    fn default() -> Self {
        Self {
            rl_volts: (0.5, 1.0),
            sweep: vec![
                (0.3, 150.0),
                (0.3, 300.0),
                (0.3, 450.0),
                (0.45, 150.0),
                (0.45, 450.0),
            ],
            accel: (300.0, 900.0),
            accel_amps: 0.5,
        }
    }
}

/// What a stage needs from the motor shaft. No stage needs a mechanical
/// clamp; the distinction is whether the shaft must be able to rotate.
#[derive(Copy, Clone, PartialEq)]
pub enum Rotor {
    /// The motor spins: decouple external loads, keep fingers clear.
    FreeSpinning,
    /// The rotor is magnetically parked/held; a clamped shaft is also fine.
    /// Expect a small twitch as the current vector engages.
    Parks,
}

impl Rotor {
    fn label(self) -> &'static str {
        match self {
            Rotor::FreeSpinning => "FREE-SPINNING",
            Rotor::Parks => "PARKS ROTOR  ",
        }
    }
}

pub struct StageDef {
    pub id: &'static str,
    pub title: &'static str,
    pub rotor: Rotor,
    /// What the stage measures and how.
    pub what: &'static str,
    /// What the operator must set up before it runs.
    pub needs: &'static str,
    /// Execution order — spinning stages first, parking probes last
    /// (separatrix lesson: a parked rotor ejects on the next I-f start).
    pub order: u32,
}

pub const STAGES: [StageDef; 5] = [
    StageDef {
        id: "sweep",
        title: "Rotating I-f flux sweep",
        rotor: Rotor::FreeSpinning,
        what: "5 I-f operating points; the hang-angle-aware fit extracts the \
               rotor flux linkage psi (-> kt)",
        needs: "shaft free to spin with no external load; motor may run up to \
                ~450 rad/s electrical",
        order: 1,
    },
    StageDef {
        id: "accel",
        title: "Sensorless accel run",
        rotor: Rotor::FreeSpinning,
        what: "sensorless 300->900 rad/s el speed step; inertia J from i_q \
               during the slew, friction from the steady i_q levels",
        needs: "shaft free to spin; requires working sensorless startup (R, \
                L, flux already roughly right)",
        order: 2,
    },
    StageDef {
        id: "rl",
        title: "R/L step probe",
        rotor: Rotor::Parks,
        what: "aligns the rotor, then d-axis square-wave; R from the level \
               change (differential), L from the folded exponential",
        needs: "nothing - the align phase parks the rotor magnetically; a \
                clamped shaft is equally fine",
        order: 10,
    },
    StageDef {
        id: "saliency",
        title: "Saliency (Ld/Lq) sweep",
        rotor: Rotor::Parks,
        what: "square-wave excitation along +/-paired electrical angles; the \
               cross-axis (i_q) transient exists only if Ld != Lq — the \
               zero-speed-sensorless feasibility gate",
        needs: "nothing - +/- angle pairing cancels net torque so a free \
                rotor only dithers a few electrical degrees; a clamp gives \
                the cleanest data",
        order: 11,
    },
    StageDef {
        id: "vdead",
        title: "Inverter dead-time voltage error",
        rotor: Rotor::Parks,
        what: "DC voltage ladder at omega_e = 0, up then back down; fits \
               v = R*i + v_dead*sign(i). The intercept is what the bridge \
               keeps -- dead time plus diode drops -- which the R/L probe \
               cancels by construction and so has never been measured",
        needs: "nothing - a DC vector parks the rotor; a clamped shaft is \
                ideal and makes the descending branch a clean thermal check",
        order: 12,
    },
];

pub struct ProfileOpts {
    /// Stage ids to run (None = all), executed in `StageDef::order`.
    pub only: Option<Vec<String>>,
    /// Rerun stages profile_state.json already marks completed.
    pub redo: bool,
    /// Skip the interactive bench-setup confirmation.
    pub yes: bool,
    /// Per-motor excitation (R/L volts, sweep points, accel targets).
    pub tuning: StageTuning,
}

/// `profile --list`: print the stage table and bench requirements.
pub fn list_stages() {
    println!("profiler stages (run order; --only <ids> selects a subset):");
    for s in &STAGES {
        println!("\n  {:10} [{}]  {}", s.id, s.rotor.label().trim(), s.title);
        println!("    does:  {}", s.what);
        println!("    needs: {}", s.needs);
    }
    println!(
        "\nordering note: spinning stages always run before the parking probes; \
         a probe-parked rotor released straight into an I-f start sits on the \
         torque-well separatrix and stalls (MS6 lesson).\n\
         fit afterwards:  python tools/profile.py <dir>   (parameters)\n\
         \x20                python tools/saliency.py <dir>  (Ld/Lq verdict)"
    );
}

/// Human translation of the firmware's NAK codes, so a refusal tells the
/// operator what to fix instead of printing a number.
pub fn nak_reason(err: u8) -> &'static str {
    match err {
        1 => "request unsupported or malformed (firmware too old for this probe?)",
        2 => "device busy or faulted - drive must be off; after a fault, send STOP/Off to re-arm",
        3 => "device still running its zero-current calibration - wait ~0.5 s after power-up",
        4 => "no finished recording to read back",
        _ => "unknown NAK code",
    }
}

pub fn run(link: &mut Link, dir: &Path, opts: &ProfileOpts) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;

    // Resolve the stage list: validate ids, then execute in declared order.
    let mut selected: Vec<&StageDef> = match &opts.only {
        None => STAGES.iter().collect(),
        Some(ids) => {
            let mut v = Vec::new();
            for id in ids {
                match STAGES.iter().find(|s| s.id == id) {
                    Some(s) => v.push(s),
                    None => {
                        let known: Vec<_> = STAGES.iter().map(|s| s.id).collect();
                        return Err(std::io::Error::other(format!(
                            "unknown stage `{id}` (known: {})",
                            known.join(", ")
                        )));
                    }
                }
            }
            v
        }
    };
    selected.sort_by_key(|s| s.order);

    let state = load_state(dir);
    let done: Vec<&StageDef> = selected
        .iter()
        .filter(|s| !opts.redo && stage_done(&state, s.id))
        .copied()
        .collect();
    selected.retain(|s| opts.redo || !stage_done(&state, s.id));
    for s in &done {
        println!(
            "profile: {} already completed - skipping (--redo reruns)",
            s.id
        );
    }
    if selected.is_empty() {
        println!("profile: nothing to do.");
        return Ok(());
    }

    // Announce the plan and the bench setup it expects, then confirm.
    println!("profile: {} stage(s) queued:", selected.len());
    for s in &selected {
        println!("  {:10} [{}] {}", s.id, s.rotor.label().trim(), s.title);
    }
    if selected.iter().any(|s| s.rotor == Rotor::FreeSpinning) {
        println!(
            "\nbench setup: the shaft MUST be free to spin (no clamp, no load) - \
             the spinning stages drive it up to ~900 rad/s electrical."
        );
    } else {
        println!(
            "\nbench setup: probes only - the rotor parks magnetically. A locked \
             or clamped shaft is fine (gives the cleanest saliency data)."
        );
    }
    if !opts.yes {
        print!("Press Enter to start, Ctrl-C to abort... ");
        std::io::stdout().flush()?;
        let mut line = String::new();
        std::io::stdin().read_line(&mut line)?;
    }

    let ids: Vec<&str> = selected.iter().map(|s| s.id).collect();
    run_stages(link, dir, &ids, &opts.tuning, &mut |line| {
        println!("{line}")
    })?;

    println!();
    println!(
        "profile: done. state -> {}",
        dir.join("profile_state.json").display()
    );
    println!(
        "  fit parameters:  python tools/profile.py {}",
        dir.display()
    );
    if selected.iter().any(|s| s.id == "saliency") {
        println!(
            "  saliency verdict: python tools/saliency.py {}",
            dir.display()
        );
    }
    println!(
        "  apply:           mmc-host apply --serial auto --baud 1000000 --profile {}/profile.json",
        dir.display()
    );
    Ok(())
}

/// The stage engine shared by the CLI (above, with its confirmation UI) and
/// the panel's profiler card: snapshot the device's parameter table into the
/// state file (the fits read `pole_pairs` from there), run the given stages
/// in `StageDef::order`, and mark each completed. `log` receives coarse
/// progress lines.
pub fn run_stages(
    link: &mut Link,
    dir: &Path,
    ids: &[&str],
    tuning: &StageTuning,
    log: &mut dyn FnMut(&str),
) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let mut stages: Vec<&StageDef> = STAGES.iter().filter(|s| ids.contains(&s.id)).collect();
    stages.sort_by_key(|s| s.order);
    if stages.is_empty() {
        return Err(std::io::Error::other("no valid stages requested"));
    }

    let mut state = load_state(dir);
    snapshot_device(link, &mut state, log);
    save_state(dir, &state)?;

    for (i, s) in stages.iter().enumerate() {
        log(&format!(
            "=== stage {}/{}: {} ({}) ===",
            i + 1,
            stages.len(),
            s.id,
            s.title
        ));
        let note = match s.id {
            "sweep" => stage_sweep(link, dir, &tuning.sweep, log)?,
            "accel" => stage_accel(link, dir, tuning.accel, tuning.accel_amps, log)?,
            "rl" => stage_rl(link, dir, tuning.rl_volts, log)?,
            "saliency" => stage_saliency(link, dir, log)?,
            "vdead" => stage_vdead(link, dir, log)?,
            _ => unreachable!(),
        };
        log(&format!("stage {}: {}", s.id, note));
        mark_done(&mut state, s.id, &note);
        if s.id == "accel" {
            // The fit needs the plateau targets this run actually used.
            state["stages"]["accel"]["targets"] =
                serde_json::json!([tuning.accel.0, tuning.accel.1]);
        }
        save_state(dir, &state)?;
    }
    Ok(())
}

/// Record the device's identity and full parameter table (as of capture
/// time) under `state["device"]` — `tools/profile.py` reads `pole_pairs`
/// from here so the kt/J fits track the connected motor.
fn snapshot_device(link: &mut Link, state: &mut serde_json::Value, log: &mut dyn FnMut(&str)) {
    let t = Duration::from_secs(2);
    let mut dev = serde_json::Map::new();
    if let Ok(Message::Info(info)) =
        link.request(&Message::GetInfo, |m| matches!(m, Message::Info(_)), t)
    {
        dev.insert("name".into(), info.name_str().into());
        dev.insert("fw".into(), info.fw_version.into());
        // The fits subtract this to report at-the-motor values (0 on the sim).
        dev.insert("r_path".into(), r_drive_path(info.kind).into());
    }
    let mut params = serde_json::Map::new();
    for (id, name) in param::NAMES.iter().enumerate() {
        match link.request(
            &Message::GetParam { id: id as u8 },
            |m| {
                matches!(m, Message::ParamValue { id: i, .. } if *i == id as u8)
                    || matches!(m, Message::Nak { of: 0x0A, .. })
            },
            t,
        ) {
            Ok(Message::ParamValue { value, .. }) => {
                params.insert((*name).into(), value.into());
            }
            _ => break, // no param table (old fw / bare sim): partial is fine
        }
    }
    if let Some(pp) = params.get("pole_pairs") {
        log(&format!(
            "device: pole_pairs = {pp} (used by the kt/J fits)"
        ));
    }
    dev.insert("params".into(), params.into());
    state["device"] = dev.into();
}

// ------------------------------------------------------------------- stages

/// Rotating I-f flux sweep (host-side captures, hang-angle fit).
fn stage_sweep(
    link: &mut Link,
    dir: &Path,
    sweep: &[(f32, f32)],
    log: &mut dyn FnMut(&str),
) -> std::io::Result<String> {
    for (i, &(amps, omega)) in sweep.iter().enumerate() {
        let name = format!("sweep_i{:03}_w{}", (amps * 100.0) as u32, omega as u32);
        let out = dir.join(format!("{name}.csv"));
        log(&format!(
            "  point {}/{}: I-f {amps} A @ {omega} rad/s el (~{:.0} s)",
            i + 1,
            sweep.len(),
            3.5 + omega / 500.0
        ));
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
    Ok(format!("{} operating points", sweep.len()))
}

/// Sensorless accel run: J from i_q during the reference slew, friction from
/// the steady i_q at two speeds.
fn stage_accel(
    link: &mut Link,
    dir: &Path,
    (lo, hi): (f32, f32),
    amps: f32,
    log: &mut dyn FnMut(&str),
) -> std::io::Result<String> {
    log(&format!(
        "  sensorless accel {lo} -> {hi} rad/s el at {amps} A (~8 s)"
    ));
    let title = format!("Accel run: sensorless {lo} -> {hi} rad/s el");
    capture::run(
        link,
        &CaptureCfg {
            divider: 20,
            mask: mmc_proto::channel::ALL,
            duration: 8.0,
            iq: None,
            drive: Some(DriveMode::Sensorless { amps, omega_e: lo }),
            drive_step: Some(DriveMode::Sensorless { amps, omega_e: hi }),
            title: &title,
            description: "Profiler inertia/friction input: i_q during the \
                          500 rad/s^2 reference slew vs the steady levels.",
            order: 10,
            command: "mmc-host profile".into(),
        },
        dir.join("accel.csv").as_path(),
    )?;
    Ok(format!("sensorless {lo}->{hi} rad/s el"))
}

/// Locked-rotor R/L step probe (on-device 20 kHz burst).
fn stage_rl(
    link: &mut Link,
    dir: &Path,
    (v_align, v_step): (f32, f32),
    log: &mut dyn FnMut(&str),
) -> std::io::Result<String> {
    log(&format!(
        "  R/L probe ({v_align} V align -> {v_step} V step, ~0.5 s; rotor parks)"
    ));
    let samples = run_probe(link, test::RL_STEP, v_align, v_step)?;
    let pairs = samples.len() / 2;
    if pairs < 1024 {
        return Err(std::io::Error::other(format!(
            "rl: only {pairs} pairs — the probe aborted early (overcurrent trip on a \
             low-R motor?). Retry with lower voltages, e.g. --rl-volts 0.2,0.4"
        )));
    }
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
        None,
    )?;
    println!("profile: {pairs} pairs -> {}", out.display());
    Ok(format!("{pairs} pairs"))
}

/// Fractions of the ladder's top voltage. Dense at the bottom because that
/// is where the knee lives: phase A leaves the zero-current band at
/// `i_d = i_thresh` and phases B/C — which carry `i_d/2` — only at `2·i_thresh`,
/// so the curve has two bends and both are needed to separate `v_dead` from
/// `i_thresh`.
const VDEAD_FRACTIONS: [f32; 10] = [0.06, 0.10, 0.15, 0.22, 0.30, 0.42, 0.55, 0.70, 0.85, 1.0];

/// Settled `i_d` from a capture the stage just wrote.
///
/// The ladder sizes itself from the device's `R` parameter, which on a fresh
/// motor is whatever the last profile left there — and a locked winding fed
/// from a too-high `R` estimate draws proportionally too much current with no
/// rotation to carry the heat away. So each rung is checked against what it
/// actually drew rather than against what it was predicted to draw.
fn settled_id(path: &Path) -> Option<f32> {
    let text = std::fs::read_to_string(path).ok()?;
    let mut lines = text.lines();
    let col = lines.next()?.split(',').position(|h| h.trim() == "i_d")?;
    let vals: Vec<f32> = lines
        .filter_map(|l| l.split(',').nth(col)?.trim().parse::<f32>().ok())
        .collect();
    if vals.len() < 20 {
        return None;
    }
    let tail = &vals[vals.len() * 3 / 5..];
    Some(tail.iter().sum::<f32>() / tail.len() as f32)
}

/// Dead-time voltage error: a DC ladder at ω_e = 0, ascending then
/// descending.
///
/// The descending branch is not redundancy — it is the thermal control. A
/// locked rotor dissipates `1.5·R·i_d²` with no rotation to carry heat away,
/// and copper gains 0.39%/°C, so a sweep that warms the winding fits a
/// resistance that was never true at any single point. If the two branches
/// disagree, the fit says so instead of quietly splitting the difference.
fn stage_vdead(link: &mut Link, dir: &Path, log: &mut dyn FnMut(&str)) -> std::io::Result<String> {
    // Size the ladder from the device's own R so it scales to whatever motor
    // is connected and still lands under the 1.5 A trip: 0.7·I_trip.
    let r = match link.request(
        &Message::GetParam { id: param::R },
        |m| matches!(m, Message::ParamValue { id, .. } if *id == param::R),
        Duration::from_secs(2),
    ) {
        Ok(Message::ParamValue { value, .. }) if value > 0.0 => value,
        _ => {
            return Err(std::io::Error::other(
                "vdead: cannot read the device's R parameter, which sizes the \
                 ladder — run `--only rl` first, or apply a profile",
            ))
        }
    };
    let v_top = 0.7 * I_TRIP_A * r;
    log(&format!(
        "  DC ladder at omega_e = 0: {} points to {v_top:.3} V (~{:.2} A peak, \
         R = {r:.3} Ω), up then down",
        2 * VDEAD_FRACTIONS.len(),
        v_top / r
    ));

    // Ascending then descending, each point its own capture so the drive
    // rests between them and the vector re-parks from the same angle.
    let legs: [(&str, Vec<f32>); 2] = [
        ("up", VDEAD_FRACTIONS.iter().map(|f| f * v_top).collect()),
        (
            "down",
            VDEAD_FRACTIONS.iter().rev().map(|f| f * v_top).collect(),
        ),
    ];
    let mut n = 0u32;
    let mut last: Option<(f32, f32)> = None; // (volts, settled amps)
    for (leg, volts) in &legs {
        for (i, &v) in volts.iter().enumerate() {
            // Look before stepping. `i/v` rises with `v` (the bridge's cut is
            // a smaller share of a bigger command), so scaling the previous
            // rung linearly *under*-predicts — by at most the error's share of
            // the command, hence the margin. The point is never to command the
            // rung that trips; the post-hoc check below is the backstop.
            if let Some((v_last, i_last)) = last {
                let predicted = i_last * (v / v_last) * VDEAD_PREDICT_MARGIN;
                if predicted > VDEAD_I_CEILING * I_TRIP_A {
                    log(&format!(
                        "  STOP before {v:.3} V: it would draw ~{predicted:.2} A, over the \
                         {:.2} A ceiling. The ladder is sized from the device's R = {r:.3} Ω, \
                         too high for this motor — profile and apply `rl` first.",
                        VDEAD_I_CEILING * I_TRIP_A
                    ));
                    return Ok(format!("{n} DC points (stopped short of {v:.3} V)"));
                }
            }
            let name = format!("vdead_{leg}_{:03}", (v * 1000.0) as u32);
            let out = dir.join(format!("{name}.csv"));
            log(&format!(
                "  {leg} {}/{}: {v:.3} V (~{:.2} A)",
                i + 1,
                volts.len(),
                v / r
            ));
            capture::run(
                link,
                &CaptureCfg {
                    divider: 20,
                    mask: mmc_proto::channel::ALL,
                    // 5 V/s slew + settle + ~0.7 s of averaging. Kept short:
                    // every millisecond here is heat into a stationary rotor.
                    duration: 1.2,
                    iq: None,
                    drive: Some(DriveMode::OpenLoopVoltage {
                        volts: v,
                        omega_e: 0.0,
                    }),
                    drive_step: None,
                    title: &format!("Dead-time ladder: {v:.3} V DC ({leg})"),
                    description: "Profiler dead-time input: locked-rotor DC \
                                  voltage at omega_e = 0. v = R*i + v_dead*sign(i), \
                                  so the intercept is the bridge's own drop.",
                    order: 20 + n,
                    command: "mmc-host profile".into(),
                },
                out.as_path(),
            )?;
            n += 1;

            // What it actually drew. Aborting keeps whatever rungs already
            // landed — a ladder that covers the knee is still fittable, and a
            // partial result beats a tripped bridge or a cooked winding.
            if let Some(i_meas) = settled_id(&out) {
                last = Some((v, i_meas.abs().max(1e-4)));
                if i_meas.abs() > VDEAD_I_CEILING * I_TRIP_A {
                    log(&format!(
                        "  ABORT: {i_meas:.2} A at {v:.3} V exceeds {:.2} A. The ladder \
                         is sized from the device's R = {r:.3} Ω, which is too high for \
                         this motor — profile and apply `rl` first.",
                        VDEAD_I_CEILING * I_TRIP_A
                    ));
                    return Ok(format!(
                        "{n} DC points (ABORTED at {v:.3} V / {i_meas:.2} A)"
                    ));
                }
            }
        }
    }
    Ok(format!("{n} DC points to {v_top:.3} V (up+down)"))
}

/// Saliency sweep: shared `mmc_core::probe` schedule, (i_d, i_q) pairs in
/// the excitation frame behind a self-describing header.
fn stage_saliency(
    link: &mut Link,
    dir: &Path,
    log: &mut dyn FnMut(&str),
) -> std::io::Result<String> {
    log(&format!(
        "  saliency sweep ({} angles, tau-adaptive, {SAL_V_LOW} -> {SAL_V_HIGH} V, ~0.5 s)",
        probe::SAL_SLOTS
    ));
    let samples = run_probe(link, test::L_THETA, SAL_V_LOW, SAL_V_HIGH)?;
    if samples.len() < probe::SAL_HDR {
        return Err(std::io::Error::other(format!(
            "saliency: burst too short ({} f32s) - probe aborted before the sweep started?",
            samples.len()
        )));
    }
    let hdr = &samples[..probe::SAL_HDR];
    // The device's actual schedule rides in the header (half-period is
    // τ-adaptive) — the CSV's derived columns must come from it, never from
    // this binary's compile-time constants.
    let half = hdr[4] as usize;
    let ctrl_freq = hdr[7];
    let pairs = &samples[probe::SAL_HDR..];
    let n = pairs.len() / 2;

    let out = dir.join("saliency.csv");
    let mut w = std::io::BufWriter::new(std::fs::File::create(&out)?);
    writeln!(w, "t,delta_e,v,i_d,i_q")?;
    for (k, p) in pairs.chunks_exact(2).enumerate() {
        let v = if probe::sal_level_is_high(k, half) {
            hdr[6]
        } else {
            hdr[5]
        };
        writeln!(
            w,
            "{},{},{},{},{}",
            k as f32 / ctrl_freq,
            probe::sal_angle(k, half),
            v,
            p[0],
            p[1]
        )?;
    }
    w.flush()?;
    // The full header rides in the meta sidecar so tools/saliency.py
    // hard-codes no schedule constants.
    let header = serde_json::json!({
        "kind": hdr[0], "slots": hdr[1], "cycles": hdr[2],
        "block_ticks": hdr[3], "half_ticks": hdr[4],
        "v_low": hdr[5], "v_high": hdr[6], "ctrl_freq": hdr[7],
    });
    write_meta(
        dir,
        "saliency",
        "Saliency (Ld/Lq) sweep",
        "Square-wave d-axis excitation along +/-paired electrical angles at \
         standstill, (i_d, i_q) in the excitation frame recorded on-device \
         per control tick. The i_q transient is a null channel: it exists \
         only if Ld != Lq. tools/saliency.py fits the ratio and gives the \
         zero-speed-sensorless verdict.",
        1,
        Some(header),
    )?;
    println!("profile: {n} pairs -> {}", out.display());
    Ok(format!("{n} pairs"))
}

// -------------------------------------------------------------- probe plumbing

/// Start a firmware test sequence and poll the burst buffer until the
/// recording is complete, keeping the deadman fed. Returns the raw f32s.
fn run_probe(link: &mut Link, kind: u8, a: f32, b: f32) -> std::io::Result<Vec<f32>> {
    let t = Duration::from_secs(2);
    let req = Message::RunTest { kind, a, b };
    match link.request(
        &req,
        |m| matches!(m, Message::Ack { of: 0x07 } | Message::Nak { of: 0x07, .. }),
        t,
    )? {
        Message::Ack { .. } => {}
        Message::Nak { err, .. } => {
            return Err(std::io::Error::other(format!(
                "device refused the probe: {}",
                nak_reason(err)
            )))
        }
        _ => unreachable!(),
    }

    // Poll for the data; each request doubles as a deadman keep-alive.
    loop {
        std::thread::sleep(Duration::from_millis(150));
        match link.request(
            &Message::ReadBurst { offset: 0 },
            |m| matches!(m, Message::BurstData(_) | Message::Nak { of: 0x08, .. }),
            t,
        )? {
            Message::BurstData(first) => return read_burst(link, &first),
            _ => continue, // still recording
        }
    }
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

// ------------------------------------------------------------------- state

fn state_path(dir: &Path) -> std::path::PathBuf {
    dir.join("profile_state.json")
}

fn load_state(dir: &Path) -> serde_json::Value {
    std::fs::read_to_string(state_path(dir))
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_else(|| serde_json::json!({ "version": 1, "stages": {} }))
}

fn stage_done(state: &serde_json::Value, id: &str) -> bool {
    state["stages"][id]["ok"].as_bool().unwrap_or(false)
}

fn mark_done(state: &mut serde_json::Value, id: &str, note: &str) {
    state["stages"][id] = serde_json::json!({
        "ok": true,
        "unix_time": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
        "note": note,
    });
}

fn save_state(dir: &Path, state: &serde_json::Value) -> std::io::Result<()> {
    std::fs::write(state_path(dir), serde_json::to_string_pretty(state)?)
}

// -------------------------------------------------------------------- apply

/// Persist the device's current parameter table to flash (survives power
/// cycle). Idempotent-ish: acks once written.
pub fn save_params(link: &mut Link) -> std::io::Result<()> {
    match link.request(
        &Message::SaveParams,
        |m| matches!(m, Message::Ack { of: 0x0B } | Message::Nak { of: 0x0B, .. }),
        Duration::from_secs(3),
    )? {
        Message::Ack { .. } => {
            println!("persist: parameters written to flash (restored at boot).");
            Ok(())
        }
        Message::Nak { err, .. } => Err(std::io::Error::other(format!(
            "device refused SaveParams: {}",
            nak_reason(err)
        ))),
        _ => unreachable!(),
    }
}

/// Apply a fitted profile (tools/profile.py JSON) and verify by read-back.
/// With `persist`, also writes the table to flash so it survives a reboot.
pub fn apply(link: &mut Link, profile: &Path, persist: bool) -> std::io::Result<()> {
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
            Message::Nak { err, .. } => {
                return Err(std::io::Error::other(format!(
                    "device refused {name} = {value}: {}",
                    nak_reason(err)
                )))
            }
            _ => unreachable!(),
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
    if persist {
        save_params(link)?;
        println!("apply: {applied} parameters set and persisted to flash.");
    } else {
        println!(
            "apply: {applied} parameters set — effective at next drive start \
             (RAM only; --persist writes them to flash)."
        );
    }
    Ok(())
}

fn write_meta(
    dir: &Path,
    stem: &str,
    title: &str,
    description: &str,
    order: u32,
    header: Option<serde_json::Value>,
) -> std::io::Result<()> {
    let mut meta = serde_json::json!({
        "title": title,
        "description": description,
        "order": order,
        "command": "mmc-host profile",
        "unix_time": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
        "notes": [],
    });
    if let Some(h) = header {
        meta["header"] = h;
    }
    std::fs::write(
        dir.join(format!("{stem}.meta.json")),
        serde_json::to_string_pretty(&meta)?,
    )
}
