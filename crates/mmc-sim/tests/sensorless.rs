//! MS4 regression: the sensorless stack (I-f startup → observer handoff →
//! speed loop) against sim ground truth, across speed and load.

use mmc_core::sensorless::Phase;
use mmc_sim::{SensorlessRunCfg, SensorlessSim};

struct RunStats {
    handoff_t: f32,
    /// Worst |wrap(θ̂ − θ)| from 100 ms after handoff completes to the end.
    theta_err_max: f32,
    theta_err_rms: f32,
    final_omega_e: f32,
    final_iq: f32,
}

fn run(cfg: SensorlessRunCfg, duration: f32, load: f32, load_step: Option<(f32, f32)>) -> RunStats {
    let mut sim = SensorlessSim::new(cfg);
    sim.set_load(load);
    let dt = 1.0 / cfg.ctrl_freq;
    let steps = (duration / dt) as usize;

    let mut handoff_t = f32::NAN;
    let mut err_max = 0.0f32;
    let mut err_sq_sum = 0.0f64;
    let mut err_n = 0u32;
    let mut last = None;
    for _ in 0..steps {
        let s = sim.step();
        if let Some((frac, nm)) = load_step {
            if s.t >= frac * duration {
                sim.set_load(nm);
            }
        }
        if handoff_t.is_nan() && s.phase == Phase::Closed {
            handoff_t = s.t;
        }
        if !handoff_t.is_nan() && s.t > handoff_t + 0.1 {
            let e = s.theta_err.abs();
            err_max = err_max.max(e);
            err_sq_sum += (e as f64) * (e as f64);
            err_n += 1;
        }
        last = Some(s);
    }
    let last = last.expect("ran");
    assert!(!handoff_t.is_nan(), "never reached closed loop");
    RunStats {
        handoff_t,
        theta_err_max: err_max,
        theta_err_rms: (err_sq_sum / err_n.max(1) as f64).sqrt() as f32,
        final_omega_e: last.omega_est,
        final_iq: last.out.i_dq.q,
    }
}

/// The canonical startup: spin up sensorless to 800 rad/s electrical and
/// take a load step mid-run. Angle stays honest, speed recovers.
#[test]
fn startup_handoff_and_load_step() {
    let cfg = SensorlessRunCfg::small_bldc(800.0);
    let stats = run(cfg, 2.0, 0.005, Some((0.6, 0.03)));

    assert!(
        stats.handoff_t < 0.6,
        "handoff too slow: {} s",
        stats.handoff_t
    );
    assert!(
        stats.theta_err_max < 0.25,
        "worst angle error {} rad",
        stats.theta_err_max
    );
    assert!(
        stats.theta_err_rms < 0.10,
        "angle error rms {} rad",
        stats.theta_err_rms
    );
    let speed_err = (stats.final_omega_e - 800.0).abs() / 800.0;
    assert!(speed_err < 0.03, "speed error {:.1} %", speed_err * 100.0);
    // The load step must be carried by real current.
    assert!(stats.final_iq > 0.3, "final i_q {}", stats.final_iq);
}

/// Speed × load sweep: the observer's angle error bound holds everywhere the
/// drive can actually operate on a 24 V bus. Ceiling: back-EMF meets the
/// voltage limit at ~1730 rad/s elec — above ~0.7× that (no field weakening
/// yet), a small overshoot erases the braking authority and the speed loop
/// cannot recover, so the sweep tops out at 1200.
#[test]
fn speed_load_sweep() {
    for &omega_ref in &[400.0f32, 800.0, 1200.0] {
        for &load in &[0.0f32, 0.02, 0.04] {
            let cfg = SensorlessRunCfg::small_bldc(omega_ref);
            let stats = run(cfg, 1.6, load, None);
            assert!(
                stats.theta_err_max < 0.3,
                "omega {omega_ref} load {load}: worst angle error {} rad",
                stats.theta_err_max
            );
            let speed_err = (stats.final_omega_e - omega_ref).abs() / omega_ref;
            assert!(
                speed_err < 0.05,
                "omega {omega_ref} load {load}: speed error {:.1} %",
                speed_err * 100.0
            );
        }
    }
}

/// Sensorless works in reverse too.
#[test]
fn negative_direction() {
    let mut cfg = SensorlessRunCfg::small_bldc(-800.0);
    cfg.seq.omega_handoff = -150.0;
    let stats = run(cfg, 1.6, 0.0, None);
    assert!(
        stats.theta_err_max < 0.3,
        "worst angle error {} rad",
        stats.theta_err_max
    );
    assert!(
        (stats.final_omega_e + 800.0).abs() / 800.0 < 0.05,
        "final speed {}",
        stats.final_omega_e
    );
}
