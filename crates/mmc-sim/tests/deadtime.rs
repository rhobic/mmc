//! Dead-time voltage error and its compensation, against the bench rig's
//! motor — the mechanism proposed for the sensorless low-speed floor.

use mmc_core::inverter::DeadtimeModel;
use mmc_core::sensorless::Phase;
use mmc_sim::{SensorlessRunCfg, SensorlessSim};

struct Run {
    reached_closed: bool,
    theta_err_max: f32,
    theta_err_rms: f32,
    omega_final: f32,
    iq_final: f32,
    /// Mean observer flux over the scoring window, as a fraction of the true
    /// magnet flux. 1.0 is honest; the stall detector trips below 0.35.
    flux_frac: f32,
}

/// Settle the drive at `cfg.omega_ref` and score the observer over the last
/// stretch of the run, after any handoff transient has decayed.
fn run(cfg: SensorlessRunCfg, duration: f32) -> Run {
    let mut sim = SensorlessSim::new(cfg);
    let dt = 1.0 / cfg.ctrl_freq;
    let steps = (duration / dt) as usize;
    let score_from = 0.6 * duration;

    let mut reached = false;
    let mut err_max = 0.0f32;
    let mut err_sq = 0.0f64;
    let mut flux_sum = 0.0f64;
    let mut n = 0u32;
    let mut last = None;
    for _ in 0..steps {
        let s = sim.step();
        reached |= s.phase == Phase::Closed;
        if s.t >= score_from {
            let e = s.theta_err.abs();
            err_max = err_max.max(e);
            err_sq += (e as f64) * (e as f64);
            flux_sum += s.flux_mag as f64;
            n += 1;
        }
        last = Some(s);
    }
    let last = last.expect("ran");
    Run {
        reached_closed: reached,
        theta_err_max: err_max,
        theta_err_rms: (err_sq / n.max(1) as f64).sqrt() as f32,
        omega_final: last.omega_m_true * cfg.params.pole_pairs as f32,
        iq_final: last.out.i_dq.q,
        flux_frac: (flux_sum / n.max(1) as f64) as f32 / cfg.params.flux,
    }
}

fn case(omega: f32, err: DeadtimeModel, comp: Option<DeadtimeModel>) -> Run {
    let mut cfg = SensorlessRunCfg::bench_g474(omega);
    cfg.seq.omega_handoff = omega.min(150.0);
    cfg.inverter_error = err;
    cfg.deadtime_comp = comp;
    run(cfg, 1.2)
}

/// The observer's flux magnitude is the drive's health indicator — the stall
/// detector trips on it — so it has to mean the same thing at every speed.
/// The leaky integrator costs `|ω|/√(ω²+leak²)` of magnitude, 11% at
/// 40 rad/s el with the default 20 rad/s leak, and `FluxObserver::flux_mag`
/// divides it back out. Without that correction this test reads 0.89 at the
/// bottom of the range: a healthy drive looking less healthy the slower it
/// runs, which is backwards for a low-speed stall detector.
#[test]
fn flux_magnitude_is_honest_across_speed() {
    for &omega in &[40.0f32, 60.0, 100.0, 300.0, 600.0] {
        let r = case(omega, DeadtimeModel::default(), None);
        assert!(r.reached_closed, "omega {omega}: never closed the loop");
        assert!(
            (r.flux_frac - 1.0).abs() < 0.02,
            "omega {omega}: observer flux {:.3}×ψ",
            r.flux_frac
        );
    }
}

/// Dead-time error is *in phase with the current*, so on this rig it is an
/// apparent resistance (`v_dead/i_thresh` = 0.24 Ω on top of a fitted
/// 0.885 Ω). Integrating a q-axis error gives a flux perturbation along +d —
/// parallel to the rotor flux — so it inflates the magnitude as `ΔR·|i|/ω`
/// and barely touches the angle.
///
/// Both halves of that matter. The angle staying good is why sensorless FOC
/// works as well as it does on an uncompensated bridge. The magnitude going
/// wrong is a real hazard: at 40 rad/s el the flux reads 1.55×ψ, so a stall
/// would have to drag it through a 1.2×ψ offset before the 0.35×ψ detector
/// noticed. The detector goes blind exactly where stalls happen.
#[test]
fn deadtime_inflates_flux_but_spares_the_angle() {
    let e = DeadtimeModel::bench_g474_estimate();
    let ideal = case(40.0, DeadtimeModel::default(), None);
    let dirty = case(40.0, e, None);

    assert!(
        dirty.flux_frac > 1.4,
        "expected an inflated flux estimate, got {:.3}×ψ",
        dirty.flux_frac
    );
    // The bias falls as 1/ω, so it must be far smaller high up.
    let dirty_fast = case(600.0, e, None);
    assert!(
        dirty_fast.flux_frac < 1.1,
        "bias should decay with speed, got {:.3}×ψ at 600",
        dirty_fast.flux_frac
    );
    // ...while the angle barely moves. Loose bound: the claim is "same order",
    // not "identical".
    assert!(
        dirty.theta_err_rms < 2.0 * ideal.theta_err_rms.max(0.005),
        "angle rms {:.4} vs ideal {:.4}",
        dirty.theta_err_rms,
        ideal.theta_err_rms
    );
}

/// Feeding the same model forward cancels it exactly — the simulator's
/// inverter subtracts `DeadtimeModel::error_ab` and `Foc` adds it back, so a
/// correctly calibrated `v_dead` returns the ideal-bridge result.
#[test]
fn compensation_restores_the_ideal_bridge() {
    let e = DeadtimeModel::bench_g474_estimate();
    for &omega in &[40.0f32, 100.0, 600.0] {
        let ideal = case(omega, DeadtimeModel::default(), None);
        let fixed = case(omega, e, Some(e));
        assert!(
            (fixed.flux_frac - ideal.flux_frac).abs() < 0.01,
            "omega {omega}: compensated {:.3} vs ideal {:.3}",
            fixed.flux_frac,
            ideal.flux_frac
        );
    }
}

/// Half the right `v_dead` removes about half the bias and nothing worse —
/// the property that makes it safe to ship a compensation fitted from an
/// imperfect measurement. (Over-compensation is the dangerous direction; it
/// is not reachable from a fit that under-reads, which is what a
/// `sign`-cancelling probe does.)
#[test]
fn undercalibrated_compensation_degrades_gracefully() {
    let e = DeadtimeModel::bench_g474_estimate();
    let none = case(40.0, e, None);
    let half = case(
        40.0,
        e,
        Some(DeadtimeModel {
            v_dead: e.v_dead * 0.5,
            ..e
        }),
    );
    let ideal = case(40.0, DeadtimeModel::default(), None);
    let bias = |r: &Run| r.flux_frac - ideal.flux_frac;
    assert!(
        bias(&half) < bias(&none) && bias(&half) > 0.0,
        "half compensation should land between: none {:.3}, half {:.3}",
        bias(&none),
        bias(&half)
    );
}

#[test]
#[ignore = "exploration: prints the table the tests above encode"]
fn sweep() {
    let e = DeadtimeModel::bench_g474_estimate();
    println!(
        "{:>6}  {:<22} {:>7} {:>9} {:>9} {:>9} {:>8} {:>9}",
        "omega", "inverter", "closed", "err_max", "err_rms", "omega_f", "iq", "flux/psi"
    );
    for &omega in &[40.0f32, 60.0, 100.0, 150.0, 300.0, 600.0] {
        for (name, err, comp) in [
            ("ideal", DeadtimeModel::default(), None),
            ("deadtime, no comp", e, None),
            ("deadtime, compensated", e, Some(e)),
            (
                "deadtime, comp 50% low",
                e,
                Some(DeadtimeModel {
                    v_dead: e.v_dead * 0.5,
                    ..e
                }),
            ),
        ] {
            let r = case(omega, err, comp);
            println!(
                "{omega:>6.0}  {name:<22} {:>7} {:>9.4} {:>9.4} {:>9.1} {:>8.3} {:>9.3}",
                r.reached_closed,
                r.theta_err_max,
                r.theta_err_rms,
                r.omega_final,
                r.iq_final,
                r.flux_frac
            );
        }
    }
}
