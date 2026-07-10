//! Regression test of the dq current loop through the full core+sim stack:
//! the exact `mmc-core` FOC step, driven through the `mmc-hal` traits against
//! the virtual motor, must produce the designed first-order step response.

use mmc_core::angle::AngleEstimator;
use mmc_core::foc::{Decoupling, Foc};
use mmc_core::transforms::{Abc, Dq};
use mmc_core::tuning::current_pi_gains;
use mmc_hal::{BusVoltageSense, CurrentSense, PwmOutput};
use mmc_sim::analysis::step_metrics;
use mmc_sim::{PmsmParams, TruthAngle, VirtualMotor};

const CTRL_DT: f32 = 1e-4; // 10 kHz
const BANDWIDTH: f32 = 2000.0; // rad/s

fn run_step(locked: bool, i_ref: Dq, duration: f32) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let params = PmsmParams::small_bldc();
    let mut rig = VirtualMotor::new(params, 24.0);
    rig.motor.locked = locked;
    rig.enable();

    let mut foc = Foc::new(current_pi_gains(params.rs, params.lq, BANDWIDTH));
    let mut angle = TruthAngle::default();

    let steps = (duration / CTRL_DT) as usize;
    let mut t = Vec::with_capacity(steps);
    let mut iq = Vec::with_capacity(steps);
    let mut id = Vec::with_capacity(steps);

    for _ in 0..steps {
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
            CTRL_DT,
        );
        rig.set_duties(out.duties);
        rig.advance(CTRL_DT);

        t.push(rig.time() as f32);
        id.push(out.i_dq.d);
        iq.push(out.i_dq.q);
    }
    (t, id, iq)
}

/// Locked rotor, 1 A q-axis step: pole-zero cancellation at ω = 2000 rad/s
/// should give a near-first-order response with rise time ln(9)/ω ≈ 1.1 ms.
#[test]
fn locked_rotor_iq_step_matches_design_bandwidth() {
    let (t, id, iq) = run_step(true, Dq { d: 0.0, q: 1.0 }, 0.03);
    let m = step_metrics(&t, &iq, 1.0).expect("trace never reached thresholds");

    let expected_rise = (9.0f32).ln() / BANDWIDTH; // ≈ 1.1 ms
    assert!(
        (m.rise_time - expected_rise).abs() < 0.5e-3,
        "rise time {} s, expected ≈ {} s",
        m.rise_time,
        expected_rise
    );
    assert!(m.overshoot < 0.10, "overshoot {}", m.overshoot);
    assert!(
        m.steady_state_error < 0.02,
        "steady-state error {}",
        m.steady_state_error
    );

    // The d axis must stay quiet while q steps.
    let id_max = id.iter().fold(0.0f32, |a, &v| a.max(v.abs()));
    assert!(id_max < 0.05, "i_d peaked at {id_max} A");
}

/// Free rotor: torque from the current step must spin the motor up, and with
/// decoupling/back-EMF feedforward the current loop must keep tracking the
/// reference while accelerating (a plain PI provably sags by
/// (dE/dt)/(R·ω_bw) against the back-EMF ramp). Kept short (40 ms): with no
/// load the motor would eventually out-run the bus voltage and the current
/// would correctly collapse — field weakening is future work.
#[test]
fn free_rotor_tracks_current_while_accelerating() {
    let params = PmsmParams::small_bldc();
    let mut rig = VirtualMotor::new(params, 24.0);
    rig.enable();
    let mut foc = Foc::with_feedforward(
        current_pi_gains(params.rs, params.lq, BANDWIDTH),
        Decoupling {
            ld: params.ld,
            lq: params.lq,
            flux: params.flux,
        },
    );
    let mut angle = TruthAngle::default();

    let mut last_iq = 0.0;
    for _ in 0..400 {
        angle.sync(&rig.motor);
        let [ia, ib, ic] = rig.phase_currents();
        let out = foc.step(
            Abc {
                a: ia,
                b: ib,
                c: ic,
            },
            angle.electrical_angle(),
            angle.electrical_velocity(),
            Dq { d: 0.0, q: 0.5 },
            rig.vbus(),
            CTRL_DT,
        );
        rig.set_duties(out.duties);
        rig.advance(CTRL_DT);
        last_iq = out.i_dq.q;
    }

    assert!(
        rig.motor.omega_m > 100.0,
        "rotor should spin up, omega_m = {}",
        rig.motor.omega_m
    );
    assert!(
        (last_iq - 0.5).abs() < 0.02,
        "i_q should still track under back-EMF, i_q = {last_iq}"
    );
}
