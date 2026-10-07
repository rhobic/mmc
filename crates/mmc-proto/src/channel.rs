//! Telemetry channel registry. A channel's bit position in the selection mask
//! is stable protocol surface; append new channels, never renumber.
//!
//! Telemetry frames carry values for the channels set in their mask, packed in
//! ascending bit order — so `NAMES` doubles as the CSV column order on the
//! host side.

pub const IQ_REF: u8 = 0;
pub const I_D: u8 = 1;
pub const I_Q: u8 = 2;
pub const V_D: u8 = 3;
pub const V_Q: u8 = 4;
pub const DUTY_A: u8 = 5;
pub const DUTY_B: u8 = 6;
pub const DUTY_C: u8 = 7;
pub const OMEGA_M: u8 = 8;
pub const THETA_E: u8 = 9;
pub const VBUS: u8 = 10;
/// Raw phase currents (hardware shunt measurements).
pub const I_A: u8 = 11;
pub const I_B: u8 = 12;
pub const I_C: u8 = 13;
/// Drive state: 0 = off, 1 = running, 2 = overcurrent fault, 3 = gate-driver
/// fault, 4 = bus-voltage fault.
pub const STATE: u8 = 14;
/// Sensorless observer estimate of the electrical angle [rad].
pub const THETA_EST: u8 = 15;
/// Sensorless observer estimate of the electrical velocity [rad/s].
pub const OMEGA_EST: u8 = 16;
/// Estimate minus reference angle, wrapped [rad]. Reference = sim truth on
/// the simulator; on hardware the calibrated hall angle when the board has
/// halls (in every drive mode — so a sensorless run is scored against the
/// rotor, not against itself), else the forced/applied angle.
pub const THETA_ERR: u8 = 17;
/// Phase terminal voltages via the shield's BEMF divider network [V].
/// With the stage Hi-Z (coasting) these are the pure back-EMFs — a direct
/// flux measurement; while PWM runs they show the switched rail unless the
/// divider is filtered. The sim reports the model's EMF.
pub const VB_U: u8 = 18;
pub const VB_V: u8 = 19;
pub const VB_W: u8 = 20;
/// Active six-step commutation sector, 0..5 (NaN-free; 0 when not commutating).
/// Tells the host which phase is floating, so a BEMF trace can be split into
/// driven and sensed segments without re-deriving it from the angle.
pub const SECTOR: u8 = 21;
/// Raw hall-sensor state, bit 0 = H1 … bit 2 = H3 (0 when the board has no
/// halls). Six valid values; 0 or 7 means a sensor supply/wiring fault.
pub const HALL: u8 = 22;
/// Electrical speed from hall edge timing [rad/s], in the hall Gray-code
/// direction — independent of the drive's own angle, so it shows whether a
/// forced or sensorless drive actually has the rotor with it.
pub const OMEGA_HALL: u8 = 23;
/// Rotor position [rad, mechanical], unwrapped, from the hall angle,
/// relative to where the current drive started. 0 outside position mode.
pub const POS_M: u8 = 24;
/// Position-loop reference [rad, mechanical] (the trapezoidal profile, not
/// the final target).
pub const POS_REF: u8 = 25;

/// Online stator-resistance [Ω] and magnet-flux [Wb] estimates
/// (`mmc_core::estim`), referred to the commanded voltage like the
/// profiler's. 0 outside the closed-loop FOC modes.
pub const R_HAT: u8 = 26;
pub const PSI_HAT: u8 = 27;

pub const COUNT: usize = 28;

/// Wire names, indexed by channel id; used as CSV headers by the host.
pub const NAMES: [&str; COUNT] = [
    "iq_ref",
    "i_d",
    "i_q",
    "v_d",
    "v_q",
    "duty_a",
    "duty_b",
    "duty_c",
    "omega_m",
    "theta_e",
    "vbus",
    "i_a",
    "i_b",
    "i_c",
    "state",
    "theta_est",
    "omega_est",
    "theta_err",
    "vb_u",
    "vb_v",
    "vb_w",
    "sector",
    "hall",
    "omega_hall",
    "pos_m",
    "pos_ref",
    "r_hat",
    "psi_hat",
];

/// Selection mask with every defined channel enabled.
pub const ALL: u32 = (1 << COUNT as u32) - 1;

/// Build a selection mask from channel ids.
pub fn mask_of(ids: &[u8]) -> u32 {
    ids.iter().fold(0, |m, &id| m | 1u32 << id)
}
