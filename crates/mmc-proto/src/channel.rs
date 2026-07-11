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

pub const COUNT: usize = 15;

/// Wire names, indexed by channel id; used as CSV headers by the host.
pub const NAMES: [&str; COUNT] = [
    "iq_ref", "i_d", "i_q", "v_d", "v_q", "duty_a", "duty_b", "duty_c", "omega_m", "theta_e",
    "vbus", "i_a", "i_b", "i_c", "state",
];

/// Selection mask with every defined channel enabled.
pub const ALL: u32 = (1 << COUNT as u32) - 1;

/// Build a selection mask from channel ids.
pub fn mask_of(ids: &[u8]) -> u32 {
    ids.iter().fold(0, |m, &id| m | 1u32 << id)
}
