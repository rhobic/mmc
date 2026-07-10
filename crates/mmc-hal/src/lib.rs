//! Hardware abstraction for the modular motor controller.
//!
//! A deliberately small trait set sized to motor control rather than a
//! general-purpose HAL. Each trait is implemented twice: by the virtual motor
//! rig in `mmc-sim` and by real hardware backends (STM32G474 first). The
//! control code in `mmc-core` is pure and does not depend on these traits;
//! they are consumed by the runner layer that owns the control loop (the sim
//! loop on a PC, the ADC interrupt handler on hardware).
//!
//! Units are SI: amps, volts, radians.

#![no_std]

/// Three-phase PWM output stage (center-aligned on real hardware).
pub trait PwmOutput {
    /// Latch per-phase duty cycles in `[0, 1]`, applied at the next PWM period.
    fn set_duties(&mut self, duties: [f32; 3]);
    /// Enable the gate drivers / power stage.
    fn enable(&mut self);
    /// Disable the power stage (outputs high-impedance).
    fn disable(&mut self);
}

/// Phase current measurement, sampled synchronously with the PWM period
/// (center-aligned trigger on real hardware).
pub trait CurrentSense {
    /// Instantaneous phase currents `[i_a, i_b, i_c]` in amps.
    /// Positive current flows into the motor terminal.
    fn phase_currents(&mut self) -> [f32; 3];
}

/// DC bus voltage measurement.
pub trait BusVoltageSense {
    /// Bus voltage in volts.
    fn vbus(&mut self) -> f32;
}

/// Mechanical rotor position sensor (encoder, magnetic sensor, ...).
///
/// Not required for the sensorless configurations; when present it backs the
/// encoder `AngleEstimator` implementation in `mmc-core`.
pub trait PositionSensor {
    /// Mechanical rotor angle in radians. Wrapping/multi-turn semantics are
    /// implementation-defined until the position-control milestone pins them.
    fn mechanical_angle(&mut self) -> f32;
}
