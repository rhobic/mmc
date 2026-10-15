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

// ------------------------------------------------------------ motor boards

/// Fixed electrical facts about a motor board (MCU + inverter stage): what
/// the shared drive application needs to know to run on it, and nothing
/// about *how* the board produces its samples.
#[derive(Copy, Clone, Debug)]
pub struct BoardSpec {
    /// Control-loop rate [Hz]: how often [`MotorBoard::sample`] has new data
    /// and the drive's `tick` runs. Every time constant in the drive is
    /// derived from it, so a slower MCU just picks a lower rate.
    pub ctrl_hz: u32,
    /// Current-sense slope [V/A] at the ADC pin. Positive phase current
    /// (into the motor) *lowers* the amplifier output by this much per amp
    /// on a low-side-shunt board; the zero-current offset is calibrated at
    /// boot, so only the slope is specified.
    pub cur_volts_per_amp: f32,
    /// Software overcurrent trip [A] (two consecutive samples).
    pub i_trip: f32,
    /// Bus over-voltage trip, and the minimum bus to start a drive [V].
    pub vbus_max: f32,
    pub vbus_min_run: f32,
    /// Duty ceiling that keeps the low-side current-sampling window open.
    pub max_duty: f32,
    /// Series resistance of the conducting drive path (switches + shunt,
    /// duty-weighted) [Ω] — reported to the host so it can separate the
    /// winding from the bridge in the R/L probe.
    pub r_path: f32,
    /// Upper bound of the terminal-voltage sample offset
    /// (see [`MotorBoard::set_terminal_sample_offset`]), in the board's own
    /// units. 0 when the board cannot move its sample point.
    pub terminal_offset_max: f32,
    /// The board reads hall sensors ([`MotorBoard::hall_state`] returns
    /// `Some`), so the hall-sensored drive modes are available.
    pub has_halls: bool,
    /// How late a duty command takes effect, as a fraction of a control
    /// tick past the sample it was computed from: 0 if it applies at once,
    /// 0.5 if the ISR finishes after the PWM's next update point and the
    /// compare values load half a tick later. The flux observer integrates
    /// the voltage actually on the winding, so it needs this.
    pub pwm_latency: f32,
    /// Largest terminal voltage [`MotorBoard::terminal_volts`] can report
    /// (the BEMF divider's ADC full scale) [V]. A driven-high terminal on a
    /// bus near or above it reads clipped; the drive then substitutes the
    /// bus voltage. `f32::INFINITY` if it cannot clip.
    pub terminal_full_scale: f32,
}

/// One synchronous set of conversions, taken at the PWM instant where the
/// low-side shunts carry the phase currents.
#[derive(Copy, Clone, Debug, Default)]
pub struct Sample {
    /// Current-sense amplifier outputs [V], phases U, V, W.
    pub phase_volts: [f32; 3],
    /// DC bus voltage [V] (already scaled through the divider).
    pub vbus: f32,
}

/// A motor board as the drive application sees it: one power stage with
/// shunt current sensing, optional back-EMF dividers and hall sensors.
///
/// Implemented once per board, on top of the MCU's HAL. The drive never
/// touches a peripheral register — everything it does to the hardware goes
/// through here, which is what lets the same application run on a 170 MHz
/// and a 72 MHz MCU, or against the simulator.
pub trait MotorBoard {
    /// The latest synchronous sample. Called exactly once per control tick,
    /// from the control interrupt the board raises at [`BoardSpec::ctrl_hz`].
    fn sample(&mut self) -> Sample;

    /// Phase-to-ground terminal voltages [V] from the back-EMF dividers
    /// (U, V, W), sampled inside the PWM on-time. Boards without dividers
    /// return zeros, and six-step sensorless will not lock on them.
    fn terminal_volts(&mut self) -> [f32; 3];

    /// Move the terminal-voltage sample point (board units, bounded by
    /// [`BoardSpec::terminal_offset_max`]). Default: fixed point, ignored.
    fn set_terminal_sample_offset(&mut self, _offset: f32) {}

    /// Per-phase duty in `[0, 1]`, latched at the next PWM period. The
    /// board clamps to [`BoardSpec::max_duty`].
    fn set_duties(&mut self, duties: [f32; 3]);

    /// Enable phases by bitmask: bit 0 = U, 1 = V, 2 = W. A disabled phase
    /// is high-impedance (both switches open) — six-step floats one to read
    /// its back-EMF. `0` takes the whole stage off.
    fn set_phase_enables(&mut self, mask: u8);

    /// The gate driver has reported a fault (overcurrent, thermal, UVLO)
    /// since the last [`MotorBoard::clear_driver_fault`].
    ///
    /// Latched, not a level: a driver's fault line can pulse for a few
    /// microseconds per event (an L6230 overcurrent retry), far shorter than
    /// a control tick, so a board reads it through a latch (the timer's
    /// hardware break input) that also takes the PWM outputs to their safe
    /// state the moment the line drops. The drive polls this once per tick.
    fn driver_fault(&mut self) -> bool;

    /// Re-arm the fault latch and the PWM outputs it stopped. The drive
    /// calls it only when starting from Off, never while running, so a
    /// fault cannot be cleared under a live stage. If the driver still
    /// holds its fault line, the latch sets again and the next
    /// [`MotorBoard::driver_fault`] reads it. Default: no latch to clear.
    fn clear_driver_fault(&mut self) {}

    /// Raw hall-sensor state, bit 0 = H1, 1 = H2, 2 = H3; `None` when the
    /// board has no hall inputs.
    fn hall_state(&mut self) -> Option<u8> {
        None
    }

    /// A free-running cycle counter, for the ISR-cost diagnostic. Boards
    /// without one return 0.
    fn cycles(&self) -> u32 {
        0
    }
}
