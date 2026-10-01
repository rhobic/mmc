//! Portable motor-control core.
//!
//! Pure math over `f32`: no I/O, no allocation, no hardware knowledge. The
//! runner layer (simulator on a PC, ADC interrupt on an MCU) feeds
//! measurements in and carries PWM duties out, so this crate runs — and is
//! tested — identically on both.
//!
//! Conventions: SI units, angles in radians (electrical unless noted),
//! amplitude-invariant Clarke transform.

#![no_std]

// Always available: the arithmetic and building blocks both control
// methodologies are made of.
pub mod angle;
pub mod hall;
pub mod math;
pub mod pi;
pub mod transforms;
pub mod tuning;

// Field-oriented control and the sensorless stack built on it.
#[cfg(feature = "foc")]
pub mod current_loop;
#[cfg(feature = "foc")]
pub mod foc;
#[cfg(feature = "foc")]
pub mod inverter;
#[cfg(feature = "foc")]
pub mod observer;
#[cfg(feature = "foc")]
pub mod probe;
#[cfg(feature = "foc")]
pub mod sensorless;
#[cfg(feature = "foc")]
pub mod svpwm;

// Six-step trapezoidal commutation. Independent of `foc`: a low-resource
// target can build this alone, which is the point of the split.
#[cfg(feature = "sixstep")]
pub mod sixstep;
