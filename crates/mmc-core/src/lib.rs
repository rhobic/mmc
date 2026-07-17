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

pub mod angle;
pub mod current_loop;
pub mod foc;
pub mod math;
pub mod observer;
pub mod pi;
pub mod probe;
pub mod sensorless;
pub mod svpwm;
pub mod transforms;
pub mod tuning;
