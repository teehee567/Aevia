#![no_std]
#![forbid(unsafe_code)]
//! Portable hardware interfaces and component drivers for Aevia boards.
//!
//! Constructors never configure hardware. Call each component's explicit setup
//! methods with settings validated for the fitted board. Drivers use blocking
//! embedded-hal buses; scheduling, bus locking and deadlines belong to the caller.
//! This library is intentionally independent of the existing device runtime.

pub use embedded_hal;
pub use embedded_io;
pub use embedded_storage;

#[cfg(test)]
extern crate std;

pub mod ap33772s;
pub mod board;
pub mod bq25622;
pub mod bq25798;
pub mod bus;
pub mod display;
pub mod esp32s31;
pub mod interfaces;
pub mod leds;
pub mod lp5813;
pub mod max16169;
pub mod max17048;
pub mod max17320;
pub mod memory;
pub mod regulators;
pub mod sch16t;
pub mod sdcard;
pub mod tca9536;
pub mod temperature;
pub mod um980;
