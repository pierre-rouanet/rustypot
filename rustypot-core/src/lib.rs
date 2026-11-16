//! Dynamixel (and Feetech) protocol packets, without a heap and without IO.
//!
//! The bytes of the bus and nothing else: encoding instructions and status packets into a
//! buffer you own, decoding them (whole, or a byte at a time as they arrive), the checks
//! of both protocols, and the device side of Protocol 2.0 in [`device`]. Moving the bytes
//! is up to you, so the same code runs on a host, in an async executor or in a
//! microcontroller's main loop.
//!
//! [`rustypot`](https://docs.rs/rustypot) builds its controllers on it.
//!
//! * [`v1`]: Protocol 1.0 (Dynamixel AX/MX/XL320's predecessor, Feetech STS/SCS).
//! * [`v2`]: Protocol 2.0, with Fast Sync Read for both sides of the bus.
//! * [`device`]: answer a Protocol 2.0 controller as a device would: a sensor board that
//!   sits on a servo bus, say.

#![no_std]

#[cfg(test)]
#[macro_use]
extern crate std;

pub mod crc;
pub mod device;
mod error;
pub mod v1;
pub mod v2;

pub use error::Error;
