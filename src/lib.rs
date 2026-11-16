//! A low-level communication library for servo (Dynamixel and Feetech motors).
//!
//! ## Feature Overview
//!
//! * Relies on [serialport] for serial communication, or on any link implementing [`Transport`]
//! * Support for dynamixel protocol v1 and v2 (both can be used on the same io)
//! * Support for sync read and sync write operations, plus fast sync read (protocol v2)
//! * Easy support for new type of motors (register definition through macros)
//! * Pure Rust
//! * `no_std` (with `alloc`) when the default `std` feature is off, with an async API
//!   (feature `async`) that runs on embassy
//!
//! ## APIs
//!
//! It exposes three APIs, from the wire up:
//! * `DynamixelProtocolHandler`: low-level API. It handles the serial communication and the Dynamixel protocol parsing. It can be used for fine-grained control of the shared bus with other communication.
//! * `Controller`: one per servo model, with a typed accessor for each register. Simpler and cleaner API but it takes full ownership of the io (it can still be shared if wrapped with a mutex for instance).
//! * [`bus::Bus`] and [`servo::ServoDefinition`]: a higher-level, family-agnostic API, designed mostly for the [LeRobot](https://github.com/huggingface/lerobot) library. Motors of several models and of both protocols on one port, registers by name, the facts about each servo, finding and setting up motors, and the torque of a whole bus. Some of its choices are LeRobot conventions rather than vendor rules; the controllers stay the per-servo API underneath.
//!
//! With the `async` feature, [`asynch`] and each servo's `asynch` module offer the
//! protocol handler, the per-register functions and the controllers as `async fn`s, over
//! any [`asynch::AsyncTransport`] -- for instance an embassy UART through
//! [`asynch::EmbeddedIoTransport`].
//!
//! See the examples below for usage.
//!
//! ## Examples
//!
//! ### With the low-level API
//! ```no_run
//! use rustypot::{DynamixelProtocolHandler, servo::dynamixel::mx};
//! use std::time::Duration;
//!
//! let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
//!     .timeout(Duration::from_millis(10))
//!     .open()
//!     .expect("Failed to open port");
//!
//! let dph = DynamixelProtocolHandler::v1();
//!
//! let raw_pos: i16 = mx::read_raw_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
//! let pos: f64 =
//!     mx::read_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
//! println!("Motor 11 present position: {:?}rads (raw: {:?})", pos, raw_pos);
//! ```
//!
//! ### With the high-level API
//! ```no_run
//! use rustypot::servo::feetech::sts3215::Sts3215Controller;
//! use std::time::Duration;
//!
//! let serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
//!     .timeout(Duration::from_millis(1000))
//!     .open()
//!     .unwrap();
//!
//! let mut c = Sts3215Controller::new()
//!         .with_protocol_v1()
//!         .with_serial_port(serial_port);
//!
//! let pos = c.sync_read_present_position(&vec![1, 2]).unwrap();
//! println!("Motors present position: {:?}", pos);
//!
//! c.sync_write_goal_position(&vec![1, 2], &vec![0.0, 90.0_f64.to_radians()]).unwrap();
//! ```

#![cfg_attr(not(any(feature = "std", test)), no_std)]
// Without `std` or `async` nothing talks to a bus: the register conversions and some
// imports only the IO code uses are left unused, and that is expected.
#![cfg_attr(
    not(any(feature = "std", feature = "async")),
    allow(unused_imports, dead_code)
)]

extern crate alloc;

/// What `std`'s prelude brings and `core`'s does not, for the modules that build without
/// `std`. Macro expansions resolve these names at their call site, so every module that
/// invokes `generate_servo!` imports it.
#[allow(unused_imports)]
pub(crate) mod prelude {
    pub(crate) use alloc::{
        borrow::ToOwned,
        boxed::Box,
        format,
        string::{String, ToString},
        vec,
        vec::Vec,
    };
}

#[cfg(feature = "std")]
pub mod bus;
pub mod servo;

#[cfg(feature = "async")]
pub mod asynch;
mod dynamixel_protocol;
#[cfg(feature = "std")]
pub mod transport;
#[cfg(feature = "std")]
pub use transport::Transport;
#[cfg(test)]
mod fake_port;
#[cfg(feature = "std")]
pub use dynamixel_protocol::DynamixelProtocolHandler;
pub use dynamixel_protocol::{
    CommunicationErrorKind, DynamixelErrorV1, StatusError, StatusPacketError, SyncReadError,
};

type Result<T> = core::result::Result<T, alloc::boxed::Box<dyn core::error::Error>>;

#[cfg(feature = "python")]
use pyo3::prelude::*;
#[cfg(feature = "python")]
use pyo3_stub_gen::define_stub_info_gatherer;

#[cfg(feature = "python")]
#[pymodule]
fn rustypot(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();

    servo::register_class(m)?;
    m.add_class::<servo::RegisterInfo>()?;
    m.add_class::<servo::RegisterAccess>()?;
    m.add_class::<servo::ServoDefinition>()?;
    m.add_class::<bus::PyBus>()?;

    Ok(())
}

#[cfg(feature = "python")]
define_stub_info_gatherer!(stub_info);
