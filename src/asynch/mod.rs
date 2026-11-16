//! The async API: the protocol handler over any [`AsyncTransport`].
//!
//! It mirrors the blocking [`crate::DynamixelProtocolHandler`] call for call, as
//! `async fn`s, and needs neither `std` nor a particular executor: it runs on embassy
//! as well as on tokio. Both APIs run the same exchanges, built on
//! [`rustypot_core`]; the blocking one drives them over its serial port. Each servo module has an `asynch` module on top of it with the
//! typed register functions and a controller, e.g.
//! [`xl330::asynch`](crate::servo::dynamixel::xl330::asynch).
//!
//! Register access by name through a [`ServoDefinition`](crate::servo::ServoDefinition),
//! scanning and motor setup live in [`definition`]. What stays blocking-only is
//! [`Bus`](crate::bus::Bus).
//!
//! ```no_run
//! # async fn demo(uart: impl rustypot::asynch::AsyncTransport) -> Result<(), Box<dyn core::error::Error>> {
//! use rustypot::servo::dynamixel::xl330;
//!
//! let mut c = xl330::asynch::Xl330Controller::new(uart);
//! c.write_torque_enable(1, true).await?;
//! let positions = c.sync_read_present_position(&[1, 2]).await?;
//! # Ok(()) }
//! ```

pub mod definition;
mod embedded;
mod half_duplex;

pub use crate::link::{AsyncTransport, TransportError};
pub use embedded::EmbeddedIoTransport;
pub use half_duplex::HalfDuplex;

use core::time::Duration;

use crate::dynamixel_protocol;
#[allow(unused_imports)]
use crate::prelude::*;
use crate::{Result, StatusError};

/// Raw Dynamixel communication (protocol v1 or v2), over an [`AsyncTransport`].
///
/// The async counterpart of [`crate::DynamixelProtocolHandler`]: the same settings,
/// the same instructions, the same errors.
#[derive(Debug, Clone)]
pub struct DynamixelProtocolHandler(pub(crate) dynamixel_protocol::DynamixelProtocolHandler);

impl DynamixelProtocolHandler {
    /// A protocol v1 handler.
    pub fn v1() -> Self {
        Self(dynamixel_protocol::DynamixelProtocolHandler::v1())
    }

    /// A protocol v2 handler.
    pub fn v2() -> Self {
        Self(dynamixel_protocol::DynamixelProtocolHandler::v2())
    }

    /// The handler of protocol `version` (1 or 2), `None` for any other.
    pub fn of_version(version: u8) -> Option<Self> {
        match version {
            1 => Some(Self::v1()),
            2 => Some(Self::v2()),
            _ => None,
        }
    }

    /// Wait `delay` after each transaction, through [`AsyncTransport::delay`].
    pub fn with_post_delay(self, delay: Duration) -> Self {
        Self(self.0.with_post_delay(delay))
    }

    /// Make [`sync_read`](Self::sync_read) use Fast Sync Read (protocol v2 only; ignored
    /// on v1). Needs firmware accepting instruction 0x8A (XL330: v46+).
    pub fn with_fast_sync_read(self) -> Self {
        Self(self.0.with_fast_sync_read())
    }

    /// Turn the Fast Sync Read routing of [`sync_read`](Self::sync_read) on or off.
    pub fn set_fast_sync_read(&mut self, enabled: bool) {
        self.0.set_fast_sync_read(enabled)
    }

    /// Whether [`sync_read`](Self::sync_read) currently uses Fast Sync Read.
    pub fn fast_sync_read_enabled(&self) -> bool {
        self.0.fast_sync_read_enabled()
    }

    /// The protocol version this handler speaks: 1 or 2.
    pub fn protocol_version(&self) -> u8 {
        self.0.protocol_version()
    }

    /// The highest id a motor can take on this protocol.
    pub fn max_id(&self) -> u8 {
        self.0.max_id()
    }

    /// How long a broadcast ping listens at `baudrate`, see
    /// [`crate::DynamixelProtocolHandler::broadcast_ping_window`].
    pub fn broadcast_ping_window(&self, baudrate: u32) -> Duration {
        self.0.broadcast_ping_window(baudrate)
    }

    /// Ping motor `id`: whether it answered.
    pub async fn ping(&self, port: &mut impl AsyncTransport, id: u8) -> Result<bool> {
        self.0.ping_async(port, id).await
    }

    /// Ping every motor at once, and return the ids that answer.
    ///
    /// Listens for [`broadcast_ping_window`](Self::broadcast_ping_window) at `baudrate`,
    /// the bus speed, through [`AsyncTransport::listen`]. Only some firmwares answer:
    /// protocol v2 Dynamixel and Feetech STS do; protocol v1 Dynamixel and Feetech SCS
    /// stay silent, which reads as no motor at all.
    pub async fn broadcast_ping(
        &self,
        port: &mut impl AsyncTransport,
        baudrate: u32,
    ) -> Result<Vec<u8>> {
        let window = self.broadcast_ping_window(baudrate);
        self.0.broadcast_ping_async(port, window).await
    }

    /// Reboot motor `id`: whether it answered.
    pub async fn reboot(&self, port: &mut impl AsyncTransport, id: u8) -> Result<bool> {
        self.0.reboot_async(port, id).await
    }

    /// Reset the control table of motor `id` to the factory values. Keeping the id
    /// and/or the baud rate is only supported on protocol v2.
    pub async fn factory_reset(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        conserve_id_only: bool,
        conserve_id_and_baudrate: bool,
    ) -> Result<()> {
        self.0
            .factory_reset_async(port, id, conserve_id_only, conserve_id_and_baudrate)
            .await
    }

    /// Read `length` raw bytes from register `addr` of motor `id`.
    pub async fn read(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        length: u8,
    ) -> Result<Vec<u8>> {
        Ok(self.read_with_error(port, id, addr, length).await?.0)
    }

    /// Write raw bytes to register `addr` of motor `id`, and wait for its status packet.
    pub async fn write(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        data: &[u8],
    ) -> Result<()> {
        self.write_with_error(port, id, addr, data)
            .await
            .map(|_| ())
    }

    /// Same as [`read`](Self::read), and also returns the status packet's error field. See
    /// [`crate::DynamixelProtocolHandler::read_with_error`] for how to read it.
    pub async fn read_with_error(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        length: u8,
    ) -> Result<(Vec<u8>, StatusError)> {
        self.0.read_with_error_async(port, id, addr, length).await
    }

    /// Same as [`write`](Self::write), and also returns the status packet's error field.
    pub async fn write_with_error(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        data: &[u8],
    ) -> Result<StatusError> {
        self.0.write_with_error_async(port, id, addr, data).await
    }

    /// Write raw bytes and return the parameters of the status packet (protocol v1 only,
    /// for the servos that answer a write with feedback).
    pub async fn write_fb(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        data: &[u8],
    ) -> Result<Vec<u8>> {
        self.0.write_fb_async(port, id, addr, data).await
    }

    /// Read `length` raw bytes from register `addr` of every motor of `ids`, in one
    /// instruction. Routed to Fast Sync Read when that is enabled.
    pub async fn sync_read(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u16,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        Ok(self
            .sync_read_with_error(port, ids, addr, length)
            .await?
            .into_iter()
            .map(|(values, _)| values)
            .collect())
    }

    /// Same as [`sync_read`](Self::sync_read), and also returns each motor's error field.
    pub async fn sync_read_with_error(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u16,
        length: u8,
    ) -> Result<Vec<(Vec<u8>, StatusError)>> {
        self.0
            .sync_read_with_error_async(port, ids, addr, length)
            .await
    }

    /// Same as [`sync_read`](Self::sync_read), always as a Fast Sync Read (instruction
    /// 0x8A): every motor appends its answer to one status packet. Protocol v2 only.
    pub async fn fast_sync_read(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u16,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        self.0.fast_sync_read_async(port, ids, addr, length).await
    }

    /// Write `data[i]` to register `addr` of motor `ids[i]`, in one instruction. No motor
    /// answers.
    pub async fn sync_write(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u16,
        data: &[Vec<u8>],
    ) -> Result<()> {
        self.0.sync_write_async(port, ids, addr, data).await
    }
}

#[cfg(test)]
mod tests;
