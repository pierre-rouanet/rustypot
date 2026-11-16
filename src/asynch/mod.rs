//! The async API: the protocol handler over any [`AsyncTransport`].
//!
//! It mirrors the blocking [`crate::DynamixelProtocolHandler`] call for call, as
//! `async fn`s, and needs neither `std` nor a particular executor: it runs on embassy
//! as well as on tokio. Each servo module has an `asynch` module on top of it with the
//! typed register functions and a controller, e.g.
//! [`xl330::asynch`](crate::servo::dynamixel::xl330::asynch).
//!
//! What it leaves out, for now: broadcast ping and scanning (they need a clock to bound a
//! listening window), and register access by name through a
//! [`ServoDefinition`](crate::servo::ServoDefinition).
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

mod embedded;
mod protocol;
mod transport;

pub use embedded::EmbeddedIoTransport;
pub use transport::{AsyncTransport, TransportError};

use core::time::Duration;

use crate::dynamixel_protocol::v1::PacketV1;
use crate::dynamixel_protocol::v2::PacketV2;
use crate::dynamixel_protocol::{self, CommunicationErrorKind, ProtocolKind};
#[allow(unused_imports)]
use crate::prelude::*;
use crate::{Result, StatusError};

/// Raw Dynamixel communication (protocol v1 or v2), over an [`AsyncTransport`].
///
/// The async counterpart of [`crate::DynamixelProtocolHandler`]: the same settings,
/// the same instructions, the same errors.
#[derive(Debug)]
pub struct DynamixelProtocolHandler(dynamixel_protocol::DynamixelProtocolHandler);

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

    async fn post_delay(&self, port: &mut impl AsyncTransport) {
        if let Some(delay) = self.0.post_delay {
            port.delay(delay).await;
        }
    }

    /// Ping motor `id`: whether it answered.
    pub async fn ping(&self, port: &mut impl AsyncTransport, id: u8) -> Result<bool> {
        match self.0.protocol {
            ProtocolKind::V1(_) => protocol::ping::<PacketV1>(port, id).await,
            ProtocolKind::V2(..) => protocol::ping::<PacketV2>(port, id).await,
        }
    }

    /// Reboot motor `id`: whether it answered.
    pub async fn reboot(&self, port: &mut impl AsyncTransport, id: u8) -> Result<bool> {
        match self.0.protocol {
            ProtocolKind::V1(_) => protocol::reboot::<PacketV1>(port, id).await,
            ProtocolKind::V2(..) => protocol::reboot::<PacketV2>(port, id).await,
        }
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
        match self.0.protocol {
            ProtocolKind::V1(_) => {
                if conserve_id_only || conserve_id_and_baudrate {
                    return Err(Box::new(CommunicationErrorKind::Unsupported));
                }
                protocol::factory_reset::<PacketV1>(port, id, false, false).await
            }
            ProtocolKind::V2(..) => {
                protocol::factory_reset::<PacketV2>(
                    port,
                    id,
                    conserve_id_only,
                    conserve_id_and_baudrate,
                )
                .await
            }
        }
    }

    /// Read `length` raw bytes from register `addr` of motor `id`.
    pub async fn read(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<Vec<u8>> {
        Ok(self.read_with_error(port, id, addr, length).await?.0)
    }

    /// Write raw bytes to register `addr` of motor `id`, and wait for its status packet.
    pub async fn write(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u8,
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
        addr: u8,
        length: u8,
    ) -> Result<(Vec<u8>, StatusError)> {
        let res = match self.0.protocol {
            ProtocolKind::V1(_) => {
                protocol::exchange(port, id, PacketV1::read(id, addr, length)).await
            }
            ProtocolKind::V2(..) => {
                protocol::exchange(port, id, PacketV2::read(id, addr, length)).await
            }
        };
        self.post_delay(port).await;
        res.map(|(values, e)| (values, StatusError::new(e)))
    }

    /// Same as [`write`](Self::write), and also returns the status packet's error field.
    pub async fn write_with_error(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<StatusError> {
        let res = match self.0.protocol {
            ProtocolKind::V1(_) => {
                protocol::exchange(port, id, PacketV1::write(id, addr, data)).await
            }
            ProtocolKind::V2(..) => {
                protocol::exchange(port, id, PacketV2::write(id, addr, data)).await
            }
        };
        self.post_delay(port).await;
        res.map(|(_, e)| StatusError::new(e))
    }

    /// Write raw bytes and return the parameters of the status packet (protocol v1 only,
    /// for the servos that answer a write with feedback).
    pub async fn write_fb(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<Vec<u8>> {
        match self.0.protocol {
            ProtocolKind::V1(_) => {
                let res = protocol::exchange(port, id, PacketV1::write(id, addr, data)).await;
                self.post_delay(port).await;
                res.map(|(values, _)| values)
            }
            ProtocolKind::V2(..) => Err(Box::new(CommunicationErrorKind::Unsupported)),
        }
    }

    /// Read `length` raw bytes from register `addr` of every motor of `ids`, in one
    /// instruction. Routed to Fast Sync Read when that is enabled.
    pub async fn sync_read(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u8,
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
        addr: u8,
        length: u8,
    ) -> Result<Vec<(Vec<u8>, StatusError)>> {
        let res = match self.0.protocol {
            ProtocolKind::V1(_) => protocol::sync_read::<PacketV1>(port, ids, addr, length).await,
            ProtocolKind::V2(_, false) => {
                protocol::sync_read::<PacketV2>(port, ids, addr, length).await
            }
            ProtocolKind::V2(_, true) => protocol::fast_sync_read(port, ids, addr, length).await,
        };
        self.post_delay(port).await;
        Ok(res?
            .into_iter()
            .map(|(values, e)| (values, StatusError::new(e)))
            .collect())
    }

    /// Same as [`sync_read`](Self::sync_read), always as a Fast Sync Read (instruction
    /// 0x8A): every motor appends its answer to one status packet. Protocol v2 only.
    pub async fn fast_sync_read(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        match self.0.protocol {
            ProtocolKind::V1(_) => Err(Box::new(CommunicationErrorKind::Unsupported)),
            ProtocolKind::V2(..) => {
                let res = protocol::fast_sync_read(port, ids, addr, length).await;
                self.post_delay(port).await;
                Ok(res?.into_iter().map(|(values, _)| values).collect())
            }
        }
    }

    /// Write `data[i]` to register `addr` of motor `ids[i]`, in one instruction. No motor
    /// answers.
    pub async fn sync_write(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u8,
        data: &[Vec<u8>],
    ) -> Result<()> {
        let res = match self.0.protocol {
            ProtocolKind::V1(_) => {
                protocol::send(port, PacketV1::sync_write(ids, addr, data).as_ref()).await
            }
            ProtocolKind::V2(..) => {
                protocol::send(port, PacketV2::sync_write(ids, addr, data).as_ref()).await
            }
        };
        self.post_delay(port).await;
        res
    }
}

/// Short names for the packet constructors, generic over the protocol.
trait Packets: dynamixel_protocol::packet::Packet + Sized {
    fn read(
        id: u8,
        addr: u8,
        length: u8,
    ) -> Box<dyn dynamixel_protocol::packet::InstructionPacket<Self>> {
        Self::read_packet(id, addr, length)
    }
    fn write(
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Box<dyn dynamixel_protocol::packet::InstructionPacket<Self>> {
        Self::write_packet(id, addr, data)
    }
    fn sync_write(
        ids: &[u8],
        addr: u8,
        data: &[Vec<u8>],
    ) -> Box<dyn dynamixel_protocol::packet::InstructionPacket<Self>> {
        Self::sync_write_packet(ids, addr, data)
    }
}
impl Packets for PacketV1 {}
impl Packets for PacketV2 {}

#[cfg(test)]
mod tests;
