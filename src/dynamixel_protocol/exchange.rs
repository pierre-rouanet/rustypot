//! Every exchange with the motors, written once.
//!
//! The async API calls these directly. The blocking API runs them through `block_on` over a
//! link whose futures complete at once (see `blocking.rs`), so both send the same bytes,
//! read the same way and fail the same way. Packets are built and parsed by
//! [`rustypot_core`].

use core::time::Duration;

use rustypot_core::{v1, v2, Error as CoreError};

use super::{
    CommunicationErrorKind, DynamixelProtocolHandler, ProtocolKind, StatusError, SyncReadError,
};
use crate::link::{AsyncTransport, TransportError};
#[allow(unused_imports)]
use crate::prelude::*;
use crate::Result;

/// A packet that does not encode or decode, as rustypot reports it.
pub(crate) fn communication_error(e: CoreError) -> Box<dyn core::error::Error> {
    Box::new(match e {
        CoreError::Checksum => CommunicationErrorKind::ChecksumError,
        CoreError::IncorrectId { expected, got } => {
            CommunicationErrorKind::IncorrectId(expected, got)
        }
        CoreError::AddressOutOfRange(addr) => CommunicationErrorKind::AddressOutOfRange(addr),
        // What a read that stopped waiting leaves.
        CoreError::Truncated => CommunicationErrorKind::TimeoutError,
        CoreError::Parsing | CoreError::BufferTooSmall | CoreError::TooLong => {
            CommunicationErrorKind::ParsingError
        }
    })
}

/// The packet `encode` writes, in a buffer grown until it fits.
fn packet(
    mut encode: impl FnMut(&mut [u8]) -> core::result::Result<usize, CoreError>,
) -> Result<Vec<u8>> {
    let mut size = 256;
    loop {
        let mut buf = vec![0u8; size];
        match encode(&mut buf) {
            Ok(n) => {
                buf.truncate(n);
                return Ok(buf);
            }
            // A v2 packet is at most 64 KiB, stuffed.
            Err(CoreError::BufferTooSmall) if size < 1 << 18 => size *= 4,
            Err(e) => return Err(communication_error(e)),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Proto {
    V1,
    V2,
}

impl Proto {
    fn header_size(self) -> usize {
        match self {
            Proto::V1 => v1::HEADER_SIZE,
            Proto::V2 => v2::HEADER_SIZE,
        }
    }

    fn broadcast_id(self) -> u8 {
        match self {
            Proto::V1 => v1::BROADCAST_ID,
            Proto::V2 => v2::BROADCAST_ID,
        }
    }

    fn payload_size(self, header: &[u8]) -> Result<usize> {
        match self {
            Proto::V1 => v1::payload_size(header),
            Proto::V2 => v2::payload_size(header),
        }
        .map_err(communication_error)
    }

    /// The parameters and error byte of the status packet `frame`, from `id`.
    fn status(self, frame: &mut [u8], id: u8) -> Result<(Vec<u8>, u8)> {
        match self {
            Proto::V1 => v1::parse_status(frame, id).map(|s| (s.params.to_vec(), s.code)),
            Proto::V2 => v2::parse_status(frame, id).map(|s| (s.params.to_vec(), s.error)),
        }
        .map_err(communication_error)
    }

    fn ping(self, id: u8) -> Result<Vec<u8>> {
        match self {
            Proto::V1 => packet(|b| v1::ping(b, id)),
            Proto::V2 => packet(|b| v2::ping(b, id)),
        }
    }

    fn reboot(self, id: u8) -> Result<Vec<u8>> {
        match self {
            Proto::V1 => packet(|b| v1::reboot(b, id)),
            Proto::V2 => packet(|b| v2::reboot(b, id)),
        }
    }

    fn read(self, id: u8, addr: u16, length: u8) -> Result<Vec<u8>> {
        match self {
            Proto::V1 => packet(|b| v1::read(b, id, addr, length)),
            Proto::V2 => packet(|b| v2::read(b, id, addr, length.into())),
        }
    }

    fn write(self, id: u8, addr: u16, data: &[u8]) -> Result<Vec<u8>> {
        match self {
            Proto::V1 => packet(|b| v1::write(b, id, addr, data)),
            Proto::V2 => packet(|b| v2::write(b, id, addr, data)),
        }
    }

    fn sync_read(self, ids: &[u8], addr: u16, length: u8) -> Result<Vec<u8>> {
        match self {
            Proto::V1 => packet(|b| v1::sync_read(b, ids, addr, length)),
            Proto::V2 => packet(|b| v2::sync_read(b, ids, addr, length.into())),
        }
    }

    fn sync_write(self, ids: &[u8], addr: u16, data: &[Vec<u8>]) -> Result<Vec<u8>> {
        match self {
            Proto::V1 => packet(|b| v1::sync_write(b, ids, addr, data)),
            Proto::V2 => packet(|b| v2::sync_write(b, ids, addr, data)),
        }
    }
}

const MAX_FLUSH_RETRIES: usize = 3;
const FLUSH_RETRY_DELAY: Duration = Duration::from_millis(5);

/// Send `packet`, after dropping whatever is left of earlier exchanges.
///
/// Leftovers mean an old, corrupted message: drop them and wait a little for the rest of
/// it, a few times, then give up rather than mix it with the answer to come.
async fn send(port: &mut impl AsyncTransport, packet: &[u8]) -> Result<()> {
    let mut clean = false;
    for _ in 0..MAX_FLUSH_RETRIES {
        if port.discard_input().await? == 0 {
            clean = true;
            break;
        }
        port.delay(FLUSH_RETRY_DELAY).await;
    }
    if !clean && port.discard_input().await? != 0 {
        return Err(Box::new(CommunicationErrorKind::TimeoutError));
    }
    match port.write_all(packet).await {
        Ok(()) => Ok(()),
        Err(_) => Err(Box::new(CommunicationErrorKind::TimeoutError)),
    }
}

/// One status packet off the wire from `id`: its parameters and error byte.
async fn status(port: &mut impl AsyncTransport, proto: Proto, id: u8) -> Result<(Vec<u8>, u8)> {
    let mut frame = vec![0u8; proto.header_size()];
    port.read_exact(&mut frame).await?;
    let payload = proto.payload_size(&frame)?;
    frame.resize(proto.header_size() + payload, 0);
    port.read_exact(&mut frame[proto.header_size()..]).await?;
    proto.status(&mut frame, id)
}

/// Send `request` to `id` and read its answer.
async fn exchange(
    port: &mut impl AsyncTransport,
    proto: Proto,
    id: u8,
    request: &[u8],
) -> Result<(Vec<u8>, u8)> {
    send(port, request).await?;
    status(port, proto, id).await
}

/// Up to `len` bytes, fewer if the link times out first.
async fn read_until_timeout(port: &mut impl AsyncTransport, len: usize) -> Result<Vec<u8>> {
    let mut data = vec![0u8; len];
    let mut filled = 0;
    while filled < len {
        match port.read(&mut data[filled..]).await {
            Ok(0) | Err(TransportError::TimedOut) => break,
            Ok(n) => filled += n,
            Err(e) => return Err(Box::new(e)),
        }
    }
    data.truncate(filled);
    Ok(data)
}

/// The ids of the status packets in `bytes`. Anything that does not frame one (a stray
/// byte, a packet cut short, a bad checksum) is skipped a byte at a time until the next
/// header, so it costs that packet and not the ones after it.
fn status_packet_ids(proto: Proto, bytes: &[u8]) -> Vec<u8> {
    let header = proto.header_size();
    let mut ids = Vec::new();
    let mut start = 0;
    while start + header <= bytes.len() {
        let packet = proto
            .payload_size(&bytes[start..start + header])
            .ok()
            .map(|payload| bytes.len().min(start + header + payload))
            .map(|end| bytes[start..end].to_vec());
        match packet {
            Some(mut packet) => {
                let id = packet[if proto == Proto::V1 { 2 } else { 4 }];
                if proto.status(&mut packet, id).is_ok() {
                    ids.push(id);
                    start += packet.len();
                } else {
                    start += 1;
                }
            }
            None => start += 1,
        }
    }
    ids
}

impl DynamixelProtocolHandler {
    fn proto(&self) -> Proto {
        match self.protocol {
            ProtocolKind::V1(_) => Proto::V1,
            ProtocolKind::V2(..) => Proto::V2,
        }
    }

    /// Leave the configured gap on the bus after a transaction, whether or not it
    /// succeeded: a transaction that timed out has used the bus just the same.
    async fn post_delay(&self, port: &mut impl AsyncTransport) {
        if let Some(delay) = self.post_delay {
            port.delay(delay).await;
        }
    }

    pub(crate) async fn ping_async(&self, port: &mut impl AsyncTransport, id: u8) -> Result<bool> {
        let proto = self.proto();
        send(port, &proto.ping(id)?).await?;
        Ok(status(port, proto, id).await.is_ok())
    }

    pub(crate) async fn broadcast_ping_async(
        &self,
        port: &mut impl AsyncTransport,
        window: Duration,
    ) -> Result<Vec<u8>> {
        let proto = self.proto();
        let ids = async {
            send(port, &proto.ping(proto.broadcast_id())?).await?;
            let mut received = Vec::new();
            port.listen(window, &mut received).await?;
            Ok(status_packet_ids(proto, &received))
        }
        .await;
        self.post_delay(port).await;
        ids
    }

    pub(crate) async fn reboot_async(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
    ) -> Result<bool> {
        let proto = self.proto();
        send(port, &proto.reboot(id)?).await?;
        Ok(status(port, proto, id).await.is_ok())
    }

    pub(crate) async fn factory_reset_async(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        conserve_id_only: bool,
        conserve_id_and_baudrate: bool,
    ) -> Result<()> {
        let request = match self.protocol {
            ProtocolKind::V1(_) => {
                if conserve_id_only || conserve_id_and_baudrate {
                    return Err(Box::new(CommunicationErrorKind::Unsupported));
                }
                packet(|b| v1::factory_reset(b, id))?
            }
            ProtocolKind::V2(..) => {
                // See https://emanual.robotis.com/docs/en/dxl/protocol2/
                let keep = match (conserve_id_only, conserve_id_and_baudrate) {
                    (false, false) => 0xFF,
                    (true, false) => 0x01,
                    (_, true) => 0x02,
                };
                packet(|b| v2::factory_reset(b, id, keep))?
            }
        };
        exchange(port, self.proto(), id, &request).await.map(|_| ())
    }

    pub(crate) async fn read_with_error_async(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        length: u8,
    ) -> Result<(Vec<u8>, StatusError)> {
        let proto = self.proto();
        let request = proto.read(id, addr, length)?;
        let res = exchange(port, proto, id, &request).await;
        self.post_delay(port).await;
        res.map(|(values, e)| (values, StatusError::new(e)))
    }

    pub(crate) async fn write_with_error_async(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        data: &[u8],
    ) -> Result<StatusError> {
        let proto = self.proto();
        let request = proto.write(id, addr, data)?;
        let res = exchange(port, proto, id, &request).await;
        self.post_delay(port).await;
        res.map(|(_, e)| StatusError::new(e))
    }

    pub(crate) async fn write_fb_async(
        &self,
        port: &mut impl AsyncTransport,
        id: u8,
        addr: u16,
        data: &[u8],
    ) -> Result<Vec<u8>> {
        if self.proto() == Proto::V2 {
            return Err(Box::new(CommunicationErrorKind::Unsupported));
        }
        let request = Proto::V1.write(id, addr, data)?;
        let res = exchange(port, Proto::V1, id, &request).await;
        self.post_delay(port).await;
        res.map(|(values, _)| values)
    }

    pub(crate) async fn sync_read_with_error_async(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u16,
        length: u8,
    ) -> Result<Vec<(Vec<u8>, StatusError)>> {
        let res = match self.protocol {
            ProtocolKind::V2(_, true) => fast_sync_read(port, ids, addr, length).await,
            _ => sync_read(port, self.proto(), ids, addr, length).await,
        };
        self.post_delay(port).await;
        Ok(res?
            .into_iter()
            .map(|(values, e)| (values, StatusError::new(e)))
            .collect())
    }

    pub(crate) async fn fast_sync_read_async(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u16,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        // The v1 arm never reaches the bus, so it has no gap to leave.
        if self.proto() == Proto::V1 {
            return Err(Box::new(CommunicationErrorKind::Unsupported));
        }
        let res = fast_sync_read(port, ids, addr, length).await;
        self.post_delay(port).await;
        Ok(res?.into_iter().map(|(values, _)| values).collect())
    }

    pub(crate) async fn sync_write_async(
        &self,
        port: &mut impl AsyncTransport,
        ids: &[u8],
        addr: u16,
        data: &[Vec<u8>],
    ) -> Result<()> {
        let request = self.proto().sync_write(ids, addr, data)?;
        let res = send(port, &request).await;
        self.post_delay(port).await;
        res
    }
}

/// One status packet per motor, in the order of `ids`; a failure names its motor.
async fn sync_read(
    port: &mut impl AsyncTransport,
    proto: Proto,
    ids: &[u8],
    addr: u16,
    length: u8,
) -> Result<Vec<(Vec<u8>, u8)>> {
    send(port, &proto.sync_read(ids, addr, length)?).await?;
    let mut result = Vec::with_capacity(ids.len());
    for &id in ids {
        let answer = status(port, proto, id)
            .await
            .map_err(|cause| SyncReadError::boxed(id, cause))?;
        result.push(answer);
    }
    Ok(result)
}

/// One status packet for all the motors (protocol v2, instruction 0x8A). A short packet is
/// not an error here but what says which motor did not answer: the parser finds where it
/// stops.
async fn fast_sync_read(
    port: &mut impl AsyncTransport,
    ids: &[u8],
    addr: u16,
    length: u8,
) -> Result<Vec<(Vec<u8>, u8)>> {
    send(
        port,
        &packet(|b| v2::fast_sync_read(b, ids, addr, length.into()))?,
    )
    .await?;
    let mut data = read_until_timeout(port, v2::HEADER_SIZE).await?;
    if data.len() == v2::HEADER_SIZE {
        if let Ok(payload) = v2::payload_size(&data) {
            data.extend(read_until_timeout(port, payload).await?);
        }
    }
    let mut values = Vec::with_capacity(ids.len());
    v2::parse_fast_sync_status(&data, ids, length.into(), |_, error, bytes| {
        values.push((bytes.to_vec(), error))
    })
    .map_err(|e| {
        let cause = communication_error(e.error);
        match e.index.and_then(|i| ids.get(i)) {
            Some(&id) => SyncReadError::boxed(id, cause),
            None => cause,
        }
    })?;
    Ok(values)
}
