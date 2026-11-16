//! The exchanges of the async handler, written once for both protocols.
//!
//! Each is the async twin of a method of the blocking `Protocol` trait in
//! `dynamixel_protocol`; keep the two in step.

use core::time::Duration;

use super::AsyncTransport;
use crate::dynamixel_protocol::packet::{InstructionPacket, Packet};
use crate::dynamixel_protocol::v2::{self, PacketV2};
use crate::dynamixel_protocol::{CommunicationErrorKind, SyncReadError};
#[allow(unused_imports)]
use crate::prelude::*;
use crate::Result;

const MAX_FLUSH_RETRIES: usize = 3;
const FLUSH_RETRY_DELAY: Duration = Duration::from_millis(5);

/// Send `packet`, after dropping whatever is left of earlier exchanges.
///
/// Leftovers mean an old, corrupted message: drop them and wait a little for the rest of
/// it, a few times, then give up rather than mix it with the answer to come.
pub(super) async fn send<P: Packet>(
    port: &mut impl AsyncTransport,
    packet: &dyn InstructionPacket<P>,
) -> Result<()> {
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
    match port.write_all(&packet.to_bytes()).await {
        Ok(()) => Ok(()),
        Err(_) => Err(Box::new(CommunicationErrorKind::TimeoutError)),
    }
}

/// One status packet off the wire, header included, checked to come from `sender_id`:
/// its parameters and error byte.
async fn status<P: Packet>(port: &mut impl AsyncTransport, sender_id: u8) -> Result<(Vec<u8>, u8)> {
    let mut data = vec![0u8; P::HEADER_SIZE];
    port.read_exact(&mut data).await?;
    let payload_size = P::get_payload_size(&data)?;
    data.resize(P::HEADER_SIZE + payload_size, 0);
    port.read_exact(&mut data[P::HEADER_SIZE..]).await?;
    let sp = P::status_packet(&data, sender_id)?;
    Ok((sp.params().to_vec(), sp.error_byte()))
}

/// Send `packet` to motor `id` and read its answer.
pub(super) async fn exchange<P: Packet>(
    port: &mut impl AsyncTransport,
    id: u8,
    packet: Box<dyn InstructionPacket<P>>,
) -> Result<(Vec<u8>, u8)> {
    send(port, packet.as_ref()).await?;
    status::<P>(port, id).await
}

pub(super) async fn ping<P: Packet>(port: &mut impl AsyncTransport, id: u8) -> Result<bool> {
    send(port, P::ping_packet(id).as_ref()).await?;
    Ok(status::<P>(port, id).await.is_ok())
}

pub(super) async fn reboot<P: Packet>(port: &mut impl AsyncTransport, id: u8) -> Result<bool> {
    send(port, P::reboot_packet(id).as_ref()).await?;
    Ok(status::<P>(port, id).await.is_ok())
}

pub(super) async fn factory_reset<P: Packet>(
    port: &mut impl AsyncTransport,
    id: u8,
    conserve_id_only: bool,
    conserve_id_and_baudrate: bool,
) -> Result<()> {
    let packet = P::factory_reset_packet(id, conserve_id_only, conserve_id_and_baudrate);
    exchange(port, id, packet).await.map(|_| ())
}

/// One status packet per motor, in the order of `ids`; a failure names its motor.
pub(super) async fn sync_read<P: Packet>(
    port: &mut impl AsyncTransport,
    ids: &[u8],
    addr: u8,
    length: u8,
) -> Result<Vec<(Vec<u8>, u8)>> {
    send(port, P::sync_read_packet(ids, addr, length).as_ref()).await?;
    let mut result = Vec::with_capacity(ids.len());
    for &id in ids {
        let answer = status::<P>(port, id)
            .await
            .map_err(|cause| SyncReadError::boxed(id, cause))?;
        result.push(answer);
    }
    Ok(result)
}

/// One status packet for all the motors (protocol v2, instruction 0x8A).
pub(super) async fn fast_sync_read(
    port: &mut impl AsyncTransport,
    ids: &[u8],
    addr: u8,
    length: u8,
) -> Result<Vec<(Vec<u8>, u8)>> {
    send(
        port,
        PacketV2::fast_sync_read_packet(ids, addr, length).as_ref(),
    )
    .await?;
    // A short packet is not an error here but what says which motor did not answer: keep
    // what arrived, the parser finds where it stops.
    let mut data = read_until_timeout(port, PacketV2::HEADER_SIZE).await?;
    if data.len() == PacketV2::HEADER_SIZE {
        if let Ok(payload_size) = PacketV2::get_payload_size(&data) {
            data.extend(read_until_timeout(port, payload_size).await?);
        }
    }
    v2::parse_fast_sync_read_status_with_error(&data, ids, length)
}

/// Up to `len` bytes, fewer if the link times out first.
async fn read_until_timeout(port: &mut impl AsyncTransport, len: usize) -> Result<Vec<u8>> {
    let mut data = vec![0u8; len];
    let mut filled = 0;
    while filled < len {
        match port.read(&mut data[filled..]).await {
            Ok(0) | Err(super::TransportError::TimedOut) => break,
            Ok(n) => filled += n,
            Err(e) => return Err(Box::new(e)),
        }
    }
    data.truncate(filled);
    Ok(data)
}
