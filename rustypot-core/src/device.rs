//! The device side of Protocol 2.0: answer a controller as a servo would.
//!
//! [`Device`] is fed the bytes of the bus one at a time and says what to send back, from a
//! [`ControlTable`] you provide. It handles Ping, Read, Write, Sync Write, Sync Read and
//! Fast Sync Read (including waiting for its turn after the devices listed before it),
//! Reboot and Factory Reset; other instructions addressed to it get an Instruction Error.
//!
//! It does no IO and has no clock, so it fits a main loop, an interrupt handler or an
//! async task alike:
//!
//! ```
//! use rustypot_core::device::{Action, Device, Identity};
//! use rustypot_core::v2;
//!
//! // A sensor that passes for an XL330 (model 1200) at id 200.
//! let mut device: Device<128> = Device::new(Identity { id: 200, model_number: 1200, firmware_version: 1 });
//! let mut table = [0u8; 160];
//! table[124..128].copy_from_slice(&[1, 2, 3, 4]);
//!
//! // A controller reads 4 bytes at 124.
//! let mut request = [0u8; 16];
//! let n = v2::read(&mut request, 200, 124, 4)?;
//!
//! let mut out = [0u8; 64];
//! let mut action = Action::Nothing;
//! for &byte in &request[..n] {
//!     action = device.push(byte, &mut table, &mut out);
//! }
//! let Action::Reply(len) = action else { panic!() };
//! let status = v2::parse_status(&mut out[..len], 200)?;
//! assert_eq!(status.params, [1, 2, 3, 4]);
//! # Ok::<(), rustypot_core::Error>(())
//! ```
//!
//! Two things are left to the application, since they need a clock:
//!
//! * The return delay before a [`Reply`](Action::Reply), and the turn of a
//!   [`BroadcastPingReply`](Action::BroadcastPingReply), which devices send one after
//!   another.
//! * Giving up on a turn. While [`is_waiting`](Device::is_waiting), the device counts the
//!   answers of the devices listed before it in a Sync Read; if one of them never answers,
//!   call [`bus_idle`](Device::bus_idle) once the bus has been quiet for a while (a few
//!   milliseconds), and the device stays silent rather than collide with the next request.
//!
//! Do not feed the device its own transmissions (a bus that echoes TX into RX).

use crate::crc::crc16_update;
use crate::v2::{self, error, instruction, Decoder, Packet};

/// The registers a [`Device`] serves.
pub trait ControlTable {
    /// The `len` bytes at `addr`, or the error number to answer with (usually
    /// [`error::DATA_RANGE`]).
    fn read(&mut self, addr: u16, len: u16) -> Result<&[u8], u8>;

    /// Write `data` at `addr`, or say which error number to answer with:
    /// [`error::DATA_RANGE`] outside the table, [`error::ACCESS`] for a read-only
    /// register...
    fn write(&mut self, addr: u16, data: &[u8]) -> Result<(), u8>;
}

/// A plain byte array, read and written as is.
impl ControlTable for [u8] {
    fn read(&mut self, addr: u16, len: u16) -> Result<&[u8], u8> {
        let start = usize::from(addr);
        self.get(start..start + usize::from(len))
            .ok_or(error::DATA_RANGE)
    }

    fn write(&mut self, addr: u16, data: &[u8]) -> Result<(), u8> {
        let start = usize::from(addr);
        self.get_mut(start..start + data.len())
            .ok_or(error::DATA_RANGE)?
            .copy_from_slice(data);
        Ok(())
    }
}

impl<const N: usize> ControlTable for [u8; N] {
    fn read(&mut self, addr: u16, len: u16) -> Result<&[u8], u8> {
        self.as_mut_slice().read(addr, len)
    }

    fn write(&mut self, addr: u16, data: &[u8]) -> Result<(), u8> {
        self.as_mut_slice().write(addr, data)
    }
}

/// Who the device says it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identity {
    pub id: u8,
    /// Answered to a Ping, with `firmware_version`.
    pub model_number: u16,
    pub firmware_version: u8,
}

/// What to do after a byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Nothing to send.
    Nothing,
    /// Send `out[..n]`, after the device's return delay.
    Reply(usize),
    /// Send `out[..n]` right away: this device's block of a Fast Sync Read answer, which
    /// follows the block of the device before it with no gap.
    Append(usize),
    /// Send `out[..n]`, the answer to a broadcast Ping, once this device's turn comes:
    /// every device answers, one after another.
    BroadcastPingReply(usize),
    /// Send `out[..n]`, then reboot.
    RebootAfter(usize),
    /// Send `out[..n]`, then reset the control table: everything (`keep` 0xFF), all but
    /// the id (0x01), or all but the id and the baud rate (0x02).
    FactoryResetAfter { reply: usize, keep: u8 },
}

/// The bytes of the bus while waiting for this device's turn in a Sync Read.
#[derive(Debug, Clone, Copy)]
enum Turn {
    None,
    /// Plain Sync Read: let `left` status packets go by, then answer.
    Sync {
        left: u16,
        addr: u16,
        length: u16,
        skip: Skip,
    },
    /// Fast Sync Read: check the header and `index` blocks, then append ours.
    Fast {
        index: u16,
        n_ids: u16,
        addr: u16,
        length: u16,
        crc: u16,
        stage: FastStage,
    },
}

/// Lets one packet go by, framed by its length (stuffing cannot fool it).
#[derive(Debug, Clone, Copy)]
enum Skip {
    Header(u8),
    Prefix { read: u8, len: u16 },
    Body(u16),
}

impl Skip {
    const START: Skip = Skip::Header(0);

    /// Feed one byte; true once a whole packet went by.
    fn push(&mut self, b: u8) -> bool {
        *self = match (*self, b) {
            (Skip::Header(m @ (0 | 1)), 0xFF) => Skip::Header(m + 1),
            (Skip::Header(2), 0xFF) => Skip::Header(2),
            (Skip::Header(2), 0xFD) => Skip::Header(3),
            (Skip::Header(3), 0x00) => Skip::Prefix { read: 0, len: 0 },
            (Skip::Header(_), 0xFF) => Skip::Header(1),
            (Skip::Header(_), _) => Skip::Header(0),
            // id, then the length on two bytes
            (Skip::Prefix { read: 0, .. }, _) => Skip::Prefix { read: 1, len: 0 },
            (Skip::Prefix { read: 1, .. }, lo) => Skip::Prefix {
                read: 2,
                len: lo.into(),
            },
            (Skip::Prefix { len, .. }, hi) => match len | u16::from(hi) << 8 {
                0 => Skip::START,
                len => Skip::Body(len),
            },
            (Skip::Body(1), _) => {
                *self = Skip::START;
                return true;
            }
            (Skip::Body(left), _) => Skip::Body(left - 1),
        };
        false
    }
}

#[derive(Debug, Clone, Copy)]
enum FastStage {
    /// Matched this many bytes of the expected header.
    Header(u8),
    /// In the block `done` (0-based), at byte `at` of its error, id and data.
    Block { done: u16, at: u16 },
    /// The block's CRC: its low byte once read.
    Crc { done: u16, low: Option<u8> },
}

/// A Protocol 2.0 device on the bus, holding packets of up to `N` bytes. See the
/// [module](self).
pub struct Device<const N: usize> {
    identity: Identity,
    decoder: Decoder<N>,
    turn: Turn,
}

impl<const N: usize> Device<N> {
    pub const fn new(identity: Identity) -> Self {
        Device {
            identity,
            decoder: Decoder::new(),
            turn: Turn::None,
        }
    }

    pub fn identity(&self) -> Identity {
        self.identity
    }

    /// Answer to another id from now on.
    pub fn set_id(&mut self, id: u8) {
        self.identity.id = id;
    }

    /// Whether the device is waiting for the devices listed before it in a Sync Read.
    pub fn is_waiting(&self) -> bool {
        !matches!(self.turn, Turn::None)
    }

    /// The bus has been quiet too long: give up any turn being waited for, and drop a
    /// half-read packet.
    pub fn bus_idle(&mut self) {
        self.turn = Turn::None;
        self.decoder.reset();
    }

    /// Feed one byte received from the bus; any reply is written to `out`.
    pub fn push(
        &mut self,
        b: u8,
        table: &mut (impl ControlTable + ?Sized),
        out: &mut [u8],
    ) -> Action {
        let id = self.identity.id;
        match &mut self.turn {
            Turn::None => {}
            Turn::Sync {
                left,
                addr,
                length,
                skip,
            } => {
                if skip.push(b) {
                    *left -= 1;
                    if *left == 0 {
                        let (addr, length) = (*addr, *length);
                        self.turn = Turn::None;
                        return reply(read_status(id, table, addr, length, out));
                    }
                }
                return Action::Nothing;
            }
            Turn::Fast {
                index,
                n_ids,
                addr,
                length,
                crc,
                stage,
            } => {
                return match fast_turn(b, *index, *n_ids, *length, crc, stage) {
                    FastStep::Wait => Action::Nothing,
                    FastStep::Abort => {
                        self.turn = Turn::None;
                        Action::Nothing
                    }
                    FastStep::Go => {
                        let (addr, length, mut crc) = (*addr, *length, *crc);
                        self.turn = Turn::None;
                        match fast_block(id, table, addr, length, &mut crc, out) {
                            Some(n) => Action::Append(n),
                            None => Action::Nothing,
                        }
                    }
                };
            }
        }

        let identity = self.identity;
        match self.decoder.push(b) {
            Some(Ok(packet)) => handle(identity, &mut self.turn, packet, table, out),
            _ => Action::Nothing,
        }
    }
}

fn reply(n: Option<usize>) -> Action {
    n.map_or(Action::Nothing, Action::Reply)
}

/// The status answering a read of `length` bytes at `addr`: the data, or the error.
fn read_status(
    id: u8,
    table: &mut (impl ControlTable + ?Sized),
    addr: u16,
    length: u16,
    out: &mut [u8],
) -> Option<usize> {
    let res = match table.read(addr, length) {
        Ok(data) => v2::status(out, id, 0, data),
        Err(code) => v2::status(out, id, code, &[]),
    };
    // Too long for `out`: say so rather than stay silent.
    res.or_else(|_| v2::status(out, id, error::DATA_LENGTH, &[]))
        .ok()
}

/// This device's block of a Fast Sync Read answer, zeros with the error when the table
/// cannot serve the read (so the blocks after it stay in place).
fn fast_block(
    id: u8,
    table: &mut (impl ControlTable + ?Sized),
    addr: u16,
    length: u16,
    crc: &mut u16,
    out: &mut [u8],
) -> Option<usize> {
    let (code, data) = match table.read(addr, length) {
        Ok(data) => (0, Some(data)),
        Err(code) => (code, None),
    };
    v2::fast_sync_block(out, id, code, data, length, crc).ok()
}

fn u16_at(params: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([params[i], params[i + 1]])
}

fn handle(
    identity: Identity,
    turn: &mut Turn,
    packet: Packet<'_>,
    table: &mut (impl ControlTable + ?Sized),
    out: &mut [u8],
) -> Action {
    let id = identity.id;
    let ours = packet.id == id;
    let broadcast = packet.id == v2::BROADCAST_ID;
    if !(ours || broadcast) {
        return Action::Nothing;
    }
    let params = packet.params;
    let error_status = |code: u8, out: &mut [u8]| reply(v2::status(out, id, code, &[]).ok());

    match packet.instruction {
        instruction::PING => {
            let [lo, hi] = identity.model_number.to_le_bytes();
            match v2::status(out, id, 0, &[lo, hi, identity.firmware_version]) {
                Ok(n) if broadcast => Action::BroadcastPingReply(n),
                Ok(n) => Action::Reply(n),
                Err(_) => Action::Nothing,
            }
        }
        instruction::READ if ours => {
            if params.len() < 4 {
                return error_status(error::DATA_LENGTH, out);
            }
            match u16_at(params, 2) {
                0 => error_status(error::DATA_RANGE, out),
                length => reply(read_status(id, table, u16_at(params, 0), length, out)),
            }
        }
        instruction::WRITE => {
            let res = if params.len() < 3 {
                Err(error::DATA_LENGTH)
            } else {
                table.write(u16_at(params, 0), &params[2..])
            };
            match (ours, res) {
                (false, _) => Action::Nothing,
                (true, Ok(())) => error_status(0, out),
                (true, Err(code)) => error_status(code, out),
            }
        }
        instruction::SYNC_WRITE if broadcast && params.len() >= 4 => {
            let addr = u16_at(params, 0);
            let length = usize::from(u16_at(params, 2));
            if let Some(chunk) = params[4..]
                .chunks_exact(length + 1)
                .find(|chunk| chunk[0] == id)
            {
                let _ = table.write(addr, &chunk[1..]);
            }
            Action::Nothing
        }
        instruction::SYNC_READ | instruction::FAST_SYNC_READ if broadcast && params.len() >= 5 => {
            let addr = u16_at(params, 0);
            let length = u16_at(params, 2);
            let ids = &params[4..];
            let Some(index) = ids.iter().position(|&i| i == id) else {
                return Action::Nothing;
            };
            let index = index as u16;
            if packet.instruction == instruction::SYNC_READ {
                if index == 0 {
                    return reply(read_status(id, table, addr, length, out));
                }
                *turn = Turn::Sync {
                    left: index,
                    addr,
                    length,
                    skip: Skip::START,
                };
                return Action::Nothing;
            }
            let n_ids = ids.len() as u16;
            if index == 0 {
                let Ok(header) = v2::fast_sync_header(out, ids.len(), length) else {
                    return Action::Nothing;
                };
                let mut crc = crate::crc::crc16(&out[..header]);
                return match fast_block(id, table, addr, length, &mut crc, &mut out[header..]) {
                    Some(n) => Action::Reply(header + n),
                    None => Action::Nothing,
                };
            }
            *turn = Turn::Fast {
                index,
                n_ids,
                addr,
                length,
                crc: 0,
                stage: FastStage::Header(0),
            };
            Action::Nothing
        }
        instruction::REBOOT if ours => match v2::status(out, id, 0, &[]) {
            Ok(n) => Action::RebootAfter(n),
            Err(_) => Action::Nothing,
        },
        instruction::FACTORY_RESET if ours => match v2::status(out, id, 0, &[]) {
            Ok(reply) => Action::FactoryResetAfter {
                reply,
                keep: params.first().copied().unwrap_or(0xFF),
            },
            Err(_) => Action::Nothing,
        },
        instruction::STATUS => Action::Nothing,
        _ if ours => error_status(error::INSTRUCTION, out),
        _ => Action::Nothing,
    }
}

enum FastStep {
    Wait,
    Abort,
    Go,
}

/// One byte of the Fast Sync Read answer of the devices before this one: the header the
/// first one sends, then `index` blocks, each checked against the running CRC so a lost
/// byte cannot shift this device's block out of place.
fn fast_turn(
    b: u8,
    index: u16,
    n_ids: u16,
    length: u16,
    crc: &mut u16,
    stage: &mut FastStage,
) -> FastStep {
    match *stage {
        FastStage::Header(at) => {
            let mut header = [0u8; v2::FAST_SYNC_HEADER_SIZE];
            if v2::fast_sync_header(&mut header, n_ids.into(), length).is_err()
                || header[usize::from(at)] != b
            {
                return FastStep::Abort;
            }
            *crc = crc16_update(*crc, b);
            *stage = if usize::from(at) + 1 == v2::FAST_SYNC_HEADER_SIZE {
                FastStage::Block { done: 0, at: 0 }
            } else {
                FastStage::Header(at + 1)
            };
        }
        FastStage::Block { done, at } => {
            *crc = crc16_update(*crc, b);
            *stage = if at + 1 == length + 2 {
                FastStage::Crc { done, low: None }
            } else {
                FastStage::Block { done, at: at + 1 }
            };
        }
        FastStage::Crc { done, low: None } => *stage = FastStage::Crc { done, low: Some(b) },
        FastStage::Crc {
            done,
            low: Some(low),
        } => {
            if u16::from_le_bytes([low, b]) != *crc {
                return FastStep::Abort;
            }
            *crc = crc16_update(crc16_update(*crc, low), b);
            if done + 1 == index {
                return FastStep::Go;
            }
            *stage = FastStage::Block {
                done: done + 1,
                at: 0,
            };
        }
    }
    FastStep::Wait
}

#[cfg(test)]
mod tests;
