//! Dynamixel Protocol 2.0.
//!
//! ```text
//! FF FF FD 00 | ID | LEN_L LEN_H | INSTRUCTION | PARAM 1 ... PARAM N | CRC_L CRC_H
//! ```
//!
//! `LEN` counts the instruction, the (stuffed) parameters and the CRC. A status packet is
//! an instruction packet whose instruction is [`instruction::STATUS`] and whose first
//! parameter is the error field. Whenever `FF FF FD` shows up in the parameters, the
//! sender inserts an extra `FD` after it, so the data can never read as a header; the
//! length and the CRC cover those stuffed bytes, and the receiver drops them after
//! checking the CRC.
//!
//! See <https://emanual.robotis.com/docs/en/dxl/protocol2/>.

use crate::crc::{crc16, crc16_update};
use crate::Error;

/// The id every device listens to.
pub const BROADCAST_ID: u8 = 0xFE;
/// The highest id a device can take: 253 is reserved, 254 is the broadcast id.
pub const MAX_ID: u8 = 252;
/// The four bytes every packet starts with.
pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
/// Header, id and length: what to read before knowing how long the rest is.
pub const HEADER_SIZE: usize = 7;
/// The smallest packet: header, id, length, instruction and CRC.
pub const MIN_PACKET_SIZE: usize = HEADER_SIZE + 3;

/// Instruction codes.
pub mod instruction {
    pub const PING: u8 = 0x01;
    pub const READ: u8 = 0x02;
    pub const WRITE: u8 = 0x03;
    pub const REG_WRITE: u8 = 0x04;
    pub const ACTION: u8 = 0x05;
    pub const FACTORY_RESET: u8 = 0x06;
    pub const REBOOT: u8 = 0x08;
    pub const CLEAR: u8 = 0x10;
    /// Not an instruction: what a status packet carries in the instruction field.
    pub const STATUS: u8 = 0x55;
    pub const SYNC_READ: u8 = 0x82;
    pub const SYNC_WRITE: u8 = 0x83;
    pub const FAST_SYNC_READ: u8 = 0x8A;
    pub const BULK_READ: u8 = 0x92;
    pub const BULK_WRITE: u8 = 0x93;
}

/// The error numbers of a status packet's error field (bits 0-6; bit 7 is
/// [`error::ALERT`]).
pub mod error {
    pub const RESULT_FAIL: u8 = 1;
    pub const INSTRUCTION: u8 = 2;
    pub const CRC: u8 = 3;
    pub const DATA_RANGE: u8 = 4;
    pub const DATA_LENGTH: u8 = 5;
    pub const DATA_LIMIT: u8 = 6;
    pub const ACCESS: u8 = 7;
    /// A hardware fault is latched; which one lives in the Hardware Error Status register.
    pub const ALERT: u8 = 0x80;
}

/// Tracks the `FF FF FD` pattern through the parameters, for stuffing.
#[derive(Debug, Clone, Copy, Default)]
struct Run(u8);

impl Run {
    /// Feed one byte of the original data; true when it completes `FF FF FD`, i.e. a
    /// stuffing `FD` goes right after it.
    fn completes_pattern(&mut self, b: u8) -> bool {
        let (next, complete) = match (self.0, b) {
            (0, 0xFF) | (1, 0xFF) => (self.0 + 1, false),
            (2, 0xFF) => (2, false), // FF FF FF keeps an FF FF suffix alive
            (2, 0xFD) => (0, true),
            (_, 0xFF) => (1, false),
            _ => (0, false),
        };
        self.0 = next;
        complete
    }
}

/// Writes one packet into a buffer: the header, then parameters one at a time (stuffed as
/// they go), then [`finish`](Writer::finish) fills in the length and the CRC.
///
/// ```
/// use rustypot_core::v2::{instruction, Writer};
///
/// let mut buf = [0u8; 16];
/// let mut w = Writer::new(&mut buf, 1, instruction::READ)?;
/// w.extend(&132u16.to_le_bytes())?;
/// w.extend(&4u16.to_le_bytes())?;
/// let n = w.finish()?;
/// assert_eq!(buf[..n], [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15]);
/// # Ok::<(), rustypot_core::Error>(())
/// ```
pub struct Writer<'a> {
    buf: &'a mut [u8],
    len: usize,
    run: Run,
}

impl<'a> Writer<'a> {
    /// Start a packet from `id` with `instruction` (or [`instruction::STATUS`]).
    pub fn new(buf: &'a mut [u8], id: u8, instruction: u8) -> Result<Self, Error> {
        if buf.len() < MIN_PACKET_SIZE {
            return Err(Error::BufferTooSmall);
        }
        buf[..4].copy_from_slice(&HEADER);
        buf[4] = id;
        buf[7] = instruction;
        Ok(Writer {
            buf,
            len: HEADER_SIZE + 1,
            run: Run::default(),
        })
    }

    /// Start a status packet from `id`, with the error field `error`.
    pub fn status(buf: &'a mut [u8], id: u8, error: u8) -> Result<Self, Error> {
        let mut w = Writer::new(buf, id, instruction::STATUS)?;
        w.push(error)?;
        Ok(w)
    }

    fn raw(&mut self, b: u8) -> Result<(), Error> {
        // Keep room for the CRC.
        if self.len + 2 >= self.buf.len() {
            return Err(Error::BufferTooSmall);
        }
        self.buf[self.len] = b;
        self.len += 1;
        Ok(())
    }

    /// Append one parameter byte.
    pub fn push(&mut self, b: u8) -> Result<(), Error> {
        self.raw(b)?;
        if self.run.completes_pattern(b) {
            self.raw(0xFD)?;
        }
        Ok(())
    }

    /// Append parameter bytes.
    pub fn extend(&mut self, bytes: &[u8]) -> Result<(), Error> {
        bytes.iter().try_for_each(|&b| self.push(b))
    }

    /// Fill in the length and the CRC: the packet is `buf[..n]`.
    pub fn finish(self) -> Result<usize, Error> {
        let length = u16::try_from(self.len - HEADER_SIZE + 2).map_err(|_| Error::TooLong)?;
        self.buf[5..7].copy_from_slice(&length.to_le_bytes());
        let crc = crc16(&self.buf[..self.len]);
        self.buf[self.len..self.len + 2].copy_from_slice(&crc.to_le_bytes());
        Ok(self.len + 2)
    }
}

/// A Ping of `id`.
pub fn ping(buf: &mut [u8], id: u8) -> Result<usize, Error> {
    Writer::new(buf, id, instruction::PING)?.finish()
}

/// A Reboot of `id`.
pub fn reboot(buf: &mut [u8], id: u8) -> Result<usize, Error> {
    Writer::new(buf, id, instruction::REBOOT)?.finish()
}

/// A Factory Reset of `id`: everything (`0xFF`), all but the id (`0x01`), or all but the
/// id and the baud rate (`0x02`).
pub fn factory_reset(buf: &mut [u8], id: u8, keep: u8) -> Result<usize, Error> {
    let mut w = Writer::new(buf, id, instruction::FACTORY_RESET)?;
    w.push(keep)?;
    w.finish()
}

/// A Read of `length` bytes from `addr` of `id`.
pub fn read(buf: &mut [u8], id: u8, addr: u16, length: u16) -> Result<usize, Error> {
    let mut w = Writer::new(buf, id, instruction::READ)?;
    w.extend(&addr.to_le_bytes())?;
    w.extend(&length.to_le_bytes())?;
    w.finish()
}

/// A Write of `data` to `addr` of `id`.
pub fn write(buf: &mut [u8], id: u8, addr: u16, data: &[u8]) -> Result<usize, Error> {
    let mut w = Writer::new(buf, id, instruction::WRITE)?;
    w.extend(&addr.to_le_bytes())?;
    w.extend(data)?;
    w.finish()
}

fn sync_read_like(
    buf: &mut [u8],
    instruction: u8,
    ids: &[u8],
    addr: u16,
    length: u16,
) -> Result<usize, Error> {
    let mut w = Writer::new(buf, BROADCAST_ID, instruction)?;
    w.extend(&addr.to_le_bytes())?;
    w.extend(&length.to_le_bytes())?;
    w.extend(ids)?;
    w.finish()
}

/// A Sync Read of `length` bytes from `addr` of each of `ids`.
pub fn sync_read(buf: &mut [u8], ids: &[u8], addr: u16, length: u16) -> Result<usize, Error> {
    sync_read_like(buf, instruction::SYNC_READ, ids, addr, length)
}

/// A Fast Sync Read: the request of a Sync Read, answered with one status packet for all.
pub fn fast_sync_read(buf: &mut [u8], ids: &[u8], addr: u16, length: u16) -> Result<usize, Error> {
    sync_read_like(buf, instruction::FAST_SYNC_READ, ids, addr, length)
}

/// A Sync Write of `data[i]` to `addr` of `ids[i]`. Every `data[i]` must have the same
/// length (it is sent once, for all).
pub fn sync_write<D: AsRef<[u8]>>(
    buf: &mut [u8],
    ids: &[u8],
    addr: u16,
    data: &[D],
) -> Result<usize, Error> {
    let length = data.first().map_or(0, |d| d.as_ref().len());
    if ids.len() != data.len() || data.iter().any(|d| d.as_ref().len() != length) {
        return Err(Error::Parsing);
    }
    let length = u16::try_from(length).map_err(|_| Error::TooLong)?;
    let mut w = Writer::new(buf, BROADCAST_ID, instruction::SYNC_WRITE)?;
    w.extend(&addr.to_le_bytes())?;
    w.extend(&length.to_le_bytes())?;
    for (&id, d) in ids.iter().zip(data) {
        w.push(id)?;
        w.extend(d.as_ref())?;
    }
    w.finish()
}

/// A status packet from `id`: the error field, then `params`.
pub fn status(buf: &mut [u8], id: u8, error: u8, params: &[u8]) -> Result<usize, Error> {
    let mut w = Writer::status(buf, id, error)?;
    w.extend(params)?;
    w.finish()
}

/// A decoded packet. For a status packet, `params[0]` is the error field; see
/// [`Packet::as_status`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet<'a> {
    pub id: u8,
    pub instruction: u8,
    /// De-stuffed.
    pub params: &'a [u8],
}

/// A decoded status packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status<'a> {
    pub id: u8,
    pub error: u8,
    pub params: &'a [u8],
}

impl<'a> Packet<'a> {
    /// This packet as a status packet, if it is one.
    pub fn as_status(&self) -> Option<Status<'a>> {
        match (self.instruction, self.params.split_first()) {
            (instruction::STATUS, Some((&error, params))) => Some(Status {
                id: self.id,
                error,
                params,
            }),
            _ => None,
        }
    }
}

/// The length field of a packet's first [`HEADER_SIZE`] bytes: how many bytes follow.
pub fn payload_size(header: &[u8]) -> Result<usize, Error> {
    if header.len() != HEADER_SIZE || header[..4] != HEADER {
        return Err(Error::Parsing);
    }
    Ok(u16::from_le_bytes([header[5], header[6]]) as usize)
}

/// Remove the stuffing of `data` in place; returns the new length. Mirrors the ROBOTIS SDK
/// `removeStuffing`: the scan restarts after a removed byte.
fn destuff(data: &mut [u8]) -> usize {
    let mut out = 0;
    let mut run = 0u8;
    for i in 0..data.len() {
        let b = data[i];
        if run == 3 {
            run = 0;
            if b == 0xFD {
                continue;
            }
        }
        run = match (run, b) {
            (0, 0xFF) | (1, 0xFF) => run + 1,
            (2, 0xFF) => 2,
            (2, 0xFD) => 3,
            (_, 0xFF) => 1,
            _ => 0,
        };
        data[out] = b;
        out += 1;
    }
    out
}

/// Decode a whole packet in `frame`, checking its header, length and CRC; the parameters
/// are de-stuffed in place.
pub fn parse(frame: &mut [u8]) -> Result<Packet<'_>, Error> {
    if frame.len() < MIN_PACKET_SIZE {
        return Err(Error::Parsing);
    }
    if payload_size(&frame[..HEADER_SIZE])? != frame.len() - HEADER_SIZE {
        return Err(Error::Parsing);
    }
    let n = frame.len();
    let crc = u16::from_le_bytes([frame[n - 2], frame[n - 1]]);
    if crc != crc16(&frame[..n - 2]) {
        return Err(Error::Checksum);
    }
    let params = destuff(&mut frame[HEADER_SIZE + 1..n - 2]);
    Ok(Packet {
        id: frame[4],
        instruction: frame[7],
        params: &frame[HEADER_SIZE + 1..HEADER_SIZE + 1 + params],
    })
}

/// Decode the status packet in `frame`, which must come from `id`.
pub fn parse_status(frame: &mut [u8], id: u8) -> Result<Status<'_>, Error> {
    let packet = parse(frame)?;
    if packet.id != id {
        return Err(Error::IncorrectId {
            expected: id,
            got: packet.id,
        });
    }
    packet.as_status().ok_or(Error::Parsing)
}

/// Where a [`Decoder`] stands in the packet it is reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// Matched this many bytes of [`HEADER`].
    Header(u8),
    /// Reading id and length.
    Prefix,
    /// Storing the rest of a packet of this total size.
    Body(usize),
    /// Dropping the rest of a packet too large to store: this many bytes left.
    Skip(usize),
}

/// Decodes packets from a stream of bytes, one byte at a time, holding at most `N` bytes.
///
/// It resyncs on the next header after anything that does not frame a packet. A packet
/// longer than `N` is skipped by its length (its parameters cannot be mistaken for a
/// header) and reported as [`Error::TooLong`].
pub struct Decoder<const N: usize> {
    buf: [u8; N],
    len: usize,
    stage: Stage,
}

impl<const N: usize> Default for Decoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Decoder<N> {
    pub const fn new() -> Self {
        Decoder {
            buf: [0; N],
            len: 0,
            stage: Stage::Header(0),
        }
    }

    /// Drop whatever was half read.
    pub fn reset(&mut self) {
        self.len = 0;
        self.stage = Stage::Header(0);
    }

    /// Whether the decoder is between packets.
    pub fn is_idle(&self) -> bool {
        self.stage == Stage::Header(0)
    }

    /// Feed one byte. Once it completes a packet, the packet (or why it is not one).
    pub fn push(&mut self, b: u8) -> Option<Result<Packet<'_>, Error>> {
        match self.stage {
            Stage::Header(matched) => {
                self.stage = Stage::Header(match (matched, b) {
                    (0 | 1, 0xFF) => matched + 1,
                    (2, 0xFF) => 2,
                    (2, 0xFD) => 3,
                    (3, 0x00) => {
                        self.buf[..4].copy_from_slice(&HEADER);
                        self.len = 4;
                        self.stage = Stage::Prefix;
                        return None;
                    }
                    (_, 0xFF) => 1,
                    _ => 0,
                });
                None
            }
            Stage::Prefix => {
                self.buf[self.len] = b;
                self.len += 1;
                if self.len == HEADER_SIZE {
                    let rest = u16::from_le_bytes([self.buf[5], self.buf[6]]) as usize;
                    self.stage = if rest < 3 {
                        self.len = 0;
                        Stage::Header(0)
                    } else if HEADER_SIZE + rest > N {
                        Stage::Skip(rest)
                    } else {
                        Stage::Body(HEADER_SIZE + rest)
                    };
                }
                None
            }
            Stage::Skip(left) => {
                if left > 1 {
                    self.stage = Stage::Skip(left - 1);
                    None
                } else {
                    self.reset();
                    Some(Err(Error::TooLong))
                }
            }
            Stage::Body(total) => {
                self.buf[self.len] = b;
                self.len += 1;
                if self.len < total {
                    return None;
                }
                self.stage = Stage::Header(0);
                self.len = 0;
                Some(parse(&mut self.buf[..total]))
            }
        }
    }
}

// --- Fast Sync Read -------------------------------------------------------------------

/// The header of a Fast Sync Read status packet: `FF FF FD 00 FE LEN_L LEN_H 55`.
pub const FAST_SYNC_HEADER_SIZE: usize = 8;

/// Why a Fast Sync Read status packet is not as asked, and at which device's block when
/// that can be told. The header goes out with the first device's block, so a header
/// problem is put on index 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FastSyncError {
    /// Position in the id list of the first block that is not there as asked.
    pub index: Option<usize>,
    /// [`Error::Truncated`] when the packet stops before that block is whole.
    pub error: Error,
}

/// The `LEN` field of a Fast Sync Read status packet for `n_ids` blocks of `length` bytes.
fn fast_sync_len(n_ids: usize, length: u16) -> Result<u16, Error> {
    u16::try_from(1 + n_ids * (usize::from(length) + 4)).map_err(|_| Error::TooLong)
}

/// Write the header of a Fast Sync Read answer (sent by the first device listed).
pub fn fast_sync_header(buf: &mut [u8], n_ids: usize, length: u16) -> Result<usize, Error> {
    let len = fast_sync_len(n_ids, length)?;
    let out = buf
        .get_mut(..FAST_SYNC_HEADER_SIZE)
        .ok_or(Error::BufferTooSmall)?;
    out[..4].copy_from_slice(&HEADER);
    out[4] = BROADCAST_ID;
    out[5..7].copy_from_slice(&len.to_le_bytes());
    out[7] = instruction::STATUS;
    Ok(FAST_SYNC_HEADER_SIZE)
}

/// Write one device's block of a Fast Sync Read answer: `ERR ID DATA CRC_L CRC_H`, not
/// stuffed.
///
/// `crc` is the running CRC over every byte of the packet before this block; on return it
/// also covers this block, CRC included, so blocks chain. `data: None` sends `length`
/// zeros, the block of a device that cannot serve the read: it keeps its size so the
/// blocks after it stay where the controller expects them.
pub fn fast_sync_block(
    buf: &mut [u8],
    id: u8,
    error: u8,
    data: Option<&[u8]>,
    length: u16,
    crc: &mut u16,
) -> Result<usize, Error> {
    let length = usize::from(length);
    let size = length + 4;
    let out = buf.get_mut(..size).ok_or(Error::BufferTooSmall)?;
    out[0] = error;
    out[1] = id;
    match data {
        Some(data) if data.len() == length => out[2..2 + length].copy_from_slice(data),
        Some(_) => return Err(Error::Parsing),
        None => out[2..2 + length].fill(0),
    }
    let mut c = *crc;
    for &b in &out[..2 + length] {
        c = crc16_update(c, b);
    }
    out[2 + length..].copy_from_slice(&c.to_le_bytes());
    for &b in &out[2 + length..] {
        c = crc16_update(c, b);
    }
    *crc = c;
    Ok(size)
}

/// Split the status packet answering a Fast Sync Read of `length` bytes from `ids` into one
/// block per id, in the order asked: `each(index, error, data)`.
///
/// Every block's CRC is checked (it covers the whole packet up to it), so a block that
/// slid out of place shows as a checksum error. A packet that stops short, which is what a
/// read that timed out leaves, is [`Error::Truncated`] at the first block missing.
pub fn parse_fast_sync_status<'a>(
    data: &'a [u8],
    ids: &[u8],
    length: u16,
    mut each: impl FnMut(usize, u8, &'a [u8]),
) -> Result<(), FastSyncError> {
    let at = |i: usize, error: Error| FastSyncError {
        index: Some(i),
        error,
    };
    let length = usize::from(length);
    let block_size = length + 4;
    let expected = FAST_SYNC_HEADER_SIZE + ids.len() * block_size;

    if data.len() > expected {
        return Err(FastSyncError {
            index: None,
            error: Error::Parsing,
        });
    }
    // Something answered, and it was not this packet's header.
    if data.len() >= 4 && data[..4] != HEADER {
        return Err(at(0, Error::Parsing));
    }
    if data.len() < FAST_SYNC_HEADER_SIZE {
        return Err(at(0, Error::Truncated));
    }
    let payload = u16::from_le_bytes([data[5], data[6]]) as usize;
    if data[4] != BROADCAST_ID
        || data[7] != instruction::STATUS
        || payload != expected - HEADER_SIZE
    {
        return Err(at(0, Error::Parsing));
    }

    for (i, &id) in ids.iter().enumerate() {
        let block = FAST_SYNC_HEADER_SIZE + i * block_size;
        let crc_at = block + 2 + length;
        if data.len() < crc_at + 2 {
            return Err(at(i, Error::Truncated));
        }
        let crc = u16::from_le_bytes([data[crc_at], data[crc_at + 1]]);
        if crc != crc16(&data[..crc_at]) {
            return Err(at(i, Error::Checksum));
        }
        if data[block + 1] != id {
            return Err(at(
                i,
                Error::IncorrectId {
                    expected: id,
                    got: data[block + 1],
                },
            ));
        }
        each(i, data[block], &data[block + 2..crc_at]);
    }
    Ok(())
}

#[cfg(test)]
mod tests;
