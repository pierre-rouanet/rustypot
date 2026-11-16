//! Dynamixel Protocol 1.0, also spoken by the Feetech STS and SCS series.
//!
//! ```text
//! FF FF | ID | LEN | INSTRUCTION or ERROR | PARAM 1 ... PARAM N | CHECKSUM
//! ```
//!
//! `LEN` counts the parameters plus two; the checksum is the inverted low byte of the sum
//! from the id on. Status and instruction packets share the layout: a status packet
//! carries its error field where an instruction packet has the instruction. Register
//! addresses are one byte.
//!
//! See <https://emanual.robotis.com/docs/en/dxl/protocol1/>.

use crate::crc::checksum_v1;
use crate::Error;

/// The id every device listens to.
pub const BROADCAST_ID: u8 = 0xFE;
/// The highest id a device can take: everything below the broadcast id.
pub const MAX_ID: u8 = 253;
/// Header, id and length: what to read before knowing how long the rest is.
pub const HEADER_SIZE: usize = 4;

/// Instruction codes.
pub mod instruction {
    pub const PING: u8 = 0x01;
    pub const READ: u8 = 0x02;
    pub const WRITE: u8 = 0x03;
    pub const REG_WRITE: u8 = 0x04;
    pub const ACTION: u8 = 0x05;
    pub const FACTORY_RESET: u8 = 0x06;
    pub const REBOOT: u8 = 0x08;
    pub const SYNC_READ: u8 = 0x82;
    pub const SYNC_WRITE: u8 = 0x83;
    pub const BULK_READ: u8 = 0x92;
}

/// The bits of a status packet's error field.
pub mod error {
    pub const INPUT_VOLTAGE: u8 = 1 << 0;
    pub const ANGLE_LIMIT: u8 = 1 << 1;
    pub const OVERHEATING: u8 = 1 << 2;
    pub const RANGE: u8 = 1 << 3;
    pub const CHECKSUM: u8 = 1 << 4;
    pub const OVERLOAD: u8 = 1 << 5;
    pub const INSTRUCTION: u8 = 1 << 6;
}

/// A register address as protocol v1 carries it, on one byte.
pub fn address(addr: u16) -> Result<u8, Error> {
    u8::try_from(addr).map_err(|_| Error::AddressOutOfRange(addr))
}

/// Writes one packet into a buffer; see [`v2::Writer`](crate::v2::Writer).
pub struct Writer<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> Writer<'a> {
    /// Start a packet from `id` with `instruction`, or a status packet with its error
    /// field in its place.
    pub fn new(buf: &'a mut [u8], id: u8, instruction: u8) -> Result<Self, Error> {
        if buf.len() < HEADER_SIZE + 2 {
            return Err(Error::BufferTooSmall);
        }
        buf[..2].copy_from_slice(&[0xFF, 0xFF]);
        buf[2] = id;
        buf[4] = instruction;
        Ok(Writer { buf, len: 5 })
    }

    /// Append one parameter byte.
    pub fn push(&mut self, b: u8) -> Result<(), Error> {
        // Keep room for the checksum.
        if self.len + 1 >= self.buf.len() {
            return Err(Error::BufferTooSmall);
        }
        self.buf[self.len] = b;
        self.len += 1;
        Ok(())
    }

    /// Append parameter bytes.
    pub fn extend(&mut self, bytes: &[u8]) -> Result<(), Error> {
        bytes.iter().try_for_each(|&b| self.push(b))
    }

    /// Fill in the length and the checksum: the packet is `buf[..n]`.
    pub fn finish(self) -> Result<usize, Error> {
        self.buf[3] = u8::try_from(self.len - 3).map_err(|_| Error::TooLong)?;
        self.buf[self.len] = checksum_v1(&self.buf[2..self.len]);
        Ok(self.len + 1)
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

/// A Factory Reset of `id`.
pub fn factory_reset(buf: &mut [u8], id: u8) -> Result<usize, Error> {
    Writer::new(buf, id, instruction::FACTORY_RESET)?.finish()
}

/// A Read of `length` bytes from `addr` of `id`.
pub fn read(buf: &mut [u8], id: u8, addr: u16, length: u8) -> Result<usize, Error> {
    let mut w = Writer::new(buf, id, instruction::READ)?;
    w.extend(&[address(addr)?, length])?;
    w.finish()
}

/// A Write of `data` to `addr` of `id`.
pub fn write(buf: &mut [u8], id: u8, addr: u16, data: &[u8]) -> Result<usize, Error> {
    let mut w = Writer::new(buf, id, instruction::WRITE)?;
    w.push(address(addr)?)?;
    w.extend(data)?;
    w.finish()
}

/// A Sync Read of `length` bytes from `addr` of each of `ids` (Feetech, and Dynamixel
/// adapters that support it).
pub fn sync_read(buf: &mut [u8], ids: &[u8], addr: u16, length: u8) -> Result<usize, Error> {
    let mut w = Writer::new(buf, BROADCAST_ID, instruction::SYNC_READ)?;
    w.extend(&[address(addr)?, length])?;
    w.extend(ids)?;
    w.finish()
}

/// A Sync Write of `data[i]` to `addr` of `ids[i]`. Every `data[i]` must have the same
/// length.
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
    let mut w = Writer::new(buf, BROADCAST_ID, instruction::SYNC_WRITE)?;
    w.push(address(addr)?)?;
    w.push(u8::try_from(length).map_err(|_| Error::TooLong)?)?;
    for (&id, d) in ids.iter().zip(data) {
        w.push(id)?;
        w.extend(d.as_ref())?;
    }
    w.finish()
}

/// A status packet from `id`: the error field, then `params`.
pub fn status(buf: &mut [u8], id: u8, error: u8, params: &[u8]) -> Result<usize, Error> {
    let mut w = Writer::new(buf, id, error)?;
    w.extend(params)?;
    w.finish()
}

/// A decoded packet: `code` is the instruction of an instruction packet, the error field
/// of a status packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet<'a> {
    pub id: u8,
    pub code: u8,
    pub params: &'a [u8],
}

/// The length field of a packet's first [`HEADER_SIZE`] bytes: how many bytes follow.
pub fn payload_size(header: &[u8]) -> Result<usize, Error> {
    if header.len() == HEADER_SIZE && header[0] == 0xFF && header[1] == 0xFF {
        Ok(header[3].into())
    } else {
        Err(Error::Parsing)
    }
}

/// Decode a whole packet in `frame`, checking its header, length and checksum.
pub fn parse(frame: &[u8]) -> Result<Packet<'_>, Error> {
    if frame.len() < HEADER_SIZE + 2 {
        return Err(Error::Parsing);
    }
    let n = frame.len();
    if frame[n - 1] != checksum_v1(&frame[2..n - 1]) {
        return Err(Error::Checksum);
    }
    if payload_size(&frame[..HEADER_SIZE])? != n - HEADER_SIZE {
        return Err(Error::Parsing);
    }
    Ok(Packet {
        id: frame[2],
        code: frame[4],
        params: &frame[5..n - 1],
    })
}

/// Decode the status packet in `frame`, which must come from `id`.
pub fn parse_status(frame: &[u8], id: u8) -> Result<Packet<'_>, Error> {
    let packet = parse(frame)?;
    if packet.id != id {
        return Err(Error::IncorrectId {
            expected: id,
            got: packet.id,
        });
    }
    Ok(packet)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode(f: impl FnOnce(&mut [u8]) -> Result<usize, Error>) -> ([u8; 64], usize) {
        let mut buf = [0u8; 64];
        let n = f(&mut buf).unwrap();
        (buf, n)
    }

    #[test]
    fn instructions_are_encoded_as_rustypot_sends_them() {
        let (b, n) = encode(|b| ping(b, 1));
        assert_eq!(b[..n], [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]);
        let (b, n) = encode(|b| read(b, 1, 0x2B, 1));
        assert_eq!(b[..n], [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC]);
        let (b, n) = encode(|b| write(b, 1, 0x2A, &[0x64, 0x80]));
        assert_eq!(
            b[..n],
            [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x64, 0x80, 0xE8]
        );
        let (b, n) = encode(|b| sync_write(b, &[11, 12], 30, &[[0x0, 0x0], [0xA, 0x14]]));
        assert_eq!(
            b[..n],
            [0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x1E, 0x02, 0x0B, 0x00, 0x00, 0x0C, 0x0A, 0x14, 0x1F]
        );
    }

    #[test]
    fn addresses_above_255_are_refused() {
        let mut buf = [0u8; 16];
        assert_eq!(
            read(&mut buf, 1, 256, 1),
            Err(Error::AddressOutOfRange(256))
        );
    }

    #[test]
    fn a_status_packet_round_trips() {
        let (b, n) = encode(|b| status(b, 1, error::OVERLOAD, &[0x20]));
        let p = parse_status(&b[..n], 1).unwrap();
        assert_eq!((p.code, p.params), (error::OVERLOAD, &[0x20][..]));
        assert_eq!(
            parse_status(&b[..n], 2),
            Err(Error::IncorrectId {
                expected: 2,
                got: 1
            })
        );
        let mut bad = b;
        bad[n - 1] ^= 1;
        assert_eq!(parse(&bad[..n]), Err(Error::Checksum));
    }
}
