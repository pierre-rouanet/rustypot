use core::fmt;

/// Why a packet could not be encoded or decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The output buffer cannot hold the packet.
    BufferTooSmall,
    /// The bytes do not frame a packet of this protocol: wrong header, a length that does
    /// not match, a status where an instruction was expected...
    Parsing,
    /// The checksum (v1) or CRC (v2) does not match.
    Checksum,
    /// The packet comes from motor `got`, not `expected`.
    IncorrectId { expected: u8, got: u8 },
    /// A register address the protocol cannot carry: v1 addresses stop at 255.
    AddressOutOfRange(u16),
    /// More parameters than the protocol's length field can count, or than a decoder
    /// can hold.
    TooLong,
    /// The packet stops before it is whole: what a read that timed out leaves.
    Truncated,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::BufferTooSmall => write!(f, "Buffer too small for the packet"),
            Error::Parsing => write!(f, "Parsing error"),
            Error::Checksum => write!(f, "Checksum error"),
            Error::IncorrectId { expected, got } => {
                write!(f, "Incorrect id ({got} instead of {expected})")
            }
            Error::AddressOutOfRange(addr) => {
                write!(
                    f,
                    "Register address {addr} is out of range for this protocol"
                )
            }
            Error::TooLong => write!(f, "Packet too long"),
            Error::Truncated => write!(f, "Packet cut short"),
        }
    }
}

impl core::error::Error for Error {}
