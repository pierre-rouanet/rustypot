//! Protocol v2 specifics; the packets are built and parsed by `rustypot_core::v2`.

/// The highest id a motor can take: 253 (0xFD) is reserved, 254 is the broadcast id.
pub(crate) const MAX_ID: u8 = rustypot_core::v2::MAX_ID;

#[derive(Debug, Clone, Copy)]
pub(crate) struct V2;
