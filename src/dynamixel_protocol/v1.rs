//! Protocol v1 specifics that rustypot exposes: the error bits of a status packet.
//! The packets themselves are built and parsed by `rustypot_core::v1`.

#[allow(unused_imports)]
use crate::prelude::*;

/// The highest id a motor can take: everything below the broadcast id.
pub(crate) const MAX_ID: u8 = rustypot_core::v1::MAX_ID;

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum DynamixelErrorV1 {
    Instruction,
    Overload,
    Checksum,
    Range,
    Overheating,
    AngleLimit,
    InputVoltage,
}
impl DynamixelErrorV1 {
    pub(crate) fn from_byte(error: u8) -> Vec<Self> {
        (0..7)
            .filter(|i| error & (1 << i) != 0)
            .map(|i| DynamixelErrorV1::from_bit(i).unwrap())
            .collect()
    }
    fn from_bit(b: u8) -> Option<Self> {
        match b {
            6 => Some(DynamixelErrorV1::Instruction),
            5 => Some(DynamixelErrorV1::Overload),
            4 => Some(DynamixelErrorV1::Checksum),
            3 => Some(DynamixelErrorV1::Range),
            2 => Some(DynamixelErrorV1::Overheating),
            1 => Some(DynamixelErrorV1::AngleLimit),
            0 => Some(DynamixelErrorV1::InputVoltage),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct V1;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_error_bits_read_as_conditions() {
        assert_eq!(
            DynamixelErrorV1::from_byte(0x24),
            vec![DynamixelErrorV1::Overheating, DynamixelErrorV1::Overload]
        );
    }
}
