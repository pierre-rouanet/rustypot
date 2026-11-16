//! The checksums of both protocols.

/// Protocol v1 checksum: the inverted low byte of the sum of `data` (id to last
/// parameter).
pub fn checksum_v1(data: &[u8]) -> u8 {
    !data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

/// Protocol v2 CRC-16 (polynomial 0x8005, MSB first, initial value 0) of `data`.
pub fn crc16(data: &[u8]) -> u16 {
    crc16_continue(0, data)
}

/// Carry the CRC-16 `crc` on over `data`, for a packet that comes in pieces.
pub fn crc16_continue(crc: u16, data: &[u8]) -> u16 {
    data.iter().fold(crc, |crc, &b| crc16_update(crc, b))
}

/// Carry the CRC-16 `crc` on over one byte.
pub fn crc16_update(crc: u16, byte: u8) -> u16 {
    let i = ((crc >> 8) as u8 ^ byte) as usize;
    (crc << 8) ^ CRC_TABLE[i]
}

const CRC_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x8005
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_table_matches_the_robotis_one() {
        // First and last rows of the table in the ROBOTIS SDK.
        assert_eq!(CRC_TABLE[..4], [0x0000, 0x8005, 0x800F, 0x000A]);
        assert_eq!(CRC_TABLE[252..], [0x0208, 0x820D, 0x8207, 0x0202]);
    }

    #[test]
    fn the_crc_of_a_ping_matches_the_e_manual() {
        let data = [0xFF, 0xFF, 0xFD, 0x00, 0x2A, 0x03, 0x00, 0x01];
        assert_eq!(crc16(&data).to_le_bytes(), [0x16, 0xD2]);
        // In pieces, the same.
        assert_eq!(crc16_continue(crc16(&data[..3]), &data[3..]), crc16(&data));
    }

    #[test]
    fn the_v1_checksum_inverts_the_sum() {
        // Read of motor 1, address 0x2B, 1 byte: checksum 0xCC.
        assert_eq!(checksum_v1(&[0x01, 0x04, 0x02, 0x2B, 0x01]), 0xCC);
    }
}
