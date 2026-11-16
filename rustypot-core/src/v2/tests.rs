use super::*;
use crate::crc::crc16;
use std::vec::Vec;

fn encode(f: impl FnOnce(&mut [u8]) -> Result<usize, Error>) -> ([u8; 128], usize) {
    let mut buf = [0u8; 128];
    let n = f(&mut buf).unwrap();
    (buf, n)
}

/// The e-manual's Fast Sync Read answer: present position of motors 3, 7 and 4.
const FAST_SYNC_READ_STATUS: [u8; 32] = [
    0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, //
    0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, //
    0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA, //
    0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E,
];

#[test]
fn instructions_are_encoded_as_rustypot_sends_them() {
    let (b, n) = encode(|b| ping(b, 2));
    assert_eq!(
        b[..n],
        [0xff, 0xff, 0xfd, 0x0, 0x2, 0x3, 0x0, 0x1, 0x19, 0x72]
    );
    let (b, n) = encode(|b| reboot(b, 2));
    assert_eq!(
        b[..n],
        [0xff, 0xff, 0xfd, 0x0, 0x2, 0x3, 0x0, 0x8, 0x2f, 0x72]
    );
    let (b, n) = encode(|b| read(b, 1, 0x2B, 2));
    assert_eq!(
        b[..n],
        [0xff, 0xff, 0xfd, 0x0, 0x1, 0x7, 0x0, 0x2, 0x2b, 0x0, 0x2, 0x0, 0x2e, 0xcd]
    );
}

#[test]
fn parameters_are_stuffed_and_the_length_counts_it() {
    let (b, _) = encode(|b| write(b, 1, 116, &[0xFF, 0xFF, 0xFD, 0x00]));
    assert_eq!(b[5..7], [0x0A, 0x00]);
    assert_eq!(b[8..14], [0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD]);

    // The scan restarts after a stuffed byte, and FF FF FF FD still matches.
    for (data, stuffed) in [
        (
            &[0xFF, 0xFF, 0xFD, 0xFD][..],
            &[0xFF, 0xFF, 0xFD, 0xFD, 0xFD][..],
        ),
        (&[0xFF, 0xFF, 0xFF, 0xFD], &[0xFF, 0xFF, 0xFF, 0xFD, 0xFD]),
        (&[1, 2, 0xFF, 0xFD, 3], &[1, 2, 0xFF, 0xFD, 3]),
    ] {
        let (mut b, n) = encode(|b| status(b, 1, 0, data));
        assert_eq!(&b[9..n - 2], stuffed);
        let p = parse_status(&mut b[..n], 1).unwrap();
        assert_eq!(p.params, data);
    }
}

#[test]
fn a_status_packet_round_trips_with_its_error_byte() {
    let (mut b, n) = encode(|b| status(b, 7, 0x83, &[0xA6, 0, 0, 0]));
    let s = parse_status(&mut b[..n], 7).unwrap();
    assert_eq!((s.id, s.error, s.params), (7, 0x83, &[0xA6, 0, 0, 0][..]));

    let (mut b, n) = encode(|b| status(b, 7, 0, &[]));
    assert_eq!(
        parse_status(&mut b[..n], 8),
        Err(Error::IncorrectId {
            expected: 8,
            got: 7
        })
    );
    b[n - 1] ^= 1;
    assert_eq!(parse(&mut b[..n]), Err(Error::Checksum));
    // An instruction is not a status.
    let (mut b, n) = encode(|b| ping(b, 7));
    assert_eq!(parse_status(&mut b[..n], 7), Err(Error::Parsing));
}

#[test]
fn a_buffer_too_small_is_reported_not_overrun() {
    let mut small = [0u8; 12];
    assert_eq!(
        write(&mut small, 1, 0, &[1, 2, 3]),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(ping(&mut small[..9], 1), Err(Error::BufferTooSmall));
}

#[test]
fn the_decoder_resyncs_and_skips_what_it_cannot_hold() {
    let (ping, n) = encode(|b| ping(b, 3));
    let (big, m) = encode(|b| write(b, 9, 0, &[0x42; 80]));
    let mut d: Decoder<32> = Decoder::new();
    let stream = [&[0x00, 0xFF, 0xFF, 0xFF][..], &big[..m], &ping[..n]].concat();
    let mut seen = vec![];
    for &b in &stream {
        if let Some(r) = d.push(b) {
            seen.push(r.map(|p| (p.id, p.instruction)));
        }
    }
    assert_eq!(seen, [Err(Error::TooLong), Ok((3, instruction::PING))]);
    assert!(d.is_idle());
}

#[test]
fn the_decoder_reports_a_bad_crc_and_carries_on() {
    let (mut bad, n) = encode(|b| ping(b, 3));
    bad[n - 1] ^= 1;
    let (good, m) = encode(|b| ping(b, 4));
    let mut d: Decoder<32> = Decoder::new();
    let results: Vec<_> = [&bad[..n], &good[..m]]
        .concat()
        .into_iter()
        .filter_map(|b| d.push(b).map(|r| r.map(|p| p.id)))
        .collect();
    assert_eq!(results, [Err(Error::Checksum), Ok(4)]);
}

#[test]
fn fast_sync_blocks_chain_into_the_e_manual_packet() {
    let mut buf = [0u8; 32];
    let mut n = fast_sync_header(&mut buf, 3, 4).unwrap();
    let mut crc = crc16(&buf[..n]);
    for (id, data) in [
        (3, [0xA6, 0, 0, 0]),
        (7, [0x1F, 0x08, 0, 0]),
        (4, [0xFF, 0x03, 0, 0]),
    ] {
        n += fast_sync_block(&mut buf[n..], id, 0, Some(&data), 4, &mut crc).unwrap();
    }
    assert_eq!(buf, FAST_SYNC_READ_STATUS);
    assert_eq!(crc, crc16(&FAST_SYNC_READ_STATUS));
}

#[test]
fn a_fast_sync_answer_splits_per_motor_and_names_the_first_one_missing() {
    let mut values = vec![];
    parse_fast_sync_status(&FAST_SYNC_READ_STATUS, &[3, 7, 4], 4, |i, error, data| {
        values.push((i, error, data.to_vec()))
    })
    .unwrap();
    assert_eq!(values[2], (2, 0, vec![0xFF, 0x03, 0x00, 0x00]));

    let cut = &FAST_SYNC_READ_STATUS[..24];
    assert_eq!(
        parse_fast_sync_status(cut, &[3, 7, 4], 4, |_, _, _| {}),
        Err(FastSyncError {
            index: Some(2),
            error: Error::Truncated
        })
    );
    let mut swapped = FAST_SYNC_READ_STATUS;
    swapped[15] ^= 1;
    assert_eq!(
        parse_fast_sync_status(&swapped, &[3, 7, 4], 4, |_, _, _| {}),
        Err(FastSyncError {
            index: Some(0),
            error: Error::Checksum
        })
    );
}
