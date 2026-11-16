//! The behaviour of imu_to_dxl's firmware (`firmware/test_native/test_dxl.cpp`), on
//! which this module is modelled, plus what it leaves out.

use std::vec::Vec;

use super::*;
use crate::v2::{parse_fast_sync_status, parse_status, Writer};

const ID: u8 = 200;
const IMU_ADDR: u16 = 124;

fn device() -> Device<128> {
    Device::new(Identity {
        id: ID,
        model_number: 1200,
        firmware_version: 6,
    })
}

/// A control table with the 12 IMU bytes at 124.
fn table() -> [u8; 150] {
    let mut table = [0u8; 150];
    for i in 0..12 {
        table[usize::from(IMU_ADDR) + i] = (0xA0 + i) as u8;
    }
    table
}

/// Feed `bytes`, and return the last action with what it asks to send.
fn feed<const N: usize>(
    device: &mut Device<N>,
    table: &mut [u8],
    bytes: &[u8],
) -> (Action, Vec<u8>) {
    let mut out = [0u8; 256];
    let mut last = Action::Nothing;
    for &b in bytes {
        let action = device.push(b, table, &mut out);
        if action != Action::Nothing {
            last = action;
        }
    }
    let sent = match last {
        Action::Nothing => vec![],
        Action::Reply(n)
        | Action::Append(n)
        | Action::BroadcastPingReply(n)
        | Action::RebootAfter(n)
        | Action::FactoryResetAfter { reply: n, .. } => out[..n].to_vec(),
    };
    (last, sent)
}

fn encode(f: impl FnOnce(&mut [u8]) -> Result<usize, crate::Error>) -> Vec<u8> {
    let mut buf = [0u8; 256];
    let n = f(&mut buf).unwrap();
    buf[..n].to_vec()
}

#[test]
fn a_ping_gets_the_model_and_firmware_a_foreign_one_nothing() {
    let (action, mut sent) = feed(&mut device(), &mut table(), &encode(|b| v2::ping(b, ID)));
    assert!(matches!(action, Action::Reply(_)));
    let status = parse_status(&mut sent, ID).unwrap();
    assert_eq!(status.params, [0xB0, 0x04, 6]);

    let (action, _) = feed(&mut device(), &mut table(), &encode(|b| v2::ping(b, 21)));
    assert_eq!(action, Action::Nothing);

    let (action, _) = feed(
        &mut device(),
        &mut table(),
        &encode(|b| v2::ping(b, v2::BROADCAST_ID)),
    );
    assert!(matches!(action, Action::BroadcastPingReply(_)));
}

#[test]
fn a_read_returns_the_table_or_a_range_error() {
    let mut t = table();
    let (_, mut sent) = feed(
        &mut device(),
        &mut t,
        &encode(|b| v2::read(b, ID, IMU_ADDR, 12)),
    );
    let status = parse_status(&mut sent, ID).unwrap();
    assert_eq!((status.error, status.params), (0, &t[124..136]));

    let (_, mut sent) = feed(&mut device(), &mut t, &encode(|b| v2::read(b, ID, 140, 20)));
    assert_eq!(
        parse_status(&mut sent, ID).unwrap().error,
        error::DATA_RANGE
    );

    // A read too short to say what to read.
    let short = encode(|b| {
        let mut w = Writer::new(b, ID, instruction::READ)?;
        w.extend(&[124, 0])?;
        w.finish()
    });
    let (_, mut sent) = feed(&mut device(), &mut t, &short);
    assert_eq!(
        parse_status(&mut sent, ID).unwrap().error,
        error::DATA_LENGTH
    );
}

#[test]
fn writes_reach_the_table_and_a_broadcast_one_is_not_answered() {
    let mut t = table();
    let (_, mut sent) = feed(
        &mut device(),
        &mut t,
        &encode(|b| v2::write(b, ID, 64, &[1, 2])),
    );
    assert_eq!(parse_status(&mut sent, ID).unwrap().error, 0);
    assert_eq!(t[64..66], [1, 2]);

    let (_, mut sent) = feed(
        &mut device(),
        &mut t,
        &encode(|b| v2::write(b, ID, 149, &[1, 2])),
    );
    assert_eq!(
        parse_status(&mut sent, ID).unwrap().error,
        error::DATA_RANGE
    );

    let (action, _) = feed(
        &mut device(),
        &mut t,
        &encode(|b| v2::write(b, v2::BROADCAST_ID, 70, &[9])),
    );
    assert_eq!((action, t[70]), (Action::Nothing, 9));
}

#[test]
fn a_sync_write_takes_this_device_s_slice_silently() {
    let mut t = table();
    let request = encode(|b| v2::sync_write(b, &[20, ID, 21], 64, &[[1, 1], [2, 2], [3, 3]]));
    let (action, _) = feed(&mut device(), &mut t, &request);
    assert_eq!((action, t[64..66].to_vec()), (Action::Nothing, vec![2, 2]));
}

#[test]
fn a_sync_read_at_index_0_answers_at_once() {
    let mut d = device();
    let request = encode(|b| v2::sync_read(b, &[ID, 20, 21], IMU_ADDR, 12));
    let (action, mut sent) = feed(&mut d, &mut table(), &request);
    assert!(matches!(action, Action::Reply(_)));
    assert_eq!(
        parse_status(&mut sent, ID).unwrap().params,
        &table()[124..136]
    );
    assert!(!d.is_waiting());

    // Not listed: nothing.
    let request = encode(|b| v2::sync_read(b, &[20, 21], IMU_ADDR, 12));
    let mut d = device();
    assert_eq!(feed(&mut d, &mut table(), &request).0, Action::Nothing);
    assert!(!d.is_waiting());
}

#[test]
fn a_sync_read_at_index_1_waits_for_the_answer_before_it() {
    let mut d = device();
    let mut t = table();
    let request = encode(|b| v2::sync_read(b, &[20, ID, 21], IMU_ADDR, 12));
    assert_eq!(feed(&mut d, &mut t, &request).0, Action::Nothing);
    assert!(d.is_waiting());

    // Motor 20's answer, with FF FF FD in its data so stuffing is on the wire.
    let answer = encode(|b| v2::status(b, 20, 0, &[0xFF, 0xFF, 0xFD, 1, 2, 3, 4, 5, 6, 7, 8, 9]));
    let (action, mut sent) = feed(&mut d, &mut t, &answer);
    assert!(matches!(action, Action::Reply(_)));
    assert_eq!(parse_status(&mut sent, ID).unwrap().params, &t[124..136]);
    assert!(!d.is_waiting());
}

#[test]
fn a_device_before_this_one_that_never_answers_leaves_it_silent() {
    let mut d = device();
    let request = encode(|b| v2::sync_read(b, &[20, ID], IMU_ADDR, 12));
    feed(&mut d, &mut table(), &request);
    assert!(d.is_waiting());

    d.bus_idle();
    assert!(!d.is_waiting());
    // And the next request is served normally.
    let (action, _) = feed(&mut d, &mut table(), &encode(|b| v2::ping(b, ID)));
    assert!(matches!(action, Action::Reply(_)));
}

#[test]
fn a_fast_sync_read_at_index_0_sends_the_header_and_its_block() {
    let mut t = table();
    let request = encode(|b| v2::fast_sync_read(b, &[ID, 20], IMU_ADDR, 12));
    let (action, sent) = feed(&mut device(), &mut t, &request);
    assert!(matches!(action, Action::Reply(_)));

    let mut expected = [0u8; 64];
    let mut n = v2::fast_sync_header(&mut expected, 2, 12).unwrap();
    let mut crc = crate::crc::crc16(&expected[..n]);
    n += v2::fast_sync_block(&mut expected[n..], ID, 0, Some(&t[124..136]), 12, &mut crc).unwrap();
    assert_eq!(sent, expected[..n]);
}

#[test]
fn a_fast_sync_read_at_index_1_appends_its_block_to_the_one_before() {
    let mut t = table();
    let request = encode(|b| v2::fast_sync_read(b, &[20, ID], IMU_ADDR, 12));
    // Motor 20's part: the header and its block (not stuffed, FF FF FD and all).
    let mut wire = [0u8; 64];
    let mut n = v2::fast_sync_header(&mut wire, 2, 12).unwrap();
    let mut crc = crate::crc::crc16(&wire[..n]);
    let d20 = [0xFF, 0xFF, 0xFD, 9, 8, 7, 6, 5, 4, 3, 2, 1];
    n += v2::fast_sync_block(&mut wire[n..], 20, 0, Some(&d20), 12, &mut crc).unwrap();

    let mut d = device();
    feed(&mut d, &mut t, &request);
    let (action, sent) = feed(&mut d, &mut t, &wire[..n]);
    assert!(matches!(action, Action::Append(_)));

    // Together they make one packet the controller accepts.
    let whole = [&wire[..n], &sent[..]].concat();
    let mut blocks = vec![];
    parse_fast_sync_status(&whole, &[20, ID], 12, |_, error, data| {
        blocks.push((error, data.to_vec()))
    })
    .unwrap();
    assert_eq!(blocks, [(0, d20.to_vec()), (0, t[124..136].to_vec())]);

    // A corrupted CRC in motor 20's block: the device stays out of it.
    let mut d = device();
    feed(&mut d, &mut t, &request);
    let mut bad = wire;
    bad[n - 1] ^= 0x55;
    assert_eq!(feed(&mut d, &mut t, &bad[..n]).0, Action::Nothing);
    assert!(!d.is_waiting());
}

#[test]
fn a_fast_sync_read_out_of_the_table_still_sends_a_block_of_the_right_size() {
    let mut t = table();
    let request = encode(|b| v2::fast_sync_read(b, &[ID], 145, 12));
    let (_, sent) = feed(&mut device(), &mut t, &request);
    let mut blocks = vec![];
    parse_fast_sync_status(&sent, &[ID], 12, |_, error, data| {
        blocks.push((error, data.to_vec()))
    })
    .unwrap();
    assert_eq!(blocks, [(error::DATA_RANGE, vec![0; 12])]);
}

#[test]
fn a_foreign_packet_too_large_to_hold_is_skipped_whole() {
    // A 15-motor sync write of goal positions (~89 bytes) through a 64-byte device:
    // dropped by its length, and the request right after it is still answered.
    let ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34];
    let data: Vec<[u8; 4]> = (0..15)
        .map(|m| [m * 7, m * 7 + 1, m * 7 + 2, m * 7 + 3])
        .collect();
    let big = encode(|b| v2::sync_write(b, &ids, 116, &data));
    assert!(big.len() > 64);

    let mut d: Device<64> = Device::new(device().identity());
    let mut t = table();
    let stream = [big, encode(|b| v2::ping(b, ID))].concat();
    let (action, _) = feed(&mut d, &mut t, &stream);
    assert!(matches!(action, Action::Reply(_)));
}

#[test]
fn reboot_factory_reset_and_unknown_instructions() {
    let (action, _) = feed(&mut device(), &mut table(), &encode(|b| v2::reboot(b, ID)));
    assert!(matches!(action, Action::RebootAfter(_)));

    let (action, _) = feed(
        &mut device(),
        &mut table(),
        &encode(|b| v2::factory_reset(b, ID, 0x01)),
    );
    assert!(matches!(
        action,
        Action::FactoryResetAfter { keep: 0x01, .. }
    ));

    let action_packet = encode(|b| Writer::new(b, ID, instruction::ACTION)?.finish());
    let (_, mut sent) = feed(&mut device(), &mut table(), &action_packet);
    assert_eq!(
        parse_status(&mut sent, ID).unwrap().error,
        error::INSTRUCTION
    );
}

#[test]
fn garbage_before_a_request_is_skipped() {
    let stream = [
        &[0x00, 0xFF, 0x13, 0xFF, 0xFF, 0xFF][..],
        &encode(|b| v2::ping(b, ID)),
    ]
    .concat();
    let (action, _) = feed(&mut device(), &mut table(), &stream);
    assert!(matches!(action, Action::Reply(_)));
}
