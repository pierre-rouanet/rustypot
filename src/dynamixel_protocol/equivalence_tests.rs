//! rustypot-core against the packet code it replaces, over generated inputs: the same
//! bytes out, the same values in. Temporary: the old code goes once this passes.

use super::packet::{Packet, StatusPacket};
use super::v1::PacketV1;
use super::v2::PacketV2;
use rustypot_core::{v1 as cv1, v2 as cv2};
#[allow(unused_imports)]
use StatusPacket as _;

/// xorshift: deterministic, so a failure reproduces.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn byte(&mut self) -> u8 {
        // Plenty of FF and FD, so stuffing patterns come up often.
        match self.next() % 4 {
            0 => 0xFF,
            1 => 0xFD,
            _ => self.next() as u8,
        }
    }
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.byte()).collect()
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn core_bytes(f: impl FnOnce(&mut [u8]) -> Result<usize, rustypot_core::Error>) -> Vec<u8> {
    let mut buf = [0u8; 2048];
    let n = f(&mut buf).unwrap();
    buf[..n].to_vec()
}

const ROUNDS: usize = 2000;

#[test]
fn v1_instructions_are_byte_identical() {
    let mut r = Rng(0x9E3779B97F4A7C15);
    for _ in 0..ROUNDS {
        let id = r.byte();
        let addr = r.below(256) as u16;
        let len = r.byte();
        let n = r.below(20) as usize + 1;
        let data = r.bytes(n);
        let n_ids = r.below(10) as usize + 1;
        let ids = r.bytes(n_ids);
        let per = r.below(6) as usize + 1;
        let values: Vec<Vec<u8>> = (0..n_ids).map(|_| r.bytes(per)).collect();

        assert_eq!(
            PacketV1::ping_packet(id).to_bytes(),
            core_bytes(|b| cv1::ping(b, id))
        );
        assert_eq!(
            PacketV1::reboot_packet(id).to_bytes(),
            core_bytes(|b| cv1::reboot(b, id))
        );
        assert_eq!(
            PacketV1::factory_reset_packet(id, false, false).to_bytes(),
            core_bytes(|b| cv1::factory_reset(b, id))
        );
        assert_eq!(
            PacketV1::read_packet(id, addr, len).unwrap().to_bytes(),
            core_bytes(|b| cv1::read(b, id, addr, len))
        );
        assert_eq!(
            PacketV1::write_packet(id, addr, &data).unwrap().to_bytes(),
            core_bytes(|b| cv1::write(b, id, addr, &data))
        );
        assert_eq!(
            PacketV1::sync_read_packet(&ids, addr, len)
                .unwrap()
                .to_bytes(),
            core_bytes(|b| cv1::sync_read(b, &ids, addr, len))
        );
        assert_eq!(
            PacketV1::sync_write_packet(&ids, addr, &values)
                .unwrap()
                .to_bytes(),
            core_bytes(|b| cv1::sync_write(b, &ids, addr, &values))
        );
    }
}

#[test]
fn v2_instructions_are_byte_identical() {
    let mut r = Rng(0xD1B54A32D192ED03);
    for _ in 0..ROUNDS {
        let id = r.byte();
        let addr = r.next() as u16;
        let len = r.byte();
        let n = r.below(40) as usize + 1;
        let data = r.bytes(n);
        let n_ids = r.below(12) as usize + 1;
        let ids = r.bytes(n_ids);
        let per = r.below(8) as usize + 1;
        let values: Vec<Vec<u8>> = (0..n_ids).map(|_| r.bytes(per)).collect();
        let (only, and_baud) = (r.below(2) == 1, r.below(2) == 1);
        let keep = match (only, and_baud) {
            (false, false) => 0xFF,
            (true, false) => 0x01,
            _ => 0x02,
        };

        assert_eq!(
            PacketV2::ping_packet(id).to_bytes(),
            core_bytes(|b| cv2::ping(b, id))
        );
        assert_eq!(
            PacketV2::reboot_packet(id).to_bytes(),
            core_bytes(|b| cv2::reboot(b, id))
        );
        assert_eq!(
            PacketV2::factory_reset_packet(id, only, and_baud).to_bytes(),
            core_bytes(|b| cv2::factory_reset(b, id, keep))
        );
        assert_eq!(
            PacketV2::read_packet(id, addr, len).unwrap().to_bytes(),
            core_bytes(|b| cv2::read(b, id, addr, len.into()))
        );
        assert_eq!(
            PacketV2::write_packet(id, addr, &data).unwrap().to_bytes(),
            core_bytes(|b| cv2::write(b, id, addr, &data))
        );
        assert_eq!(
            PacketV2::sync_read_packet(&ids, addr, len)
                .unwrap()
                .to_bytes(),
            core_bytes(|b| cv2::sync_read(b, &ids, addr, len.into()))
        );
        assert_eq!(
            PacketV2::fast_sync_read_packet(&ids, addr, len).to_bytes(),
            core_bytes(|b| cv2::fast_sync_read(b, &ids, addr, len.into()))
        );
        assert_eq!(
            PacketV2::sync_write_packet(&ids, addr, &values)
                .unwrap()
                .to_bytes(),
            core_bytes(|b| cv2::sync_write(b, &ids, addr, &values))
        );
    }
}

#[test]
fn status_packets_parse_the_same() {
    let mut r = Rng(0x2545F4914F6CDD1D);
    for round in 0..ROUNDS {
        let id = r.byte();
        let error = r.byte();
        let n = r.below(30) as usize;
        let params = r.bytes(n);
        let mut frame = core_bytes(|b| cv2::status(b, id, error, &params));
        let mut v1_frame = core_bytes(|b| cv1::status(b, id, error, &params));
        // Every fourth one corrupted somewhere past the header, which the reader checks
        // before handing the packet over (the old parsers assert on it).
        if round % 4 == 0 {
            let i = 4 + r.below(frame.len() as u64 - 4) as usize;
            frame[i] ^= 1 << r.below(8);
            let j = 2 + r.below(v1_frame.len() as u64 - 2) as usize;
            v1_frame[j] ^= 1 << r.below(8);
        }
        let asked = if r.below(8) == 0 {
            id.wrapping_add(1)
        } else {
            id
        };

        let old =
            PacketV2::status_packet(&frame, asked).map(|s| (s.error_byte(), s.params().clone()));
        let new =
            cv2::parse_status(&mut frame.clone(), asked).map(|s| (s.error, s.params.to_vec()));
        assert_eq!(old.ok(), new.ok(), "v2 round {round}");

        let old =
            PacketV1::status_packet(&v1_frame, asked).map(|s| (s.error_byte(), s.params().clone()));
        let new = cv1::parse_status(&v1_frame, asked).map(|s| (s.code, s.params.to_vec()));
        assert_eq!(old.ok(), new.ok(), "v1 round {round}");
    }
}

#[test]
fn fast_sync_answers_parse_the_same() {
    let mut r = Rng(0x94D049BB133111EB);
    for round in 0..ROUNDS {
        let n = r.below(6) as usize + 1;
        let length = r.below(8) as u16 + 1;
        let ids: Vec<u8> = (0..n).map(|i| i as u8 + 1).collect();
        let mut packet = vec![0u8; 8 + n * (usize::from(length) + 4)];
        let mut at = cv2::fast_sync_header(&mut packet, n, length).unwrap();
        let mut crc = rustypot_core::crc::crc16(&packet[..at]);
        for &id in &ids {
            let data = r.bytes(length.into());
            at += cv2::fast_sync_block(
                &mut packet[at..],
                id,
                r.byte(),
                Some(&data),
                length,
                &mut crc,
            )
            .unwrap();
        }
        match round % 3 {
            0 => packet.truncate(r.below(packet.len() as u64) as usize),
            1 => {
                let i = r.below(packet.len() as u64) as usize;
                packet[i] ^= 1 << r.below(8);
            }
            _ => {}
        }

        let old = super::v2::parse_fast_sync_read_status_with_error(&packet, &ids, length as u8)
            .map_err(|e| e.to_string());
        let mut blocks = vec![];
        let new = cv2::parse_fast_sync_status(&packet, &ids, length, |_, e, d| {
            blocks.push((d.to_vec(), e))
        })
        .map(|()| blocks);
        // Same values, and failures at the same block.
        assert_eq!(old.as_ref().ok(), new.as_ref().ok(), "round {round}");
        if let (Err(old), Err(new)) = (&old, &new) {
            let named = old.strip_prefix("motor ").and_then(|s| s.split(':').next());
            let index = new.index.and_then(|i| ids.get(i)).map(|id| id.to_string());
            assert_eq!(
                named.map(str::to_string),
                index,
                "round {round}: {old} / {new:?}"
            );
        }
    }
}
