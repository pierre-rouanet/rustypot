use core::future::{pending, Future};
use core::time::Duration;
use std::collections::VecDeque;

use embassy_futures::block_on;

use super::*;
use crate::{CommunicationErrorKind, SyncReadError};

/// A link that replays canned answers: each write loads the next one, which then comes
/// back a byte per read; an exhausted answer reads as `Ok(0)`, the end of the data.
/// `stale` bytes sit in the input before the first write, as a late answer would.
#[derive(Default)]
pub(crate) struct Link {
    answers: VecDeque<Vec<u8>>,
    pending: VecDeque<u8>,
    pub(crate) written: Vec<Vec<u8>>,
    pub(crate) delays: Vec<Duration>,
    /// Every timeout set, oldest first.
    pub(crate) timeouts: Vec<Duration>,
    timeout: Duration,
}

impl Link {
    pub(crate) fn new(answers: Vec<Vec<u8>>) -> Self {
        Link {
            answers: answers.into(),
            timeout: Duration::from_millis(10),
            ..Link::default()
        }
    }

    fn with_stale_input(mut self, stale: &[u8]) -> Self {
        self.pending.extend(stale);
        self
    }
}

impl AsyncTransport for Link {
    async fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, TransportError> {
        match (self.pending.pop_front(), buf.first_mut()) {
            (Some(b), Some(slot)) => {
                *slot = b;
                Ok(1)
            }
            _ => Ok(0),
        }
    }
    async fn write_all(&mut self, buf: &[u8]) -> core::result::Result<(), TransportError> {
        self.written.push(buf.to_vec());
        self.pending = self.answers.pop_front().unwrap_or_default().into();
        Ok(())
    }
    async fn discard_input(&mut self) -> core::result::Result<usize, TransportError> {
        let n = self.pending.len();
        self.pending.clear();
        Ok(n)
    }
    async fn delay(&mut self, duration: Duration) {
        self.delays.push(duration);
    }
    fn timeout(&self) -> Duration {
        self.timeout
    }
    fn set_timeout(&mut self, timeout: Duration) -> core::result::Result<(), TransportError> {
        self.timeouts.push(timeout);
        self.timeout = timeout;
        Ok(())
    }
}

/// Present position 2048 from STS3215 id 1, protocol v1.
const PRESENT_POSITION_2048: [u8; 8] = [0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2];

const SYNC_READ_RESPONSE: [u8; 21] = [
    0xFF, 0xFF, 0x0A, 0x03, 0x00, 0x20, 0xD2, // id 10
    0xFF, 0xFF, 0x0B, 0x03, 0x00, 0x20, 0xD1, // id 11
    0xFF, 0xFF, 0x0C, 0x03, 0x00, 0x20, 0xD0, // id 12
];

#[test]
fn a_read_sends_the_same_bytes_as_the_blocking_handler() {
    let mut link = Link::new(vec![PRESENT_POSITION_2048.to_vec()]);
    let dph = DynamixelProtocolHandler::v1();

    let value = block_on(dph.read(&mut link, 1, 56, 2)).unwrap();

    assert_eq!(value, [0x00, 0x08]);
    assert_eq!(
        link.written,
        [vec![0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]]
    );
}

#[test]
fn an_answer_cut_short_is_an_error_and_a_silent_motor_does_not_ping() {
    let mut link = Link::new(vec![PRESENT_POSITION_2048[..6].to_vec(), vec![]]);
    let dph = DynamixelProtocolHandler::v1();

    assert!(block_on(dph.read(&mut link, 1, 56, 2)).is_err());
    assert!(!block_on(dph.ping(&mut link, 1)).unwrap());
}

#[test]
fn a_sync_read_answers_in_the_order_asked_and_names_a_silent_motor() {
    let dph = DynamixelProtocolHandler::v1();

    let mut link = Link::new(vec![SYNC_READ_RESPONSE.to_vec()]);
    let values = block_on(dph.sync_read(&mut link, &[10, 11, 12], 43, 1)).unwrap();
    assert_eq!(values, [vec![0x20], vec![0x20], vec![0x20]]);

    let mut link = Link::new(vec![SYNC_READ_RESPONSE[..14].to_vec()]);
    let err = block_on(dph.sync_read(&mut link, &[10, 11, 12], 43, 1)).unwrap_err();
    assert_eq!(err.downcast_ref::<SyncReadError>().unwrap().id, 12);
}

#[test]
fn a_fast_sync_read_names_the_motor_that_did_not_answer() {
    // The e-manual's answer from motors 3, 7 and 4, with motor 4's block missing.
    let answer = [
        0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, //
        0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, //
        0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA,
    ];
    let dph = DynamixelProtocolHandler::v2().with_fast_sync_read();
    let mut link = Link::new(vec![answer.to_vec()]);

    let err = block_on(dph.sync_read(&mut link, &[3, 7, 4], 132, 4)).unwrap_err();

    // Fast sync read is instruction 0x8A.
    assert_eq!(link.written[0][7], 0x8A);
    let err = err.downcast_ref::<SyncReadError>().unwrap();
    assert_eq!(err.id, 4);
    assert_eq!(err.to_string(), "motor 4: Timeout error");
}

#[test]
fn fast_sync_read_is_refused_on_protocol_v1_without_touching_the_bus() {
    let mut link = Link::new(vec![]);
    let dph = DynamixelProtocolHandler::v1();

    assert!(block_on(dph.fast_sync_read(&mut link, &[1], 0, 1)).is_err());
    assert!(link.written.is_empty());
}

#[test]
fn the_post_delay_follows_reads_and_writes_but_not_pings() {
    let delay = Duration::from_millis(20);
    let dph = DynamixelProtocolHandler::v1().with_post_delay(delay);
    let mut link = Link::new(vec![
        vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC],
        PRESENT_POSITION_2048.to_vec(),
        vec![],
    ]);

    assert!(block_on(dph.ping(&mut link, 1)).unwrap());
    assert!(link.delays.is_empty());

    block_on(dph.read(&mut link, 1, 56, 2)).unwrap();
    // A transaction that failed has used the bus all the same.
    assert!(block_on(dph.read(&mut link, 1, 56, 2)).is_err());
    block_on(dph.sync_write(&mut link, &[1, 2], 42, &[vec![0], vec![1]])).unwrap();
    assert_eq!(link.delays, [delay, delay, delay]);
}

#[test]
fn leftovers_of_an_earlier_exchange_are_dropped_before_sending() {
    let mut link =
        Link::new(vec![PRESENT_POSITION_2048.to_vec()]).with_stale_input(&[0xFF, 0x00, 0x42]);
    let dph = DynamixelProtocolHandler::v1();

    let value = block_on(dph.read(&mut link, 1, 56, 2)).unwrap();

    assert_eq!(value, [0x00, 0x08]);
    // One pause for the rest of the old message, then a clean input.
    assert_eq!(link.delays, [Duration::from_millis(5)]);
}

#[test]
fn addresses_above_255_go_out_on_v2_and_are_refused_on_v1() {
    let mut link = Link::new(vec![]);
    let _ = block_on(DynamixelProtocolHandler::v2().read(&mut link, 1, 634, 2));
    assert_eq!(link.written[0][7..12], [0x02, 0x7A, 0x02, 0x02, 0x00]);

    let mut link = Link::new(vec![]);
    let err = block_on(DynamixelProtocolHandler::v1().write(&mut link, 1, 256, &[0])).unwrap_err();
    assert!(matches!(
        err.downcast_ref::<CommunicationErrorKind>(),
        Some(CommunicationErrorKind::AddressOutOfRange(256))
    ));
    assert!(link.written.is_empty());
}

#[test]
fn a_v1_factory_reset_cannot_keep_the_id() {
    let mut link = Link::new(vec![]);
    let dph = DynamixelProtocolHandler::v1();

    assert!(block_on(dph.factory_reset(&mut link, 1, true, false)).is_err());
    assert!(link.written.is_empty());
}

mod embedded_io_transport {
    use super::*;
    use embedded_io_async::{ErrorKind, ErrorType, Read, ReadReady, Write};

    /// A serial port whose reads either come from `rx` at once or never complete.
    #[derive(Default)]
    struct Port {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
        flushed: usize,
        silent: bool,
    }

    impl ErrorType for Port {
        type Error = ErrorKind;
    }

    impl Read for Port {
        async fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, ErrorKind> {
            if self.silent || self.rx.is_empty() {
                pending::<()>().await;
            }
            let n = buf.len().min(self.rx.len());
            for slot in &mut buf[..n] {
                *slot = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
    }

    impl ReadReady for Port {
        fn read_ready(&mut self) -> core::result::Result<bool, ErrorKind> {
            Ok(!self.rx.is_empty())
        }
    }

    impl Write for Port {
        async fn write(&mut self, buf: &[u8]) -> core::result::Result<usize, ErrorKind> {
            self.tx.extend_from_slice(buf);
            Ok(buf.len())
        }
        async fn flush(&mut self) -> core::result::Result<(), ErrorKind> {
            self.flushed += 1;
            Ok(())
        }
    }

    /// A delay that either elapses at once or never: enough to pick a branch of the
    /// read's race between data and timeout.
    struct Delay {
        elapses: bool,
        asked_us: Vec<u32>,
    }

    impl embedded_hal_async::delay::DelayNs for Delay {
        async fn delay_ns(&mut self, ns: u32) {
            self.asked_us.push(ns / 1_000);
            if !self.elapses {
                pending::<()>().await;
            }
        }
        async fn delay_us(&mut self, us: u32) {
            self.asked_us.push(us);
            if !self.elapses {
                pending::<()>().await;
            }
        }
    }

    fn run<F: Future>(f: F) -> F::Output {
        block_on(f)
    }

    #[test]
    fn a_read_returns_what_arrived() {
        let port = Port {
            rx: [1, 2, 3].into(),
            ..Port::default()
        };
        let delay = Delay {
            elapses: false,
            asked_us: vec![],
        };
        let mut link = EmbeddedIoTransport::new(port, delay, Duration::from_millis(10));

        let mut buf = [0u8; 8];
        assert_eq!(run(link.read(&mut buf)).unwrap(), 3);
        assert_eq!(buf[..3], [1, 2, 3]);
    }

    #[test]
    fn a_read_times_out_after_the_link_timeout() {
        let port = Port {
            silent: true,
            ..Port::default()
        };
        let delay = Delay {
            elapses: true,
            asked_us: vec![],
        };
        let mut link = EmbeddedIoTransport::new(port, delay, Duration::from_millis(10));

        let mut buf = [0u8; 8];
        assert!(matches!(
            run(link.read(&mut buf)),
            Err(TransportError::TimedOut)
        ));
        let (_, delay) = link.into_inner();
        assert_eq!(delay.asked_us, [10_000]);
    }

    #[test]
    fn a_write_is_flushed_and_stale_input_dropped() {
        let port = Port {
            rx: [9; 40].into(),
            ..Port::default()
        };
        let delay = Delay {
            elapses: true,
            asked_us: vec![],
        };
        let mut link = EmbeddedIoTransport::new(port, delay, Duration::from_millis(10));

        assert_eq!(run(link.discard_input()).unwrap(), 40);
        assert_eq!(run(link.discard_input()).unwrap(), 0);
        run(link.write_all(&[0xFF, 0xFF])).unwrap();
        let (port, _) = link.into_inner();
        assert_eq!(port.tx, [0xFF, 0xFF]);
        assert_eq!(port.flushed, 1);
    }

    #[test]
    fn listen_keeps_what_arrived_until_the_window_closes() {
        let port = Port {
            rx: [1, 2, 3, 4].into(),
            ..Port::default()
        };
        let delay = Delay {
            elapses: true,
            asked_us: vec![],
        };
        let mut link = EmbeddedIoTransport::new(port, delay, Duration::from_millis(10));

        let mut received = Vec::new();
        run(link.listen(Duration::from_millis(50), &mut received)).unwrap();

        assert_eq!(received, [1, 2, 3, 4]);
        let (_, delay) = link.into_inner();
        assert_eq!(delay.asked_us, [50_000]);
    }

    #[test]
    fn the_handler_reads_a_motor_through_it() {
        // The answer is already waiting: the instruction goes out, the reply comes back.
        // (A real UART only gets it after the write; the port here does not model that.)
        let port = Port::default();
        let delay = Delay {
            elapses: false,
            asked_us: vec![],
        };
        let mut link = EmbeddedIoTransport::new(port, delay, Duration::from_millis(10));
        let dph = DynamixelProtocolHandler::v1();

        // Nothing waiting before the write; the answer lands right after it.
        let mut wrapped = AnswerAfterWrite {
            inner: &mut link,
            answer: Some(PRESENT_POSITION_2048.to_vec()),
        };
        assert_eq!(run(dph.read(&mut wrapped, 1, 56, 2)).unwrap(), [0x00, 0x08]);
    }

    /// Puts `answer` in the port's input once the instruction has gone out.
    struct AnswerAfterWrite<'a> {
        inner: &'a mut EmbeddedIoTransport<Port, Delay>,
        answer: Option<Vec<u8>>,
    }

    impl AsyncTransport for AnswerAfterWrite<'_> {
        async fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, TransportError> {
            self.inner.read(buf).await
        }
        async fn write_all(&mut self, buf: &[u8]) -> core::result::Result<(), TransportError> {
            self.inner.write_all(buf).await?;
            if let Some(answer) = self.answer.take() {
                self.inner.io_mut().rx.extend(answer);
            }
            Ok(())
        }
        async fn discard_input(&mut self) -> core::result::Result<usize, TransportError> {
            self.inner.discard_input().await
        }
        async fn delay(&mut self, duration: Duration) {
            self.inner.delay(duration).await
        }
        fn timeout(&self) -> Duration {
            self.inner.timeout()
        }
        fn set_timeout(&mut self, timeout: Duration) -> core::result::Result<(), TransportError> {
            self.inner.set_timeout(timeout)
        }
    }
}

/// The generated async controllers against the blocking ones: same calls, same bytes on
/// the wire, same values back.
#[cfg(feature = "std")]
mod controllers {
    use super::*;
    use crate::fake_port::FakePort;
    use crate::servo::dynamixel::xl330;
    use crate::servo::feetech::sts3215;

    /// The e-manual's fast sync read answer: present position of motors 3, 7 and 4.
    const FAST_SYNC_READ_STATUS: [u8; 32] = [
        0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, //
        0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, //
        0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA, //
        0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E,
    ];

    /// An empty protocol v2 status from motor 1, as a write gets.
    const V2_ACK: [u8; 11] = [
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0xA1, 0x0C,
    ];

    #[test]
    fn a_write_goes_out_as_the_blocking_controller_sends_it() {
        let port = FakePort::new(vec![V2_ACK.to_vec(), V2_ACK.to_vec()]);
        let written = port.written();
        let mut blocking = xl330::Xl330Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v2();
        blocking.write_torque_enable(1, true).unwrap();
        blocking.write_goal_position(1, 0.5).unwrap();

        let mut c =
            xl330::asynch::Xl330Controller::new(Link::new(vec![V2_ACK.to_vec(), V2_ACK.to_vec()]));
        block_on(c.write_torque_enable(1, true)).unwrap();
        block_on(c.write_goal_position(1, 0.5)).unwrap();

        assert_eq!(c.into_transport().written, *written.lock().unwrap());
    }

    #[test]
    fn a_fast_sync_read_returns_what_the_blocking_controller_returns() {
        let port = FakePort::new(vec![FAST_SYNC_READ_STATUS.to_vec()]);
        let mut blocking = xl330::Xl330Controller::new()
            .with_serial_port(Box::new(port))
            .with_fast_sync_read();
        let expected = blocking.sync_read_present_position(&[3, 7, 4]).unwrap();

        let mut c =
            xl330::asynch::Xl330Controller::new(Link::new(vec![FAST_SYNC_READ_STATUS.to_vec()]))
                .with_fast_sync_read();
        let raw = block_on(c.sync_read_raw_present_position(&[3, 7, 4]));

        let mut c =
            xl330::asynch::Xl330Controller::new(Link::new(vec![FAST_SYNC_READ_STATUS.to_vec()]))
                .with_fast_sync_read();
        let positions = block_on(c.sync_read_present_position(&[3, 7, 4])).unwrap();

        assert_eq!(raw.unwrap(), [166, 2079, 1023]);
        assert_eq!(positions, expected);
    }

    #[test]
    fn a_converted_read_matches_the_blocking_one() {
        let port = FakePort::new(vec![PRESENT_POSITION_2048.to_vec()]);
        let written = port.written();
        let mut blocking = sts3215::Sts3215Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v1();
        let expected = blocking.read_present_position(1).unwrap();

        let mut c =
            sts3215::asynch::Sts3215Controller::new(Link::new(
                vec![PRESENT_POSITION_2048.to_vec()],
            ));
        let position = block_on(c.read_present_position(1)).unwrap();

        // The blocking controller wraps a single read in a Vec; the async one does not.
        assert_eq!(vec![position], expected);
        assert_eq!(c.into_transport().written, *written.lock().unwrap());
    }

    #[test]
    fn an_answer_of_the_wrong_size_is_an_error_not_a_panic() {
        // Motor 1 answers a 2-byte read with a single byte.
        let short = vec![0xFF, 0xFF, 0x01, 0x03, 0x00, 0x08, 0xF3];
        let mut c = sts3215::asynch::Sts3215Controller::new(Link::new(vec![short]));

        let err = block_on(c.read_raw_present_position(1)).unwrap_err();
        assert!(err.is::<CommunicationErrorKind>(), "{err}");
    }

    #[test]
    fn registers_by_name_match_the_blocking_controller() {
        // homing_offset of motor 1 as 0x0AC5 (sign-magnitude on bit 11: -709), then the
        // status of a write.
        let answers = vec![
            vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0xC5, 0x0A, 0x2B],
            vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC],
        ];
        let port = FakePort::new(answers.clone());
        let written = port.written();
        let mut blocking = sts3215::Sts3215Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v1();
        assert_eq!(blocking.read_register(1, "homing_offset").unwrap(), -709);
        blocking.write_register(1, "goal_position", -100).unwrap();

        let mut c = sts3215::asynch::Sts3215Controller::new(Link::new(answers));
        assert_eq!(block_on(c.read_register(1, "homing_offset")).unwrap(), -709);
        block_on(c.write_register(1, "goal_position", -100)).unwrap();

        assert_eq!(c.into_transport().written, *written.lock().unwrap());
    }

    #[test]
    fn a_bad_register_name_never_reaches_the_bus() {
        let mut c = sts3215::asynch::Sts3215Controller::new(Link::new(vec![]));
        let err = block_on(c.write_register(1, "no_such_register", 0)).unwrap_err();
        assert!(err.is::<crate::servo::RegisterError>(), "{err}");
        assert!(c.into_transport().written.is_empty());
    }

    #[test]
    fn a_scan_reports_the_ids_that_answer_and_puts_the_timeout_back() {
        // Only motor 2 answers, with model number 777.
        let mut c = sts3215::asynch::Sts3215Controller::new(Link::new(vec![
            vec![],
            vec![0xFF, 0xFF, 0x02, 0x04, 0x00, 0x09, 0x03, 0xED],
            vec![],
        ]));

        let found = block_on(c.scan(&[1, 2, 3], 1_000_000)).unwrap();

        assert_eq!(found, alloc::collections::BTreeMap::from([(2, 777)]));
        let link = c.into_transport();
        assert_eq!(
            link.timeouts,
            [
                crate::servo::scan_timeout(1_000_000),
                Duration::from_millis(10)
            ]
        );
    }

    #[test]
    fn a_broadcast_scan_pings_once_then_reads_the_motors_that_answered() {
        // Motors 2 and 5 answer the ping; 5 was not asked for. Motor 2 then gives 777.
        let mut c = sts3215::asynch::Sts3215Controller::new(Link::new(vec![
            [
                [0xFF, 0xFF, 0x02, 0x02, 0x00, 0xFB],
                [0xFF, 0xFF, 0x05, 0x02, 0x00, 0xF8],
            ]
            .concat(),
            vec![0xFF, 0xFF, 0x02, 0x04, 0x00, 0x09, 0x03, 0xED],
        ]));

        let found = block_on(c.broadcast_scan(&[1, 2, 3], 1_000_000)).unwrap();

        assert_eq!(found, alloc::collections::BTreeMap::from([(2, 777)]));
        let link = c.into_transport();
        assert_eq!(link.written.len(), 2);
        assert_eq!(link.written[0][2], 0xFE);
        // The default listen ran under the ping window, then put the timeout back.
        assert_eq!(
            link.timeouts,
            [
                DynamixelProtocolHandler::v1().broadcast_ping_window(1_000_000),
                Duration::from_millis(10)
            ]
        );
    }

    #[test]
    fn changing_an_id_writes_what_the_blocking_definition_writes() {
        let status = vec![0xFF, 0xFF, 0x09, 0x02, 0x00, 0xF4];
        let answers = vec![status.clone(), status.clone(), status];
        let mut port = FakePort::new(answers.clone());
        let written = port.written();
        sts3215::DEFINITION
            .change_id(&crate::DynamixelProtocolHandler::v1(), &mut port, 9, 3)
            .unwrap();

        let mut link = Link::new(answers);
        block_on(crate::asynch::definition::change_id(
            &sts3215::DEFINITION,
            &DynamixelProtocolHandler::v1(),
            &mut link,
            9,
            3,
        ))
        .unwrap();

        assert_eq!(link.written, *written.lock().unwrap());
    }

    #[test]
    fn a_mixed_sync_read_decodes_each_motor_through_its_definition() {
        // Present position 2048 from the STS3215 (little-endian), 16 from the SCS0009
        // (big-endian, no sync read: read one at a time).
        use crate::servo::feetech::scs0009;
        let sts = vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2];
        let scs = vec![0xFF, 0xFF, 0x02, 0x04, 0x00, 0x00, 0x10, 0xE9];
        let mut link = Link::new(vec![sts, scs]);

        let values = block_on(crate::asynch::definition::sync_read_register(
            &DynamixelProtocolHandler::v1(),
            &mut link,
            &[(1, sts3215::DEFINITION), (2, scs0009::DEFINITION)],
            "present_position",
        ))
        .unwrap();

        assert_eq!(values, [2048, 16]);
        assert_eq!(link.written.len(), 2);
    }

    #[test]
    fn a_controller_can_borrow_its_link() {
        let mut link = Link::new(vec![vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC]]);
        {
            let mut c = sts3215::asynch::Sts3215Controller::new(&mut link);
            assert!(block_on(c.ping(1)).unwrap());
        }
        assert_eq!(link.written.len(), 1);
    }
}

mod half_duplex {
    use super::*;
    use core::convert::Infallible;
    use std::cell::RefCell;
    use std::rc::Rc;

    /// Records pin levels and writes in one timeline.
    #[derive(Clone, Default)]
    struct Timeline(Rc<RefCell<Vec<&'static str>>>);

    struct Pin(Timeline);
    impl embedded_hal::digital::ErrorType for Pin {
        type Error = Infallible;
    }
    impl embedded_hal::digital::OutputPin for Pin {
        fn set_high(&mut self) -> core::result::Result<(), Infallible> {
            self.0 .0.borrow_mut().push("high");
            Ok(())
        }
        fn set_low(&mut self) -> core::result::Result<(), Infallible> {
            self.0 .0.borrow_mut().push("low");
            Ok(())
        }
    }

    /// A link whose UART stays busy for `busy_polls` polls after each write.
    struct Uart {
        link: Link,
        timeline: Timeline,
        busy_polls: usize,
    }
    impl AsyncTransport for Uart {
        async fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, TransportError> {
            self.link.read(buf).await
        }
        async fn write_all(&mut self, buf: &[u8]) -> core::result::Result<(), TransportError> {
            self.timeline.0.borrow_mut().push("write");
            self.busy_polls = 3;
            self.link.write_all(buf).await
        }
        async fn discard_input(&mut self) -> core::result::Result<usize, TransportError> {
            self.link.discard_input().await
        }
        async fn delay(&mut self, duration: Duration) {
            self.link.delay(duration).await
        }
        fn timeout(&self) -> Duration {
            self.link.timeout()
        }
        fn set_timeout(&mut self, timeout: Duration) -> core::result::Result<(), TransportError> {
            self.link.set_timeout(timeout)
        }
    }

    fn busy(uart: &mut Uart) -> bool {
        if uart.busy_polls == 0 {
            return false;
        }
        uart.busy_polls -= 1;
        uart.timeline.0.borrow_mut().push("busy");
        true
    }

    #[test]
    fn the_pin_transmits_until_the_uart_is_done_then_listens() {
        let timeline = Timeline::default();
        let uart = Uart {
            link: Link::new(vec![PRESENT_POSITION_2048.to_vec()]),
            timeline: timeline.clone(),
            busy_polls: 0,
        };
        let mut link = HalfDuplex::new(uart, Pin(timeline.clone())).with_tx_busy(busy);

        let value = block_on(DynamixelProtocolHandler::v1().read(&mut link, 1, 56, 2)).unwrap();

        assert_eq!(value, [0x00, 0x08]);
        assert_eq!(
            *timeline.0.borrow(),
            ["low", "high", "write", "busy", "busy", "busy", "low"]
        );
    }
}
