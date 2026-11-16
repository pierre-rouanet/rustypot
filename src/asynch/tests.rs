use core::future::{pending, Future};
use core::time::Duration;
use std::collections::VecDeque;

use embassy_futures::block_on;

use super::*;
use crate::SyncReadError;

/// A link that replays canned answers: each write loads the next one, which then comes
/// back a byte per read; an exhausted answer reads as `Ok(0)`, the end of the data.
/// `stale` bytes sit in the input before the first write, as a late answer would.
#[derive(Default)]
pub(crate) struct Link {
    answers: VecDeque<Vec<u8>>,
    pending: VecDeque<u8>,
    pub(crate) written: Vec<Vec<u8>>,
    pub(crate) delays: Vec<Duration>,
}

impl Link {
    pub(crate) fn new(answers: Vec<Vec<u8>>) -> Self {
        Link {
            answers: answers.into(),
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
        assert_eq!(run(link.read(&mut buf)), Ok(3));
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
        assert_eq!(run(link.read(&mut buf)), Err(TransportError::TimedOut));
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

        assert_eq!(run(link.discard_input()), Ok(40));
        assert_eq!(run(link.discard_input()), Ok(0));
        run(link.write_all(&[0xFF, 0xFF])).unwrap();
        let (port, _) = link.into_inner();
        assert_eq!(port.tx, [0xFF, 0xFF]);
        assert_eq!(port.flushed, 1);
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
    fn a_controller_can_borrow_its_link() {
        let mut link = Link::new(vec![vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC]]);
        {
            let mut c = sts3215::asynch::Sts3215Controller::new(&mut link);
            assert!(block_on(c.ping(1)).unwrap());
        }
        assert_eq!(link.written.len(), 1);
    }
}
