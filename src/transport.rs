//! The byte link the protocol talks over.
//!
//! rustypot only needs a handful of operations from the link to the motors: send bytes,
//! read bytes with a timeout, see whether stale bytes are waiting, and know (or change)
//! the baud rate the bus runs at. [`Transport`] names exactly those, so the protocol can
//! run over anything that provides them: a [`serialport`] port (implemented here for
//! every [`serialport::SerialPort`]), a bridge over the network, a test double...

use std::io;
use std::time::Duration;

/// A half-duplex byte link to a bus of servos.
///
/// Every [`serialport::SerialPort`] is a `Transport`, so the existing
/// `serial_port.as_mut()` calls keep working. Implement it directly for any other link.
///
/// The trait is object safe: controllers and [`Bus`](crate::bus::Bus) keep their link as
/// a `Box<dyn Transport + Send>`.
pub trait Transport {
    /// Read whatever arrives, up to `buf.len()` bytes.
    ///
    /// Block until at least one byte is available or the [timeout](Self::timeout)
    /// elapses. A timeout must be reported as an [`io::ErrorKind::TimedOut`] error (or as
    /// `Ok(0)`): the protocol relies on it to tell a motor that did not answer, and a
    /// short fast sync read, from a failure of the link itself.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;

    /// Send all of `buf`.
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()>;

    /// Fill all of `buf`, failing with [`io::ErrorKind::TimedOut`] if the bytes stop
    /// coming before it is full.
    fn read_exact(&mut self, mut buf: &mut [u8]) -> io::Result<()> {
        while !buf.is_empty() {
            match self.read(buf) {
                Ok(0) => return Err(io::ErrorKind::TimedOut.into()),
                Ok(n) => buf = &mut buf[n..],
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Number of received bytes waiting to be read.
    ///
    /// Used before each instruction to drop the leftovers of an earlier exchange, so an
    /// answer is never matched with the wrong request.
    fn bytes_to_read(&mut self) -> io::Result<usize>;

    /// How long [`read`](Self::read) waits for a byte.
    fn timeout(&self) -> Duration;

    /// Change how long [`read`](Self::read) waits for a byte.
    ///
    /// Broadcast ping and scanning lengthen it for their listening window, then put the
    /// previous value back.
    fn set_timeout(&mut self, timeout: Duration) -> io::Result<()>;

    /// The bus speed, in bits per second.
    ///
    /// Used to size listening windows to the time the answers take on the wire.
    fn baud_rate(&self) -> io::Result<u32>;

    /// Switch the bus speed.
    ///
    /// A link with no notion of baud rate may answer [`io::ErrorKind::Unsupported`].
    fn set_baud_rate(&mut self, baud_rate: u32) -> io::Result<()>;
}

impl<T: serialport::SerialPort + ?Sized> Transport for T {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        io::Read::read(self, buf)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        io::Write::write_all(self, buf)
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> io::Result<()> {
        io::Read::read_exact(self, buf)
    }

    fn bytes_to_read(&mut self) -> io::Result<usize> {
        Ok(serialport::SerialPort::bytes_to_read(self)? as usize)
    }

    fn timeout(&self) -> Duration {
        serialport::SerialPort::timeout(self)
    }

    fn set_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        Ok(serialport::SerialPort::set_timeout(self, timeout)?)
    }

    fn baud_rate(&self) -> io::Result<u32> {
        Ok(serialport::SerialPort::baud_rate(self)?)
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> io::Result<()> {
        Ok(serialport::SerialPort::set_baud_rate(self, baud_rate)?)
    }
}

/// An owned serial port, as a [`Transport`] that can be boxed.
///
/// `Box<dyn SerialPort>` is not itself a `SerialPort`, so it cannot be turned into a
/// `Box<dyn Transport + Send>` directly; this wraps it.
pub(crate) struct BoxedSerialPort(pub(crate) Box<dyn serialport::SerialPort>);

impl BoxedSerialPort {
    pub(crate) fn boxed(port: Box<dyn serialport::SerialPort>) -> Box<dyn Transport + Send> {
        Box::new(BoxedSerialPort(port))
    }
}

impl Transport for BoxedSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        Transport::read(self.0.as_mut(), buf)
    }
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        Transport::write_all(self.0.as_mut(), buf)
    }
    fn read_exact(&mut self, buf: &mut [u8]) -> io::Result<()> {
        Transport::read_exact(self.0.as_mut(), buf)
    }
    fn bytes_to_read(&mut self) -> io::Result<usize> {
        Transport::bytes_to_read(self.0.as_mut())
    }
    fn timeout(&self) -> Duration {
        Transport::timeout(self.0.as_ref())
    }
    fn set_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        Transport::set_timeout(self.0.as_mut(), timeout)
    }
    fn baud_rate(&self) -> io::Result<u32> {
        Transport::baud_rate(self.0.as_ref())
    }
    fn set_baud_rate(&mut self, baud_rate: u32) -> io::Result<()> {
        Transport::set_baud_rate(self.0.as_mut(), baud_rate)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, VecDeque};
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::bus::Bus;
    use crate::servo::feetech::sts3215;
    use crate::DynamixelProtocolHandler;

    /// What a [`Link`] saw, kept outside it so a test can look after giving it away.
    #[derive(Default)]
    struct Wire {
        written: Vec<Vec<u8>>,
        baud_rate: u32,
    }

    /// A link that is a [`Transport`] and nothing else, in particular not a
    /// `SerialPort`. Each write loads the next scripted answer, which then comes back
    /// one byte per read; an exhausted answer reads as `Ok(0)`, the end of the data.
    struct Link {
        answers: VecDeque<Vec<u8>>,
        pending: VecDeque<u8>,
        wire: Arc<Mutex<Wire>>,
    }

    impl Link {
        fn new(answers: Vec<Vec<u8>>) -> (Self, Arc<Mutex<Wire>>) {
            let wire = Arc::new(Mutex::new(Wire {
                baud_rate: 1_000_000,
                ..Wire::default()
            }));
            let link = Link {
                answers: answers.into(),
                pending: VecDeque::new(),
                wire: Arc::clone(&wire),
            };
            (link, wire)
        }
    }

    impl Transport for Link {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            match (self.pending.pop_front(), buf.first_mut()) {
                (Some(b), Some(slot)) => {
                    *slot = b;
                    Ok(1)
                }
                _ => Ok(0),
            }
        }
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            self.wire.lock().unwrap().written.push(buf.to_vec());
            self.pending = self.answers.pop_front().unwrap_or_default().into();
            Ok(())
        }
        fn bytes_to_read(&mut self) -> io::Result<usize> {
            Ok(0)
        }
        fn timeout(&self) -> Duration {
            Duration::from_millis(10)
        }
        fn set_timeout(&mut self, _: Duration) -> io::Result<()> {
            Ok(())
        }
        fn baud_rate(&self) -> io::Result<u32> {
            Ok(self.wire.lock().unwrap().baud_rate)
        }
        fn set_baud_rate(&mut self, baud_rate: u32) -> io::Result<()> {
            self.wire.lock().unwrap().baud_rate = baud_rate;
            Ok(())
        }
    }

    /// Present position 2048 from STS3215 id 1, protocol v1.
    const PRESENT_POSITION_2048: [u8; 8] = [0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2];

    #[test]
    fn the_protocol_handler_talks_over_any_transport() {
        let (mut link, wire) = Link::new(vec![PRESENT_POSITION_2048.to_vec()]);
        let dph = DynamixelProtocolHandler::v1();

        // A concrete link, passed as is: no SerialPort, no trait object.
        assert_eq!(dph.read(&mut link, 1, 56, 2).unwrap(), [0x00, 0x08]);
        assert_eq!(
            wire.lock().unwrap().written[0],
            [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]
        );
    }

    #[test]
    fn a_link_that_runs_dry_reads_as_a_motor_that_did_not_answer() {
        // The answer stops two bytes short: read_exact must give up, not spin.
        let short = PRESENT_POSITION_2048[..6].to_vec();
        let (mut link, _) = Link::new(vec![short, vec![]]);
        let dph = DynamixelProtocolHandler::v1();

        assert!(dph.read(&mut link, 1, 56, 2).is_err());
        assert!(!dph.ping(&mut link, 1).unwrap());
    }

    #[test]
    fn a_controller_runs_over_a_boxed_transport() {
        let (link, wire) = Link::new(vec![PRESENT_POSITION_2048.to_vec()]);
        let mut c = sts3215::Sts3215Controller::new()
            .with_transport(Box::new(link))
            .with_protocol_v1();

        assert_eq!(c.read_register(1, "present_position").unwrap(), 2048);
        c.set_baudrate(500_000).unwrap();
        assert_eq!(wire.lock().unwrap().baud_rate, 500_000);
    }

    #[test]
    fn a_bus_runs_over_a_boxed_transport() {
        let (link, wire) = Link::new(vec![PRESENT_POSITION_2048.to_vec()]);
        let mut bus =
            Bus::from_transport(Box::new(link), BTreeMap::from([(1, sts3215::DEFINITION)]));

        assert_eq!(bus.read_register(1, "present_position").unwrap(), 2048);
        assert_eq!(wire.lock().unwrap().written.len(), 1);
    }
}
