use core::fmt;
use core::time::Duration;

/// Why an [`AsyncTransport`] operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    /// Nothing arrived within the link's timeout.
    TimedOut,
    /// The link itself failed, as an `embedded-io` error kind.
    Io(embedded_io::ErrorKind),
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            TransportError::TimedOut => write!(f, "Timeout error"),
            TransportError::Io(kind) => write!(f, "I/O error: {kind}"),
        }
    }
}

impl core::error::Error for TransportError {}

impl<E: embedded_io::Error> From<E> for TransportError {
    fn from(e: E) -> Self {
        TransportError::Io(e.kind())
    }
}

/// A half-duplex byte link to a bus of servos, as `async fn`s.
///
/// The async counterpart of the blocking [`Transport`](crate::Transport), slimmed to what
/// the async handler uses: the link owns its timeout and its baud rate, set up when it is
/// built. [`EmbeddedIoTransport`](super::EmbeddedIoTransport) implements it over any
/// `embedded-io-async` serial port (an embassy UART, say) and an `embedded-hal-async`
/// delay.
///
/// The futures carry no `Send` bound: on a multi-threaded executor, drive the bus from
/// one task.
#[allow(async_fn_in_trait)]
pub trait AsyncTransport {
    /// Read whatever arrives, up to `buf.len()` bytes.
    ///
    /// Wait until at least one byte is there, or the link's timeout elapses: then answer
    /// [`TransportError::TimedOut`] (or `Ok(0)`). The protocol relies on it to tell a motor
    /// that did not answer, and a short fast sync read, from a failure of the link.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Send all of `buf`, and return once it has left (so a half-duplex link can turn
    /// around and listen).
    async fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError>;

    /// Fill all of `buf`, failing with [`TransportError::TimedOut`] if the bytes stop
    /// coming before it is full.
    async fn read_exact(&mut self, mut buf: &mut [u8]) -> Result<(), TransportError> {
        while !buf.is_empty() {
            match self.read(buf).await? {
                0 => return Err(TransportError::TimedOut),
                n => buf = &mut buf[n..],
            }
        }
        Ok(())
    }

    /// Drop the bytes already received, without waiting for more, and say how many there
    /// were.
    ///
    /// Called before each instruction, so a late answer to an earlier one is never taken
    /// for the answer to this one.
    async fn discard_input(&mut self) -> Result<usize, TransportError>;

    /// Wait `duration`: the handler's post delay, and the pause between flush retries.
    async fn delay(&mut self, duration: Duration);
}

impl<T: AsyncTransport + ?Sized> AsyncTransport for &mut T {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        T::read(self, buf).await
    }
    async fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        T::write_all(self, buf).await
    }
    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), TransportError> {
        T::read_exact(self, buf).await
    }
    async fn discard_input(&mut self) -> Result<usize, TransportError> {
        T::discard_input(self).await
    }
    async fn delay(&mut self, duration: Duration) {
        T::delay(self, duration).await
    }
}
