//! The blocking API runs the exchanges of `exchange.rs` over its [`Transport`], through a
//! link whose futures complete at once: [`block_on`] only ever polls each future once.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::io;
use std::time::{Duration, Instant};

use crate::link::{AsyncTransport, TransportError};
use crate::Transport;

/// Run `future`, which never waits: every await in it is on a [`BlockingLink`], whose IO
/// is done by the time it returns.
pub(crate) fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    match future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
    {
        Poll::Ready(output) => output,
        Poll::Pending => unreachable!("a blocking exchange never waits on a future"),
    }
}

/// A blocking [`Transport`] seen as an [`AsyncTransport`].
pub(crate) struct BlockingLink<'a, T: Transport + ?Sized> {
    port: &'a mut T,
}

impl<'a, T: Transport + ?Sized> BlockingLink<'a, T> {
    pub(crate) fn new(port: &'a mut T) -> Self {
        BlockingLink { port }
    }
}

/// A timeout is the protocol's business (a motor that did not answer); anything else is
/// the port's, and keeps the OS error.
fn link_error(e: io::Error) -> TransportError {
    match e.kind() {
        // `read_exact` over a port that read nothing before its timeout.
        io::ErrorKind::TimedOut | io::ErrorKind::UnexpectedEof => TransportError::TimedOut,
        _ => TransportError::Link(Box::new(e)),
    }
}

impl<T: Transport + ?Sized> AsyncTransport for BlockingLink<'_, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        loop {
            match self.port.read(buf) {
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                res => return res.map_err(link_error),
            }
        }
    }

    async fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(buf).map_err(link_error)
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), TransportError> {
        self.port.read_exact(buf).map_err(link_error)
    }

    async fn discard_input(&mut self) -> Result<usize, TransportError> {
        let n = self.port.bytes_to_read().map_err(link_error)?;
        if n > 0 {
            let mut stale = vec![0u8; n];
            self.port.read_exact(&mut stale).map_err(link_error)?;
        }
        Ok(n)
    }

    async fn delay(&mut self, duration: Duration) {
        std::thread::sleep(duration);
    }

    fn timeout(&self) -> Duration {
        self.port.timeout()
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<(), TransportError> {
        self.port.set_timeout(timeout).map_err(link_error)
    }

    /// Exactly `window`, against the clock, then the port's own timeout back.
    async fn listen(
        &mut self,
        window: Duration,
        received: &mut Vec<u8>,
    ) -> Result<(), TransportError> {
        let timeout = self.port.timeout();
        let deadline = Instant::now() + window;
        let mut chunk = [0u8; 256];
        let res = loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break Ok(());
            }
            if let Err(e) = self.port.set_timeout(left) {
                break Err(link_error(e));
            }
            match self.port.read(&mut chunk) {
                Ok(0) => break Ok(()),
                Ok(n) => received.extend_from_slice(&chunk[..n]),
                Err(e) if e.kind() == io::ErrorKind::TimedOut => break Ok(()),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => break Err(link_error(e)),
            }
        };
        self.port.set_timeout(timeout).map_err(link_error)?;
        res
    }
}
