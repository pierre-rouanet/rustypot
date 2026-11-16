use core::time::Duration;

use embedded_hal::digital::OutputPin;

use super::{AsyncTransport, TransportError};
#[allow(unused_imports)]
use crate::prelude::*;

/// An [`AsyncTransport`] that drives the direction pin of a half-duplex buffer: high to
/// transmit, low to listen.
///
/// The bus has to be handed back to the motors as soon as the last stop bit is out, and no
/// sooner. When the link's `write_all` returns before that (embassy-rp's `BufferedUart`
/// flush returns with bytes still in the hardware FIFO, for instance), give
/// [`with_tx_busy`](Self::with_tx_busy) a function that says whether the UART is still
/// sending: the pin stays high until it answers `false`.
///
/// ```ignore
/// let link = HalfDuplex::new(EmbeddedIoTransport::new(uart, Delay, timeout), dir_pin)
///     .with_tx_busy(|link| link.io_mut().busy());
/// ```
pub struct HalfDuplex<L, P> {
    link: L,
    dir: P,
    tx_busy: fn(&mut L) -> bool,
}

impl<L, P: OutputPin> HalfDuplex<L, P> {
    /// `link`, with `dir` set low (listening) now and high around each write.
    pub fn new(link: L, mut dir: P) -> Self {
        let _ = dir.set_low();
        Self {
            link,
            dir,
            tx_busy: |_| false,
        }
    }

    /// Keep transmitting after `write_all` returns, until `tx_busy` says the UART is done.
    pub fn with_tx_busy(self, tx_busy: fn(&mut L) -> bool) -> Self {
        Self { tx_busy, ..self }
    }

    /// The wrapped link.
    pub fn link_mut(&mut self) -> &mut L {
        &mut self.link
    }

    /// The link and the pin back.
    pub fn into_inner(self) -> (L, P) {
        (self.link, self.dir)
    }
}

fn pin_error<E>(_: E) -> TransportError {
    TransportError::Io(embedded_io::ErrorKind::Other)
}

impl<L: AsyncTransport, P: OutputPin> AsyncTransport for HalfDuplex<L, P> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        self.link.read(buf).await
    }

    async fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        self.dir.set_high().map_err(pin_error)?;
        let res = self.link.write_all(buf).await;
        while (self.tx_busy)(&mut self.link) {
            embassy_futures::yield_now().await;
        }
        // Listen again whatever happened, so the motors can answer the next instruction.
        self.dir.set_low().map_err(pin_error)?;
        res
    }

    async fn discard_input(&mut self) -> Result<usize, TransportError> {
        self.link.discard_input().await
    }

    async fn delay(&mut self, duration: Duration) {
        self.link.delay(duration).await
    }

    fn timeout(&self) -> Duration {
        self.link.timeout()
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<(), TransportError> {
        self.link.set_timeout(timeout)
    }

    async fn listen(
        &mut self,
        window: Duration,
        received: &mut Vec<u8>,
    ) -> Result<(), TransportError> {
        self.link.listen(window, received).await
    }
}
