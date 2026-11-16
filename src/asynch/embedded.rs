use core::time::Duration;

use embassy_futures::select::{select, Either};
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, ReadReady, Write};

use super::{AsyncTransport, TransportError};
#[allow(unused_imports)]
use crate::prelude::*;

/// An [`AsyncTransport`] over an `embedded-io-async` serial port and an
/// `embedded-hal-async` delay.
///
/// With embassy, `io` is a UART that implements `Read`, `Write` and `ReadReady` (e.g. a
/// `BufferedUart`) and `delay` is `embassy_time::Delay`. Reads time out after `timeout`;
/// writes return once [`Write::flush`] says the bytes have left.
///
/// The bus direction of a half-duplex adapter is not handled here: use one that switches
/// on its own (TX-driven enable, as on the ROBOTIS and Waveshare boards), or wrap this in
/// an [`AsyncTransport`] of your own that drives the direction pin around `write_all`.
pub struct EmbeddedIoTransport<T, D> {
    io: T,
    delay: D,
    timeout: Duration,
}

impl<T, D> EmbeddedIoTransport<T, D> {
    /// The link over `io`, waiting at most `timeout` for each read.
    pub fn new(io: T, delay: D, timeout: Duration) -> Self {
        Self { io, delay, timeout }
    }

    /// The serial port, e.g. to change its baud rate.
    pub fn io_mut(&mut self) -> &mut T {
        &mut self.io
    }

    /// The serial port and the delay back.
    pub fn into_inner(self) -> (T, D) {
        (self.io, self.delay)
    }
}

/// `duration` in microseconds, saturating at what a `u32` holds (over an hour).
fn micros(duration: Duration) -> u32 {
    u32::try_from(duration.as_micros()).unwrap_or(u32::MAX)
}

impl<T, D> AsyncTransport for EmbeddedIoTransport<T, D>
where
    T: Read + Write + ReadReady,
    D: DelayNs,
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        let timeout = micros(self.timeout);
        match select(self.io.read(buf), self.delay.delay_us(timeout)).await {
            Either::First(read) => Ok(read?),
            Either::Second(()) => Err(TransportError::TimedOut),
        }
    }

    async fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        self.io.write_all(buf).await?;
        self.io.flush().await?;
        Ok(())
    }

    async fn discard_input(&mut self) -> Result<usize, TransportError> {
        let mut dropped = 0;
        let mut chunk = [0u8; 32];
        while self.io.read_ready()? {
            match self.io.read(&mut chunk).await? {
                0 => break,
                n => dropped += n,
            }
        }
        Ok(dropped)
    }

    async fn delay(&mut self, duration: Duration) {
        self.delay.delay_us(micros(duration)).await
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<(), TransportError> {
        self.timeout = timeout;
        Ok(())
    }

    /// Everything that arrives until `window` closes, exactly: the read loop races the
    /// delay. Bytes already read when the window closes are kept.
    async fn listen(
        &mut self,
        window: Duration,
        received: &mut Vec<u8>,
    ) -> Result<(), TransportError> {
        let io = &mut self.io;
        let collect = async {
            let mut chunk = [0u8; 64];
            loop {
                let n = io.read(&mut chunk).await?;
                if n == 0 {
                    return Ok::<(), TransportError>(());
                }
                received.extend_from_slice(&chunk[..n]);
            }
        };
        match select(collect, self.delay.delay_us(micros(window))).await {
            Either::First(res) => res,
            Either::Second(()) => Ok(()),
        }
    }
}
