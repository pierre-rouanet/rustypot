//! Drive Dynamixel XL330s from a Raspberry Pi Pico with rustypot's async API.
//!
//! Wiring (3.3 V logic, through a half-duplex buffer such as a 74LVC2G241 to the
//! Dynamixel TTL data line):
//!
//! | Pico | |
//! |---|---|
//! | GP0 (UART0 TX) | buffer input, TX side |
//! | GP1 (UART0 RX) | buffer output, RX side |
//! | GP2 | buffer direction: high to transmit |
//!
//! Boards that switch direction on their own (TX-driven enable) need no GP2: replace
//! `HalfDuplex` by the `EmbeddedIoTransport` it wraps.
//!
//! It pings the motors in `IDS`, turns their torque on, then moves each one back and forth
//! by `SWING` radians around where it started, logging the positions over defmt.
//!
//! ```text
//! cargo run --release
//! ```

#![no_std]
#![no_main]

extern crate alloc;

use core::time::Duration as CoreDuration;

use defmt::{info, warn, Display2Format};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config};
use embassy_time::{Delay, Duration, Ticker};
use embedded_alloc::LlffHeap as Heap;
use rustypot::asynch::{EmbeddedIoTransport, HalfDuplex};
use rustypot::servo::dynamixel::xl330;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

/// The motors on the bus.
const IDS: [u8; 2] = [1, 2];
/// The bus speed. An XL330 leaves the factory at 57 600 bps.
const BAUDRATE: u32 = 57_600;
/// How far each motor swings either side of where it started, in radians.
const SWING: f64 = 0.2;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    {
        const HEAP_SIZE: usize = 16 * 1024;
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        // SAFETY: called once, before anything allocates.
        unsafe { HEAP.init(core::ptr::addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    let p = embassy_rp::init(Default::default());

    static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let mut config = Config::default();
    config.baudrate = BAUDRATE;
    let uart = BufferedUart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        TX_BUF.init([0; 256]),
        RX_BUF.init([0; 256]),
        config,
    );
    // `flush` on a `BufferedUart` returns once its software buffer is empty, while the
    // last bytes may still be in the UART's FIFO: keep transmitting until it is idle.
    let link = HalfDuplex::new(
        EmbeddedIoTransport::new(uart, Delay, CoreDuration::from_millis(10)),
        Output::new(p.PIN_2, Level::Low),
    )
    .with_tx_busy(|link| link.io_mut().busy());

    let mut motors = xl330::asynch::Xl330Controller::new(link);

    for id in IDS {
        match motors.ping(id).await {
            Ok(true) => info!("motor {} answered", id),
            Ok(false) => warn!("motor {} did not answer", id),
            Err(e) => warn!("ping {}: {}", id, Display2Format(&e)),
        }
    }

    let center = loop {
        match motors.sync_read_present_position(&IDS).await {
            Ok(positions) => break positions,
            Err(e) => warn!("reading the start positions: {}", Display2Format(&e)),
        }
    };
    if let Err(e) = motors
        .sync_write_torque_enable(&IDS, &[true; IDS.len()])
        .await
    {
        warn!("torque on: {}", Display2Format(&e));
    }

    let mut ticker = Ticker::every(Duration::from_millis(20));
    let mut tick: u32 = 0;
    loop {
        // One second each way.
        let side = if (tick / 50).is_multiple_of(2) {
            SWING
        } else {
            -SWING
        };
        let goals: alloc::vec::Vec<f64> = center.iter().map(|c| c + side).collect();
        if let Err(e) = motors.sync_write_goal_position(&IDS, &goals).await {
            warn!("goal: {}", Display2Format(&e));
        }
        if tick.is_multiple_of(10) {
            match motors.sync_read_present_position(&IDS).await {
                Ok(positions) => info!("present positions: {}", positions.as_slice()),
                Err(e) => warn!("present positions: {}", Display2Format(&e)),
            }
        }
        tick = tick.wrapping_add(1);
        ticker.next().await;
    }
}
