# rustypot on a Raspberry Pi Pico (embassy)

Drives Dynamixel XL330 motors from an RP2040 with rustypot's async API, `no_std`:
it pings the motors, turns their torque on, then swings each one back and forth
around where it started while logging the positions over defmt.

## Wiring

The Dynamixel TTL bus is a single half-duplex data line. Connect the Pico to it
through a 3.3 V half-duplex buffer (e.g. a 74LVC2G241):

| Pico | to |
|---|---|
| GP0 (UART0 TX) | buffer, transmit side |
| GP1 (UART0 RX) | buffer, receive side |
| GP2 | buffer direction (high = transmit) |
| GND | motor GND |

The motors need their own supply (5 V for an XL330).

Boards that switch direction from the TX line on their own need no GP2: use the
`EmbeddedIoTransport` directly instead of wrapping it in `HalfDuplex`.

## Settings

At the top of `src/main.rs`: the motor ids (`IDS`), the bus speed (`BAUDRATE`,
57 600 bps out of the factory for an XL330) and the swing (`SWING`, radians).

## Run

With a debug probe (a second Pico running
[debugprobe](https://github.com/raspberrypi/debugprobe) works) and
[probe-rs](https://probe.rs):

```bash
rustup target add thumbv6m-none-eabi
```

```bash
cargo run --release
```

`cargo build --release` alone builds the firmware, into
`target/thumbv6m-none-eabi/release/rustypot-embassy-rp2040`.

## Half-duplex turnaround

`flush` on an embassy-rp `BufferedUart` returns once its software buffer is empty,
while the last bytes may still be in the UART's hardware FIFO. rustypot's `HalfDuplex`
is given `BufferedUart::busy` (`with_tx_busy`), so it keeps transmitting until the UART
is idle before switching the buffer back to receive; without that, the end of each
instruction is cut off and the motors never answer.
