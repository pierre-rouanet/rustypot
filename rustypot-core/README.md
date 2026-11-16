# rustypot-core

Dynamixel (and Feetech) protocol packets for Rust, without a heap and without IO:
`no_std`, no `alloc`, no dependencies.

- **`v1`, `v2`**: encode instructions and status packets into a buffer you own, decode
  them whole or a byte at a time (`v2::Decoder`), with the checksums, Protocol 2.0 byte
  stuffing, and Fast Sync Read for both sides of the bus.
- **`device`**: answer a Protocol 2.0 controller as a servo would, from a control table
  you provide: Ping, Read, Write, Sync Write, Sync Read and Fast Sync Read (waiting for its
  turn behind the devices listed before it), Reboot and Factory Reset. It is fed the bus a
  byte at a time and says what to send, so it fits a microcontroller's main loop: about
  3 KB of flash and a few hundred bytes of RAM on a Cortex-M0+.

[`rustypot`](https://crates.io/crates/rustypot) builds its motor controllers on it.

```rust
use rustypot_core::device::{Action, Device, Identity};

// A sensor board that passes for an XL330 (model 1200) at id 200.
let mut device: Device<128> = Device::new(Identity { id: 200, model_number: 1200, firmware_version: 1 });
let mut table = [0u8; 160];
let mut out = [0u8; 64];

// For each byte received from the bus:
# let byte = 0u8;
match device.push(byte, &mut table, &mut out) {
    Action::Reply(n) => { /* after the return delay, send out[..n] */ }
    Action::Append(n) => { /* send out[..n] now (Fast Sync Read) */ }
    _ => {}
}
```

## License

Apache-2.0
