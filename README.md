# c6-lvgl

Rust/Embassy firmware for the Waveshare ESP32-C6 Touch LCD 1.47 board. It boots the touch controller, starts the BLE stack scaffold, and logs both heartbeat messages and touch coordinates over RTT.

## Build & Run

- `cargo build` to compile the firmware with the configured esp-hal toolchain.
- `cargo run` (or flash via your usual workflow) and monitor RTT to see touch events.

Example RTT log while touching the panel:

```
INFO  Hello world!
INFO  touch: (171, 204)
INFO  touch: (155, 193)
INFO  touch: (162, 248)
```

Touch rotation defaults to `Rotate0`; adjust in `src/bin/main.rs` if you need a different orientation.
