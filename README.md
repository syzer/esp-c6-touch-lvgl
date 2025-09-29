# c6-lvgl

Rust/Embassy firmware for the Waveshare ESP32-C6 Touch LCD 1.47 board. It boots the touch controller, brings up the ST7789 display through an SPI driver, and logs both heartbeat messages and touch coordinates over RTT while changing the screen colour on each touch.

## Build & Run

- `cargo build` to compile the firmware with the configured esp-hal toolchain.
- `cargo run` (or flash via your usual workflow) and monitor RTT to see touch events and colour changes.

Example RTT log while touching the panel:

```
INFO  Hello world!
INFO  touch: (171, 204)
INFO  display color: 0xf800
INFO  touch: (155, 193)
INFO  display color: 0xfd20
INFO  touch: (162, 248)
INFO  display color: 0x07e0
```

Touch rotation defaults to `Rotate0`; adjust in `src/bin/main.rs` if you need a different orientation. The LVGL-style colour cycling is backed by a small embassy channel, so the display task and touch task remain independent.
