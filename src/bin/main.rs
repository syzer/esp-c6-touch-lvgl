#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use bt_hci::controller::ExternalController;
use core::convert::Infallible;
use defmt::info;
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::MonoTextStyle,
    mono_font::ascii::FONT_9X18,
    primitives::{Primitive, Rectangle},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
    Drawable,
};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::Channel,
};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::{
    master::{Config as SpiConfig, Spi},
    Mode,
};
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Blocking;
use esp_wifi::ble::controller::BleConnector;
use panic_rtt_target as _;
use static_cell::StaticCell;

use embedded_hal_bus::spi::ExclusiveDevice;
use mipidsi::{
    interface::SpiInterface,
    models::ST7789,
    options::{Orientation, Rotation},
    Builder as DisplayBuilder,
};

use axs5106l::{Axs5106l, Rotation as TouchRotation};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

type SpiDevice = ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>;
type DisplayInterface = SpiInterface<'static, SpiDevice, Output<'static>>;
type DisplayDriver = mipidsi::Display<DisplayInterface, ST7789, Output<'static>>;
type TouchEvent = ();

const DISPLAY_WIDTH: u16 = 172;
const DISPLAY_HEIGHT: u16 = 320;

const DISPLAY_X_OFFSET: u16 = 34;
const DISPLAY_Y_OFFSET: u16 = 0;
const SPI_BUFFER_SIZE: usize = 1024;
const RAINBOW: [Rgb565; 7] = [
    Rgb565::new(31, 0, 0),   // Red
    Rgb565::new(31, 20, 0),  // Orange
    Rgb565::new(31, 63, 0),  // Yellow
    Rgb565::new(0, 63, 0),   // Green
    Rgb565::new(0, 0, 31),   // Blue
    Rgb565::new(8, 0, 31),   // Indigo
    Rgb565::new(31, 0, 31),  // Violet
];

const FB_W: usize = 86;   // enough for "+5" in 9x18 with padding
const FB_H: usize = 24;   // 18px font + margin

static FB_STORAGE: StaticCell<[Rgb565; FB_W * FB_H]> = StaticCell::new();
static SPI_BUFFER: StaticCell<[u8; SPI_BUFFER_SIZE]> = StaticCell::new();
static DISPLAY: StaticCell<DisplayDriver> = StaticCell::new();
static TOUCH: StaticCell<Axs5106l<I2c<'static, Blocking>, Output<'static>>> = StaticCell::new();
static TOUCH_EVENTS: StaticCell<Channel<NoopRawMutex, TouchEvent, 4>> = StaticCell::new();

struct SimpleFb<'a> {
    buf: &'a mut [Rgb565],
}

impl<'a> SimpleFb<'a> {
    fn new(storage: &'a mut [Rgb565]) -> Self { Self { buf: storage } }
    fn clear(&mut self, color: Rgb565) {
        for px in self.buf.iter_mut() { *px = color; }
    }
}

impl OriginDimensions for SimpleFb<'_> {
    fn size(&self) -> Size { Size::new(FB_W as u32, FB_H as u32) }
}

impl DrawTarget for SimpleFb<'_> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(Point { x, y }, color) in pixels {
            if x >= 0 && y >= 0 {
                let (x, y) = (x as usize, y as usize);
                if x < FB_W && y < FB_H {
                    self.buf[y * FB_W + x] = color;
                }
            }
        }
        Ok(())
    }
}

#[embassy_executor::task]
async fn display_task(
    display: &'static mut DisplayDriver,
    events: &'static Channel<NoopRawMutex, TouchEvent, 4>,
) {
    let mut lfsr: u32 = 0xACE1u32;
        let fb_buf = FB_STORAGE.init([Rgb565::BLACK; FB_W * FB_H]);

    loop {
        let _ = events.receive().await;
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        let color = RAINBOW[(lfsr as usize) % RAINBOW.len()]; // TODO random color
        let area = display.bounding_box();
        match display.fill_solid(&area, color) {
            Ok(_) => {
                let raw = color.into_storage();
                info!("display color: {=u16:x}", raw);
                
                // 1) Render small text into offscreen framebuffer
                let text_style = MonoTextStyle::new(&FONT_9X18, Rgb565::BLACK);
                let mut fb = SimpleFb::new(fb_buf);
                fb.clear(Rgb565::WHITE);
                let text = Text::with_alignment("+5", Point::new(0, 18), text_style, Alignment::Left);
                let _ = text.draw(&mut fb);

                // 2) Choose scale so height > 100 px but fits the screen
                let min_target_h = 100i32;
                let mut scale = (min_target_h + FB_H as i32 - 1) / FB_H as i32; // ceil
                if scale < 2 { scale = 2; }
                // keep within screen bounds
                while (FB_W as i32 * scale) as u16 > DISPLAY_WIDTH || (FB_H as i32 * scale) as u16 > DISPLAY_HEIGHT {
                    scale -= 1;
                    if scale <= 1 { break; }
                }
                let out_w = (FB_W as i32 * scale) as u16;
                let out_h = (FB_H as i32 * scale) as u16;

                // 3) Center on screen
                let bb = display.bounding_box();
                let cx = bb.center().x.max(0) as i32;
                let cy = bb.center().y.max(0) as i32;
                let origin_x = cx - (out_w as i32 / 2);
                let origin_y = cy - (out_h as i32 / 2);

                // 4) Blit with nearest-neighbor: draw each source pixel as a filled rect of size scale
                for sy in 0..FB_H {
                    for sx in 0..FB_W {
                        let src = fb_buf[sy * FB_W + sx];
                        if src != Rgb565::WHITE { // treat white as transparent/background
                            let x = origin_x + (sx as i32 * scale);
                            let y = origin_y + (sy as i32 * scale);
                            let rect = Rectangle::new(Point::new(x, y), Size::new(scale as u32, scale as u32));
                            let _ = rect.into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(src)).draw(display);
                        }
                    }
                }
            }
            Err(_) => info!("display fill error"),
        }
    }
}

#[embassy_executor::task]
async fn touch_task(
    touch: &'static mut Axs5106l<I2c<'static, Blocking>, Output<'static>>,
    events: &'static Channel<NoopRawMutex, TouchEvent, 4>,
) {
    // Simple polling loop: ~100 Hz with rising-edge detection for touches
    let mut touch_active = false;

    loop {
        match touch.get_touch_data() {
            Ok(Some(report)) => {
                if !touch_active {
                    events.send(()).await;
                    for point in report.points.iter() {
                        info!("touch: ({}, {})", point.x, point.y);
                    }
                    touch_active = true;
                }
            }
            Ok(None) => {
                touch_active = false;
            }
            Err(_) => {
                info!("touch read error");
                touch_active = false;
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // --- Display setup (ST7789 over SPI) ---
    let _io = Io::new(peripherals.IO_MUX);
    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_hz(40_000_000))
            .with_mode(Mode::_0),
    )
    .expect("failed to initialise SPI2")
    .with_sck(peripherals.GPIO1)
    .with_mosi(peripherals.GPIO2);

    let cs = Output::new(peripherals.GPIO14, Level::High, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO15, Level::High, OutputConfig::default());
    let mut backlight = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let rst_lcd = Output::new(peripherals.GPIO22, Level::High, OutputConfig::default());

    let spi_device = ExclusiveDevice::new(spi, cs, Delay::new()).expect("failed to bind SPI device");
    let spi_buffer = SPI_BUFFER.init([0u8; SPI_BUFFER_SIZE]);
    let di = SpiInterface::new(spi_device, dc, spi_buffer);

    let mut delay = Delay::new();
    let mut display = DisplayBuilder::new(ST7789, di)
        .display_size(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        .display_offset(DISPLAY_X_OFFSET, DISPLAY_Y_OFFSET)
        .orientation(Orientation::default().rotate(Rotation::Deg90).flip_horizontal())   // or .mirror_x(true) depending on version
        .reset_pin(rst_lcd)
        .init(&mut delay)
        .expect("failed to initialise display");

    let area = display.bounding_box();
    let _ = display.fill_solid(&area, RAINBOW[0]);
    backlight.set_high();

    let display = DISPLAY.init(display);
    let touch_events = TOUCH_EVENTS.init(Channel::new());
    spawner.spawn(display_task(display, touch_events)).ok();
    // --- End display setup ---

    // --- Touch (AXS5106L) setup for Waveshare ESP32-C6-Touch-LCD-1.47 ---
    let sda = peripherals.GPIO18;
    let scl = peripherals.GPIO19;
    let rst = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());
    let _int = peripherals.GPIO21; // optional interrupt pin (unused in polling)

    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("failed to initialise I2C0")
        .with_sda(sda)
        .with_scl(scl);

    let mut touch = Axs5106l::new(i2c, rst, DISPLAY_WIDTH, DISPLAY_HEIGHT, TouchRotation::Rotate0);
    if touch.init(&mut delay).is_err() {
        info!("touch init failed");
    }

    let touch = TOUCH.init(touch);

    spawner.spawn(touch_task(touch, touch_events)).ok();
    // --- End touch setup ---

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&wifi_init, peripherals.BT);
    let _ble_controller = ExternalController::<_, 20>::new(transport);

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}
