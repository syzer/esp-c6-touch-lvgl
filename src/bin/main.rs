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
    blocking_mutex::raw::{NoopRawMutex, CriticalSectionRawMutex},
    channel::Channel,
    mutex::Mutex,
};
use embassy_time::{Duration, Timer, Instant};
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

#[derive(Clone, Copy, Debug)]
pub enum GestureEvent {
    Touch { x: u16, y: u16 },
    Swipe { start_x: u16, start_y: u16, end_x: u16, end_y: u16 },
}

type TouchEvent = GestureEvent;

const DISPLAY_WIDTH: u16 = 172;
const DISPLAY_HEIGHT: u16 = 320;

const DISPLAY_X_OFFSET: u16 = 34;
const DISPLAY_Y_OFFSET: u16 = 0;
const SPI_BUFFER_SIZE: usize = 1024;
// Original RGB RAINBOW (for reference)
// const RAINBOW: [Rgb565; 7] = [
//     Rgb565::new(31, 0, 0),   // Red
//     Rgb565::new(31, 20, 0),  // Orange
//     Rgb565::new(31, 63, 0),  // Yellow
//     Rgb565::new(0, 63, 0),   // Green
//     Rgb565::new(0, 0, 31),   // Blue
//     Rgb565::new(8, 0, 31),   // Indigo
//     Rgb565::new(31, 0, 31),  // Violet
// ];

// BGR RAINBOW (R and B channels swapped for BGR display)
const RAINBOW: [Rgb565; 7] = [
    Rgb565::new(0, 0, 31),   // Red (R and B swapped)
    Rgb565::new(0, 20, 31),  // Orange (R and B swapped)
    Rgb565::new(0, 63, 31),  // Yellow (R and B swapped)
    Rgb565::new(0, 63, 0),   // Green (stays same)
    Rgb565::new(31, 0, 0),   // Blue (R and B swapped)
    Rgb565::new(31, 0, 8),   // Indigo (R and B swapped)
    Rgb565::new(31, 0, 31),  // Violet (stays same - symmetric)
];

const FB_W: usize = 100;  // Extra wide to ensure no cut-off in framebuffer
const FB_H: usize = 24;   // Keep height small for big vertical scaling

static FB_STORAGE: StaticCell<[Rgb565; FB_W * FB_H]> = StaticCell::new();
static SPI_BUFFER: StaticCell<[u8; SPI_BUFFER_SIZE]> = StaticCell::new();
static DISPLAY: StaticCell<DisplayDriver> = StaticCell::new();
static TOUCH: StaticCell<Axs5106l<I2c<'static, Blocking>, Output<'static>>> = StaticCell::new();
static TOUCH_EVENTS: StaticCell<Channel<NoopRawMutex, TouchEvent, 4>> = StaticCell::new();

// Global timer state
static TIMER_START: Mutex<CriticalSectionRawMutex, Option<Instant>> = Mutex::new(None);

// Global points tracker
static TOTAL_POINTS: Mutex<CriticalSectionRawMutex, i32> = Mutex::new(0);

fn draw_num(display: &mut DisplayDriver, fb_buf: &mut [Rgb565], num: u32) {
    draw_num_with_prefix(display, fb_buf, num, false);
}

fn draw_negative_num(display: &mut DisplayDriver, fb_buf: &mut [Rgb565], num: u32) {
    // 1) Render small text into offscreen framebuffer
    let text_style = MonoTextStyle::new(&FONT_9X18, Rgb565::BLACK);
    let mut fb = SimpleFb::new(fb_buf);
    fb.clear(Rgb565::WHITE);
    
    // Format the number as a string with negative sign
    let mut buffer = [0u8; 12]; // enough for "-4294967295"
    let num_str = format_num_with_minus_to_str(num, &mut buffer);
    
    let text = Text::with_alignment(num_str, Point::new(0, 18), text_style, Alignment::Left);
    let _ = text.draw(&mut fb);

    // 2) Choose scale to make text HUGE - use most of the screen height
    let max_usable_height = (DISPLAY_HEIGHT - 10) as i32; // Use almost full height
    let scale = max_usable_height / FB_H as i32; // Calculate maximum possible scale
    let out_h = (FB_H as i32 * scale) as u16;

    // 3) Position on screen - left-aligned with some margin
    let bb = display.bounding_box();
    let margin = 10i32; // Left margin from screen edge
    let cy = bb.center().y.max(0) as i32;
    let origin_x = margin; // Left-aligned instead of centered
    let origin_y = cy - (out_h as i32 / 2); // Still center vertically

    // 4) Blit with nearest-neighbor: draw each source pixel as a filled rect of size scale
    for sy in 0..FB_H {
        for sx in 0..FB_W {
            let src = fb_buf[sy * FB_W + sx];
            if src != Rgb565::WHITE { // treat white as transparent/background
                let x = origin_x + (sx as i32 * scale);
                let y = origin_y + (sy as i32 * scale);
                
                // Check bounds to avoid drawing beyond screen edges
                if x >= 0 && y >= 0 && (y + scale) <= DISPLAY_HEIGHT as i32 {
                    let rect = Rectangle::new(Point::new(x, y), Size::new(scale as u32, scale as u32));
                    let _ = rect.into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(src)).draw(display);
                }
            }
        }
    }
}

fn draw_num_with_prefix(display: &mut DisplayDriver, fb_buf: &mut [Rgb565], num: u32, show_plus: bool) {
    // 1) Render small text into offscreen framebuffer
    let text_style = MonoTextStyle::new(&FONT_9X18, Rgb565::BLACK);
    let mut fb = SimpleFb::new(fb_buf);
    fb.clear(Rgb565::WHITE);
    
    // Format the number as a string
    let mut buffer = [0u8; 12]; // enough for "+4294967295"
    let num_str = if show_plus {
        format_num_with_plus_to_str(num, &mut buffer)
    } else {
        format_num_to_str(num, &mut buffer)
    };
    
    let text = Text::with_alignment(num_str, Point::new(0, 18), text_style, Alignment::Left);
    let _ = text.draw(&mut fb);

    // 2) Choose scale to make text HUGE - use most of the screen height
    let max_usable_height = (DISPLAY_HEIGHT - 10) as i32; // Use almost full height
    let mut scale = max_usable_height / FB_H as i32; // Calculate maximum possible scale
    let _out_w = (FB_W as i32 * scale) as u16; // Width not needed for left-aligned positioning
    let out_h = (FB_H as i32 * scale) as u16;

    // 3) Position on screen - left-aligned with some margin
    let bb = display.bounding_box();
    let margin = 10i32; // Left margin from screen edge
    let cy = bb.center().y.max(0) as i32;
    let origin_x = margin; // Left-aligned instead of centered
    let origin_y = cy - (out_h as i32 / 2); // Still center vertically

    // 4) Blit with nearest-neighbor: draw each source pixel as a filled rect of size scale
    for sy in 0..FB_H {
        for sx in 0..FB_W {
            let src = fb_buf[sy * FB_W + sx];
            if src != Rgb565::WHITE { // treat white as transparent/background
                let x = origin_x + (sx as i32 * scale);
                let y = origin_y + (sy as i32 * scale);
                
                // // Check bounds to avoid drawing beyond screen edges
                if x >= 0 && y >= 0 && 
                //    (x + scale) <= DISPLAY_WIDTH as i32 && 
                   (y + scale) <= DISPLAY_HEIGHT as i32 {
                    let rect = Rectangle::new(Point::new(x, y), Size::new(scale as u32, scale as u32));
                    let _ = rect.into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(src)).draw(display);
                }
            }
        }
    }
}

fn format_num_to_str(num: u32, buffer: &mut [u8]) -> &str {
    if num == 0 {
        buffer[0] = b'0';
        return core::str::from_utf8(&buffer[0..1]).unwrap();
    }
    
    let mut n = num;
    let mut len = 0;
    
    // Count digits
    let mut temp = num;
    while temp > 0 {
        len += 1;
        temp /= 10;
    }
    
    // Fill buffer from right to left
    for i in 0..len {
        buffer[len - 1 - i] = (n % 10) as u8 + b'0';
        n /= 10;
    }
    
    core::str::from_utf8(&buffer[0..len]).unwrap()
}

fn format_num_with_plus_to_str(num: u32, buffer: &mut [u8]) -> &str {
    if num == 0 {
        buffer[0] = b'+';
        buffer[1] = b'0';
        return core::str::from_utf8(&buffer[0..2]).unwrap();
    }
    
    let mut n = num;
    let mut len = 0;
    
    // Count digits
    let mut temp = num;
    while temp > 0 {
        len += 1;
        temp /= 10;
    }
    
    // Fill buffer from right to left (leaving space for '+')
    for i in 0..len {
        buffer[len - i] = (n % 10) as u8 + b'0';
        n /= 10;
    }
    
    // Add the '+' prefix
    buffer[0] = b'+';
    
    core::str::from_utf8(&buffer[0..len + 1]).unwrap()
}

fn format_num_with_minus_to_str(num: u32, buffer: &mut [u8]) -> &str {
    if num == 0 {
        buffer[0] = b'-';
        buffer[1] = b'0';
        return core::str::from_utf8(&buffer[0..2]).unwrap();
    }
    
    let mut n = num;
    let mut len = 0;
    
    // Count digits
    let mut temp = num;
    while temp > 0 {
        len += 1;
        temp /= 10;
    }
    
    // Fill buffer from right to left (leaving space for '-')
    for i in 0..len {
        buffer[len - i] = (n % 10) as u8 + b'0';
        n /= 10;
    }
    
    // Add the '-' prefix
    buffer[0] = b'-';
    
    core::str::from_utf8(&buffer[0..len + 1]).unwrap()
}

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

fn draw_text_label(display: &mut DisplayDriver, fb_buf: &mut [Rgb565], text: &str) {
    // 1) Render text into offscreen framebuffer
    let text_style = MonoTextStyle::new(&FONT_9X18, Rgb565::BLACK);
    let mut fb = SimpleFb::new(fb_buf);
    fb.clear(Rgb565::WHITE);
    
    let text_obj = Text::with_alignment(text, Point::new(0, 18), text_style, Alignment::Left);
    let _ = text_obj.draw(&mut fb);

    // 2) Choose scale to make text large
    let max_usable_height = (DISPLAY_HEIGHT - 10) as i32;
    let scale = max_usable_height / FB_H as i32;
    let out_h = (FB_H as i32 * scale) as u16;

    // 3) Position on screen - left-aligned with margin
    let bb = display.bounding_box();
    let margin = 10i32;
    let cy = bb.center().y.max(0) as i32;
    let origin_x = margin;
    let origin_y = cy - (out_h as i32 / 2);

    // 4) Blit with nearest-neighbor scaling
    for sy in 0..FB_H {
        for sx in 0..FB_W {
            let src = fb_buf[sy * FB_W + sx];
            if src != Rgb565::WHITE {
                let x = origin_x + (sx as i32 * scale);
                let y = origin_y + (sy as i32 * scale);
                
                if x >= 0 && y >= 0 && (y + scale) <= DISPLAY_HEIGHT as i32 {
                    let rect = Rectangle::new(Point::new(x, y), Size::new(scale as u32, scale as u32));
                    let _ = rect.into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(src)).draw(display);
                }
            }
        }
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
        let gesture_event = events.receive().await;
        
        let now = Instant::now();
        let ms_elapsed = {
            let mut timer_start = TIMER_START.lock().await;
            let elapsed_ms = match *timer_start {
                Some(start_time) => {
                    let elapsed = now.duration_since(start_time);
                    elapsed.as_millis() as u32
                }
                None => 0, // First gesture
            };
            
            // Reset timer for next measurement
            *timer_start = Some(now);
            elapsed_ms
        };
        
        // Generate new color
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        let color = RAINBOW[(lfsr as usize) % RAINBOW.len()];
        let area = display.bounding_box();
        
        match display.fill_solid(&area, color) {
            Ok(_) => {
                // Print current color to console
                let raw_color = color.into_storage();
                
                // Generate color name by comparing with actual RAINBOW array values
                let color_names = ["Red", "Orange", "Yellow", "Green", "Blue", "Indigo", "Violet"];
                let mut color_name = "Unknown";
                let mut color_index = None;
                
                for (i, rainbow_color) in RAINBOW.iter().enumerate() {
                    if rainbow_color.into_storage() == raw_color {
                        color_name = color_names[i];
                        color_index = Some(i);
                        break;
                    }
                }
                
                info!("Current color displayed: {} (raw: {=u16:x})", color_name, raw_color);
                
                // Check if current color is blue (index 4 in RAINBOW array)
                let is_blue = color_index == Some(4); // Blue is at index 4 in RAINBOW
                
                // Calculate base points using linear formula
                let base_points = if ms_elapsed > 0 {
                    let a = -0.2_f32;
                    let b = 110.0_f32;
                    let result = a * (ms_elapsed as f32) + b;
                    (result as i32).max(1) // Minimum 1 point
                } else {
                    100 // First gesture gets 100 points
                };
                
                // Apply gesture rules based on color
                let points = match (gesture_event, is_blue) {
                    (GestureEvent::Touch { x, y }, true) => {
                        // Touch on blue = penalty (blue requires swipe)
                        info!("TOUCH on BLUE at ({}, {}) - PENALTY! Blue requires SWIPE! -99 points", x, y);
                        -99
                    }
                    (GestureEvent::Touch { x, y }, false) => {
                        // Touch on non-blue = correct
                        info!("TOUCH on {} at ({}, {}) - CORRECT! +{} points", color_name, x, y, base_points);
                        base_points
                    }
                    (GestureEvent::Swipe { start_x, start_y, end_x, end_y }, true) => {
                        // Swipe on blue = correct
                        info!("SWIPE on BLUE from ({}, {}) to ({}, {}) - CORRECT! +{} points", 
                              start_x, start_y, end_x, end_y, base_points);
                        base_points
                    }
                    (GestureEvent::Swipe { start_x, start_y, end_x, end_y }, false) => {
                        // Swipe on non-blue = penalty (non-blue requires touch)
                        info!("SWIPE on {} from ({}, {}) to ({}, {}) - PENALTY! {} requires TOUCH! -99 points", 
                              color_name, start_x, start_y, end_x, end_y, color_name);
                        -99
                    }
                };
                
                // Update total points
                let total_points = {
                    let mut total = TOTAL_POINTS.lock().await;
                    *total += points;
                    *total
                };
                
                info!("Points this round: {}, Total points: {}", points, total_points);
                
                // Display the points (show penalty with negative sign, others with + sign)
                if points < 0 {
                    draw_negative_num(display, fb_buf, (-points) as u32); // Show as negative
                } else {
                    draw_num_with_prefix(display, fb_buf, points as u32, true); // Show with + prefix
                }
            }
            Err(_) => info!("display fill error"),
        }
    }
}

// Simple integer square root approximation
fn approximate_sqrt(n: u32) -> u32 {
    if n == 0 { return 0; }
    if n == 1 { return 1; }
    
    let mut x = n;
    let mut y = (x + 1) / 2;
    
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

#[embassy_executor::task]
async fn touch_task(
    touch: &'static mut Axs5106l<I2c<'static, Blocking>, Output<'static>>,
    events: &'static Channel<NoopRawMutex, TouchEvent, 4>,
) {
    // Enhanced touch detection with swipe recognition
    let mut touch_active = false;
    let mut touch_start: Option<(u16, u16)> = None;
    let mut last_position: Option<(u16, u16)> = None;
    
    // Swipe detection parameters
    const MIN_SWIPE_DISTANCE: u16 = 30; // Minimum distance to consider it a swipe
    const MAX_TOUCH_DISTANCE: u16 = 10;  // Maximum movement for a tap

    loop {
        match touch.get_touch_data() {
            Ok(Some(report)) => {
                if let Some(point) = report.points.first() {
                    let current_pos = (point.x, point.y);
                    
                    if !touch_active {
                        // First touch detected - record starting position
                        touch_start = Some(current_pos);
                        last_position = Some(current_pos);
                        touch_active = true;
                        info!("touch start: ({}, {})", point.x, point.y);
                    } else {
                        // Update last known position while touch is active
                        last_position = Some(current_pos);
                    }
                }
            }
            Ok(None) => {
                // Touch ended - determine if it was a touch or swipe
                if touch_active {
                    if let (Some((start_x, start_y)), Some((end_x, end_y))) = (touch_start, last_position) {
                        // Calculate distance moved (using squared distance to avoid sqrt)
                        let dx = (end_x as i32) - (start_x as i32);
                        let dy = (end_y as i32) - (start_y as i32);
                        let distance_squared = (dx * dx + dy * dy) as u32;
                        let distance = approximate_sqrt(distance_squared) as u16;
                        
                        let gesture = if distance >= MIN_SWIPE_DISTANCE {
                            // Swipe detected
                            info!("swipe detected: ({}, {}) -> ({}, {}), distance: {}", 
                                  start_x, start_y, end_x, end_y, distance);
                            GestureEvent::Swipe { 
                                start_x, 
                                start_y, 
                                end_x, 
                                end_y 
                            }
                        } else {
                            // Simple touch/tap
                            info!("touch detected: ({}, {}), distance: {}", 
                                  start_x, start_y, distance);
                            GestureEvent::Touch { 
                                x: start_x, 
                                y: start_y 
                            }
                        };
                        
                        events.send(gesture).await;
                    }
                    
                    // Reset state
                    touch_active = false;
                    touch_start = None;
                    last_position = None;
                }
            }
            Err(_) => {
                info!("touch read error");
                touch_active = false;
                touch_start = None;
                last_position = None;
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
