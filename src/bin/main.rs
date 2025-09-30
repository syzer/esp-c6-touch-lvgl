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

#[derive(Clone, Copy, Debug, defmt::Format)]
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

// BGR RAINBOW (R and B channels swapped for BGR display)
const RAINBOW: [Rgb565; 7] = [
    Rgb565::new(0, 0, 31),   // Red (R and B swapped)
    Rgb565::new(0, 45, 31),  // Orange (R and B swapped) - more yellow-orange
    Rgb565::new(0, 63, 31),  // Yellow (R and B swapped)
    Rgb565::new(0, 63, 0),   // Green (stays same)
    Rgb565::new(31, 0, 0),   // Blue (R and B swapped)
    Rgb565::new(31, 63, 0),  // Cyan (R and B swapped - bright blue-green)
    Rgb565::new(31, 0, 31),  // Violet (stays same - symmetric)
];

const FB_W: usize = 100;  // Extra wide to ensure no cut-off in framebuffer
const FB_H: usize = 24;   // Keep height small for big vertical scaling

// Timing constants for display sequence
const COLOR_DISPLAY_DURATION_MS: u64 = 1000;  // Time each color is displayed (1 second)
const CYCLE_PAUSE_DURATION_MS: u64 = 2000;    // Pause between color cycles (2 seconds)

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

    // 2) Calculate scale based on both width and height constraints
    let max_usable_height = (DISPLAY_HEIGHT - 10) as i32;
    let max_usable_width = (DISPLAY_WIDTH * 3 / 4) as i32;  // 3/4 of screen width
    
    // Estimate text width in pixels (each character is roughly 9 pixels wide in FONT_9X18)
    let text_width_pixels = text.len() as i32 * 9;
    
    // Calculate scale factors for both dimensions
    let height_scale = max_usable_height / FB_H as i32;
    let width_scale = max_usable_width / text_width_pixels;
    
    // Use the smaller scale to ensure text fits in both dimensions
    let scale = height_scale.min(width_scale).max(1); // Ensure scale is at least 1
    let out_h = (FB_H as i32 * scale) as u16;

    // 3) Position on screen - centered horizontally
    let bb = display.bounding_box();
    let cy = bb.center().y.max(0) as i32;
    let cx = bb.center().x.max(0) as i32;
    let text_display_width = text_width_pixels * scale;
    let origin_x = cx - (text_display_width / 2);
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
    let area = display.bounding_box();
    let color_names = ["Red", "Orange", "Yellow", "Green", "Blue", "Cyan", "Violet"];

    loop {
        info!("Starting 3-second sequence with 3 random colors");
        
        // Show 3 random colors in 3 seconds (1 second each)
        let text_overlays = ["All", "ways", "blue"];
        
        for i in 0..3 {
            // Generate random color
            lfsr ^= lfsr << 13;
            lfsr ^= lfsr >> 17;
            lfsr ^= lfsr << 5;
            let color_index = (lfsr as usize) % RAINBOW.len();
            let color = RAINBOW[color_index];
            
            // Display color
            match display.fill_solid(&area, color) {
                Ok(_) => {
                    info!("Color {}: {} (index: {})", i + 1, color_names[color_index], color_index);
                    // Display text overlay on the color
                    draw_text_label(display, fb_buf, text_overlays[i]);
                }
                Err(_) => info!("display fill error for color {}", i + 1),
            }
            
            // Wait for touch within the time limit or timeout
            let timeout_duration = Duration::from_millis(COLOR_DISPLAY_DURATION_MS);
            match embassy_time::with_timeout(timeout_duration, events.receive()).await {
                Ok(touch_event) => {
                    // Touch received in time - continue game
                    info!("Touch received: {:?}", touch_event);
                }
                Err(_) => {
                    // Timeout - no touch received, show DEAD screen
                    info!("No touch received - GAME OVER");
                    show_dead_screen(display, fb_buf);
                    
                    // Wait for touch to restart game
                    info!("Waiting for touch to restart...");
                    let restart_touch = events.receive().await;
                    info!("Touch received - RESTARTING GAME: {:?}", restart_touch);
                    
                    // Clear any remaining events in the queue
                    while events.try_receive().is_ok() {
                        // Drain any pending touch events
                    }
                    
                    // Break out of current color loop to restart the game
                    break;
                }
            }
        }
        
        info!("3-color sequence complete, waiting for next cycle...");
        
        // Wait before starting next sequence
        Timer::after(Duration::from_millis(CYCLE_PAUSE_DURATION_MS)).await;
    }
}

fn show_dead_screen(display: &mut DisplayDriver, fb_buf: &mut [Rgb565]) {
    let area = display.bounding_box();
    
    // Fill screen with black
    let _ = display.fill_solid(&area, Rgb565::BLACK);
    
    // Draw "DEAD" text in white
    draw_dead_text(display, fb_buf);
}

fn draw_dead_text(display: &mut DisplayDriver, fb_buf: &mut [Rgb565]) {
    // 1) Render "DEAD" text into offscreen framebuffer
    let text_style = MonoTextStyle::new(&FONT_9X18, Rgb565::WHITE); // White text
    let mut fb = SimpleFb::new(fb_buf);
    fb.clear(Rgb565::BLACK); // Black background
    
    let text_obj = Text::with_alignment("DEAD", Point::new(0, 18), text_style, Alignment::Left);
    let _ = text_obj.draw(&mut fb);

    // 2) Calculate scale to make "DEAD" text large
    let max_usable_height = (DISPLAY_HEIGHT - 10) as i32;
    let max_usable_width = (DISPLAY_WIDTH * 3 / 4) as i32;
    
    // Text width for "DEAD" (4 characters * 9 pixels each)
    let text_width_pixels = 4 * 9;
    
    // Calculate scale factors
    let height_scale = max_usable_height / FB_H as i32;
    let width_scale = max_usable_width / text_width_pixels;
    
    // Use the smaller scale
    let scale = height_scale.min(width_scale).max(1);
    let out_h = (FB_H as i32 * scale) as u16;

    // 3) Position on screen - centered
    let bb = display.bounding_box();
    let cy = bb.center().y.max(0) as i32;
    let cx = bb.center().x.max(0) as i32;
    let text_display_width = text_width_pixels * scale;
    let origin_x = cx - (text_display_width / 2);
    let origin_y = cy - (out_h as i32 / 2);

    // 4) Blit white text on black background
    for sy in 0..FB_H {
        for sx in 0..FB_W {
            let src = fb_buf[sy * FB_W + sx];
            if src != Rgb565::BLACK { // Only draw white pixels
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

    let mut poll_count = 0;
    let mut last_touch_time = 0u32;
    loop {
        poll_count += 1;
        if poll_count % 1500 == 0 {  // Log every 30 seconds (1500 * 20ms) - much less frequent
            info!("Touch task alive: poll #{}, last touch at poll #{}", poll_count, last_touch_time);
        }
        
        match touch.get_touch_data() {
            Ok(Some(report)) => {
                if let Some(point) = report.points.first() {
                    let current_pos = (point.x, point.y);
                    
                    if !touch_active {
                        // First touch detected - record starting position
                        touch_start = Some(current_pos);
                        last_position = Some(current_pos);
                        touch_active = true;
                        last_touch_time = poll_count;
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
                info!("touch read error, resetting touch state");
                touch_active = false;
                touch_start = None;
                last_position = None;
                // Add a longer delay after error to let the controller recover
                Timer::after(Duration::from_millis(100)).await;
            }
        }
        Timer::after(Duration::from_millis(20)).await; // Increased polling interval
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    rtt_target::rtt_init_defmt!();

    // Give RTT time to initialize
    let mut delay = Delay::new();
    delay.delay_millis(100);

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
