#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use bt_hci::controller::ExternalController;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi::ble::controller::BleConnector;
use panic_rtt_target as _;
use static_cell::StaticCell;

use esp_hal::delay::Delay;
use esp_hal::gpio::{Io, Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::time::Rate;
use esp_hal::Blocking;

use axs5106l::{Axs5106l, Rotation};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static TOUCH: StaticCell<Axs5106l<I2c<'static, Blocking>, Output<'static>>> = StaticCell::new();

#[embassy_executor::task]
async fn touch_task(touch: &'static mut Axs5106l<I2c<'static, Blocking>, Output<'static>>) {
    // Simple polling loop: ~100 Hz
    loop {
        match touch.get_touch_data() {
            Ok(Some(report)) => {
                for point in report.points.iter() {
                    info!("touch: ({}, {})", point.x, point.y);
                }
            }
            Ok(None) => {}
            Err(_) => {
                info!("touch read error");
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

    // --- Touch (AXS5106L) setup for Waveshare ESP32-C6-Touch-LCD-1.47 ---
    // Pins per board: SDA=GPIO18, SCL=GPIO19, RST=GPIO20, INT=GPIO21
    let _io = Io::new(peripherals.IO_MUX);
    let sda = peripherals.GPIO18;
    let scl = peripherals.GPIO19;
    let rst = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());
    let _int = peripherals.GPIO21; // optional interrupt pin (unused in polling)

    // Configure blocking I2C @ 400 kHz
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("failed to initialise I2C0")
        .with_sda(sda)
        .with_scl(scl);

    // Create touch driver (172x320 panel)
    let mut touch = Axs5106l::new(i2c, rst, 172, 320, Rotation::Rotate0);
    let mut delay = Delay::new();
    if touch.init(&mut delay).is_err() {
        info!("touch init failed");
    }

    let touch = TOUCH.init(touch);

    // Spawn the polling task
    spawner.spawn(touch_task(touch)).ok();
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
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}
