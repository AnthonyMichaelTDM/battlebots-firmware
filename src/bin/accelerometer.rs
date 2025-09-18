//! Accelerometer experiments
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use battlebots_firmware::drivers::h3lis331::{accelerometer::Accelerometer, H3lis331, SlaveAddr};
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
    main,
    time::{Duration, Instant},
};

use log::info;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    info!("Panic occurred!");
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO1)
        .with_scl(peripherals.GPIO2);

    let mut accelerometer = H3lis331::new_i2c(i2c, SlaveAddr::Default);

    let who_am_i = accelerometer.who_am_i().unwrap();
    info!("Who am I: {:#b}", who_am_i);
    // assert_eq!(who_am_i, 0b00110011, "Unexpected WHO_AM_I value");

    accelerometer
        .set_datarate(battlebots_firmware::drivers::h3lis331::DataRate::Hz100)
        .unwrap();

    loop {
        let data = accelerometer.accel_norm().unwrap();
        info!("Accel data: {:?}", data);
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(100) {
            // Wait
        }
    }
}
