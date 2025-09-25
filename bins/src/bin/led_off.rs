#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use battlebots_firmware::ws2812::WS2812B;
use esp_hal::{clock::CpuClock, main};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // the lonelybinary ESP32-S3 has a WS2812 on GPIO 48
    let freq_mhz = 80;
    let mut led = WS2812B::new(peripherals.RMT, freq_mhz, peripherals.GPIO48).unwrap();

    led.set_colors(0, 0, 0);
    _ = led.play(1).unwrap();

    loop {}
}
