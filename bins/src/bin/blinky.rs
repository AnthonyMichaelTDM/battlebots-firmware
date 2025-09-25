#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use battlebots_firmware::ws2812::WS2812B;
use esp_hal::{
    clock::CpuClock,
    main,
    time::{Duration, Instant},
};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // the lonelybinary ESP32-S3 has a WS2812 on GPIO 48
    let freq_mhz = 80;
    let mut led = WS2812B::new(peripherals.RMT, freq_mhz, peripherals.GPIO48).unwrap();

    const BRIGHTNESS: u8 = 16;
    let sequence: [((u8, u8, u8), Duration); 3] = [
        ((BRIGHTNESS, 0, 0), Duration::from_millis(1000)), // Red
        ((0, BRIGHTNESS, 0), Duration::from_millis(1000)), // Green
        ((0, 0, BRIGHTNESS), Duration::from_millis(1000)), // Blue
    ];
    let mut seq_index = 0;

    loop {
        let ((r, g, b), duration) = &sequence[seq_index];
        seq_index = (seq_index + 1) % sequence.len();

        led.set_colors(*r, *g, *b);
        led = led.play(1).unwrap();

        let delay_start = Instant::now();
        while delay_start.elapsed() < *duration {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}
