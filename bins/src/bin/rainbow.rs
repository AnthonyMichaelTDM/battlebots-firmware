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

const SATURATION: u8 = 100; // Full saturation for vibrant colors
const BRIGHTNESS: u8 = 5; // Brightness (0-100)
const HUE_STEP: u16 = 1; // How much to increment hue each step
const DELAY_MS: u64 = 10; // Delay between color changes in milliseconds

/// Convert HSV to RGB
/// h: hue (0-360)
/// s: saturation (0-100)
/// v: value/brightness (0-100)
/// Returns (r, g, b) values (0-255)
fn hsv_to_rgb(h: u16, s: u8, v: u8) -> (u8, u8, u8) {
    let h = h % 360;
    let s = s.min(100) as f32 / 100.0;
    let v = v.min(100) as f32 / 100.0;

    let c = v * s;
    let x = c * (1.0 - ((h as f32 / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;

    let (r, g, b) = match h / 60 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        5 => (c, 0.0, x),
        _ => (0.0, 0.0, 0.0),
    };

    (
        ((r + m) * 255.0) as u8,
        ((g + m) * 255.0) as u8,
        ((b + m) * 255.0) as u8,
    )
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // the lonelybinary ESP32-S3 has a WS2812 on GPIO 48
    let freq_mhz = 80;
    let mut led = WS2812B::new(peripherals.RMT, freq_mhz, peripherals.GPIO48).unwrap();

    let mut hue: u16 = 0;
    loop {
        // Convert HSV to RGB
        let (r, g, b) = hsv_to_rgb(hue, SATURATION, BRIGHTNESS);

        // Set the LED color
        led.set_colors(r, g, b);
        led = led.play(1).unwrap();

        // Increment hue for next iteration, wrapping around at 360
        hue = (hue + HUE_STEP) % 360;

        // Small delay to control the speed of color transition
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(DELAY_MS) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}
