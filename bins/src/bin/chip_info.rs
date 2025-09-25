//! This binary prints information about the chip to the RTT console, then loops forever.
//! As an experiment, it also enables a watchdog that should reset the chip after a few seconds of inactivity.
//! - note: I've since disabled the watchdog for simplicity, but the code is still here commented out
//!
//! Some information that is printed:
//! - Chip model
//! - MAC address
//! - Flash size
//! - PSRAM size
//! - Features (WiFi, Bluetooth, etc.)
//! - ...
//!
//! This is useful for debugging and verifying that the chip is functioning correctly.
//!
//! # Notes
//!
//! - watchdog documentation: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html
//!
//! Things I learned from this:
//! - the SWD watchdog can recover from a panic (if the panic handler halts or aborts)
//!   - it seems that it can also recover from infinite loops, but there's seemingly no way to feed it so i don't think it's safe to use
//! - the RWDT watchdog can recover from an infinite loop
//!   - it can not recover from a panic if the panic handler stays in a critical section forever (like panic_rtt_target)
//!   - if the panic handler in esp-backtrace does not stay in a critical section forever, so the RWDT can recover from a panic when using esp-backtrace
//!
//! - if you set a watchdog in one boot, it can sometimes stay enabled across resets, even if the next boot doesn't enable it
//!
//! - without a watchdog, a panic (that halts or aborts) can make the chip unresponsive to JTAG until I press the reset button
//!   - this is especially dangerous if the panic occurs early in the boot process, before I can re-flash the chip
//!     - last time I accidentally did this, espflash couldn't reflash the chip at all, but `probe-rs erase` worked to get the chip in a useable state

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use battlebots_firmware::ws2812::WS2812B;
use defmt::{info, warn};
use esp_hal::{
    chip,
    clock::CpuClock,
    // config::{WatchdogConfig, WatchdogStatus},
    efuse::{Efuse, FLASH_CAP, FLASH_ECC_MODE, FLASH_PAGE_SIZE},
    main,
    system::Cpu,
    time::{Duration, Instant},
};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

fn print_flash_info() {
    info!("Flash info:");

    // the FLASH_CAP field is a 3-bit value, but isn't well documented in the datasheet
    let flash_size_raw = Efuse::read_field_le::<u8>(FLASH_CAP) & 0b111;
    // let flash_size: u8 = match flash_size_raw {
    //     0 => 4,
    //     1 => 2,
    //     2 => 8,
    //     3 => 16,
    //     4 => 32,
    //     _ => {
    //         warn!("\tUnknown flash size");
    //         0
    //     }
    // };
    info!("\tFlash size (raw: 0b{=u8:03b})", flash_size_raw);

    // FLASH_ECC_MODE is 1 bit, and represents the flash ECC mode in ROM, 0 = 16-to-18 byte mode, 1 = 16-to-17 byte mode
    let flash_ecc_raw = Efuse::read_field_le::<u8>(FLASH_ECC_MODE) & 0b1;
    let flash_ecc = match flash_ecc_raw {
        0 => "16-to-18 byte mode",
        1 => "16-to-17 byte mode",
        _ => {
            warn!("\tUnknown FLASH_ECC_MODE");
            ""
        }
    };
    info!(
        "\tFLASH_ECC_MODE (raw: 0b{=u8:01b}): {=str}",
        flash_ecc_raw, flash_ecc
    );

    // FLASH_PAGE_SIZE is 2 bits
    // the meaning of the bits isn't documents, the following is a guess
    let flash_page_size_raw = Efuse::read_field_le::<u8>(FLASH_PAGE_SIZE) & 0b11;
    // let flash_page_size: u16 = match flash_page_size_raw {
    //     0 => 256,
    //     1 => 512,
    //     2 => 1024,
    //     3 => 2048,
    //     _ => {
    //         warn!("\tUnknown FLASH_PAGE_SIZE");
    //         0
    //     }
    // };
    info!("\tFLASH_PAGE_SIZE (raw: 0b{=u8:02b})", flash_page_size_raw);

    // FLASH_TYPE is 1 bit and represents the maximum data lines of SPI flash, 0 = 4
    let flash_type_raw = Efuse::read_field_le::<u8>(esp_hal::efuse::FLASH_TYPE) & 0b1;
    let flash_type: u8 = match flash_type_raw {
        0 => 4,
        1 => 8,
        _ => {
            warn!("\tUnknown FLASH_TYPE");
            0
        }
    };

    info!(
        "\tFLASH_TYPE (raw: 0b{=u8:01b}): SPI Flash maximum {=u8} data lines",
        flash_type_raw, flash_type
    );

    // FLASH_VENDOR is 3 bits, but not documented in the datasheet
    let flash_vendor_raw = Efuse::read_field_le::<u8>(esp_hal::efuse::FLASH_VENDOR) & 0b111;
    info!("\tFLASH_VENDOR (raw: 0b{=u8:03b})", flash_vendor_raw);

    // FLASH_TEMP is 2 bits, but not documented in the datasheet
    let flash_temp_raw = Efuse::read_field_le::<u8>(esp_hal::efuse::FLASH_TEMP) & 0b11;
    info!("\tFLASH_TEMP (raw: 0b{=u8:02b})", flash_temp_raw);
}

fn print_wafer_version_info() {
    // WAFER_VERSION_MAJOR is 2 bits
    let wafer_version_major_raw =
        Efuse::read_field_le::<u8>(esp_hal::efuse::WAFER_VERSION_MAJOR) & 0b11;
    info!(
        "\tWAFER_VERSION_MAJOR (raw: 0b{=u8:02b})",
        wafer_version_major_raw
    );

    // WAFER_VERSION_MINOR_HI is 1 bit
    let wafer_version_minor_raw =
        Efuse::read_field_le::<u8>(esp_hal::efuse::WAFER_VERSION_MINOR_HI) & 0b1;
    info!(
        "\tWAFER_VERSION_MINOR_HI (raw: 0b{=u8:01b})",
        wafer_version_minor_raw
    );

    // WAFER_VERSION_MINOR_LO is 3 bits
    let wafer_version_minor_lo_raw =
        Efuse::read_field_le::<u8>(esp_hal::efuse::WAFER_VERSION_MINOR_LO) & 0b111;
    info!(
        "\tWAFER_VERSION_MINOR_LO (raw: 0b{=u8:03b})",
        wafer_version_minor_lo_raw
    );
}

fn print_psram_info() {
    info!("PSRAM info:");
    // PSRAM_CAP is 2 bits, with the 3rd bit in PSRAM_CAP_3.
    // The meaning of the bits isn't documented in the datasheet
    let psram_cap_raw = Efuse::read_field_le::<u8>(esp_hal::efuse::PSRAM_CAP) & 0b11;
    let psram_cap_3_raw = Efuse::read_field_le::<u8>(esp_hal::efuse::PSRAM_CAP_3) & 0b1;
    let psram_cap_combined = (psram_cap_3_raw << 2) | psram_cap_raw;
    info!("\tPSRAM_CAP (raw: 0b{=u8:03b})", psram_cap_combined);

    // PSRAM_TEMP is 2 bits, but not documented in the datasheet
    let psram_temp_raw = Efuse::read_field_le::<u8>(esp_hal::efuse::PSRAM_TEMP) & 0b11;
    info!("\tPSRAM_TEMP (raw: 0b{=u8:02b})", psram_temp_raw);

    // PSRAM_VENDOR is 2 bits, but not documented in the datasheet
    let psram_vendor_raw = Efuse::read_field_le::<u8>(esp_hal::efuse::PSRAM_VENDOR) & 0b11;
    info!("\tPSRAM_VENDOR (raw: 0b{=u8:02b})", psram_vendor_raw);
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    // .with_watchdog(
    //     WatchdogConfig::default()
    //         // .with_swd(true)
    //         .with_swd(false)
    //         .with_rwdt(WatchdogStatus::Disabled),
    //     // .with_rwdt(WatchdogStatus::Enabled(Duration::from_millis(5000u64))),
    // );
    let peripherals = esp_hal::init(config);
    let mut led = WS2812B::new(peripherals.RMT, 80, peripherals.GPIO48).unwrap();
    led.set_colors(32, 0, 0);
    led = led.play(1).unwrap();

    // we need to allocate some memory for logging
    esp_alloc::heap_allocator!(size: 64 * 1024);

    {
        // print the reset reason, if available
        for cpu in [Cpu::ProCpu, Cpu::AppCpu] {
            if let Some(reason) = esp_hal::rtc_cntl::reset_reason(cpu) {
                warn!(
                    "Reset reason for {:?}: {:?}",
                    defmt::Debug2Format(&cpu),
                    defmt::Debug2Format(&reason)
                );
            }
        }
    }

    // wait a bit for the chip to stabilize
    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(2000) {}
    esp_hal::rtc_cntl::Rwdt.feed(); // feed the RWDT watchdog
    led.set_colors(0, 0, 0);
    _ = led.play(1).unwrap();

    let name: &str = chip!();
    let mac = Efuse::mac_address();

    info!("Chip info:");
    info!("\tChip Name: {=str}", name);
    info!("\tMAC Address: {=[u8; 6]:02X}", mac);
    print_wafer_version_info();
    print_flash_info();
    print_psram_info();

    // let delay_start = Instant::now();
    // while delay_start.elapsed() < Duration::from_millis(10000) {
    //     // inner loop to keep the watchdog from resetting the chip
    //     let inner_delay = Instant::now();
    //     while inner_delay.elapsed() < Duration::from_millis(500) {}
    //     esp_hal::rtc_cntl::Rwdt.feed(); // feed the RWDT watchdog
    //     info!(
    //         "Still alive after {=u64} ms",
    //         delay_start.elapsed().as_millis()
    //     );
    // }

    // panic!("done");
    loop {
        // infinite loop
    }
}
