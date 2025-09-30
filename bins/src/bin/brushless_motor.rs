//! Brushless motor control
//!
//! Drive a brushless motor on a 3-phase ESC using dshot protocol driven by the RMT peripheral
//!
//! From this experiment, it seems that the ESC needs to be continuously receiving commands to keep the motor running after arming.
//! If no commands are received for a short period of time, the motor stops.
//!
//! I'm not sure exactly what this timeout period is, but it seems to be at most 20 milliseconds, which isn't great.
//!
//! This means that when making the robot, I should have an embassy task dedicated to sending dshot commands to the ESC at a regular interval,
//! and the control loop can update the desired throttle value by sending messages to that task. If the task doesn't receive any new
//! throttle values for, say, 2 seconds, it should begin ramping down to 0 throttle.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(clippy::empty_loop)]
use battlebots_firmware::dshot::{BlockingChannel, DShot, DShotSpeed};
use defmt::{info, warn};
use embedded_hal::delay::DelayNs;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig, Pull},
    main,
    rmt::{Rmt, TxChannelConfig, TxChannelCreator as _},
    time::{Duration, Instant, Rate},
};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

const PROTOCOL: DShotSpeed = DShotSpeed::DShot600;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

fn ramp_up_down<const CHANNEL: u8>(
    target: u16,
    duration: Duration,
    mut dshot: DShot<BlockingChannel<CHANNEL>>,
    delay: &mut impl DelayNs,
) -> DShot<BlockingChannel<CHANNEL>> {
    // ramp up to the target throttle value
    info!("Ramping up throttle to {}", target);
    let start = Instant::now();
    loop {
        let elapsed = start.elapsed();
        if elapsed > duration {
            break;
        }
        let throttle = (elapsed.as_millis() * target as u64 / duration.as_millis()) as u16 + 48;
        dshot = match dshot.write_throttle_blocking(throttle.clamp(48, 2047), false) {
            Ok(dshot) => dshot,
            Err(e) => {
                warn!("Failed to set throttle: {:?}", e);
                loop {}
            }
        };
        info!("Throttle: {}", throttle);
        // wait 20 ms between updates
        delay.delay_ms(20);
    }
    // ramping down
    info!("Ramping down throttle to 0");
    let start = Instant::now();
    loop {
        let elapsed = start.elapsed();
        if elapsed > duration {
            break;
        }
        let throttle =
            target - (elapsed.as_millis() * target as u64 / duration.as_millis()) as u16 + 48;
        dshot = match dshot.write_throttle_blocking(throttle.clamp(48, 2047), false) {
            Ok(dshot) => dshot,
            Err(e) => {
                warn!("Failed to set throttle: {:?}", e);
                loop {}
            }
        };
        info!("Throttle: {}", throttle);
        // wait 20 ms between updates
        delay.delay_ms(20);
    }

    dshot
}

fn hold_throttle<const CHANNEL: u8>(
    throttle: u16,
    duration: Duration,
    mut dshot: DShot<BlockingChannel<CHANNEL>>,
    delay: &mut impl DelayNs,
) -> DShot<BlockingChannel<CHANNEL>> {
    info!("Holding throttle at {} for {:?}", throttle, duration);
    let start = Instant::now();
    loop {
        let elapsed = start.elapsed();
        if elapsed > duration {
            break;
        }
        dshot = match dshot.write_throttle_blocking(throttle.clamp(48, 2047), false) {
            Ok(dshot) => dshot,
            Err(e) => {
                warn!("Failed to set throttle: {:?}", e);
                loop {}
            }
        };
        // wait between updates
        delay.delay_ms(20);
    }

    dshot
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let mut delay = &mut esp_hal::delay::Delay::new();

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // // wait a moment for me to plug in the ESC
    // info!("Starting in 5 seconds...");
    // let start = Instant::now();
    // while start.elapsed() < Duration::from_secs(5) {
    //     let sec = start.elapsed().as_secs();
    //     if sec < 5 {
    //         info!("{}...", 5 - sec);
    //     }

    //     // busy wait
    //     delay.delay_millis(900);
    // }

    // set up the RMT channel and GPIO pin for DShot
    // let gpio = peripherals.GPIO46;
    let gpio = peripherals.GPIO14;
    let rmt_rate = Rate::from_mhz(80);
    let rmt_clk_divider = 1; // 1 tick = 12.5 ns
    let rmt = Rmt::new(peripherals.RMT, rmt_rate).unwrap();
    let output = Output::new(
        gpio,
        Level::Low,
        OutputConfig::default().with_pull(Pull::Down),
    );
    let channel = rmt
        .channel1
        .configure_tx(
            output,
            TxChannelConfig::default().with_clk_divider(rmt_clk_divider),
        )
        .unwrap();

    // create the DShot instance
    let mut dshot = DShot::new(channel, PROTOCOL, rmt_rate, rmt_clk_divider);

    // not sure why, but this wait is needed
    delay.delay_millis(1500);

    // arm the ESC
    info!("Arming ESC...");
    dshot = match dshot.arm_blocking(&mut delay) {
        Ok(dshot) => dshot,
        Err(e) => {
            warn!("Failed to arm ESC: {:?}", e);
            loop {}
        }
    };

    delay.delay_millis(20);

    // ramp up over 5 seconds and then ramp back back down
    const TARGET_THROTTLE_PERCENT: f32 = 0.4;
    const TARGET_THROTTLE: u16 = 48 + ((2047 - 48) as f32 * TARGET_THROTTLE_PERCENT) as u16;
    dshot = ramp_up_down(TARGET_THROTTLE, Duration::from_secs(4), dshot, delay);

    // let's maintain 0% throttle for another 2 seconds to see if we can trigger the "beacon beeps"
    dshot = hold_throttle(100, Duration::from_secs(2), dshot, delay);

    info!("sleeping a bit");
    delay.delay_millis(500);

    warn!(
        "if we try to set the throttle again now, the motor will not start again without re-arming"
    );

    drop(dshot);

    loop {}
}
