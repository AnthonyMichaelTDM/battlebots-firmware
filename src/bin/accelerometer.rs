//! Accelerometer experiments
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use accelerometer::vector::VectorExt;
use battlebots_firmware::drivers::h3lis331::{
    accelerometer::Accelerometer, Configuration, DataRate, H3lis331, H3lis331Interface,
    H3lis331Range, Register, SlaveAddr, CHIP_ID,
};

use defmt::info;
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
    main,
    time::{Duration, Instant},
};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

fn print_register_dump<I>(accelerometer: &mut H3lis331<I>)
where
    I: H3lis331Interface,
    I::PinError: core::fmt::Debug,
    I::BusError: core::fmt::Debug,
{
    info!("Register dump:");
    const REGISTERS: &[Register] = &[
        Register::WhoAmI,
        Register::Ctrl1,
        Register::Ctrl2,
        Register::Ctrl3,
        Register::Ctrl4,
        Register::Ctrl5,
        Register::HpFilterReset,
        Register::Reference,
        Register::Status,
        Register::OutXL,
        Register::OutXH,
        Register::OutYL,
        Register::OutYH,
        Register::OutZL,
        Register::OutZH,
        Register::Int1Cfg,
        Register::Int1Src,
        Register::Int1Ths,
        Register::Int1Dur,
        Register::Int2Cfg,
        Register::Int2Src,
        Register::Int2Ths,
        Register::Int2Dur,
    ];
    for &reg in REGISTERS.iter() {
        let val = accelerometer.read_register(reg).unwrap();
        info!("Reg {:?}: {:#04X}", reg, val);
    }
}

fn print_accelerometer_info<I>(accelerometer: &mut H3lis331<I>)
where
    I: H3lis331Interface,
    I::PinError: core::fmt::Debug,
    I::BusError: core::fmt::Debug,
{
    let who_am_i = accelerometer.who_am_i().unwrap();
    info!("Who am I: {:#b}", who_am_i);
    let stat = accelerometer.get_status().unwrap();
    info!("Status: {:?}", stat);
    let datarate = accelerometer.get_datarate().unwrap();
    info!("Data rate: {:?}", datarate);
    let range = accelerometer.get_range().unwrap();
    info!("Range: {:?}", range);
    info!("-----------------------------------");
    print_register_dump(accelerometer);
    info!("-----------------------------------");
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO1)
        .with_scl(peripherals.GPIO2);

    let mut accelerometer = H3lis331::new_i2c(i2c, SlaveAddr::Default);

    print_accelerometer_info(&mut accelerometer);

    let who_am_i = accelerometer.who_am_i().unwrap();
    assert_eq!(who_am_i, CHIP_ID, "Unexpected WHO_AM_I value");

    // set configuration
    let config = Configuration {
        datarate: DataRate::Hz50,
        ..Configuration::default()
    };
    accelerometer.configure(&config).unwrap();
    accelerometer.set_range(H3lis331Range::G100).unwrap();

    print_accelerometer_info(&mut accelerometer);
    info!("Starting main loop...");
    const TICK: Duration = Duration::from_millis(100);

    let mut tick_count = 0u8;

    loop {
        tick_count = tick_count.wrapping_add(1);

        match accelerometer.all_data_available() {
            Ok(true) => {
                let data = accelerometer.accel_norm().unwrap();

                info!(
                    "Accel data: x={=f32}, y={=f32}, z={=f32} (units of g). Magnitude={=f32}",
                    data.x,
                    data.y,
                    data.z,
                    data.magnitude(),
                );
            }
            Ok(false) => {
                // No new data available
            }
            Err(e) => {
                info!(
                    "Error checking data availability: {=str}",
                    alloc::fmt::format(format_args!("{:?}", e))
                );
            }
        }

        // Delay for a bit
        let delay_start = Instant::now();
        while delay_start.elapsed() < TICK {
            // Wait
        }
    }
}
