//! A SPI and I2C driver for the Adafruit H3LIS331DL 3-axis accelerometer
//!
//! Datasheet: https://www.st.com/resource/en/datasheet/h3lis331dl.pdf
//!
//! Official Driver:
//! - https://github.com/STMicroelectronics/h3lis331dl-pid/tree/master
//! - https://github.com/adafruit/Adafruit_LIS331
//!
//! Based on https://github.com/mmalecki/lis331-rs, but modified to:
//! 1. Work with the H3LIS331DL specifically
//! 2. use embedded-hal 1.0.0
//!
//! This driver is based on the `embedded-hal` traits and should work with any
//! microcontroller that has an `embedded-hal` implementation.
//!
//! # Example
//!
//! ## Using SPI (esp32)
//!
//! TODO
//!
//! ## Using I2C
//!
//! TODO

use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;

use embedded_hal as hal;

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer};

pub mod interrupt;
pub mod register;

pub use register::{
    DataRate, DataStatus, Duration, H3lis331Range, Register, SlaveAddr, Threshold, CHIP_ID,
};
use register::{BDU, X_EN, Y_EN, Z_EN};

use crate::drivers::h3lis331::interrupt::{
    Detect4D, Interrupt, InterruptConfig, InterruptMode, InterruptSource, IrqPin,
    LatchInterruptRequest,
};

/// Accelerometer errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
pub enum Error<BusError, PinError> {
    /// I2C bus error
    Bus(BusError),
    Pin(PinError),

    /// Invalid data rate selection
    InvalidDataRate,

    /// Invalid full-scale selection
    InvalidRange,

    /// Attempted to write to a read-only register
    WriteToReadOnly,

    /// Invalid address provided
    WrongAddress,
}

pub struct AccelerationData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Common interface for H3lis331 over different communication protocols
pub trait H3lis331Interface {
    type BusError;
    type PinError;

    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>>;

    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<Self::BusError, Self::PinError>>;

    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<Self::BusError, Self::PinError>>;
}

pub struct H3lis331<Interface> {
    interface: Interface,
}

/// Marker that we're using SPI to communicate with the H3lis331
pub struct H3lis331Spi<SPI, CS> {
    /// underlying SPI bus
    spi: SPI,
    /// chip select pin
    cs: CS,
}

/// Marker that we're using I2C to communicate with the H3lis331
pub struct H3lis331I2c<I2C> {
    /// underlying I2C bus
    i2c: I2C,
    /// current I2C slave address
    address: SlaveAddr,
}

impl H3lis331<()> {
    pub fn new_spi<SPI, CS>(spi: SPI, cs: CS) -> H3lis331<H3lis331Spi<SPI, CS>>
    where
        SPI: hal::spi::SpiDevice,
        CS: hal::digital::OutputPin,
    {
        H3lis331 {
            interface: H3lis331Spi { spi, cs },
        }
    }

    pub fn new_i2c<I2C>(i2c: I2C, address: SlaveAddr) -> H3lis331<H3lis331I2c<I2C>>
    where
        I2C: hal::i2c::I2c,
    {
        H3lis331 {
            interface: H3lis331I2c { i2c, address },
        }
    }
}

/// Sensor configuration options
#[derive(defmt::Format, Clone, Copy)]
pub struct Configuration {
    /// The output data rate, default [`DataRate::Hz400`].
    pub datarate: register::DataRate,
    /// Measure changes in the x axis, default `true`.
    pub enable_x_axis: bool,
    /// Measure changes in the y axis, default `true`.
    pub enable_y_axis: bool,
    /// Measure changes in the z axis, default `true`.
    pub enable_z_axis: bool,
    /// When is data updated
    ///
    /// - when `true`: only after data is read
    /// - when `false`: continually
    ///
    /// default `true`
    pub block_data_update: bool,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            block_data_update: true,
            datarate: register::DataRate::Hz400,
            enable_x_axis: true,
            enable_y_axis: true,
            enable_z_axis: true,
        }
    }
}

impl<Interface> H3lis331Interface for H3lis331<Interface>
where
    Interface: H3lis331Interface,
{
    type BusError = Interface::BusError;
    type PinError = Interface::PinError;

    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        self.interface.write_register(register, value)
    }

    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<Self::BusError, Self::PinError>> {
        self.interface.read_register(register)
    }

    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<Self::BusError, Self::PinError>> {
        self.interface.read_accel_bytes()
    }
}

impl<Interface> H3lis331<Interface>
where
    Interface: H3lis331Interface,
{
    /// Configure the sensor with the provided configuration
    pub fn configure(
        &mut self,
        config: &Configuration,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        if config.block_data_update {
            self.write_register(Register::Ctrl4, BDU)?;
        }

        self.set_datarate(config.datarate)?;

        self.enable_axes((
            config.enable_x_axis,
            config.enable_y_axis,
            config.enable_z_axis,
        ))
    }

    /// Enable or disable measurement on each axis
    pub fn enable_axes(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.modify_register(Register::Ctrl1, |mut ctrl1| {
            ctrl1 &= !(X_EN | Y_EN | Z_EN);
            ctrl1 |= if x { X_EN } else { 0 };
            ctrl1 |= if y { Y_EN } else { 0 };
            ctrl1 |= if z { Z_EN } else { 0 };
            ctrl1
        })
    }

    /// Set the output data rate
    pub fn set_datarate(
        &mut self,
        datarate: DataRate,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.modify_register(Register::Ctrl1, |mut ctrl1| {
            // mask off the ODR bits
            ctrl1 &= !register::ODR_MASK;
            // write the new ODR bits
            ctrl1 |= (datarate as u8) << 4;
            ctrl1
        })
    }

    /// Read the current data rate setting
    pub fn get_datarate(
        &mut self,
    ) -> Result<DataRate, Error<Interface::BusError, Interface::PinError>> {
        let ctrl1 = self.read_register(Register::Ctrl1)?;
        let odr_bits = (ctrl1 & register::ODR_MASK) >> 4;
        DataRate::try_from(odr_bits).map_err(|_| Error::InvalidDataRate)
    }

    /// Set current device range
    /// This affects the sensitivity and therefore the conversion from raw
    /// data to g's.
    pub fn set_range(
        &mut self,
        range: H3lis331Range,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.modify_register(Register::Ctrl4, |mut ctrl4| {
            // mask off the FS bits
            ctrl4 &= !register::FS_MASK;
            // write the new FS bits
            ctrl4 |= range.bits() << 4;
            ctrl4
        })
    }

    /// Read the current device range setting
    pub fn get_range(
        &mut self,
    ) -> Result<H3lis331Range, Error<Interface::BusError, Interface::PinError>> {
        let ctrl4 = self.read_register(Register::Ctrl4)?;
        let fs_bits = (ctrl4 & register::FS_MASK) >> 4;
        fs_bits.try_into().map_err(|_| Error::InvalidRange)
    }

    /// Set the `Register::Reference` register to the provided value
    pub fn set_reference(
        &mut self,
        reference: u8,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.write_register(Register::Reference, reference)
    }

    /// Read the current value of the `Register::Reference` register
    pub fn get_reference(&mut self) -> Result<u8, Error<Interface::BusError, Interface::PinError>> {
        self.read_register(Register::Reference)
    }

    /// Accelerometer data-available status
    pub fn get_status(
        &mut self,
    ) -> Result<DataStatus, Error<Interface::BusError, Interface::PinError>> {
        let status = self.read_register(Register::Status)?;
        Ok(DataStatus::from_register(status))
    }

    /// Convenience function for `STATUS_REG` to confirm all three X, Y and
    /// Z-axis have new data available for reading by accel_raw and associated
    /// function calls.
    pub fn all_data_available(
        &mut self,
    ) -> Result<bool, Error<Interface::BusError, Interface::PinError>> {
        let status = self.get_status()?;
        Ok(status.zyxda)
    }

    /// Read the WHO_AM_I register. Should always return `0b00110011`.
    pub fn who_am_i(&mut self) -> Result<u8, Error<Interface::BusError, Interface::PinError>> {
        self.read_register(Register::WhoAmI)
    }

    /// Modify a register's value. Read the current value of the register,
    /// update the value with the provided function, and set the register to
    /// the return value.
    fn modify_register<F>(
        &mut self,
        register: Register,
        f: F,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register)?;

        self.write_register(register, f(value))
    }

    /// Clear the given bits in the given register. For example:
    ///
    ///     h3lis311.register_clear_bits(some_register, 0b0110)
    ///
    /// This call clears (sets to 0) the bits at index 1 and 2. Other bits of the register are not touched.
    pub fn register_clear_bits(
        &mut self,
        register: Register,
        bits: u8,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.modify_register(register, |value| value & !bits)
    }

    /// Set the given bits in the given register. For example:
    ///
    ///     h3lis311.register_set_bits(some_register, 0b0110)
    ///
    /// This call sets (writes 1) the bits at index 1 and 2. Other bits of the register are not touched.
    pub fn register_set_bits(
        &mut self,
        register: Register,
        bits: u8,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.modify_register(register, |value| value | bits)
    }

    /// Configure one of the interrupt pins
    ///
    ///     h3lis311.configure_interrupt_pin(IrqPin1Config {
    ///         // Raise if interrupt 1 is raised
    ///         ia1_en: true,
    ///         // Disable for all other interrupts
    ///         ..IrqPin1Config::default()
    ///     })?;
    pub fn configure_interrupt<P: IrqPin>(
        &mut self,
        pin: P,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.write_register(P::ctrl_reg(), pin.bits())
    }

    /// Configure an IRQ source
    ///
    /// For example, to configure interrupt 1 to trigger when there is movement
    /// on any axis:
    ///
    ///     h3lis311.configure_interrupt_source(Irq1SourceConfig {
    ///         // Trigger on any axis
    ///         ia1_en: true,
    ///         ..Irq1SourceConfig::default()
    ///     })?;
    pub fn configure_irq_src<I: Interrupt>(
        &mut self,
        int: I,
        interrupt_mode: InterruptMode,
        interrupt_config: InterruptConfig,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.configure_irq_src_and_control(
            int,
            interrupt_mode,
            interrupt_config,
            LatchInterruptRequest::default(),
            Detect4D::default(),
        )
    }

    /// Configure an IRQ source.
    ///
    /// LIS (latch interrupt request) will latch (keep active) the interrupt until the [`Lis331::get_irq_src`] is read.
    ///
    /// 4D detection is a subset of the 6D detection where detection on the Z axis is disabled.
    /// This setting only has effect when the interrupt mode is either `Movement` or `Position`.
    ///
    /// Example: configure interrupt 1 to fire when there is movement along any of the axes.
    ///
    ///     h3lis331.configure_irq_src(
    ///         h3lis331::Interrupt1,
    ///         h3lis331::InterruptMode::Movement,
    ///         h3lis331::InterruptConfig::high_and_low(),
    ///         h3lis331::LatchInterruptRequest::Enable,
    ///         h3lis331::Detect4D::Enable,
    ///     )?;
    pub fn configure_irq_src_and_control<I: Interrupt>(
        &mut self,
        _int: I,
        interrupt_mode: InterruptMode,
        interrupt_config: InterruptConfig,
        latch_interrupt_request: LatchInterruptRequest,
        detect_4d: Detect4D,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        let latch_interrupt_request =
            matches!(latch_interrupt_request, LatchInterruptRequest::Enable);

        let detect_4d = matches!(detect_4d, Detect4D::Enable);

        if latch_interrupt_request || detect_4d {
            let latch = (latch_interrupt_request as u8) << I::lir_int_bit();
            let d4d = (detect_4d as u8) << I::d4d_int_bit();
            self.register_set_bits(Register::Ctrl5, latch | d4d)?;
        }
        self.write_register(I::cfg_reg(), interrupt_config.to_bits(interrupt_mode))
    }

    /// Set the minimum duration for the Interrupt event to be recognized.
    ///
    /// Example: the event has to last at least 25 miliseconds to be recognized.
    ///
    ///     // let mut h3lis331 = ...
    ///     let duration = Duration::miliseconds(DataRate::Hz_400, 25.0);
    ///     h3lis331.configure_irq_duration(duration);
    #[doc(alias = "INT1_DURATION")]
    #[doc(alias = "INT2_DURATION")]
    pub fn configure_irq_duration<I: Interrupt>(
        &mut self,
        _int: I,
        duration: Duration,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.write_register(I::duration_reg(), duration.0)
    }

    /// Set the minimum magnitude for the Interrupt event to be recognized.
    ///
    /// Example: the event has to have a magnitude of at least 1.1g to be recognized.
    ///
    ///     // let mut h3lis331 = ...
    ///     let threshold = Threshold::g(Range::G2, 1.1);
    ///     h3lis331.configure_irq_threshold(threshold);
    #[doc(alias = "INT1_THS")]
    #[doc(alias = "INT2_THS")]
    pub fn configure_irq_threshold<I: Interrupt>(
        &mut self,
        _int: I,
        threshold: Threshold,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.write_register(I::ths_reg(), threshold.0)
    }

    /// Get interrupt source. The `interrupt_active` field is true when an interrupt is active.
    /// The other fields specify what measurement caused the interrupt.
    pub fn get_irq_src<I: Interrupt>(
        &mut self,
        _int: I,
    ) -> Result<InterruptSource, Error<Interface::BusError, Interface::PinError>> {
        let irq_src = self.read_register(I::src_reg())?;
        Ok(InterruptSource::from_bits(irq_src))
    }

    /// Reboot memory content
    pub fn reboot_memory_content(
        &mut self,
    ) -> Result<(), Error<Interface::BusError, Interface::PinError>> {
        self.register_set_bits(Register::Ctrl5, 0b1000_0000)
    }
}

impl<Interface> Accelerometer for H3lis331<Interface>
where
    Interface: H3lis331Interface,
    Interface::BusError: Debug,
    Interface::PinError: Debug,
{
    type Error = Error<Interface::BusError, Interface::PinError>;

    /// Get normalized +- g reading from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `all_data_available`.
    ///
    /// the official driver: https://github.com/adafruit/Adafruit_LIS331/blob/master/Adafruit_H3LIS331.cpp#L119C1-L138C10
    /// was used as a reference.
    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        let range = self.get_range()?;

        // See "2.1 Mechanical characteristics" in the datasheet
        // to find the sensitivity values.
        //
        // Sensitivity is in mg/digit, but we want g/digit, so divide by 1000 here to
        // avoid division on the hardware.
        //
        // Alternatively, we could use what the Adafruit driver does and calculate the
        // sensitivity on the fly based on the range:
        // lsb_value = 2 * scale_max * (float) 1 / 4096;
        // where scale_max is the maximum g value for the range (e.g. 400 for +/- 400g)
        //
        // this could give us a more precise value
        let sensitivity = match range {
            H3lis331Range::G100 => 0.049, // ~= 49 mg/digit ~= 2 * 100 * (1.0 / 4096)
            H3lis331Range::G200 => 0.098, // ~= 98 mg/digit ~= 2 * 200 * (1.0 / 4096)
            H3lis331Range::G400 => 0.195, // ~= 195 mg/digit ~= 2 * 400 * (1.0 / 4096)
        };

        // The H3LIS331 gives us 12-bit left-justified values, since it is two's complement
        // we can just interpret it as a normal i16 and then shift right by 4
        let raw = self.accel_raw()?;
        let x = (raw.x >> 4) as f32 * sensitivity;
        let y = (raw.y >> 4) as f32 * sensitivity;
        let z = (raw.z >> 4) as f32 * sensitivity;

        Ok(F32x3::new(x, y, z))
    }

    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_datarate()?.sample_rate())
    }
}

impl<Interface> RawAccelerometer<I16x3> for H3lis331<Interface>
where
    Interface: H3lis331Interface,
    Interface::PinError: Debug,
    Interface::BusError: Debug,
{
    type Error = Error<Interface::BusError, Interface::PinError>;

    /// Get raw acceleration data from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    ///
    /// See the following reference implementations:
    /// - https://github.com/STMicroelectronics/h3lis331dl-pid/blob/master/h3lis331dl_reg.c#L610-L626
    /// - https://github.com/mmalecki/lis331-rs/blob/master/src/lib.rs#L523-L530
    ///
    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        // read OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H in one go
        let accel_bytes = self.read_accel_bytes()?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}

/// H3lis331Interface implementation for I2C
impl<I2C, E> H3lis331Interface for H3lis331I2c<I2C>
where
    I2C: hal::i2c::I2c<Error = E>,
{
    type BusError = E;
    type PinError = core::convert::Infallible;

    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(self.address as u8, &[register as u8, value])
            .map_err(Error::Bus)
    }

    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<Self::BusError, Self::PinError>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address as u8, &[register as u8], &mut buf)
            .map_err(Error::Bus)?;
        Ok(buf[0])
    }

    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<Self::BusError, Self::PinError>> {
        let mut buf = [0u8; 6];
        // set auto-increment bit in register address
        let register = Register::OutXL as u8 | 0x80;
        self.i2c
            .write_read(self.address as u8, &[register], &mut buf)
            .map_err(Error::Bus)?;
        Ok(buf)
    }
}

impl<SPI, CS, E, PinError> H3lis331Spi<SPI, CS>
where
    SPI: hal::spi::SpiDevice<Error = E>,
    CS: hal::digital::OutputPin<Error = PinError>,
{
    /// Turn on the SPI slave by setting CS low
    fn select(&mut self) -> Result<(), Error<E, PinError>> {
        self.cs.set_low().map_err(Error::Pin)
    }

    /// Turn off the SPI slave by setting CS high
    fn deselect(&mut self) -> Result<(), Error<E, PinError>> {
        self.cs.set_high().map_err(Error::Pin)
    }
}

/// H3lis331Interface implementation for SPI
impl<SPI, CS, E, PinError> H3lis331Interface for H3lis331Spi<SPI, CS>
where
    SPI: hal::spi::SpiDevice<Error = E>,
    CS: hal::digital::OutputPin<Error = PinError>,
{
    type BusError = E;
    type PinError = PinError;

    fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<Self::BusError, Self::PinError>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.select()?;
        // clear the read bit (bit 7) and auto-increment bit (bit 6)
        let mut addr = [register as u8 & 0x3F];
        let write_buf = [value];
        self.spi
            .transfer(&mut addr, &mut write_buf.clone())
            .map_err(Error::Bus)?;
        self.deselect()?;

        Ok(())
    }

    fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<Self::BusError, Self::PinError>> {
        self.select()?;
        // set the read bit (bit 7), clear auto-increment bit (bit 6)
        let mut addr = [register as u8 | 0x80];
        let mut write_buf = [0];
        self.spi
            .transfer(&mut addr, &mut write_buf)
            .map_err(Error::Bus)?;
        self.deselect()?;

        Ok(write_buf[0])
    }

    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<Self::BusError, Self::PinError>> {
        let mut buf = [0u8; 6];
        self.select()?;
        // set auto-increment bit in register address
        let mut register = [Register::OutXL as u8 | 0xC0];
        self.spi
            .transfer(&mut register, &mut buf)
            .map_err(Error::Bus)?;
        self.deselect()?;

        Ok(buf)
    }
}
