//! see <https://github.com/adafruit/Adafruit_LIS331/blob/master/Adafruit_LIS331.h> for reference

/// Possible I2C slave addresses.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SlaveAddr {
    /// Default slave address (`0x18`)
    Default = 0x18,

    /// Alternate slave address (`0x19`),
    /// set by connecting the SDO/SA0 pin to 3.3V
    Alternate = 0x19,
}

impl SlaveAddr {
    pub fn addr(self) -> u8 {
        self as u8
    }
}

pub const CHIP_ID: u8 = 0x32; // The default response to WHO_AM_I for the H3LIS331 and LIS331HH

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    WhoAmI = 0x0F,        // Device identification register. 0b00110011
    Ctrl1 = 0x20,         // Data rate, power mode, axis enable
    Ctrl2 = 0x21,         // Memory reboot, HPF config
    Ctrl3 = 0x22,         // Interrupt config, polarity, pin mode, latching, pin enable
    Ctrl4 = 0x23,         // BDU, Endianness, Range, SPI mode
    Ctrl5 = 0x24,         // Sleep to wake enable
    HpFilterReset = 0x25, // Dummy register to reset filter
    Reference = 0x26,     // HPF reference value
    Status = 0x27,        // Data overrun status, Data available status
    OutXL = 0x28,         // X-axis acceleration data. Low value (bits 0-7)
    OutXH = 0x29,         // X-axis acceleration data. High value (bits 8-15)
    OutYL = 0x2A,         // Y-axis acceleration data. Low value (bits 0-7)
    OutYH = 0x2B,         // Y-axis acceleration data. High value (bits 8-15)
    OutZL = 0x2C,         // Z-axis acceleration data. Low value (bits 0-7)
    OutZH = 0x2D,         // Z-axis acceleration data. High value (bits 8-15)
    Int1Cfg = 0x30,       // INT1 config. Enable on hi/low for each axis
    Int1Src = 0x31,       // INT1 source info
    Int1Ths = 0x32,       // INT1 acceleration threshold
    Int1Dur = 0x33,       // INT1 duration threshold
    Int2Cfg = 0x34,       // INT2 config. Enable on hi/low for each axis
    Int2Src = 0x35,       // INT2 source info
    Int2Ths = 0x36,       // INT2 acceleration threshold
    Int2Dur = 0x37,       // INT3 duration threshold
}

impl Register {
    pub const fn addr(self) -> u8 {
        self as u8
    }

    /// Is the register read-only?
    pub const fn read_only(self) -> bool {
        matches!(
            self,
            Register::Status
                | Register::OutXL
                | Register::OutXH
                | Register::OutYL
                | Register::OutYH
                | Register::OutZL
                | Register::OutZH
                | Register::Int1Src
                | Register::Int2Src
        )
    }
}

/// Measurement range options
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum H3lis331Range {
    G100 = 0b00, // +-100g
    G200 = 0b01, // +-200g
    G400 = 0b10, // +-400g
}

impl Default for H3lis331Range {
    fn default() -> Self {
        H3lis331Range::G100
    }
}

impl H3lis331Range {
    pub const fn bits(self) -> u8 {
        self as u8
    }
    /// Convert the range into an value in mili-g
    pub const fn as_mg(self) -> u8 {
        match self {
            H3lis331Range::G100 => 49,
            H3lis331Range::G200 => 98,
            H3lis331Range::G400 => 195,
        }
    }
}

impl TryFrom<u8> for H3lis331Range {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b00 => Ok(H3lis331Range::G100),
            0b01 => Ok(H3lis331Range::G200),
            0b10 => Ok(H3lis331Range::G400),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct Threshold(pub(crate) u8);

impl Threshold {
    /// Convert a value in multiples of the `g` constant (roughly 9.81) to a threshold.
    ///
    ///     assert_eq!(Threshold::g(H3lis331Range::G200, 1.1), 69);
    #[inline(always)]
    pub fn g(range: H3lis331Range, gs: f32) -> Self {
        Self::mg(range, gs * 1000.0)
    }

    #[inline(always)]
    pub fn mg(range: H3lis331Range, mgs: f32) -> Self {
        let value = mgs / (range.as_mg() as f32);

        let result = crude_ceil(value);

        Threshold(result.try_into().unwrap())
    }

    pub const ZERO: Self = Threshold(0);
}

/// a crude `.ceil()`, the std one is not currently available when using no_std
fn crude_ceil(value: f32) -> u64 {
    let truncated = value as u64;

    let round_up = value - (truncated as f32) > 0.0;

    if round_up {
        truncated + 1
    } else {
        truncated
    }
}

/// Data rate options, used with CTRL_REG1 to set bandwidth
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DataRate {
    PowerDown = 0,
    Hz50 = 0x4,
    Hz100 = 0x5,
    Hz400 = 0x6,
    Hz1000 = 0x7,
    LowPower0_5Hz = 0x8,
    LowPower1Hz = 0xC,
    LowPower2Hz = 0x10,
    LowPower5Hz = 0x14,
    LowPower10Hz = 0x18,
}

impl DataRate {
    pub const fn bits(self) -> u8 {
        self as u8
    }

    pub const fn sample_rate(self) -> f32 {
        match self {
            DataRate::PowerDown => 0.0,
            DataRate::Hz50 => 50.0,
            DataRate::Hz100 => 100.0,
            DataRate::Hz400 => 400.0,
            DataRate::Hz1000 => 1000.0,
            DataRate::LowPower0_5Hz => 0.5,
            DataRate::LowPower1Hz => 1.0,
            DataRate::LowPower2Hz => 2.0,
            DataRate::LowPower5Hz => 5.0,
            DataRate::LowPower10Hz => 10.0,
        }
    }
}

impl TryFrom<u8> for DataRate {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(DataRate::PowerDown),
            0x4 => Ok(DataRate::Hz50),
            0x5 => Ok(DataRate::Hz100),
            0x6 => Ok(DataRate::Hz400),
            0x7 => Ok(DataRate::Hz1000),
            0x8 => Ok(DataRate::LowPower0_5Hz),
            0xC => Ok(DataRate::LowPower1Hz),
            0x10 => Ok(DataRate::LowPower2Hz),
            0x14 => Ok(DataRate::LowPower5Hz),
            0x18 => Ok(DataRate::LowPower10Hz),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct Duration(pub(crate) u8);

impl Duration {
    /// Convert a number of seconds into a duration. Internally a duration is represented
    /// as a multiple of `1 / ODR` where ODR (the output data rate) is of type [`DataRate`].
    #[inline(always)]
    pub fn seconds(output_data_rate: DataRate, seconds: f32) -> Self {
        let duration = output_data_rate.sample_rate() * seconds;

        Self(duration as u8)
    }

    /// Convert a number of miliseconds into a duration. Internally a duration is represented
    /// as a multiple of `1 / ODR` where ODR (the output data rate) is of type [`DataRate`].
    ///
    ///     assert_eq!(Duration::miliseconds(DataRate::Hz_400, 25.0), 10);
    #[inline(always)]
    pub fn miliseconds(output_data_rate: DataRate, miliseconds: f32) -> Self {
        Self::seconds(output_data_rate, miliseconds * 1000.0)
    }

    pub const ZERO: Self = Duration(0);
}

/// Data status structure. Decoded from the `STATUS_REG` register.
///
/// `STATUS_REG` has the following bit fields:
///   * `ZYXOR` - X, Y and Z-axis data overrun
///   * `ZOR` - Z-axis data overrun
///   * `YOR` - Y-axis data overrun
///   * `XOR` - X-axis data overrun
///   * `ZYXDA` - X, Y and Z-axis new data available
///   * `ZDA` - Z-axis new data available
///   * `YDA` Y-axis new data available
///   * `XDA` X-axis new data available
///
/// This struct splits the fields into more convenient groups:
///  * `zyxor` -> `ZYXOR`
///  * `xyzor` -> (`XOR`, `YOR`, `ZOR`)
///  * `zyxda` -> `ZYXDA`
///  * `xyzda` -> (`XDA`, `YDA`, `ZDA`)
#[derive(Debug)]
pub struct DataStatus {
    /// ZYXOR bit
    pub zyxor: bool,

    /// (XOR, YOR, ZOR) bits
    pub xyzor: (bool, bool, bool),

    /// ZYXDA bit
    pub zyxda: bool,

    /// (XDA, YDA, ZDA) bits
    pub xyzda: (bool, bool, bool),
}

impl DataStatus {
    /// Decode a `u8` value read from the `STATUS_REG` register into a `DataStatus` struct.
    pub fn from_register(value: u8) -> Self {
        Self {
            zyxor: (value & ZYXOR) != 0,
            xyzor: ((value & XOR) != 0, (value & YOR) != 0, (value & ZOR) != 0),
            zyxda: (value & ZYXDA) != 0,
            xyzda: ((value & XDA) != 0, (value & YDA) != 0, (value & ZDA) != 0),
        }
    }
}

// === CTRL_REG1 (20h) ===

pub const ODR_MASK: u8 = 0b1111_0000;
pub const LP_EN: u8 = 0b0000_1000;
pub const Z_EN: u8 = 0b0000_0100;
pub const Y_EN: u8 = 0b0000_0010;
pub const X_EN: u8 = 0b0000_0001;

// === CTRL_REG4 (23h) ===

pub const BDU: u8 = 0b1000_0000;
pub const FS_MASK: u8 = 0b0011_0000;
pub const HR: u8 = 0b0000_1000;

// === STATUS_REG (27h) ===

pub const ZYXOR: u8 = 0b1000_0000;
pub const ZOR: u8 = 0b0100_0000;
pub const YOR: u8 = 0b0010_0000;
pub const XOR: u8 = 0b0001_0000;
pub const ZYXDA: u8 = 0b0000_1000;
pub const ZDA: u8 = 0b0000_0100;
pub const YDA: u8 = 0b0000_0010;
pub const XDA: u8 = 0b0000_0001;

/// High pass filter cutoff frequency
#[repr(u8)]
pub enum HpfCutoff {
    Odr50,  // ODR/50
    Odr100, // ODR/100
    Odr200, // ODR/200
    Odr400, // ODR/400
}

/// Low pass filter cutoff frequency
#[repr(u8)]
pub enum LpfCutoff {
    Hz37,
    Hz74,
    Hz292,
    Hz780,
}

/// Mode options
#[repr(u8)]
pub enum Mode {
    Shutdown,
    Normal,
    LowPower, // Low power is from 2-6 so checks against this should be 'mode >=LIS331_MODE_LOW_POWER'
}
