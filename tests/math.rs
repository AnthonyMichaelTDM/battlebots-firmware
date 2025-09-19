//! double-checking some math we do in drivers and such
#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests]
mod h3lis311_math {
    use battlebots_firmware::drivers::h3lis331::register::H3lis331Range;
    use core::convert::TryInto;
    use defmt::{assert, assert_eq, info};
    use esp_hal as _;

    #[init]
    fn init() {
        let _ = esp_hal::init(esp_hal::Config::default());

        rtt_target::rtt_init_defmt!();
    }

    #[test]
    fn raw_acceleration_parsing() {
        // read OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H in one go
        let accel_bytes = [0x40, 0x00, 0x90, 0x00, 0xf0, 0x02]; // self.read_accel_bytes()?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        info!("Raw accel data: x=0x{:X}, y=0x{:X}, z=0x{:X}", x, y, z);
        assert_eq!(x, 0x0040);
        assert_eq!(y, 0x0090);
        assert_eq!(z, 0x02F0);
    }

    #[test]
    fn sensitivity_calculation() {
        let g100 = f32::abs(200.0f32 * (1.0f32 / 4096.0));
        let g200 = f32::abs(400.0f32 * (1.0f32 / 4096.0));
        let g400 = f32::abs(800.0f32 * (1.0f32 / 4096.0));

        assert!(
            g100 - 0.049 < 0.0001,
            "+-100G sensitivity calculation failed {=f32} !~+ 0.049",
            g100
        );
        assert!(
            g200 - 0.098 < 0.0001,
            "+-200G sensitivity calculation failed {=f32} !~+ 0.098",
            g200
        );
        assert!(
            g400 - 0.195 < 0.001,
            "+-400G sensitivity calculation failed {=f32} !~+ 0.195",
            g400
        );
    }

    #[test]
    fn normalized_acceleration_calculation() {
        // read OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H in one go
        let accel_bytes = [0x60, 0x00, 0xF0, 0xFF, 0x60, 0x03]; // self.read_accel_bytes()?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        info!("Raw accel data: x=0x{:X}, y=0x{:X}, z=0x{:X}", x, y, z);

        // assuming range is set to 200g
        let range = H3lis331Range::G200;

        let sensitivity_mg = match range {
            H3lis331Range::G100 => 200.0f32 * (1.0f32 / 4096.0),
            H3lis331Range::G200 => 400.0f32 * (1.0f32 / 4096.0),
            H3lis331Range::G400 => 800.0f32 * (1.0f32 / 4096.0),
        };

        let x_mg = (x >> 4) as f32 * sensitivity_mg;
        let y_mg = (y >> 4) as f32 * sensitivity_mg;
        let z_mg = (z >> 4) as f32 * sensitivity_mg;

        info!("Accel: x={=f32}, y={=f32}, z={=f32}", x_mg, y_mg, z_mg);
    }
}
