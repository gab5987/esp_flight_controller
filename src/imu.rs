/*
 * REQ-MPU-000: Shall initialize the mpu6050 sensor with the following
 * characteristics:
 * - Accelerometer force sensitivity: around 6 - 10 times the gravitational
 * acceleration.
 * - Gyroscopic sensitivity shall be 1000 DPS -> Better response due to the lack
 * of adc bits in the 2000 DPS sensitivity
 *
 * REQ-MPU-001: Shall Read the values using a interruption pin in NEG_EDGE mode.
 * Every time the mpu finishes a conversion cycle, it must trigger a
 * interruption and read the values.
 *
 * REQ-MPU-001: Shall calibrate the mpu during the first boot, a simple mean of
 * a couple measurements might be good enought ig
 *
 * REQ-MPU-003: Shall convert the raw data to rad/s, preferably during the read
 * function to avoid future overheading.
 *
 * REQ-MPU-004: Shall calculate the row and pitch values based on both the
 * accelerometer and the gyroscope data using Kalman's filter model.
 * */

use esp_idf_svc::{hal::{prelude::*, i2c::{I2cConfig, I2cDriver}, peripherals::Peripherals, gpio::AnyIOPin}, sys::*};

enum AcceFs
{
    AcceFs2g  = 0, // !< Accelerometer full scale range is +/- 2g
    AcceFs4g  = 1, // !< Accelerometer full scale range is +/- 4g
    AcceFs8g  = 2, // !< Accelerometer full scale range is +/- 8g
    AcceFs16g = 3, // !< Accelerometer full scale range is +/- 16g
}

enum DlpfCf
{
    Dlpf260 = 0, // !< The low pass filter is configurated to 260 Hz 
    Dlpf184 = 1, // !< The low pass filter is configurated to 184 Hz 
    Dlpf94  = 2, // !< The low pass filter is configurated to 94 Hz 
    Dlpf44  = 3, // !< The low pass filter is configurated to 44 Hz 
    Dlpf21  = 4, // !< The low pass filter is configurated to 21 Hz 
    Dlpf10  = 5, // !< The low pass filter is configurated to 10 Hz 
    Dlpf5   = 6, // !< The low pass filter is configurated to 5 Hz 
}

enum Register
{
    DlpfConfig = 0x1A,
    GyroConfig = 0x1B,
    AccelConfig = 0x1C,
    AccelXoutH = 0x3B,
    PwrMgt1 = 0x6B,
    WhoAmI = 0x75,
}

pub struct CompFilter { pub roll: f32, pub pitch: f32 }

struct MPU6050<'a> {
    driver: I2cDriver<'a>,
    addr: u8,
}

impl MPU6050<'_> {
    pub fn new() -> Result<Self, EspError>
    {
        let peripherals = Peripherals::take().unwrap();
        let sda_pin = peripherals.pins.gpio21;
        let scl_pin = peripherals.pins.gpio22;

        let config = I2cConfig::new().baudrate(400.kHz().into());
        let driver = I2cDriver::new(peripherals.i2c0, sda_pin, scl_pin, &config)?;

        return Ok(Self { addr: 0x68<<1, driver })
    }
    
    fn writeregister(&mut self, reg_start_addr: Register, data_buf: &[u8])
    {
        let reg: [u8; 1] = [reg_start_addr as u8];
        let _ = self.driver.write(self.addr, &reg, 30);
        let _ = self.driver.write(self.addr, data_buf, 30);
    }
    
    fn readregister(&mut self, reg_start_addr: Register, data_buf: &mut [u8])
    {
        let reg: [u8; 1] = [reg_start_addr as u8];
        let _ = self.driver.write_read(self.addr, &reg, data_buf, 30);
    }

    pub fn setup(&mut self)
    {
        {
            let regstp: [u8; 1] = [DlpfCf::Dlpf10 as u8];
            self.writeregister(Register::DlpfConfig, &regstp);
        }

        {
            let regstp: [u8; 2] = [DlpfCf::Dlpf10 as u8];
            self.writeregister(Register::DlpfConfig, &regstp);
        }
    } 
}

static DEV_ANGLE: CompFilter = CompFilter{ roll: 0.0, pitch: 0.0 };

pub fn initialize() -> i32 
{
    let mut mpu6050 = MPU6050::new().unwrap();

    mpu6050.setup();

    return ESP_OK; 
}
