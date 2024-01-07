/*
 * REQ-MPU-000: Shall initialize the mpu6050 sensor with the following
 * characteristics:
 * - Accelerometer force sensitivity: around 6 - 10 times the gravitational
 * acceleration.
 * - Gyroscopic sensitivity shall be 1000 Dps -> Better response due to the lack
 * of adc bits in the 2000 Dps sensitivity
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

use esp_idf_svc::{hal::{prelude::*, i2c::{I2cConfig, I2cDriver}, peripherals::Peripherals, delay::{BLOCK, FreeRtos}}, sys::*};

#[repr(u8)]
enum AcceFs
{
    Fs2g  = 0, // !< Accelerometer full scale range is +/- 2g
    Fs4g  = 1, // !< Accelerometer full scale range is +/- 4g
    Fs8g  = 2, // !< Accelerometer full scale range is +/- 8g
    Fs16g = 3, // !< Accelerometer full scale range is +/- 16g
}

#[repr(C)]
enum GyroFs
{
    Dps250 = 0, // !< Gyroscope full scale range is +/- 250 degree per sencond
    Dps500 = 1, // !< Gyroscope full scale range is +/- 500 degree per sencond 
    Dps1000 = 2, // !< Gyroscope full scale range is +/- 1000 degree per sencond 
    Dps2000 = 3, // !< Gyroscope full scale range is +/- 2000 degree per sencond
}

#[repr(u8)]
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

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
enum Register
{
    DlpfConfig = 0x1A,
    GyroConfig = 0x1B,
    AccelConfig = 0x1C,
    AccelXoutH = 0x3B,
    PwrMgt1 = 0x6B,
}

pub struct Axis
{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct Mpu6050AxisData
{
    acce: Axis,
    gyro: Axis,
}

macro_rules! init_mpu6050_axis_data {
    () => {
        Mpu6050AxisData {
            acce: Axis{x: 0.0, y: 0.0, z: 0.0},
            gyro: Axis{x: 0.0, y: 0.0, z: 0.0},
        }
    };
}

pub struct CompFilter
{
    pub roll: f32,
    pub pitch: f32,
}

struct MPU6050<'a> {
    driver: I2cDriver<'a>,
    addr: u8,

    gyro_cal: Axis,
    gyro_sens: f32,
    acce_sens: f32,
}

const CAL_ITERATIONS: u16 = 2000;

impl MPU6050<'_> {
    pub fn new() -> Result<Self, EspError>
    {
        let peripherals = Peripherals::take().unwrap();
        let sda_pin = peripherals.pins.gpio21;
        let scl_pin = peripherals.pins.gpio22;
        let addr = 0x68;

        let config = I2cConfig::new().baudrate(400.kHz().into());
        let driver = I2cDriver::new(peripherals.i2c0, sda_pin, scl_pin, &config)?;

        return Ok(Self { driver, addr, gyro_cal: Axis{x: 0.0, y: 0.0, z: 0.0}, gyro_sens: 0.0, acce_sens: 0.0 })
    }
    
    fn writeregister<const N: usize>(&mut self, reg_start_addr: Register, data_buf: &[u8; N]) -> Result<(), EspError>
    {
        let mut reg = vec![reg_start_addr as u8; N + 1];
        for i in 0..data_buf.len() { reg[i+1] = data_buf[i]; }

        if let Err(e) = self.driver.write(self.addr, &reg, BLOCK)
        {
            log::error!("Error while writting to register {:#04x} -> {}", reg[0], e.to_string());
            return Err(e);
        }

        return Ok(());
    }
    
    fn readregister(&mut self, reg_start_addr: Register, data_buf: &mut [u8]) -> Result<(), EspError>
    {
        let reg: [u8; 1] = [reg_start_addr as u8];
        if let Err(e) = self.driver.write_read(self.addr, &reg, data_buf, BLOCK)
        {
            log::error!("Error while reading from register {:#04x} -> {}", reg[0], e.to_string());
            return Err(e);
        }

        return Ok(());

    }

    fn get_gyro_sensitivity(&mut self) -> Result<f32, EspError>
    {
        let mut regcfg: [u8; 1] = [0];
        self.readregister(Register::GyroConfig, &mut regcfg)?;
        let val: f32 = match (regcfg[0] >> 3) & 0x03
        {
            0 => 131.0,
            1 => 65.5,
            2 => 32.8,
            3 => 16.4,
            _ => 0.0,
        };
        return Ok(val);
    }

    fn get_acce_sensitivity(&mut self) -> Result<f32, EspError>
    {
        let mut regcfg: [u8; 1] = [0];
        self.readregister(Register::AccelConfig, &mut regcfg)?;
        let val: f32 = match (regcfg[0] >> 3) & 0x03
        {
            0 => 16384.0,
            1 => 8192.0,
            2 => 4096.0,
            3 => 2048.0,
            _ => 0.0,
        };
        return Ok(val);
    }

    fn wakeup(&mut self) -> Result<(), EspError>
    {
        let mut recfg: [u8; 1] = [0];
        self.readregister(Register::PwrMgt1, &mut recfg)?;
        recfg[0] &= !0x00000040;
        self.writeregister(Register::PwrMgt1, &recfg)?;
        return Ok(());
    }

    fn sleep(&mut self) -> Result<(), EspError>
    {
        let mut recfg: [u8; 1] = [0];
        self.readregister(Register::PwrMgt1, &mut recfg)?;
        recfg[0] |= 0x00000040;
        self.writeregister(Register::PwrMgt1, &recfg)?;
        return Ok(());
    }

    pub fn read(&mut self, axd: &mut Mpu6050AxisData) -> Result<(), EspError>
    {
        /* Somehow i could read all the registers on a single burst by
         * requesting past the 6 bytes from the accelerometer register */
        let mut data_rd: [u8; 14] = [0; 14];

        if let Err(e) = self.readregister(Register::AccelXoutH, &mut data_rd)
        {
            log::error!("Could not read data from mpu6050! {}", e.to_string());
            return Err(e);
        }

        /*
         * Accordingly to the datasheet, all the axis data are represented as a 16
         * bit integer spread in two 8 bit registers.*/
        let acce_x: i16 = ((data_rd[0] as i16) << 8) | data_rd[1] as i16;
        axd.acce.x = (acce_x as f32) / self.acce_sens;

        let acce_y: i16 = ((data_rd[2] as i16) << 8) | data_rd[3] as i16;
        axd.acce.y = (acce_y as f32) / self.acce_sens;

        let acce_z: i16 = ((data_rd[4] as i16) << 8) | data_rd[5] as i16;
        axd.acce.z = (acce_z as f32) / self.acce_sens;

        let gyro_x: i16 = ((data_rd[8] as i16) << 8) | data_rd[9] as i16;
        axd.gyro.x = ((gyro_x as f32) / self.gyro_sens) - self.gyro_cal.x;

        let gyro_y: i16 = ((data_rd[10] as i16) << 8) | data_rd[11] as i16;
        axd.gyro.y = ((gyro_y as f32) / self.gyro_sens) - self.gyro_cal.y;

        let gyro_z: i16 = ((data_rd[12] as i16) << 8) | data_rd[13] as i16;
        axd.gyro.z = ((gyro_z as f32) / self.gyro_sens) - self.gyro_cal.z;

        return Ok(());
    }

    pub fn calibrate(&mut self) -> Result<(), EspError>
    {
        let mut cal_data = init_mpu6050_axis_data!();
        let mut cal_axis = Axis{ x: 0.0, y: 0.0, z: 0.0 };

        let mut iterations: u16 = 0;   
        while iterations < CAL_ITERATIONS
        {
            let _ = self.read(&mut cal_data)?;
            cal_axis.x += cal_data.gyro.x;
            cal_axis.y += cal_data.gyro.y;
            cal_axis.z += cal_data.gyro.z;
            iterations += 1;
            FreeRtos::delay_ms(10);
        }

        self.gyro_cal.x = cal_axis.x / (CAL_ITERATIONS as f32);
        self.gyro_cal.y = cal_axis.y / (CAL_ITERATIONS as f32);
        self.gyro_cal.z = cal_axis.z / (CAL_ITERATIONS as f32);

        log::info!(
            "---> Gyroscope calibration done <--- X: {}\t| Y: {}\t| Z: {}",
            self.gyro_cal.x, self.gyro_cal.y, self.gyro_cal.z
        );

        return Ok(());
    }

    pub fn setup(&mut self) -> Result<(), EspError>
    {
        self.wakeup()?;

        let regstp: [u8; 1] = [DlpfCf::Dlpf10 as u8];
        self.writeregister(Register::DlpfConfig, &regstp)?;

        let regstp: [u8; 2] = [(GyroFs::Dps500 as u8) << 3, (AcceFs::Fs8g as u8) << 3];
        self.writeregister(Register::GyroConfig, &regstp)?;

        self.gyro_sens = self.get_gyro_sensitivity()?;
        self.acce_sens = self.get_acce_sensitivity()?;

        return Ok(());
    } 
}

// static DEV_ANGLE: CompFilter = CompFilter{ roll: 0.0, pitch: 0.0 };

pub fn initialize() -> i32 
{
    let mut mpu6050 = MPU6050::new().unwrap();

    let _ = mpu6050.setup();

    let _ = mpu6050.calibrate();

    // let mut axd = init_mpu6050_axis_data!();
    //
    // loop 
    // {
    //     let _ = mpu6050.read(&mut axd);
    //     log::info!("\nacce -> x: {} | y: {} | z: {}\ngyro -> x: {} | y: {} | z: {}", axd.acce.x,axd.acce.y,axd.acce.x,axd.gyro.x,axd.gyro.y,axd.gyro.z);
    //     FreeRtos::delay_ms(1000);
    // }

    return ESP_OK; 
}
