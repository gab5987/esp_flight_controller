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
 * TODO:
 * REQ-MPU-004: Shall calculate the row and pitch values based on both the
 * accelerometer and the gyroscope data using Kalman's filter model.
 * */

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <math.h>

#include "cfg.h"
#include "hal.h"

enum
{
    ACCE_FS_2G  = 0, /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 1, /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 2, /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3, /*!< Accelerometer full scale range is +/- 16g */
};

enum
{
    GYRO_FS_250DPS =
        0, /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    GYRO_FS_500DPS =
        1, /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_1000DPS =
        2, /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_2000DPS =
        3, /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
};

enum
{
    DLPF_260 = 0, /*!< The low pass filter is configurated to 260 Hz */
    DLPF_184 = 1, /*!< The low pass filter is configurated to 184 Hz */
    DLPF_94  = 2, /*!< The low pass filter is configurated to 94 Hz */
    DLPF_44  = 3, /*!< The low pass filter is configurated to 44 Hz */
    DLPF_21  = 4, /*!< The low pass filter is configurated to 21 Hz */
    DLPF_10  = 5, /*!< The low pass filter is configurated to 10 Hz */
    DLPF_5   = 6, /*!< The low pass filter is configurated to 5 Hz */
};

struct mpu6050_axis_data
{
    f32 acce_x, acce_y, acce_z;
    f32 gyro_x, gyro_y, gyro_z;
};

static const char TAG[] = "imu";

// static const f32 ALPHA      = 0.96F; /*!< Weight of gyroscope */
static const f32 RAD_TO_DEG = (f32)(M_PI / (f32)180); /*!< Radians to degrees */

static const gpio_num_t SCL_PIN  = GPIO_NUM_22;
static const gpio_num_t SDA_PIN  = GPIO_NUM_21;
static const i2c_port_t I2C_PORT = I2C_NUM_0;
static const byte       I2C_ADDR = 0x68U << 1;
static const u32        I2C_FREQ = 400000;
// static const gpio_num_t INT_PIN                = GPIO_NUM_12;
static const i64 MEASUREMENT_CYCLE_TIME = 8000; /*[us]: 250 sps */

/* MPU6050 register */
static const u16 MPU6050_DLPF_CONFIG  = 0x1AU;
static const u16 MPU6050_GYRO_CONFIG  = 0x1BU;
static const u16 MPU6050_ACCEL_CONFIG = 0x1CU;
// static const u16 MPU6050_INTR_PIN_CFG = 0x37U;
// static const u16 MPU6050_INTR_ENABLE  = 0x38U;
// static const u16 MPU6050_INTR_STATUS  = 0x3AU;
static const u16 MPU6050_ACCEL_XOUT_H = 0x3BU;
// static const u16 MPU6050_GYRO_XOUT_H  = 0x43U;
// static const u16 MPU6050_TEMP_XOUT_H  = 0x41U;
static const u16 MPU6050_PWR_MGMT_1 = 0x6BU;

static const struct
{
    f32 x, y, z;
} ACCE_CAL = {.x = 0.06F, .y = 0.0F, .z = 1.02F};
static const struct
{
    f32 x, y, z;
} GYRO_CAL = {.x = -1.24F, .y = 2.75F, .z = 0.20F};

const u8 MPU6050_DATA_RDY_INT_BIT      = (u8)BIT0;
const u8 MPU6050_I2C_MASTER_INT_BIT    = (u8)BIT3;
const u8 MPU6050_FIFO_OVERFLOW_INT_BIT = (u8)BIT4;
const u8 MPU6050_MOT_DETECT_INT_BIT    = (u8)BIT6;
const u8 MPU6050_ALL_INTERRUPTS =
    (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT |
     MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

static f32                s_gyro_sensitivity = 0.0F;
static f32                s_acce_sensitivity = 0.0F;
static esp_timer_handle_t s_timer_handler;

static struct mpu6050
{
    f32 pitch, roll;
} s_device;

static esp_err_t writeregister(
    const u8 reg_start_addr, const byte *const data_buf, const u8 data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t        ret = i2c_master_start(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG, "Cannot start i2c master command: %s", esp_err_to_name(ret));
        return ret;
    }
    i2c_master_write_byte(cmd, I2C_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_start_addr, true);
    i2c_master_write(cmd, data_buf, data_len, true);
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG, "Cannot stop i2c master command: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG, "i2c command transmission failed: %s", esp_err_to_name(ret));
    }

    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t readregister(
    const u8 reg_start_addr, byte *const data_buf, const u8 data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t        ret = i2c_master_start(cmd);
    ret = i2c_master_write_byte(cmd, I2C_ADDR | I2C_MASTER_WRITE, true);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    ret = i2c_master_start(cmd);
    ret = i2c_master_write_byte(cmd, I2C_ADDR | I2C_MASTER_READ, true);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    ret = i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t mpu6050_wake_up(void)
{
    byte      tmp = 0;
    esp_err_t ret = readregister(MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) return ret;
    tmp &= (~BIT6);
    ret = writeregister(MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_sleep(void)
{
    byte      tmp = 0;
    esp_err_t ret = readregister(MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) return ret;
    tmp |= BIT6;
    ret = writeregister(MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

static esp_err_t mpu6050_get_acce_sensitivity(f32 *const acce_sensitivity)
{
    byte      acce_fs = 0;
    esp_err_t ret     = readregister(MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs           = (acce_fs >> 3) & 0x03;
    switch (acce_fs)
    {
        case ACCE_FS_2G:
            *acce_sensitivity = 16384;
            break;
        case ACCE_FS_4G:
            *acce_sensitivity = 8192;
            break;
        case ACCE_FS_8G:
            *acce_sensitivity = 4096;
            break;
        case ACCE_FS_16G:
            *acce_sensitivity = 2048;
            break;
        default:
            break;
    }
    return ret;
}

static esp_err_t mpu6050_get_gyro_sensitivity(f32 *const gyro_sensitivity)
{
    byte      gyro_fs = 0;
    esp_err_t ret     = readregister(MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs           = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs)
    {
        case GYRO_FS_250DPS:
            *gyro_sensitivity = 131.0F;
            break;
        case GYRO_FS_500DPS:
            *gyro_sensitivity = 65.5F;
            break;
        case GYRO_FS_1000DPS:
            *gyro_sensitivity = 32.8F;
            break;
        case GYRO_FS_2000DPS:
            *gyro_sensitivity = 16.4F;
            break;
        default:
            break;
    }
    return ret;
}

esp_err_t mpu6050_request_data(struct mpu6050_axis_data *axdata)
{
    static byte data_rd[16] = {0};

    /* Somehow i could read all the registers on a single burst by
     * requesting past the 6 bytes from the accelerometer register */
    esp_err_t ret = readregister(MPU6050_ACCEL_XOUT_H, data_rd, 14);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG, "Unable to read from data register: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * Accordingly to the datasheet, all the axis data are represented as a 16
     * bit integer spread in two 8 bit registers.
     * */
    axdata->acce_x =
        (f32)((i16)((data_rd[0] << 8) | data_rd[1])) / s_acce_sensitivity;
    axdata->acce_y =
        (f32)((i16)((data_rd[2] << 8) | data_rd[3])) / s_acce_sensitivity;
    axdata->acce_z =
        (f32)((i16)((data_rd[4] << 8) | data_rd[5])) / s_acce_sensitivity;

    axdata->gyro_x =
        (f32)((i16)((data_rd[8] << 8) | data_rd[9])) / s_gyro_sensitivity;
    axdata->gyro_y =
        (f32)((i16)((data_rd[10] << 8) | data_rd[11])) / s_gyro_sensitivity;
    axdata->gyro_z =
        (f32)((i16)((data_rd[12] << 8) + data_rd[13])) / s_gyro_sensitivity;

    axdata->acce_x -= ACCE_CAL.x;
    axdata->acce_y -= ACCE_CAL.y;
    axdata->acce_z -= ACCE_CAL.z;
    axdata->gyro_x -= GYRO_CAL.x;
    axdata->gyro_y -= GYRO_CAL.y;
    axdata->gyro_z -= GYRO_CAL.z;

    return ret;
}

static void sensread(UNUSED void *arg)
{
    static struct mpu6050_axis_data axis_data;

    esp_err_t err = ESP_FAIL;
    u8        ctr = 0;

    do
    {
        err = mpu6050_request_data(&axis_data);
        if (ctr > 5) return;
        ctr++;
    } while (err != ESP_OK);

// #define CALIBRATION_LOG
#ifdef CALIBRATION_LOG
    printf(
        "accelerometer -> x: %.2f\t| y: %.2f\t| z: %.2f\n", axis_data.acce_x,
        axis_data.acce_y, axis_data.acce_z);
    printf(
        "gyroscope -> x: %.2f\t| y: %.2f\t| z: %.2f\n\n", axis_data.gyro_x,
        axis_data.gyro_y, axis_data.gyro_z);
    return;
#endif /* ifdef DEBUG_LOG */

    /*
     * Given a rotation matrix R, we can compute the Euler angles, ψ, θ, and
     * φ by equating each element in R with the corresponding element in the
     * matrix product Rz(φ)Ry(θ)Rx(ψ).
     *
     * - φ is the signed angle between the x axis and the N axis
     *  (x-convention – it could also be defined between y and N, called
     * y-convention).
     * - θ is the angle between the z axis and the Z axis.
     * - ψ is the signed angle between the N axis and the X axis
     * (x-convention).
     *
     * Euler discrete angle formulas:
     * Ay(rad) = atangent( X / √(Y² + Z²) )
     * Ax(rad) = atangent( Y / √(X² + Z²) )
     * */

    f32 powacz_2 = (f32)pow(axis_data.acce_z, 2);

    f32 acce_angle_x = (f32)(atan(
        axis_data.acce_y / sqrt(pow(axis_data.acce_x, 2) + powacz_2)));
    acce_angle_x /= RAD_TO_DEG;

    f32 acce_angle_y = (f32)(atan(
        -1 * axis_data.acce_x / sqrt(pow(axis_data.acce_y, 2) + powacz_2)));
    acce_angle_y /= RAD_TO_DEG;

    static f32 pitch             = 0.0F;
    static f32 pitch_uncertainty = 4.0F;

    if (axis_data.gyro_x != INFINITY && axis_data.acce_y != INFINITY)
    {
        pitch = pitch + 0.004F * axis_data.acce_y;
        pitch_uncertainty =
            pitch_uncertainty + (const f32)(0.004 * 0.004 * 4 * 4);
        f32 pitch_gain    = pitch_uncertainty / (pitch_uncertainty + (3 * 3));
        pitch             = pitch + pitch_gain * (acce_angle_y - pitch);
        pitch_uncertainty = (1 - pitch_gain) * pitch_uncertainty;
    }

    static f32 roll             = 0.0F;
    static f32 roll_uncertainty = 4.0F;

    if (axis_data.gyro_y != INFINITY && axis_data.acce_x != INFINITY)
    {
        roll = roll + 0.004F * axis_data.acce_x;
        roll_uncertainty =
            roll_uncertainty + (const f32)(0.004 * 0.004 * 4 * 4);
        f32 roll_gain    = roll_uncertainty / (roll_uncertainty + (3 * 3));
        roll             = roll + roll_gain * (acce_angle_x - roll);
        roll_uncertainty = (1 - roll_gain) * roll_uncertainty;
    }

    printf("r: %.2f \t p: %.2f\n", roll, pitch);
}

esp_err_t imu_get_complimentary_angle(complimentary_angle_t *const cang)
{
    if (cang == NULL)
    {
        ESP_LOGE(TAG, "complimentary_angle_t argument is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    cang->roll  = s_device.roll;
    cang->pitch = s_device.pitch;

    return ESP_OK;
}

esp_err_t imu_initialize(void)
{
    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = SDA_PIN;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = SCL_PIN;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    conf.clk_flags        = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    s_device.roll = s_device.pitch = 0;

    esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c param config error: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c install error: %s", esp_err_to_name(ret));
        return ret;
    }

    byte lowpass_cfg = DLPF_10;
    writeregister(MPU6050_DLPF_CONFIG, &lowpass_cfg, sizeof(lowpass_cfg));

    byte config_regs[2] = {GYRO_FS_500DPS << 3, ACCE_FS_8G << 3};
    ret = writeregister(MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Device config error: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mpu6050_get_gyro_sensitivity(&s_gyro_sensitivity);
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG, "Failed to get gyroscope sensitivity: %s",
            esp_err_to_name(ret));
        return ret;
    }

    ret = mpu6050_get_acce_sensitivity(&s_acce_sensitivity);
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG, "Failed to get accelerometer sensitivity: %s",
            esp_err_to_name(ret));
        return ret;
    }

    ret = mpu6050_wake_up();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Device wakeup error: %s", esp_err_to_name(ret));
        return ret;
    }

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &sensread, .name = "mpu6050"};

    ret = esp_timer_create(&periodic_timer_args, &s_timer_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to setup timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_timer_start_periodic(s_timer_handler, MEASUREMENT_CYCLE_TIME);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Navigator failed to start: %s", esp_err_to_name(ret));
    }

    return ret;
}
