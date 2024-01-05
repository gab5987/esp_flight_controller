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
    f64 acce_x, acce_y, acce_z;
    f64 gyro_x, gyro_y, gyro_z;
};

static const char TAG[] = "imu";

// static const f64 ALPHA      = 0.96F; /*!< Weight of gyroscope */
static const f64 RAD_TO_DEG = 1 / (M_PI / 180); /*!< Radians to degrees */

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

const u8 MPU6050_DATA_RDY_INT_BIT      = (u8)BIT0;
const u8 MPU6050_I2C_MASTER_INT_BIT    = (u8)BIT3;
const u8 MPU6050_FIFO_OVERFLOW_INT_BIT = (u8)BIT4;
const u8 MPU6050_MOT_DETECT_INT_BIT    = (u8)BIT6;
const u8 MPU6050_ALL_INTERRUPTS =
    (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT |
     MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

static f64                s_gyro_sensitivity = 65.5F;
static f64                s_acce_sensitivity = 4096.0F;
static f64                s_roll             = 0.0F;
static f64                s_pitch            = 0.0F;
static esp_timer_handle_t s_timer_handler;
static struct
{
    f64 x, y, z;
} s_gyro_cal = {
    .x = 0.0F,  // -1.208603053435111318592021F,
    .y = 0.0F,  // 2.687190839694640231982703F,
    .z = 0.0F}; // 0.200114503816792688173365F};

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

static esp_err_t mpu6050_get_acce_sensitivity(f64 *const acce_sensitivity)
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

static esp_err_t mpu6050_get_gyro_sensitivity(f64 *const gyro_sensitivity)
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
    i16 acce_x     = (i16)((data_rd[0] << 8) | data_rd[1]);
    axdata->acce_x = ((f64)acce_x / s_acce_sensitivity);

    i16 acce_y     = (i16)((data_rd[2] << 8) | data_rd[3]);
    axdata->acce_y = ((f64)acce_y / s_acce_sensitivity);

    i16 acce_z     = (i16)((data_rd[4] << 8) | data_rd[5]);
    axdata->acce_z = ((f64)acce_z / s_acce_sensitivity);

    // i16 temp = (i16)((data_rd[6] << 8) | data_rd[7]);
    // f64 tmp  = temp / 340.0 + 36.53;
    // printf("temperature: %.2f\n", tmp);

    i16 gyro_x     = (i16)((data_rd[8] << 8) | data_rd[9]);
    axdata->gyro_x = ((f64)gyro_x / s_gyro_sensitivity) - s_gyro_cal.x;

    i16 gyro_y     = (i16)((data_rd[10] << 8) | data_rd[11]);
    axdata->gyro_y = ((f64)gyro_y / s_gyro_sensitivity) - s_gyro_cal.y;

    i16 gyro_z     = (i16)((data_rd[12] << 8) | data_rd[13]);
    axdata->gyro_z = ((f64)gyro_z / s_gyro_sensitivity) - s_gyro_cal.z;

    return ret;
}

/*
        Y Axis
         ↑ ↻
         ↑ ↻
         ↑ ↻
  * * * * * * * * *
  *               *
  *   MPU 6050    * X Axis
  *               * → → →
  *               * ↩ ↩ ↩
  *               *
  * ()            *
  * * * * * * * * *
    Z Axis ⇪ ↺
 * */

static void sensread(UNUSED void *arg)
{
    struct mpu6050_axis_data axd;

    static f64 roll_uncertainty  = 4.0F;
    static f64 pitch_uncertainty = 4.0F;

    esp_err_t err = mpu6050_request_data(&axd);
    if (err != ESP_OK) return;

// #define CALIBRATION
#ifdef CALIBRATION
    static const f32                max_iterations = 2000.0F;
    static u16                      iterations     = 0;
    static struct mpu6050_axis_data cal_data       = {
              .acce_z = 0,
              .acce_y = 0,
              .acce_x = 0,
              .gyro_y = 0,
              .gyro_z = 0,
              .gyro_x = 0};
    if (iterations > 2000) return;
    iterations++;
    if (iterations == 2000)
    {
        printf(
            "accelerometer -> x: %.24f\t| y: %.24f\t| z: %.24f\n",
            cal_data.acce_x / max_iterations, cal_data.acce_y / max_iterations,
            cal_data.acce_z / max_iterations);
        printf(
            "gyroscope -> x: %.24f\t| y: %.24f\t| z: %.24f\n\n",
            cal_data.gyro_x / max_iterations, cal_data.gyro_y / max_iterations,
            cal_data.gyro_z / max_iterations);
        return;
    }
    cal_data.acce_x += axd.acce_x;
    cal_data.acce_y += axd.acce_y;
    cal_data.acce_z += axd.acce_z;
    cal_data.gyro_x += axd.gyro_x;
    cal_data.gyro_y += axd.gyro_y;
    cal_data.gyro_z += axd.gyro_z;
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

    f64 powacz_2 = axd.acce_z * axd.acce_z;

    f64 acce_angle_x =
        atan(axd.acce_y / sqrt((axd.acce_x * axd.acce_x) + powacz_2)) *
        RAD_TO_DEG;

    f64 acce_angle_y =
        -1 * atan(axd.acce_x / sqrt(axd.acce_y * axd.acce_y + powacz_2)) *
        RAD_TO_DEG;

    // printf(
    //     "\n axis -> x: %.2f | y: %.2f | z: %.2f \t\t %.2f %.2f\n",
    //     axd.acce_x, axd.acce_y, axd.acce_z, acce_angle_x, acce_angle_y);

    if (axd.gyro_x != INFINITY && axd.acce_y != INFINITY)
    {
        s_pitch = s_pitch + 0.004F * axd.acce_y;
        pitch_uncertainty =
            pitch_uncertainty + (const f64)(0.004 * 0.004 * 4 * 4);
        f64 pitch_gain    = pitch_uncertainty / (pitch_uncertainty + (3 * 3));
        s_pitch           = s_pitch + pitch_gain * (acce_angle_y - s_pitch);
        pitch_uncertainty = (1 - pitch_gain) * pitch_uncertainty;
    }

    if (axd.gyro_y != INFINITY && axd.acce_x != INFINITY)
    {
        s_roll = s_roll + 0.004F * axd.acce_x;
        roll_uncertainty =
            roll_uncertainty + (const f64)(0.004 * 0.004 * 4 * 4);
        f64 roll_gain    = roll_uncertainty / (roll_uncertainty + (3 * 3));
        s_roll           = s_roll + roll_gain * (acce_angle_x - s_roll);
        roll_uncertainty = (1 - roll_gain) * roll_uncertainty;
    }

    printf("r: %.2f \t p: %.2f\n", s_roll, s_pitch);
}

esp_err_t imu_get_complimentary_angle(complimentary_angle_t *const cang)
{
    if (cang == NULL)
    {
        ESP_LOGE(TAG, "complimentary_angle_t argument is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    cang->roll  = s_roll;
    cang->pitch = s_pitch;

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

    /* Each function that does not return ESP_OK must abort since those
     * assertions are not fail recoverable and thus the devce cannot start */

    esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c param config error: %s", esp_err_to_name(ret));
        abort();
    }

    ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c install error: %s", esp_err_to_name(ret));
        abort();
    }

    ESP_ERROR_CHECK(mpu6050_wake_up());

    byte lowpass_cfg = DLPF_10;
    ESP_ERROR_CHECK(
        writeregister(MPU6050_DLPF_CONFIG, &lowpass_cfg, sizeof(lowpass_cfg)));

    byte config_regs[2] = {GYRO_FS_500DPS << 3, ACCE_FS_8G << 3};
    ESP_ERROR_CHECK(
        writeregister(MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs)));

    ESP_ERROR_CHECK(mpu6050_get_gyro_sensitivity(&s_gyro_sensitivity));
    ESP_ERROR_CHECK(mpu6050_get_acce_sensitivity(&s_acce_sensitivity));

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &sensread, .name = "mpu6050"};

    ret = esp_timer_create(&periodic_timer_args, &s_timer_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to setup timer: %s", esp_err_to_name(ret));
        abort();
    }

    ret = esp_timer_start_periodic(s_timer_handler, MEASUREMENT_CYCLE_TIME);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Navigator failed to start: %s", esp_err_to_name(ret));
        abort();
    }

    return ret;
}
