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

static const char TAG[] = "IMU";

static const f32 ALPHA      = 0.96F; /*!< Weight of gyroscope */
static const f32 RAD_TO_DEG = (f32)(M_PI / (f32)180); /*!< Radians to degrees */

static const gpio_num_t SCL_PIN  = GPIO_NUM_22;
static const gpio_num_t SDA_PIN  = GPIO_NUM_21;
static const i2c_port_t I2C_PORT = I2C_NUM_0;
static const byte       I2C_ADDR = 0x68U << 1;
static const u32        I2C_FREQ = 400000;
static const gpio_num_t INT_PIN  = GPIO_NUM_12;

static const u8         CALIBRATION_SAMPLES     = 200;
static const TickType_t CALIBRATION_SAMPLE_TIME = pdMS_TO_TICKS(10);

/* MPU6050 register */
static const u16 MPU6050_DLPF_CONFIG  = 0x1AU;
static const u16 MPU6050_GYRO_CONFIG  = 0x1BU;
static const u16 MPU6050_ACCEL_CONFIG = 0x1CU;
static const u16 MPU6050_INTR_PIN_CFG = 0x37U;
static const u16 MPU6050_INTR_ENABLE  = 0x38U;
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

TaskHandle_t s_task_handler;
static f32   s_gyro_sensitivity = 0.0F;
static f32   s_acce_sensitivity = 0.0F;

static struct mpu6050
{
    f32 cal_ac_x, cal_ac_y, cal_ac_z;
    f32 cal_gy_x, cal_gy_y;

    f32 pitch, roll;
    f32 pitch_uncertainty, roll_uncertainty;
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

esp_err_t mpu6050_calibrate(void)
{
    struct sens_axis
    {
        f32 x, y, z;
    };

    struct sens_axis acce_cal    = {.x = 0, .y = 0, .z = 0};
    struct sens_axis gyro_cal    = {.x = 0, .y = 0, .z = 0};
    byte             data_rd[16] = {0};

    for (u8 i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        /*
         * Calibration is done simply by taking the mean of the first 200
         * samples from the sensor
         * */

        readregister(MPU6050_ACCEL_XOUT_H, data_rd, 14);

        f32 acce_raw_x =
            (f32)((i16)((data_rd[0] << 8) | data_rd[1])) / s_acce_sensitivity;
        f32 acce_raw_y =
            (f32)((i16)((data_rd[2] << 8) | data_rd[3])) / s_acce_sensitivity;
        f32 acce_raw_z =
            (f32)((i16)((data_rd[4] << 8) | data_rd[5])) / s_acce_sensitivity;

        f32 gyro_raw_x =
            (f32)((i16)((data_rd[8] << 8) | data_rd[9])) / s_gyro_sensitivity;
        f32 gyro_raw_y =
            (f32)((i16)((data_rd[10] << 8) | data_rd[11])) / s_gyro_sensitivity;

        acce_cal.x += acce_raw_x == INFINITY ? 0 : acce_raw_x;
        acce_cal.y += acce_raw_y == INFINITY ? 0 : acce_raw_y;
        acce_cal.z += acce_raw_z == INFINITY ? 0 : acce_raw_z;
        gyro_cal.x += gyro_raw_x == INFINITY ? 0 : gyro_raw_x;
        gyro_cal.y += gyro_raw_y == INFINITY ? 0 : gyro_raw_y;
        vTaskDelay(CALIBRATION_SAMPLE_TIME);
    }

    s_device.cal_ac_x = acce_cal.x /= (f32)CALIBRATION_SAMPLES;
    s_device.cal_ac_y = acce_cal.y /= (f32)CALIBRATION_SAMPLES;
    s_device.cal_ac_z = acce_cal.z /= (f32)CALIBRATION_SAMPLES;
    s_device.cal_gy_x = gyro_cal.x /= (f32)CALIBRATION_SAMPLES;
    s_device.cal_gy_y = gyro_cal.y /= (f32)CALIBRATION_SAMPLES;

    ESP_LOGI(
        TAG,
        "---> Calibration finished <---\n Accelerometer -> |X| %.4f \t |Y| "
        "%.4f \t |Z| %.4f\n Gyroscope -> |X| %.4f \t |Y| %.4f",
        acce_cal.x, acce_cal.y, acce_cal.x, gyro_cal.x, gyro_cal.y);

    return ESP_OK;
}

NORET static void sens_read_tsk(void *args)
{
    static byte       data_rd[16] = {0};
    i64               ptime       = esp_timer_get_time();
    static UNUSED u32 tskvalue; // Cant pass a nullptr

    for (;;)
    {
        xTaskNotifyWait(0, 0, &tskvalue, portMAX_DELAY);

        /* Somehow i could read all the registers on a single burst by
         * requesting past the 6 bytes from the accelerometer register */
        readregister(MPU6050_ACCEL_XOUT_H, data_rd, 14);

        f32 acce_raw_x =
            (f32)((i16)((data_rd[0] << 8) | data_rd[1])) / s_acce_sensitivity;
        f32 acce_raw_y =
            (f32)((i16)((data_rd[2] << 8) | data_rd[3])) / s_acce_sensitivity;
        f32 acce_raw_z =
            (f32)((i16)((data_rd[4] << 8) | data_rd[5])) / s_acce_sensitivity;

        f32 gyro_raw_x =
            (f32)((i16)((data_rd[8] << 8) | data_rd[9])) / s_gyro_sensitivity;
        f32 gyro_raw_y =
            (f32)((i16)((data_rd[10] << 8) | data_rd[11])) / s_gyro_sensitivity;
        // f32 gyro_raw_z =
        //     (f32)((i16)((data_rd[12] << 8) + data_rd[13])) /
        //     s_gyro_sensitivity;

        acce_raw_x -= s_device.cal_ac_x;
        acce_raw_y -= s_device.cal_ac_y;
        acce_raw_z -= s_device.cal_ac_z;
        gyro_raw_x -= s_device.cal_gy_x;
        gyro_raw_y -= s_device.cal_gy_y;

        f32 powacz_2 = (f32)pow(acce_raw_z, 2);

        i64 now = esp_timer_get_time();
        f32 elapsed_time =
            ((f32)(now - ptime) /
             (const f32)(1000 * 1000 /* Convert to seconds so we get rad/s
             */));

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

        f32 acce_angle_x =
            (f32)(atan(acce_raw_y / sqrt(pow(acce_raw_x, 2) + powacz_2)));
        acce_angle_x /= RAD_TO_DEG;

        f32 acce_angle_y =
            (f32)(atan(-1 * acce_raw_x / sqrt(pow(acce_raw_y, 2) + powacz_2)));
        acce_angle_y /= RAD_TO_DEG;

        /*
         * For both the X and Y axis, the angle will be the integral
         * accumulation of the past angle, plus the product of the current
         * angular rotation and the time between the last measurement.
         * 𝜃b = 𝜃a + Gz * Δt
         * */

        static f32 gyro_angle_x = 0.0F;
        static f32 gyro_angle_y = 0.0F;

        f32 ngang_x =
            gyro_angle_x + ((gyro_raw_x - s_device.cal_gy_x) * elapsed_time);
        f32 ngang_y =
            gyro_angle_y + ((gyro_raw_y - s_device.cal_ac_y) * elapsed_time);

        if (ngang_x != INFINITY && ngang_y != INFINITY)
        {
            gyro_angle_x = ngang_x;
            gyro_angle_y = ngang_y;
            ptime        = now;
        }

        s_device.roll =
            (ALPHA * gyro_angle_x) + ((const f32)(1 - ALPHA) * acce_angle_x);
        s_device.pitch =
            (ALPHA * gyro_angle_y) + ((const f32)(1 - ALPHA) * acce_angle_y);

        // printf(
        //     "%.2f \t %.2f \t %.2f \t %.2f\t %.2f\n", acce_angle_x,
        //     acce_angle_x, gyro_angle_x, gyro_angle_y, elapsed_time);

        // printf("r: %.2f \t p: %.2f\n", s_device.roll, s_device.pitch);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void IRAM_ATTR isrcb(void *args)
{
    xTaskNotifyFromISR((TaskHandle_t)args, 0, eNoAction, NULL);
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

    s_device.cal_ac_x = s_device.cal_ac_y = 0;
    s_device.cal_gy_x = s_device.cal_gy_y = 0;
    s_device.roll = s_device.pitch = 0;
    s_device.roll_uncertainty = s_device.pitch_uncertainty = 0;

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

    mpu6050_calibrate();

    esp_rom_gpio_pad_select_gpio(INT_PIN);
    gpio_set_direction(INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INT_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(INT_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    xTaskCreate(sens_read_tsk, "mpu6050", 4096, NULL, 8, &s_task_handler);
    gpio_isr_handler_add(INT_PIN, isrcb, s_task_handler);

    byte enabled_interrupts = MPU6050_DATA_RDY_INT_BIT;

    byte int_pin_cfg = (BIT7 | BIT6 | BIT5 | BIT4);
    ret              = writeregister(MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);
    if (ESP_OK != ret)
    {
        ESP_LOGE(
            TAG, "Failed to write interrupt config: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = writeregister(MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    ret = mpu6050_wake_up();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Device wakeup error: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(
        pdMS_TO_TICKS(250) /* Startup time specified in the datasheet */);

    return ret;
}

