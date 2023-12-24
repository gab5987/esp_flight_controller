#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <math.h>

#include "cfg.h"
#include "hal.h"

typedef struct
{
    f32 roll;
    f32 pitch;
} complimentary_angle_t;

typedef enum
{
    ACCE_FS_2G  = 0, /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 1, /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 2, /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3, /*!< Accelerometer full scale range is +/- 16g */
} mpu6050_acce_fs_t;

typedef enum
{
    GYRO_FS_250DPS =
        0, /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    GYRO_FS_500DPS =
        1, /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_1000DPS =
        2, /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_2000DPS =
        3, /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} mpu6050_gyro_fs_t;

static const char TAG[] = "mpu6050";

static const f32 ALPHA      = 0.99F;        /*!< Weight of gyroscope */
static const f32 RAD_TO_DEG = 57.27272727F; /*!< Radians to degrees */

static const gpio_num_t SCL_PIN  = GPIO_NUM_22;
static const gpio_num_t SDA_PIN  = GPIO_NUM_21;
static const i2c_port_t I2C_PORT = I2C_NUM_0;
static const byte       I2C_ADDR = 0x68U << 1;
static const u32        I2C_FREQ = 100000;
static const gpio_num_t INT_PIN  = GPIO_NUM_12;

/* MPU6050 register */
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

static i64               s_timer   = 0;
static SemaphoreHandle_t s_semph   = NULL;
static u32               s_counter = 0;
TaskHandle_t             s_task_handler;

static f32 s_gyro_sensitivity = 0.0F;
static f32 s_acce_sensitivity = 0.0F;

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

esp_err_t mpu6050_config(
    const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    byte config_regs[2] = {gyro_fs << 3, acce_fs << 3};
    return writeregister(MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(f32 *const acce_sensitivity)
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

esp_err_t mpu6050_get_gyro_sensitivity(f32 *const gyro_sensitivity)
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

// esp_err_t mpu6050_get_gyro(mpu6050_t *const dev)
// {
//     f32       sensitivity = 0.0F;
//     esp_err_t ret         = mpu6050_get_gyro_sensitivity(&sensitivity);
//     if (ret != ESP_OK) return ret;
//
//     byte data_rd[6] = {0};
//     ret = readregister(MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));
//     if (ret != ESP_OK) return ret;
//
//     i16 raw_x = (i16)((data_rd[0] << 8) + (data_rd[1]));
//     i16 raw_y = (i16)((data_rd[2] << 8) + (data_rd[3]));
//     i16 raw_z = (i16)((data_rd[4] << 8) + (data_rd[5]));
//
//     dev->gy_x = (f32)raw_x / sensitivity;
//     dev->gy_y = (f32)raw_y / sensitivity;
//     dev->gy_z = (f32)raw_z / sensitivity;
//
//     return ret;
// }
//
// esp_err_t mpu6050_get_acce(mpu6050_t *const dev)
// {
//     f32       sensitivity = 0.0F;
//     esp_err_t ret         = mpu6050_get_acce_sensitivity(&sensitivity);
//     if (ret != ESP_OK) return ret;
//
//     byte data_rd[6] = {0};
//     ret = readregister(MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));
//     if (ret != ESP_OK) return ret;
//
//     i16 raw_x = (i16)((data_rd[0] << 8) + (data_rd[1]));
//     i16 raw_y = (i16)((data_rd[2] << 8) + (data_rd[3]));
//     i16 raw_z = (i16)((data_rd[4] << 8) + (data_rd[5]));
//
//     dev->ac_x = (f32)raw_x / sensitivity;
//     dev->ac_y = (f32)raw_y / sensitivity;
//     dev->ac_z = (f32)raw_z / sensitivity;
//
//     return ret;
// }
//
// esp_err_t mpu6050_get_temp(mpu6050_t *const dev)
// {
//     byte      data_rd[2] = {0};
//     esp_err_t ret = readregister(MPU6050_TEMP_XOUT_H, data_rd,
//     sizeof(data_rd)); if (ret != ESP_OK) return ret; dev->tmp =
//     (f32)(((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53); return ret;
// }

esp_err_t mpu6050_complimentory_filter(
    mpu6050_t *restrict dev,
    complimentary_angle_t *restrict const complimentary_angle)
{
    f32 acce_angle[2] = {0};
    f32 gyro_angle[2] = {0};
    f32 gyro_rate[2]  = {0};

    s_counter++;
    if (s_counter == 1)
    {
        acce_angle[0] = ((f32)atan2(dev->ac_y, dev->ac_z) * RAD_TO_DEG);
        acce_angle[1] = ((f32)atan2(dev->ac_x, dev->ac_z) * RAD_TO_DEG);
        complimentary_angle->roll  = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        s_timer                    = esp_timer_get_time() / 1000;
        return ESP_OK;
    }

    i64 now    = esp_timer_get_time() / 1000;
    dev->dt    = (f32)(now - dev->timer);
    dev->timer = now;

    acce_angle[0] = ((f32)atan2(dev->ac_y, dev->ac_z) * RAD_TO_DEG);
    acce_angle[1] = ((f32)atan2(dev->ac_x, dev->ac_z) * RAD_TO_DEG);

    gyro_rate[0]  = dev->gy_x;
    gyro_rate[1]  = dev->gy_y;
    gyro_angle[0] = gyro_rate[0] * dev->dt;
    gyro_angle[1] = gyro_rate[1] * dev->dt;

    complimentary_angle->roll =
        (ALPHA * (complimentary_angle->roll + gyro_angle[0])) +
        ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch =
        (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) +
        ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}

NORET static void sens_read_tsk(void *args)
{
    mpu6050_t *dev         = (mpu6050_t *)args;
    byte       data_rd[16] = {0};
    UNUSED u32 tskvalue; // Cant pass a nullptr

    for (;;)
    {
        xTaskNotifyWait(0, 0, &tskvalue, portMAX_DELAY);

        /* Somehow i could read all the registers on a single burst by
         * requesting past the 6 bytes from the accelerometer register */
        readregister(MPU6050_ACCEL_XOUT_H, data_rd, 14);

        dev->ac_x =
            (f32)((i16)((data_rd[0] << 8) + (data_rd[1]))) / s_acce_sensitivity;
        dev->ac_y =
            (f32)((i16)((data_rd[2] << 8) + (data_rd[3]))) / s_acce_sensitivity;
        dev->ac_z =
            (f32)((i16)((data_rd[4] << 8) + (data_rd[5]))) / s_acce_sensitivity;

        i16 raw_tmp = (data_rd[6] << 8) + data_rd[7];
        dev->tmp    = (f32)(raw_tmp / 340.00 + 36.53);

        dev->gy_x =
            (f32)((i16)((data_rd[8] << 8) + (data_rd[9]))) / s_gyro_sensitivity;
        dev->gy_y = (f32)((i16)((data_rd[10] << 8) + (data_rd[11]))) /
                    s_gyro_sensitivity;
        dev->gy_z = (f32)((i16)((data_rd[12] << 8) + (data_rd[13]))) /
                    s_gyro_sensitivity;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void IRAM_ATTR isrcb(void *args)
{
    xTaskNotifyFromISR((TaskHandle_t)args, 0, eNoAction, NULL);
}

esp_err_t mpu6050_initialize(mpu6050_t *dev)
{
    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = SDA_PIN;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = SCL_PIN;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    conf.clk_flags        = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

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

    ret = mpu6050_config(ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Device config error: %s", esp_err_to_name(ret));
        return ret;
    }

    byte int_pin_cfg = (BIT7 | BIT6 | BIT5 | BIT4);
    ret              = writeregister(MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);
    if (ESP_OK != ret)
    {
        ESP_LOGE(
            TAG, "Failed to write interrupt config: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_rom_gpio_pad_select_gpio(INT_PIN);
    gpio_set_direction(INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INT_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(INT_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    xTaskCreate(sens_read_tsk, "mpu6050", 4096, dev, 8, &s_task_handler);
    gpio_isr_handler_add(INT_PIN, isrcb, s_task_handler);

    ret = mpu6050_wake_up();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Device wakeup error: %s", esp_err_to_name(ret));
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

    byte enabled_interrupts = MPU6050_DATA_RDY_INT_BIT;
    ret = writeregister(MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    ret = mpu6050_get_acce_sensitivity(&s_acce_sensitivity);
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG, "Failed to get accelerometer sensitivity: %s",
            esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

