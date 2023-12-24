#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#include "hal.h"

void app_main(void)
{
    mpu6050_t device = {0};

    mpu6050_initialize(&device);

    for (;;)
    {
        // mpu6050_update_data(&device);

        printf(
            "acce X: %.4f \t gyro X: %.4f \t temp: %.2f\n", device.ac_x,
            device.gy_x, device.tmp);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

