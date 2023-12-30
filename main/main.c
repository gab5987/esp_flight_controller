#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#include "hal.h"

void app_main(void)
{
    complimentary_angle_t cma = {0};
    imu_initialize();
    for (;;)
    {
        // imu_get_complimentary_angle(&cma);
        // printf("row: %.2f \t pitch: %.2f\n\n", cma.roll, cma.pitch);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

