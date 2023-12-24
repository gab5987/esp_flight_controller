#ifndef _HAL_H_
#define _HAL_H_

#include <esp_err.h>

#include "cfg.h"

typedef struct
{
    f32 ac_x, ac_y, ac_z;
    f32 gy_x, gy_y, gy_z;
    f32 tmp;
    f32 dt;
    i64 timer;
} mpu6050_t;

esp_err_t mpu6050_initialize(mpu6050_t *);

#endif

