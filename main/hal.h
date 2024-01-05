#ifndef _HAL_H_
#define _HAL_H_

#include <esp_err.h>

#include "cfg.h"

typedef struct
{
    f64 roll;
    f64 pitch;
} complimentary_angle_t;

esp_err_t imu_get_complimentary_angle(complimentary_angle_t *);
esp_err_t imu_initialize(void);

#endif

