#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "stm32f4xx_hal.h"

#define PI 3.1415926f

typedef struct
{
    uint32_t last_update_ms;
    uint32_t age_ms;
    uint32_t frame_count;
    uint8_t valid;
    uint8_t fresh;
} SensorTimeInfo_t;

#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pressure.h"
#include "flash.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "string.h"
#include <stdlib.h>

#include "chassis.h"
#include "encoder.h"
#include "imu_get.h"
#include "imu_calc.h"
#include "motor.h"
#include "oid.h"
#include "pid.h"
#include "rk3588.h"
#include "servo.h"
#include "wireless.h"


#endif
