#ifndef _IMU_CALC_H_
#define _IMU_CALC_H_

#include "stm32f4xx_hal.h"
#include "system.h"

#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "imu_get.h"

#define IMU_FRESH_TIMEOUT_MS  30U

typedef struct{
  int16_t acc_raw[3];
  int16_t gyro_raw[3];
  int16_t mag_raw[3];
  int16_t angle_raw[3];
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float pitch;
  float roll;
  float yaw;
  float yaw_total;
  SensorTimeInfo_t time_info;
}IMU_init;

extern IMU_init imu;

typedef enum {
  IMU_ALGORITHM_6_AXIS = 6,
  IMU_ALGORITHM_9_AXIS = 9
} IMU_Algorithm;

void imu_init(void);
uint8_t imu_data_get(void);
void yaw_init(void);
int32_t imu_set_algorithm(IMU_Algorithm algorithm);

#endif
