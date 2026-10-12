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

/**
 * @brief 注册 WIT 驱动回调，并探测、初始化 IMU。
 */
void imu_init(void);
/**
 * @brief 请求 IMU 寄存器数据，并更新测量值缓存和时效状态。
 * @return 处理到新的角度数据时返回 1，否则返回 0。
 */
uint8_t imu_data_get(void);
/**
 * @brief 将当前累计航向角设为相对航向角的零点。
 */
void yaw_init(void);
/**
 * @brief 设置并保存 IMU 的六轴或九轴算法。
 * @param algorithm 要使用的 IMU 算法。
 * @return 成功返回 WIT_HAL_OK；算法无效返回 WIT_HAL_INVAL；
 *         寄存器写入失败返回 WIT_HAL_ERROR。
 */
int32_t imu_set_algorithm(IMU_Algorithm algorithm);

#endif
