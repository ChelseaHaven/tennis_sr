#ifndef _SERVO_H
#define _SERVO_H

#include "tim.h"
//SERVO_1发球电机  SERVO_2拨球电机
#define SERVO_1                         1U
#define SERVO_2                         2U

#define SERVO_1_ANGLE_MIN_DEG           0.0f
#define SERVO_1_ANGLE_MAX_DEG          90.0f
#define SERVO_1_DEFAULT_ANGLE_DEG      45.0f

#define SERVO_2_ANGLE_MIN_DEG           0.0f
#define SERVO_2_ANGLE_MAX_DEG         305.0f
#define SERVO_2_DEFAULT_ANGLE_DEG       0.0f

#define SERVO_1_PULSE_MIN_US           500U
#define SERVO_1_PULSE_MAX_US          2500U
#define SERVO_2_PULSE_MIN_US           500U
#define SERVO_2_PULSE_MAX_US          2500U

extern volatile float servo_1_target_angle;
extern volatile float servo_2_target_angle;

/**
 * @brief 使用默认角度初始化指定舵机的 PWM 输出。
 * @param servo_id 舵机编号，使用 SERVO_1 或 SERVO_2。
 * @return 成功返回 HAL_OK；舵机编号无效或 PWM 启动失败时返回 HAL_ERROR。
 */
HAL_StatusTypeDef servo_init(uint8_t servo_id);
/**
 * @brief 设置指定舵机的目标角度并更新 PWM 脉宽。
 * @param servo_id 舵机编号，使用 SERVO_1 或 SERVO_2。
 * @param angle_deg 目标角度，单位为度；超出范围时会被限幅。
 * @return 参数和舵机编号有效时返回 HAL_OK，否则返回 HAL_ERROR。
 */
HAL_StatusTypeDef servo_set_angle(uint8_t servo_id, float angle_deg);

/**
 * @brief 停止指定舵机的 PWM 输出。
 * @param servo_id 舵机编号，使用 SERVO_1 或 SERVO_2；无效编号会被忽略。
 */
void servo_stop(uint8_t servo_id);

#endif
