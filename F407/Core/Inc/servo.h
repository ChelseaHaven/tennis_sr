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

/* Each timer count is 1 us. Adjust these limits for the actual servos. */
#define SERVO_1_PULSE_MIN_US           500U
#define SERVO_1_PULSE_MAX_US          2500U
#define SERVO_2_PULSE_MIN_US           500U
#define SERVO_2_PULSE_MAX_US          2500U

extern volatile float servo_1_target_angle;
extern volatile float servo_2_target_angle;

HAL_StatusTypeDef servo_init(uint8_t servo_id);
HAL_StatusTypeDef servo_set_angle(uint8_t servo_id, float angle_deg);

void servo_stop(uint8_t servo_id);

#endif
