#include "encoder.h"
#include "motor.h"
#include "chassis.h"
#include "FreeRTOS.h"
#include "task.h"

#define ENCODER_PI_SCALED 3141593LL
#define ENCODER_PI_SCALE   1000000LL

EncoderData_t motor_encoder = {0};

static int32_t encoder_apply_chassis_direction(uint8_t motor_id, int32_t value)
{
    int8_t direction = 1;

    switch (motor_id)
    {
        case MOTOR_ID_REAR_LEFT:
            direction = ENCODER_DIRECTION_REAR_LEFT;
            break;
        case MOTOR_ID_FRONT_RIGHT:
            direction = ENCODER_DIRECTION_FRONT_RIGHT;
            break;
        case MOTOR_ID_FRONT_LEFT:
            direction = ENCODER_DIRECTION_FRONT_LEFT;
            break;
        case MOTOR_ID_REAR_RIGHT:
            direction = ENCODER_DIRECTION_REAR_RIGHT;
            break;
        default:
            break;
    }

    return value * direction;
}

int32_t encoder_erpm_to_mm_s(uint8_t motor_id, int32_t speed_erpm)
{
    int64_t numerator;
    int64_t denominator;
    int32_t speed_mm_s;

    if ((motor_id < OID_CHASSIS_NODE_ID_MIN) || (motor_id > OID_CHASSIS_NODE_ID_MAX))
    {
        return 0;
    }

    numerator = (int64_t)speed_erpm * ENCODER_PI_SCALED * ENCODER_CHASSIS_WHEEL_DIAMETER_MM * ENCODER_CHASSIS_GEAR_RATIO_DEN;
    denominator = (int64_t)ENCODER_CHASSIS_POLE_PAIRS * ENCODER_CHASSIS_GEAR_RATIO_NUM * 60LL * ENCODER_PI_SCALE;

    if (numerator >= 0)
    {
        speed_mm_s = (int32_t)((numerator + denominator / 2LL) / denominator);
    }
    else
    {
        speed_mm_s = (int32_t)(-(((-numerator) + denominator / 2LL) / denominator));
    }

    return encoder_apply_chassis_direction(motor_id, speed_mm_s);
}

int32_t encoder_mm_s_to_erpm(uint8_t motor_id, int32_t speed_mm_s)
{
    int64_t numerator;
    int64_t denominator;
    int32_t directed_speed;

    if ((motor_id < OID_CHASSIS_NODE_ID_MIN) || (motor_id > OID_CHASSIS_NODE_ID_MAX))
    {
        return 0;
    }

    directed_speed = -speed_mm_s;
    if ((motor_id == MOTOR_ID_REAR_LEFT) || (motor_id == MOTOR_ID_FRONT_LEFT))
    {
        directed_speed = speed_mm_s;
    }
    numerator = (int64_t)directed_speed * ENCODER_CHASSIS_POLE_PAIRS * ENCODER_CHASSIS_GEAR_RATIO_NUM * 60LL * ENCODER_PI_SCALE;
    denominator = ENCODER_PI_SCALED * (int64_t)ENCODER_CHASSIS_WHEEL_DIAMETER_MM * ENCODER_CHASSIS_GEAR_RATIO_DEN;

    if (numerator >= 0)
    {
        return (int32_t)((numerator + denominator / 2LL) / denominator);
    }

    return (int32_t)(-(((-numerator) + denominator / 2LL) / denominator));
}

static uint8_t encoder_get_speed_erpm(uint8_t motor_id, int32_t *speed_erpm)
{
    OidDataValue_t speed_data;

    if ((speed_erpm == NULL) || (oid_get_data_value(motor_id, OID_DATA_SPEED_ERPM, &speed_data) == 0U))
    {
        return 0U;
    }

    *speed_erpm = speed_data.value;
    return 1U;
}

uint8_t encoder_get_chassis_speed_mm_s(uint8_t motor_id, int32_t *speed_mm_s)
{
    int32_t speed_erpm;

    if ((speed_mm_s == NULL) || (motor_id < OID_CHASSIS_NODE_ID_MIN) || (motor_id > OID_CHASSIS_NODE_ID_MAX))
    {
        return 0U;
    }

    if (encoder_get_speed_erpm(motor_id, &speed_erpm) == 0U)
    {
        return 0U;
    }

    speed_erpm = encoder_erpm_to_mm_s(motor_id, speed_erpm);
    taskENTER_CRITICAL();
    *speed_mm_s = speed_erpm;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t encoder_get_shooter_speed_erpm(uint8_t motor_id, int32_t *speed_erpm)
{
    int32_t measured_speed_erpm;

    if ((speed_erpm == NULL) || (motor_id < OID_SHOOTER_NODE_ID_MIN) || (motor_id > OID_SHOOTER_NODE_ID_MAX))
    {
        return 0U;
    }

    if (encoder_get_speed_erpm(motor_id, &measured_speed_erpm) == 0U)
    {
        return 0U;
    }

    if (motor_id == OID_SHOOTER_NODE_ID_MAX)
    {
        measured_speed_erpm = -measured_speed_erpm;
    }

    taskENTER_CRITICAL();
    *speed_erpm = measured_speed_erpm;
    taskEXIT_CRITICAL();

    return 1U;
}

void encoder_process(void)
{
    (void)encoder_get_chassis_speed_mm_s(MOTOR_ID_FRONT_RIGHT, &motor_encoder.front_right);
    (void)encoder_get_chassis_speed_mm_s(MOTOR_ID_REAR_LEFT, &motor_encoder.rear_left);
    (void)encoder_get_chassis_speed_mm_s(MOTOR_ID_REAR_RIGHT, &motor_encoder.rear_right);
    (void)encoder_get_chassis_speed_mm_s(MOTOR_ID_FRONT_LEFT, &motor_encoder.front_left);
    (void)encoder_get_shooter_speed_erpm(MOTOR_ID_LAUNCHER_UP, &motor_encoder.launcher_left_erpm);
    (void)encoder_get_shooter_speed_erpm(MOTOR_ID_LAUNCHER_DOWN, &motor_encoder.launcher_right_erpm);
}

void encoder_init(void)
{
    motor_encoder.rear_left = 0;
    motor_encoder.front_right = 0;
    motor_encoder.front_left = 0;
    motor_encoder.rear_right = 0;
    motor_encoder.launcher_left_erpm = 0;
    motor_encoder.launcher_right_erpm = 0;
}
