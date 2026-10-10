#include "motor.h"

static HAL_StatusTypeDef motor_send_speed_erpm(uint8_t motor_id, int32_t speed_erpm)
{
    return oid_send_speed_erpm(motor_id, speed_erpm);
}

HAL_StatusTypeDef motor_set_chassis_speed_mm_s(uint8_t motor_id, int32_t speed_mm_s)
{
    int32_t speed_erpm;

    if ((motor_id < OID_CHASSIS_NODE_ID_MIN) || (motor_id > OID_CHASSIS_NODE_ID_MAX))
    {
        return HAL_ERROR;
    }

    speed_erpm = encoder_mm_s_to_erpm(motor_id, speed_mm_s);
    if (speed_erpm > MOTOR_CHASSIS_MAX_ABS_ERPM)
    {
        speed_erpm = MOTOR_CHASSIS_MAX_ABS_ERPM;
    }
    else if (speed_erpm < -MOTOR_CHASSIS_MAX_ABS_ERPM)
    {
        speed_erpm = -MOTOR_CHASSIS_MAX_ABS_ERPM;
    }

    return motor_send_speed_erpm(motor_id, speed_erpm);
}

HAL_StatusTypeDef motor_set_launcher_speed_erpm(uint8_t motor_id, int32_t speed_erpm)
{
    if ((motor_id < OID_LAUNCHER_NODE_ID_MIN) || (motor_id > OID_LAUNCHER_NODE_ID_MAX))
    {
        return HAL_ERROR;
    }

    if (speed_erpm > MOTOR_LAUNCHER_MAX_ABS_ERPM)
    {
        speed_erpm = MOTOR_LAUNCHER_MAX_ABS_ERPM;
    }
    else if (speed_erpm < -MOTOR_LAUNCHER_MAX_ABS_ERPM)
    {
        speed_erpm = -MOTOR_LAUNCHER_MAX_ABS_ERPM;
    }

    if (motor_id == MOTOR_ID_LAUNCHER_DOWN)
    {
        speed_erpm = -speed_erpm;
    }

    return motor_send_speed_erpm(motor_id, speed_erpm);
}

HAL_StatusTypeDef motor_set_zero_current(uint8_t motor_id)
{
    if ((motor_id < OID_NODE_ID_MIN) || (motor_id > OID_NODE_ID_MAX))
    {
        return HAL_ERROR;
    }

    return oid_send_current(motor_id, 0);
}
