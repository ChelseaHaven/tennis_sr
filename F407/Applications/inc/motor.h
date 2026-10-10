#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "system.h"
#include "can_fifo.h"
#include "oid.h"
#include "encoder.h"

#define MOTOR_ID_FRONT_RIGHT            OID_CHASSIS_NODE_ID_MIN
#define MOTOR_ID_REAR_LEFT              (OID_CHASSIS_NODE_ID_MIN + 1U)
#define MOTOR_ID_REAR_RIGHT             (OID_CHASSIS_NODE_ID_MIN + 2U)
#define MOTOR_ID_FRONT_LEFT             OID_CHASSIS_NODE_ID_MAX
#define MOTOR_ID_LAUNCHER_UP            OID_LAUNCHER_NODE_ID_MIN
#define MOTOR_ID_LAUNCHER_DOWN          OID_LAUNCHER_NODE_ID_MAX

#define MOTOR_CHASSIS_MAX_ABS_ERPM       3600L
#define MOTOR_LAUNCHER_MAX_ABS_ERPM      70000L
#define MOTOR_PWM_ZERO_DEADBAND          10.0f

HAL_StatusTypeDef motor_set_chassis_speed_mm_s(uint8_t motor_id, int32_t speed_mm_s);
HAL_StatusTypeDef motor_set_launcher_speed_erpm(uint8_t motor_id, int32_t speed_erpm);
HAL_StatusTypeDef motor_set_zero_current(uint8_t motor_id);

#endif
