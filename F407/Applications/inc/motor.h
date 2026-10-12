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

/**
 * @brief 换算、限幅并发送底盘电机速度命令。
 * @param motor_id 底盘电机节点 ID。
 * @param speed_mm_s 目标车轮线速度，单位 mm/s。
 * @return 节点无效时返回 HAL_ERROR，否则返回发送状态。
 */
HAL_StatusTypeDef motor_set_chassis_speed_mm_s(uint8_t motor_id, int32_t speed_mm_s);
/**
 * @brief 限幅并发送发球机电机速度命令。
 * @param motor_id 发球机电机节点 ID。
 * @param speed_erpm 目标电气转速，单位 eRPM；尚未进行节点方向修正。
 * @return 节点无效时返回 HAL_ERROR，否则返回发送状态。
 */
HAL_StatusTypeDef motor_set_launcher_speed_erpm(uint8_t motor_id, int32_t speed_erpm);
/**
 * @brief 向 OID 电机发送零电流命令。
 * @param motor_id OID 电机节点 ID。
 * @return 节点无效时返回 HAL_ERROR，否则返回发送状态。
 */
HAL_StatusTypeDef motor_set_zero_current(uint8_t motor_id);

#endif
