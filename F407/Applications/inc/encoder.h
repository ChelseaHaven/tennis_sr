#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include "oid.h"

#define ENCODER_DIRECTION_REAR_LEFT       1
#define ENCODER_DIRECTION_FRONT_RIGHT    -1
#define ENCODER_DIRECTION_FRONT_LEFT      1
#define ENCODER_DIRECTION_REAR_RIGHT     -1

#define ENCODER_CHASSIS_POLE_PAIRS                 2U
#define ENCODER_CHASSIS_GEAR_RATIO_NUM             6U
#define ENCODER_CHASSIS_GEAR_RATIO_DEN             1U
#define ENCODER_CHASSIS_WHEEL_DIAMETER_MM        141U
#define ENCODER_LAUNCHER_POLE_PAIRS                7U

typedef struct
{
    int32_t rear_left;       /* mm/s */
    int32_t front_right;     /* mm/s */
    int32_t front_left;      /* mm/s */
    int32_t rear_right;      /* mm/s */
    int32_t launcher_left_erpm;  /* Raw electrical RPM from node 7. */
    int32_t launcher_right_erpm; /* Raw electrical RPM from node 8. */
} EncoderData_t;

extern EncoderData_t motor_encoder;

/**
 * @brief 清零电机转速缓存。
 */
void encoder_init(void);
/**
 * @brief 更新所有底盘和发球机电机的缓存转速。
 */
void encoder_process(void);
/**
 * @brief 读取底盘电机上报转速并换算为线速度。
 * @param motor_id 底盘电机节点 ID。
 * @param[out] speed_mm_s 用于接收修正方向后的线速度，单位 mm/s。
 * @return 成功时返回 1，否则返回 0。
 */
uint8_t encoder_get_chassis_speed_mm_s(uint8_t motor_id, int32_t *speed_mm_s);
/**
 * @brief 读取发球机电机上报的电气转速。
 * @param motor_id 发球机电机节点 ID。
 * @param[out] speed_erpm 用于接收转速；8 号节点会进行方向修正。
 * @return 成功时返回 1，否则返回 0。
 */
uint8_t encoder_get_launcher_speed_erpm(uint8_t motor_id, int32_t *speed_erpm);
/**
 * @brief 将底盘电机电气转速换算为车轮线速度。
 * @param motor_id 底盘电机节点 ID。
 * @param speed_erpm 测得的电气转速，单位 eRPM。
 * @return 修正方向后的线速度，单位 mm/s；节点无效时返回 0。
 */
int32_t encoder_erpm_to_mm_s(uint8_t motor_id, int32_t speed_erpm);
/**
 * @brief 将底盘车轮目标线速度换算为电机电气转速。
 * @param motor_id 底盘电机节点 ID。
 * @param speed_mm_s 目标线速度，单位 mm/s。
 * @return 修正方向后的电气转速，单位 eRPM；节点无效时返回 0。
 */
int32_t encoder_mm_s_to_erpm(uint8_t motor_id, int32_t speed_mm_s);

#endif
