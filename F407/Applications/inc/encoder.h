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

void encoder_init(void);
void encoder_process(void);
uint8_t encoder_get_chassis_speed_mm_s(uint8_t motor_id, int32_t *speed_mm_s);
uint8_t encoder_get_shooter_speed_erpm(uint8_t motor_id,
                                       int32_t *speed_erpm);
int32_t encoder_erpm_to_mm_s(uint8_t motor_id, int32_t speed_erpm);
int32_t encoder_mm_s_to_erpm(uint8_t motor_id, int32_t speed_mm_s);

#endif
