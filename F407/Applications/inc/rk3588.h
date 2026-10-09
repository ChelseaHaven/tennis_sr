#ifndef _RK3588_H_
#define _RK3588_H_

#include "can_fifo.h"
#include "system.h"

#define RK3588_NODE_ID                  16U
#define RK3588_CMD_HEARTBEAT            0x00U
#define RK3588_FEEDBACK_ID_OFFSET       0x100U
#define RK3588_FEEDBACK_ID              (RK3588_FEEDBACK_ID_OFFSET + RK3588_NODE_ID)
#define RK3588_CMD_CHASSIS_ENABLE       0x00U
#define RK3588_CMD_01                   0x01U
#define RK3588_CMD_02                   0x02U
#define RK3588_CMD_03                   0x03U
#define RK3588_DATA_ITEM_COUNT          4U

typedef enum
{
    RK3588_VALUE_U8 = 0U,
    RK3588_VALUE_I16,
    RK3588_VALUE_U16,
    RK3588_VALUE_I32,
    RK3588_VALUE_U32,
    RK3588_VALUE_RAW
} Rk3588ValueType_t;

typedef struct
{
    int32_t value;
    SensorTimeInfo_t time_info;
    uint8_t raw[7];
    uint8_t raw_length;
    uint8_t type;
} Rk3588DataValue_t;

HAL_StatusTypeDef rk3588_send_heartbeat(void);
void rk3588_handle_can_rx_frame(const CanFifoFrame_t *frame);
void rk3588_refresh_freshness(void);
uint8_t rk3588_get_data_value(uint8_t command, Rk3588DataValue_t *out_value);

#endif
