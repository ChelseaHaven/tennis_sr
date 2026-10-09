#ifndef __OID_H__
#define __OID_H__

#include "stm32f4xx_hal.h"
#include "can_fifo.h"
#include "system.h"

#define OID_NODE_ID_MIN                 3U
#define OID_NODE_ID_MAX                 8U
#define OID_NODE_COUNT                  ((OID_NODE_ID_MAX - OID_NODE_ID_MIN) + 1U)
#define OID_CHASSIS_NODE_ID_MIN         OID_NODE_ID_MIN
#define OID_CHASSIS_NODE_ID_MAX         (OID_CHASSIS_NODE_ID_MIN + 3U)
#define OID_SHOOTER_NODE_ID_MIN         (OID_CHASSIS_NODE_ID_MAX + 1U)
#define OID_SHOOTER_NODE_ID_MAX         OID_NODE_ID_MAX
#define OID_FEEDBACK_ID_OFFSET          0x100U

#define OID_CMD_HEARTBEAT               0x00U
#define OID_CMD_CURRENT                 0x01U
#define OID_CMD_SPEED                   0x02U
#define OID_CMD_QUERY                   0x0FU

#define OID_DATA_FAULT                  0x00U
#define OID_DATA_SPEED_ERPM             0x01U
#define OID_DATA_DUTY                   0x02U
#define OID_DATA_POWER                  0x03U
#define OID_DATA_VOLTAGE                0x04U
#define OID_DATA_MOTOR_CURRENT          0x05U
#define OID_DATA_BUS_CURRENT            0x06U
#define OID_DATA_TEMPERATURE            0x07U
#define OID_DATA_POSITION               0x08U
#define OID_DATA_SINGLE_TURN_ANGLE      0x09U
#define OID_DATA_ENCODER_Z              0x0AU
#define OID_DATA_CACHED_ERRORS          0x0BU
#define OID_DATA_IO_STATE               0x0CU
#define OID_DATA_ITEM_COUNT             13U

#define OID_TIMEOUT_FAULT_MS            150U
#define OID_TIMEOUT_SPEED_MS            100U
#define OID_TIMEOUT_DUTY_MS             150U
#define OID_TIMEOUT_POWER_MS            300U
#define OID_TIMEOUT_VOLTAGE_MS          300U
#define OID_TIMEOUT_MOTOR_CURRENT_MS    100U
#define OID_TIMEOUT_BUS_CURRENT_MS      150U
#define OID_TIMEOUT_TEMPERATURE_MS     1500U
#define OID_TIMEOUT_POSITION_MS         150U
#define OID_TIMEOUT_SINGLE_ANGLE_MS     300U
#define OID_TIMEOUT_ENCODER_Z_MS        300U
#define OID_TIMEOUT_CACHED_ERRORS_MS    150U
#define OID_TIMEOUT_IO_STATE_MS         300U

typedef enum
{
    OID_VALUE_U8 = 0U,
    OID_VALUE_I16,
    OID_VALUE_U16,
    OID_VALUE_I32,
    OID_VALUE_U32,
    OID_VALUE_RAW
} OidValueType_t;

typedef struct
{
    int32_t value;
    SensorTimeInfo_t time_info;
    uint8_t raw[5];
    uint8_t raw_length;
    uint8_t type;
} OidDataValue_t;

void oid_init(void);
void oid_handle_can_rx_frame(const CanFifoFrame_t *frame);

void oid_refresh_freshness(void);
void oid_process_speed_queries(void);
void oid_process_heartbeats(void);
void oid_process_queries(void);

uint8_t oid_get_data_value(uint8_t node_id, uint8_t item, OidDataValue_t *out_value);

HAL_StatusTypeDef oid_send_heartbeat(uint8_t node_id);
HAL_StatusTypeDef oid_send_current(uint8_t node_id, int16_t current);
HAL_StatusTypeDef oid_send_speed_erpm(uint8_t node_id, int32_t speed_erpm);
HAL_StatusTypeDef oid_send_query(uint8_t node_id, uint8_t item);

#endif
