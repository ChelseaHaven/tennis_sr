#include "oid.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define OID_BACKGROUND_ITEM_COUNT        (OID_DATA_ITEM_COUNT - 1U)

typedef struct
{
    OidDataValue_t item[OID_DATA_ITEM_COUNT];
} OidTelemetry_t;

static OidTelemetry_t oid_telemetry[OID_NODE_COUNT];

static const uint8_t oid_reply_dlc[OID_DATA_ITEM_COUNT] =
{
    4U, 6U, 4U, 4U, 4U, 4U, 4U, 4U, 6U, 4U, 3U, 7U, 4U
};

static const uint8_t oid_value_type[OID_DATA_ITEM_COUNT] =
{
    OID_VALUE_U16, OID_VALUE_I32, OID_VALUE_I16,
    OID_VALUE_I16, OID_VALUE_U16, OID_VALUE_I16,
    OID_VALUE_I16, OID_VALUE_I16, OID_VALUE_I32,
    OID_VALUE_U16, OID_VALUE_U8,  OID_VALUE_RAW,
    OID_VALUE_U16
};

static const uint16_t oid_timeout_ms[OID_DATA_ITEM_COUNT] =
{
    OID_TIMEOUT_FAULT_MS,
    OID_TIMEOUT_SPEED_MS,
    OID_TIMEOUT_DUTY_MS,
    OID_TIMEOUT_POWER_MS,
    OID_TIMEOUT_VOLTAGE_MS,
    OID_TIMEOUT_MOTOR_CURRENT_MS,
    OID_TIMEOUT_BUS_CURRENT_MS,
    OID_TIMEOUT_TEMPERATURE_MS,
    OID_TIMEOUT_POSITION_MS,
    OID_TIMEOUT_SINGLE_ANGLE_MS,
    OID_TIMEOUT_ENCODER_Z_MS,
    OID_TIMEOUT_CACHED_ERRORS_MS,
    OID_TIMEOUT_IO_STATE_MS
};

static uint8_t oid_query_cursor = 0U;
static uint8_t oid_speed_query_node_id = OID_NODE_ID_MIN;
static uint8_t oid_heartbeat_node_id = OID_NODE_ID_MIN;

/**
 * @brief 检查节点 ID 是否属于当前 OID 节点范围。
 * @param node_id 待检查的节点 ID。
 * @return 节点 ID 有效返回 1，否则返回 0。
 */
static uint8_t oid_node_valid(uint8_t node_id)
{
    return (uint8_t)((node_id >= OID_NODE_ID_MIN) && (node_id <= OID_NODE_ID_MAX));
}

/**
 * @brief 将指定标准 CAN 帧加入 CAN 发送队列。
 * @param std_id 标准帧 ID。
 * @param data CAN 数据缓冲区。
 * @param length 数据长度。
 * @return CAN 发送队列返回的 HAL 状态。
 */
static HAL_StatusTypeDef oid_send_frame(uint16_t std_id, const uint8_t *data, uint8_t length)
{
    return can_fifo_tx_push(std_id, data, length);
}

/**
 * @brief 按大端序读取 16 位无符号整数。
 * @param data 指向两个数据字节的指针。
 * @return 解析后的数值。
 */
static uint16_t oid_read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

/**
 * @brief 按大端序读取 16 位有符号整数。
 * @param data 指向两个数据字节的指针。
 * @return 解析后的数值。
 */
static int16_t oid_read_i16_be(const uint8_t *data)
{
    return (int16_t)oid_read_u16_be(data);
}

/**
 * @brief 按大端序读取 32 位无符号整数。
 * @param data 指向四个数据字节的指针。
 * @return 解析后的数值。
 */
static uint32_t oid_read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) | ((uint32_t)data[1] << 16U) | ((uint32_t)data[2] << 8U) | (uint32_t)data[3];
}

/**
 * @brief 根据遥测数据类型解析 OID 反馈载荷。
 * @param type OID_VALUE_* 数据类型。
 * @param payload 指向数据载荷的指针。
 * @return 解析后的数值；原始字节类型返回 0。
 */
static int32_t oid_decode_value(uint8_t type, const uint8_t *payload)
{
    switch (type)
    {
        case OID_VALUE_U8:
            return (int32_t)payload[0];
        case OID_VALUE_I16:
            return (int32_t)oid_read_i16_be(payload);
        case OID_VALUE_U16:
            return (int32_t)oid_read_u16_be(payload);
        case OID_VALUE_I32:
            return (int32_t)oid_read_u32_be(payload);
        case OID_VALUE_U32:
            return (int32_t)oid_read_u32_be(payload);
        case OID_VALUE_RAW:
        default:
            return 0;
    }
}

HAL_StatusTypeDef oid_send_heartbeat(uint8_t node_id)
{
    static const uint8_t data[1] = {OID_CMD_HEARTBEAT};

    if (oid_node_valid(node_id) == 0U)
    {
        return HAL_ERROR;
    }

    return oid_send_frame(node_id, data, sizeof(data));
}

HAL_StatusTypeDef oid_send_current(uint8_t node_id, int16_t current)
{
    uint16_t value;
    uint8_t data[3];

    if (oid_node_valid(node_id) == 0U)
    {
        return HAL_ERROR;
    }

    value = (uint16_t)current;
    data[0] = OID_CMD_CURRENT;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)value;

    return oid_send_frame(node_id, data, sizeof(data));
}

HAL_StatusTypeDef oid_send_speed_erpm(uint8_t node_id, int32_t speed_erpm)
{
    uint32_t value;
    uint8_t data[5];

    if (oid_node_valid(node_id) == 0U)
    {
        return HAL_ERROR;
    }

    value = (uint32_t)speed_erpm;
    data[0] = OID_CMD_SPEED;
    data[1] = (uint8_t)(value >> 24U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 8U);
    data[4] = (uint8_t)value;

    return oid_send_frame(node_id, data, sizeof(data));
}

HAL_StatusTypeDef oid_send_query(uint8_t node_id, uint8_t item)
{
    uint8_t data[2];

    if ((oid_node_valid(node_id) == 0U) ||
        (item >= OID_DATA_ITEM_COUNT))
    {
        return HAL_ERROR;
    }

    data[0] = OID_CMD_QUERY;
    data[1] = item;

    return oid_send_frame(node_id, data, sizeof(data));
}

void oid_refresh_freshness(void)
{
    OidDataValue_t *value;
    uint8_t node_index;
    uint8_t item;
    uint32_t now_ms = HAL_GetTick();

    taskENTER_CRITICAL();
    for (node_index = 0U; node_index < OID_NODE_COUNT; node_index++)
    {
        for (item = 0U; item < OID_DATA_ITEM_COUNT; item++)
        {
            value = &oid_telemetry[node_index].item[item];
            if (value->time_info.valid != 0U)
            {
                value->time_info.age_ms = now_ms - value->time_info.last_update_ms;
                value->time_info.fresh = (uint8_t)(value->time_info.age_ms <= oid_timeout_ms[item]);
            }
            else
            {
                value->time_info.age_ms = 0U;
                value->time_info.fresh = 0U;
            }
        }
    }
    taskEXIT_CRITICAL();
}

void oid_process_queries(void)
{
    uint8_t node_index;
    uint8_t item_index;
    uint8_t item;

    node_index = (uint8_t)(oid_query_cursor / OID_BACKGROUND_ITEM_COUNT);
    item_index = (uint8_t)(oid_query_cursor % OID_BACKGROUND_ITEM_COUNT);
    item = item_index;
    if (item >= OID_DATA_SPEED_ERPM)
    {
        item++;
    }

    if (oid_send_query((uint8_t)(OID_NODE_ID_MIN + node_index), item) == HAL_OK)
    {
        oid_query_cursor++;
        if (oid_query_cursor >= (OID_NODE_COUNT * OID_BACKGROUND_ITEM_COUNT))
        {
            oid_query_cursor = 0U;
        }
    }
}

void oid_process_speed_queries(void)
{
    if (oid_send_query(oid_speed_query_node_id, OID_DATA_SPEED_ERPM) == HAL_OK)
    {
        oid_speed_query_node_id++;
        if (oid_speed_query_node_id > OID_NODE_ID_MAX)
        {
            oid_speed_query_node_id = OID_NODE_ID_MIN;
        }
    }
}

void oid_process_heartbeats(void)
{
    (void)oid_send_heartbeat(oid_heartbeat_node_id);

    oid_heartbeat_node_id++;
    if (oid_heartbeat_node_id > OID_NODE_ID_MAX)
    {
        oid_heartbeat_node_id = OID_NODE_ID_MIN;
    }
}

uint8_t oid_get_data_value(uint8_t node_id, uint8_t item, OidDataValue_t *out_value)
{
    uint8_t node_index;

    if ((out_value == NULL) || (oid_node_valid(node_id) == 0U) || (item >= OID_DATA_ITEM_COUNT))
    {
        return 0U;
    }
    node_index = (uint8_t)(node_id - OID_NODE_ID_MIN);
    taskENTER_CRITICAL();
    *out_value = oid_telemetry[node_index].item[item];
    taskEXIT_CRITICAL();
    return 1U;
}

void oid_handle_can_rx_frame(const CanFifoFrame_t *frame)
{
    OidTelemetry_t *telemetry;
    OidDataValue_t *item_data;
    uint32_t now_ms;
    uint8_t node_id;
    uint8_t node_index;
    uint8_t item;
    uint8_t payload_length;
    uint8_t index;

    if ((frame == NULL) || (frame->std_id < (OID_FEEDBACK_ID_OFFSET + OID_NODE_ID_MIN)) || (frame->std_id > (OID_FEEDBACK_ID_OFFSET + OID_NODE_ID_MAX)))
    {
        return;
    }

    node_id = (uint8_t)(frame->std_id - OID_FEEDBACK_ID_OFFSET);
    node_index = (uint8_t)(node_id - OID_NODE_ID_MIN);
    telemetry = &oid_telemetry[node_index];

    if ((frame->dlc < 2U) || (frame->data[0] != OID_CMD_QUERY))
    {
        return;
    }

    item = frame->data[1];
    if ((item >= OID_DATA_ITEM_COUNT) || (frame->dlc != oid_reply_dlc[item]))
    {
        return;
    }

    payload_length = (uint8_t)(frame->dlc - 2U);
    now_ms = HAL_GetTick();

    taskENTER_CRITICAL();
    item_data = &telemetry->item[item];
    for (index = 0U; index < 5U; index++)
    {
        item_data->raw[index] = (index < payload_length) ? frame->data[index + 2U] : 0U;
    }
    item_data->raw_length = payload_length;
    item_data->type = oid_value_type[item];
    item_data->value = oid_decode_value(item_data->type, item_data->raw);
    item_data->time_info.last_update_ms = now_ms;
    item_data->time_info.age_ms = 0U;
    item_data->time_info.frame_count++;
    item_data->time_info.valid = 1U;
    item_data->time_info.fresh = 1U;
    taskEXIT_CRITICAL();
}

void oid_init(void)
{
    uint8_t node_index;
    uint8_t item;

    oid_query_cursor = 0U;
    oid_speed_query_node_id = OID_NODE_ID_MIN;
    oid_heartbeat_node_id = OID_NODE_ID_MIN;
    for (node_index = 0U; node_index < OID_NODE_COUNT; node_index++)
    {
        for (item = 0U; item < OID_DATA_ITEM_COUNT; item++)
        {
            memset(&oid_telemetry[node_index].item[item], 0, sizeof(oid_telemetry[node_index].item[item]));
            oid_telemetry[node_index].item[item].type = oid_value_type[item];
        }
    }
}
