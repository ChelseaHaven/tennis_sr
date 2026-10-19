#include "rk3588.h"
#include "can_fifo.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

typedef struct
{
    Rk3588DataValue_t item[RK3588_DATA_ITEM_COUNT];
} Rk3588Telemetry_t;

static Rk3588Telemetry_t rk3588_telemetry;

static const uint8_t rk3588_reply_dlc[RK3588_DATA_ITEM_COUNT] =
{
    2U, 5U, 7U, 2U
};

static const uint8_t rk3588_value_type[RK3588_DATA_ITEM_COUNT] =
{
    RK3588_VALUE_U8, RK3588_VALUE_U32_LE, RK3588_VALUE_I16X3, RK3588_VALUE_U8
};

static const uint16_t rk3588_timeout_ms[RK3588_DATA_ITEM_COUNT] =
{
    150U, 150U, 150U, 150U
};

/**
 * @brief 按大端序解析两个字节的无符号整数。
 * @param data 指向数据载荷首字节的指针。
 * @return 解析后的无符号整数。
 */
static uint16_t rk3588_read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

/**
 * @brief 按大端序解析两个字节的有符号整数。
 * @param data 指向数据载荷首字节的指针。
 * @return 解析后的有符号整数。
 */
static int16_t rk3588_read_i16_be(const uint8_t *data)
{
    return (int16_t)rk3588_read_u16_be(data);
}

/**
 * @brief 按大端序解析四个字节的无符号整数。
 * @param data 指向数据载荷首字节的指针。
 * @return 解析后的无符号整数。
 */
static uint32_t rk3588_read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           (uint32_t)data[3];
}

/**
 * @brief 按小端序解析四个字节的无符号整数。
 * @param data 指向数据载荷首字节的指针。
 * @return 解析后的无符号整数。
 */
static uint32_t rk3588_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

uint8_t rk3588_data_parse_int16x3(const Rk3588DataValue_t *item_data, int16_t out_values[3])
{
    if ((item_data == NULL) ||
        (out_values == NULL) ||
        (item_data->type != RK3588_VALUE_I16X3) ||
        (item_data->raw_length != 6U))
    {
        return 0U;
    }

    out_values[0] = rk3588_read_i16_be(&item_data->raw[0]);
    out_values[1] = rk3588_read_i16_be(&item_data->raw[2]);
    out_values[2] = rk3588_read_i16_be(&item_data->raw[4]);
    return 1U;
}

/**
 * @brief 根据数值类型解析一条 RK3588 命令的数据载荷。
 * @param type Rk3588ValueType_t 中定义的数值类型。
 * @param payload 指向数据载荷的指针。
 * @return 解析后的数值；不支持的类型返回 0。
 */
static int32_t rk3588_decode_value(uint8_t type, const uint8_t *payload)
{
    switch (type)
    {
        case RK3588_VALUE_U8:
            return (int32_t)payload[0];
        case RK3588_VALUE_I16:
            return (int32_t)rk3588_read_i16_be(payload);
        case RK3588_VALUE_U16:
            return (int32_t)rk3588_read_u16_be(payload);
        case RK3588_VALUE_I32:
        case RK3588_VALUE_U32:
            return (int32_t)rk3588_read_u32_be(payload);
        case RK3588_VALUE_U32_LE:
            return (int32_t)rk3588_read_u32_le(payload);
        case RK3588_VALUE_I16X3:
            return 0;
        case RK3588_VALUE_RAW:
        default:
            return 0;
    }
}

HAL_StatusTypeDef rk3588_send_heartbeat(void)
{
    static const uint8_t heartbeat_data[1] = {RK3588_CMD_HEARTBEAT};

    return can_fifo_tx_push(RK3588_NODE_ID, heartbeat_data, sizeof(heartbeat_data));
}

HAL_StatusTypeDef rk3588_data_send_int16x3(uint8_t command, int16_t value0, int16_t value1, int16_t value2)
{
    uint8_t payload[7];
    uint8_t index;
    const int16_t values[3] = {value0, value1, value2};

    payload[0] = command;
    for (index = 0U; index < 3U; index++)
    {
        const uint16_t bits = (uint16_t)values[index];
        payload[1U + (2U * index)] = (uint8_t)(bits >> 8U);
        payload[2U + (2U * index)] = (uint8_t)(bits & 0xFFU);
    }

    return can_fifo_tx_push(RK3588_NODE_ID, payload, sizeof(payload));
}

int16_t telemetry_float_to_int16(float value)
{
    if (value >= 32767.0f)
    {
        return 32767;
    }
    if (value <= -32768.0f)
    {
        return -32768;
    }
    return (int16_t)lroundf(value);
}

void rk3588_handle_can_rx_frame(const CanFifoFrame_t *frame)
{
    Rk3588DataValue_t *item_data;
    uint32_t now_ms;
    uint8_t command;
    uint8_t payload_length;
    uint8_t index;

    if ((frame == NULL) ||
        (frame->std_id != RK3588_FEEDBACK_ID) ||
        (frame->dlc == 0U) ||
        (frame->dlc > 8U))
    {
        return;
    }

    command = frame->data[0];
    if ((command >= RK3588_DATA_ITEM_COUNT) ||
        (frame->dlc != rk3588_reply_dlc[command]))
    {
        return;
    }

    payload_length = (uint8_t)(frame->dlc - 1U);
    now_ms = HAL_GetTick();

    taskENTER_CRITICAL();
    item_data = &rk3588_telemetry.item[command];
    for (index = 0U; index < 7U; index++)
    {
        item_data->raw[index] = (index < payload_length) ? frame->data[index + 1U] : 0U;
    }
    item_data->raw_length = payload_length;
    item_data->type = rk3588_value_type[command];
    if (item_data->type >= RK3588_VALUE_I16X3)
    {
        /* The three values stay in raw[] until a caller explicitly parses them. */
        item_data->value = 0;
    }
    else
    {
        item_data->value = rk3588_decode_value(item_data->type, item_data->raw);
    }
    item_data->time_info.last_update_ms = now_ms;
    item_data->time_info.age_ms = 0U;
    item_data->time_info.frame_count++;
    item_data->time_info.valid = 1U;
    item_data->time_info.fresh = 1U;
    taskEXIT_CRITICAL();
}

void rk3588_refresh_freshness(void)
{
    Rk3588DataValue_t *value;
    uint32_t now_ms;
    uint8_t command;

    now_ms = HAL_GetTick();
    taskENTER_CRITICAL();
    for (command = 0U; command < RK3588_DATA_ITEM_COUNT; command++)
    {
        value = &rk3588_telemetry.item[command];
        if (value->time_info.valid != 0U)
        {
            value->time_info.age_ms = now_ms - value->time_info.last_update_ms;
            value->time_info.fresh =
                (uint8_t)(value->time_info.age_ms <= rk3588_timeout_ms[command]);
        }
        else
        {
            value->time_info.age_ms = 0U;
            value->time_info.fresh = 0U;
        }
    }
    taskEXIT_CRITICAL();
}

uint8_t rk3588_data_get_value(uint8_t command, Rk3588DataValue_t *out_value)
{
    if ((out_value == NULL) || (command >= RK3588_DATA_ITEM_COUNT))
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out_value = rk3588_telemetry.item[command];
    taskEXIT_CRITICAL();

    return 1U;
}

uint8_t rk3588_data_clear(uint8_t command)
{
    Rk3588DataValue_t *item_data;
    uint8_t index;

    if (command >= RK3588_DATA_ITEM_COUNT)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    item_data = &rk3588_telemetry.item[command];
    item_data->value = 0;
    for (index = 0U; index < sizeof(item_data->raw); index++)
    {
        item_data->raw[index] = 0U;
    }
    item_data->raw_length = 0U;
    item_data->time_info.age_ms = 0U;
    item_data->time_info.valid = 0U;
    item_data->time_info.fresh = 0U;
    taskEXIT_CRITICAL();

    return 1U;
}

uint8_t rk3588_data_is_enabled(uint32_t enable_mask)
{
    Rk3588DataValue_t manual_enable;

    if ((enable_mask == 0U) ||
        (rk3588_data_get_value(RK3588_CMD_MANUAL_ENABLE, &manual_enable) == 0U) ||
        (manual_enable.time_info.valid == 0U))
    {
        return 0U;
    }

    return (uint8_t)(((uint32_t)manual_enable.value & enable_mask) == enable_mask);
}
