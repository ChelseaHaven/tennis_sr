#ifndef _RK3588_H_
#define _RK3588_H_

#include "can_fifo.h"
#include "system.h"

#define RK3588_NODE_ID                  16U
#define RK3588_CMD_HEARTBEAT            0x00U
#define RK3588_FEEDBACK_ID_OFFSET       0x100U
#define RK3588_FEEDBACK_ID              (RK3588_FEEDBACK_ID_OFFSET + RK3588_NODE_ID)
#define RK3588_CMD_CHASSIS_ENABLE       0x00U
#define RK3588_CMD_MANUAL_ENABLE        0x01U
/* Bit mask carried by RK3588_CMD_MANUAL_ENABLE as a 32-bit little-endian value. */
#define RK3588_MANUAL_ENABLE_CHASSIS_MASK  (1UL << 0U)
#define RK3588_MANUAL_ENABLE_LAUNCHER_MASK (1UL << 1U)
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
    RK3588_VALUE_U32_LE,
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

/**
 * @brief 将发往 RK3588 CAN 节点的单字节心跳帧加入发送队列。
 * @return CAN 发送队列返回的 HAL 状态。
 */
HAL_StatusTypeDef rk3588_send_heartbeat(void);
/**
 * @brief 校验并缓存从 RK3588 收到的命令帧。
 * @param frame 接收到的 CAN 帧；无关或格式错误的帧会被忽略。
 */
void rk3588_handle_can_rx_frame(const CanFifoFrame_t *frame);
/**
 * @brief 更新各缓存命令的数据年龄和新鲜度标志。
 */
void rk3588_refresh_freshness(void);
/**
 * @brief 复制指定命令的缓存数值和时间信息。
 * @param command RK3588 命令索引。
 * @param[out] out_value 用于接收缓存数据快照。
 * @return 参数有效时返回 1，否则返回 0；是否收到数据需检查 valid 字段。
 */
uint8_t rk3588_get_data_value(uint8_t command, Rk3588DataValue_t *out_value);
/**
 * @brief 检查最近收到的手动使能命令中的指定标志位。
 * @param enable_mask 要求全部置位的模块使能掩码。
 * @return 所有指定标志位均置位时返回 1，否则返回 0；使能状态保持不变。
 */
uint8_t rk3588_is_enabled(uint32_t enable_mask);

#endif
