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
#define OID_LAUNCHER_NODE_ID_MIN        (OID_CHASSIS_NODE_ID_MAX + 1U)
#define OID_LAUNCHER_NODE_ID_MAX        OID_NODE_ID_MAX
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

/**
 * @brief 初始化 OID 遥测缓存和查询、心跳轮询状态。
 */
void oid_init(void);
/**
 * @brief 校验并解析一帧 OID 电机反馈，更新对应遥测缓存。
 * @param frame 待处理的 CAN 接收帧；空指针、非 OID 帧或格式错误帧会被忽略。
 */
void oid_handle_can_rx_frame(const CanFifoFrame_t *frame);

/**
 * @brief 根据各遥测项的超时阈值更新数据年龄和新鲜度。
 */
void oid_refresh_freshness(void);
/**
 * @brief 轮询一个 OID 节点的电气转速数据项。
 */
void oid_process_speed_queries(void);
/**
 * @brief 轮询发送一个 OID 节点的心跳帧。
 */
void oid_process_heartbeats(void);
/**
 * @brief 轮询一个非转速 OID 遥测数据项。
 */
void oid_process_queries(void);

/**
 * @brief 复制指定节点和数据项的遥测缓存。
 * @param node_id OID 节点 ID。
 * @param item OID 数据项编号，使用 OID_DATA_* 定义。
 * @param[out] out_value 用于接收遥测数据副本。
 * @return 参数有效时返回 1，否则返回 0；数据状态需检查 time_info 字段。
 */
uint8_t oid_get_data_value(uint8_t node_id, uint8_t item, OidDataValue_t *out_value);

/**
 * @brief 将指定 OID 节点的心跳帧加入 CAN 发送队列。
 * @param node_id OID 节点 ID。
 * @return 入队状态；节点无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef oid_send_heartbeat(uint8_t node_id);
/**
 * @brief 将指定电流命令加入 OID CAN 发送队列。
 * @param node_id OID 节点 ID。
 * @param current 电流命令值。
 * @return 入队状态；节点无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef oid_send_current(uint8_t node_id, int16_t current);
/**
 * @brief 将指定电气转速命令加入 OID CAN 发送队列。
 * @param node_id OID 节点 ID。
 * @param speed_erpm 目标电气转速，单位 eRPM。
 * @return 入队状态；节点无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef oid_send_speed_erpm(uint8_t node_id, int32_t speed_erpm);
/**
 * @brief 将指定遥测项的查询命令加入 OID CAN 发送队列。
 * @param node_id OID 节点 ID。
 * @param item 要查询的数据项编号，范围为 0 到 OID_DATA_ITEM_COUNT - 1。
 * @return 入队状态；节点或数据项无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef oid_send_query(uint8_t node_id, uint8_t item);

#endif
