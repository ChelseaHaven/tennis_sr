#ifndef __CAN_FIFO_H__
#define __CAN_FIFO_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define CAN_FIFO_TX_LENGTH     64U
#define CAN_FIFO_RX_LENGTH    128U
#define CAN_FIFO_ISR_BUDGET     8U

typedef struct
{
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} CanFifoFrame_t;

/**
 * @brief 初始化 CAN 收发队列及队列状态。
 */
void can_fifo_init(void);
/**
 * @brief 将一帧标准 CAN 数据帧加入发送队列。
 * @param std_id 11 位标准帧 ID。
 * @param data 待发送数据缓冲区。
 * @param dlc 数据长度，范围为 0 到 8 字节。
 * @return 入队成功返回 HAL_OK；参数无效返回 HAL_ERROR；队列已满返回 HAL_BUSY。
 */
HAL_StatusTypeDef can_fifo_tx_push(uint16_t std_id, const uint8_t *data, uint8_t dlc);
/**
 * @brief 获取当前等待发送的 CAN 帧数量。
 * @return 发送队列中的帧数。
 */
uint8_t can_fifo_tx_get_pending_count(void);
/**
 * @brief 从发送队列取帧并尝试提交给 CAN 外设发送。
 */
void can_fifo_tx_process(void);

/**
 * @brief 从接收队列取出一帧 CAN 数据。
 * @param[out] frame 用于接收出队帧的结构体。
 * @return 成功取到帧时返回 1，否则返回 0。
 */
uint8_t can_fifo_rx_pop(CanFifoFrame_t *frame);
/**
 * @brief 获取自上次读取以来最新收到的一帧 CAN 数据。
 * @param[out] frame 用于接收最新帧的结构体。
 * @return 有尚未读取的新帧时返回 1，否则返回 0。
 */
uint8_t can_fifo_rx_get_last_frame(CanFifoFrame_t *frame);
/**
 * @brief 获取 CAN 接收成功的累计帧数。
 * @return 自初始化以来成功入队的接收帧总数。
 */
uint32_t can_fifo_rx_get_count(void);
/**
 * @brief 获取因接收队列已满而丢弃的累计帧数。
 * @return 自初始化以来丢弃的接收帧总数。
 */
uint32_t can_fifo_rx_get_drop_count(void);
/**
 * @brief 获取当前等待处理的接收帧数量。
 * @return 接收队列中的帧数。
 */
uint8_t can_fifo_rx_get_pending_count(void);

#endif
