#include "can_fifo.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define CAN_FIFO_TX_BURST 3U

static CanFifoFrame_t can_fifo_tx_queue[CAN_FIFO_TX_LENGTH];
static CanFifoFrame_t can_fifo_rx_queue[CAN_FIFO_RX_LENGTH];
static CanFifoFrame_t can_fifo_rx_last_frame;

static uint8_t can_fifo_tx_head = 0U;
static uint8_t can_fifo_tx_tail = 0U;
static uint8_t can_fifo_tx_queue_count = 0U;

static uint8_t can_fifo_rx_head = 0U;
static uint8_t can_fifo_rx_tail = 0U;
static volatile uint8_t can_fifo_rx_queue_count = 0U;
static volatile uint32_t can_fifo_rx_total_count = 0U;
static volatile uint32_t can_fifo_rx_drop_count = 0U;
static volatile uint8_t can_fifo_rx_last_frame_ready = 0U;

HAL_StatusTypeDef can_fifo_tx_push(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    CanFifoFrame_t frame;

    if ((std_id > 0x7FFU) || (data == NULL) || (dlc > 8U))
    {
        return HAL_ERROR;
    }

    frame.std_id = std_id;
    frame.dlc = dlc;
    memcpy(frame.data, data, dlc);

    taskENTER_CRITICAL();
    if (can_fifo_tx_queue_count >= CAN_FIFO_TX_LENGTH)
    {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    can_fifo_tx_queue[can_fifo_tx_tail] = frame;
    can_fifo_tx_tail = (uint8_t)((can_fifo_tx_tail + 1U) % CAN_FIFO_TX_LENGTH);
    can_fifo_tx_queue_count++;
    taskEXIT_CRITICAL();

    return HAL_OK;
}

/**
 * @brief 从发送队列取出一帧。
 * @param[out] frame 用于接收出队帧的结构体。
 * @return 成功取到帧时返回 1，否则返回 0。
 */
static uint8_t can_fifo_tx_pop(CanFifoFrame_t *frame)
{
    uint8_t has_frame = 0U;

    if (frame == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (can_fifo_tx_queue_count > 0U)
    {
        *frame = can_fifo_tx_queue[can_fifo_tx_head];
        can_fifo_tx_head = (uint8_t)((can_fifo_tx_head + 1U) % CAN_FIFO_TX_LENGTH);
        can_fifo_tx_queue_count--;
        has_frame = 1U;
    }
    taskEXIT_CRITICAL();

    return has_frame;
}

uint8_t can_fifo_tx_get_pending_count(void)
{
    uint8_t count;
    taskENTER_CRITICAL();
    count = can_fifo_tx_queue_count;
    taskEXIT_CRITICAL();
    return count;
}

/**
 * @brief 将硬件收到的一帧加入接收队列并更新最新帧缓存。
 * @param frame 待入队的接收帧。
 * @return 入队成功返回 1；空指针或队列已满时返回 0。
 */
static uint8_t can_fifo_rx_push(const CanFifoFrame_t *frame)
{
    if (frame == NULL)
    {
        return 0U;
    }

    if (can_fifo_rx_queue_count >= CAN_FIFO_RX_LENGTH)
    {
        can_fifo_rx_drop_count++;
        return 0U;
    }

    can_fifo_rx_queue[can_fifo_rx_tail] = *frame;
    can_fifo_rx_last_frame = *frame;
    can_fifo_rx_last_frame_ready = 1U;
    can_fifo_rx_tail = (uint8_t)((can_fifo_rx_tail + 1U) % CAN_FIFO_RX_LENGTH);
    can_fifo_rx_queue_count++;
    can_fifo_rx_total_count++;

    return 1U;
}

uint8_t can_fifo_rx_pop(CanFifoFrame_t *frame)
{
    uint8_t has_frame = 0U;

    if (frame == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (can_fifo_rx_queue_count > 0U)
    {
        *frame = can_fifo_rx_queue[can_fifo_rx_head];
        can_fifo_rx_head = (uint8_t)((can_fifo_rx_head + 1U) % CAN_FIFO_RX_LENGTH);
        can_fifo_rx_queue_count--;
        has_frame = 1U;
    }
    taskEXIT_CRITICAL();

    return has_frame;
}

uint8_t can_fifo_rx_get_last_frame(CanFifoFrame_t *frame)
{
    uint8_t has_frame;

    if (frame == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    has_frame = can_fifo_rx_last_frame_ready;
    if (has_frame != 0U)
    {
        *frame = can_fifo_rx_last_frame;
        can_fifo_rx_last_frame_ready = 0U;
    }
    taskEXIT_CRITICAL();

    return has_frame;
}

uint32_t can_fifo_rx_get_count(void)
{
    return can_fifo_rx_total_count;
}

uint32_t can_fifo_rx_get_drop_count(void)
{
    return can_fifo_rx_drop_count;
}

uint8_t can_fifo_rx_get_pending_count(void)
{
    return can_fifo_rx_queue_count;
}

void can_fifo_tx_process(void)
{
    CanFifoFrame_t frame;
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;
    uint8_t sent = 0U;

    while (sent < CAN_FIFO_TX_BURST)
    {
        taskENTER_CRITICAL();
        if (can_fifo_tx_queue_count == 0U)
        {
            taskEXIT_CRITICAL();
            break;
        }
        frame = can_fifo_tx_queue[can_fifo_tx_head];
        taskEXIT_CRITICAL();

        tx_header.StdId = frame.std_id;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = frame.dlc;
        tx_header.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, frame.data, &tx_mailbox) != HAL_OK)
        {
            break;
        }

        (void)can_fifo_tx_pop(&frame);
        sent++;
    }
}

/**
 * @brief CAN1 FIFO0 接收中断回调，将硬件帧转存到软件接收队列。
 * @param hcan 触发回调的 CAN 外设句柄。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CanFifoFrame_t frame;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = {0};
    uint8_t budget = CAN_FIFO_ISR_BUDGET;
    uint8_t index;

    if (hcan->Instance != CAN1)
    {
        return;
    }

    while ((budget > 0U) && (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U))
    {
        budget--;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA) || (rx_header.DLC > 8U))
        {
            continue;
        }

        frame.std_id = (uint16_t)rx_header.StdId;
        frame.dlc = (uint8_t)rx_header.DLC;
        for (index = 0U; index < 8U; index++)
        {
            frame.data[index] = rx_data[index];
        }
        (void)can_fifo_rx_push(&frame);
    }
}

void can_fifo_init(void)
{
    can_fifo_tx_head = 0U;
    can_fifo_tx_tail = 0U;
    can_fifo_tx_queue_count = 0U;
    
    can_fifo_rx_head = 0U;
    can_fifo_rx_tail = 0U;
    can_fifo_rx_queue_count = 0U;
    can_fifo_rx_total_count = 0U;
    can_fifo_rx_drop_count = 0U;
    can_fifo_rx_last_frame_ready = 0U;
}
