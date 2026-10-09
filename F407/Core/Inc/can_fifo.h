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

void can_fifo_init(void);
HAL_StatusTypeDef can_fifo_tx_push(uint16_t std_id, const uint8_t *data, uint8_t dlc);
uint8_t can_fifo_tx_get_pending_count(void);
void can_fifo_tx_process(void);

uint8_t can_fifo_rx_pop(CanFifoFrame_t *frame);
uint8_t can_fifo_rx_get_last_frame(CanFifoFrame_t *frame);
uint32_t can_fifo_rx_get_count(void);
uint32_t can_fifo_rx_get_drop_count(void);
uint8_t can_fifo_rx_get_pending_count(void);

#endif
