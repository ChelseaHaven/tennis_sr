#ifndef _WIRELESS_H_
#define _WIRELESS_H_

#include "system.h"

typedef struct{

	volatile uint8_t chasis;
	volatile uint8_t launcher;
	volatile uint8_t wireless;
	volatile uint8_t driver_healthy;
	volatile uint8_t can_bus_off;
	volatile uint8_t disarm_reason;
} State_flag;

/**
 * @brief 解析已接收的无线命令，并重新启动 UART3 DMA 接收。
 */
void wireless_read(void);

extern State_flag state;
extern uint8_t wireless_data[51];
extern float wireless_test[10];

#endif
