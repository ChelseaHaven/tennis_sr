#ifndef PRESSURE_H
#define PRESSURE_H

#include "stm32f4xx_hal.h"
#include "system.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PRESSURE_SENSOR_1_ADDR  0x01U
#define PRESSURE_SENSOR_2_ADDR  0x02U
#define PRESSURE_SENSOR_COUNT   2U
#define PRESSURE_FRESH_TIMEOUT_MS 300U

typedef struct
{
    uint16_t value;               /* Raw register value for one sensor. */
    SensorTimeInfo_t time_info;
} PressureData_t;

extern PressureData_t pressure_data[PRESSURE_SENSOR_COUNT];

/* Initialize pressure polling state before the RTOS scheduler starts. */
void Pressure_Init(PressureData_t data[PRESSURE_SENSOR_COUNT]);

/* Call periodically (e.g. every 10-50 ms); queries alternate every 50 ms. */
void Pressure_Process(PressureData_t data[PRESSURE_SENSOR_COUNT]);

/* UART5 byte-receive callback forwards each received byte here. */
void Pressure_RxByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_H */
