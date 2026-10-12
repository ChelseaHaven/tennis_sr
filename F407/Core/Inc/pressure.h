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

/**
 * @brief 初始化压力传感器轮询状态及各传感器数据。
 * @param data 用于保存各传感器读数和时效信息的数组。
 */
void Pressure_Init(PressureData_t data[PRESSURE_SENSOR_COUNT]);

/**
 * @brief 处理压力传感器收发状态并更新读数及数据时效。
 * @param data 保存各传感器读数和时效信息的数组。
 * @note 应由任务周期性调用；模块按地址轮询传感器，并处理响应超时。
 */
void Pressure_Process(PressureData_t data[PRESSURE_SENSOR_COUNT]);

/**
 * @brief 将 UART5 收到的单个字节交给压力传感器帧接收器处理。
 * @param byte UART5 收到的字节。
 */
void Pressure_RxByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_H */
