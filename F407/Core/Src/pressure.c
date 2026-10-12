#include <stddef.h>
#include "pressure.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

#define PRESSURE_BUS_REQUEST_PERIOD_MS  50U
#define PRESSURE_RESPONSE_TIMEOUT_MS    35U
#define PRESSURE_FRAME_LENGTH           7U
PressureData_t pressure_data[PRESSURE_SENSOR_COUNT] = {0};

//要添加新的压力传感器只需要改PRESSURE_SENSOR_COUNT和下面的地址
static const uint8_t pressure_addresses[PRESSURE_SENSOR_COUNT] = {
    PRESSURE_SENSOR_1_ADDR, PRESSURE_SENSOR_2_ADDR
};

static volatile uint8_t pressure_rx_frame[PRESSURE_FRAME_LENGTH];
static volatile uint8_t pressure_rx_count;
static volatile uint8_t pressure_frame_ready;
static volatile uint8_t pressure_request_active;
static volatile uint8_t pressure_waiting;
static volatile uint8_t pressure_active_address;
static uint8_t pressure_next_sensor;
static uint8_t pressure_has_sent;
static uint32_t pressure_last_request_tick;
static uint32_t pressure_request_started_tick;

/**
 * @brief 计算 Modbus RTU 帧的 CRC16 校验值。
 * @param data 待校验的数据缓冲区。
 * @param length 数据字节数。
 * @return 16 位 CRC 值，低字节在前发送。
 */
static uint16_t Pressure_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t bit;

    while (length-- != 0U) {
        crc ^= *data++;
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/**
 * @brief 构造读取一个压力传感器寄存器的 Modbus 请求帧。
 * @param address 压力传感器从站地址。
 * @param[out] request 用于接收 8 字节请求帧的缓冲区。
 */
static void Pressure_BuildReadRequest(uint8_t address, uint8_t request[8])
{
    uint16_t crc;

    request[0] = address;
    request[1] = 0x03U;
    request[2] = 0x00U;
    request[3] = 0x00U;
    request[4] = 0x00U;
    request[5] = 0x01U;
    crc = Pressure_Crc16(request, 6U);
    request[6] = (uint8_t)(crc & 0x00FFU);
    request[7] = (uint8_t)(crc >> 8U);
}

/**
 * @brief 将压力传感器从站地址映射到数据数组下标。
 * @param address 待查找的从站地址。
 * @return 找到时返回数组下标，否则返回 -1。
 */
static int Pressure_AddressIndex(uint8_t address)
{
    uint8_t sensor_index;

    for (sensor_index = 0U; sensor_index < PRESSURE_SENSOR_COUNT; sensor_index++) {
        if (address == pressure_addresses[sensor_index]) {
            return (int)sensor_index;
        }
    }
    return -1;
}

/**
 * @brief 校验已接收的压力传感器帧并更新对应数据缓存。
 * @param data 用于保存传感器读数和时效信息的数组。
 * @return 收到并处理有效帧时返回 1，否则返回 0。
 */
static uint8_t Pressure_ProcessReceivedFrame(PressureData_t data[PRESSURE_SENSOR_COUNT])
{
    uint8_t frame[PRESSURE_FRAME_LENGTH];
    uint8_t i;
    uint8_t address;
    int sensor_index;
    uint16_t expected_crc;
    uint16_t received_crc;

    taskENTER_CRITICAL();
    if (pressure_frame_ready == 0U) {
        taskEXIT_CRITICAL();
        return 0U;
    }
    for (i = 0U; i < PRESSURE_FRAME_LENGTH; i++) {
        frame[i] = pressure_rx_frame[i];
    }
    pressure_frame_ready = 0U;
    pressure_rx_count = 0U;
    taskEXIT_CRITICAL();

    address = frame[0];
    sensor_index = Pressure_AddressIndex(address);
    if ((sensor_index < 0) || (address != pressure_active_address) || (frame[1] != 0x03U) || (frame[2] != 0x02U)) {
        return 0U;
    }

    expected_crc = Pressure_Crc16(frame, 5U);
    received_crc = (uint16_t)(frame[5] | ((uint16_t)frame[6] << 8U));
    if (expected_crc != received_crc) {
        return 0U;
    }

    data[sensor_index].value = (uint16_t)(((uint16_t)frame[3] << 8U) | frame[4]);
    data[sensor_index].time_info.last_update_ms = HAL_GetTick();
    data[sensor_index].time_info.age_ms = 0U;
    data[sensor_index].time_info.frame_count++;
    data[sensor_index].time_info.valid = 1U;
    data[sensor_index].time_info.fresh = 1U;
    pressure_waiting = 0U;
    pressure_request_active = 0U;
    return 1U;
}

void Pressure_Process(PressureData_t data[PRESSURE_SENSOR_COUNT])
{
    uint32_t now;
    uint8_t request[8];
    uint8_t sensor_index;

    if (data == NULL) {
        return;
    }

    (void)Pressure_ProcessReceivedFrame(data);
    now = HAL_GetTick();

    for (sensor_index = 0U; sensor_index < PRESSURE_SENSOR_COUNT; sensor_index++) {
        if (data[sensor_index].time_info.valid != 0U) {
            data[sensor_index].time_info.age_ms = now - data[sensor_index].time_info.last_update_ms;
            data[sensor_index].time_info.fresh = (uint8_t) (data[sensor_index].time_info.age_ms <= PRESSURE_FRESH_TIMEOUT_MS);
        }
    }

    if (pressure_waiting != 0U && (uint32_t)(now - pressure_request_started_tick) >= PRESSURE_RESPONSE_TIMEOUT_MS) {
        pressure_waiting = 0U;
        pressure_request_active = 0U;
        pressure_rx_count = 0U;
        pressure_frame_ready = 0U;
    }

    if (pressure_waiting == 0U && (pressure_has_sent == 0U || (uint32_t)(now - pressure_last_request_tick) >= PRESSURE_BUS_REQUEST_PERIOD_MS)) {
        pressure_active_address = pressure_addresses[pressure_next_sensor];
        pressure_next_sensor = (uint8_t)((pressure_next_sensor + 1U) % PRESSURE_SENSOR_COUNT);

        pressure_rx_count = 0U;
        pressure_frame_ready = 0U;
        Pressure_BuildReadRequest(pressure_active_address, request);
        pressure_request_active = 1U;
        pressure_waiting = 1U;
        pressure_last_request_tick = now;
        pressure_has_sent = 1U;
        Uart5Send(request, sizeof(request));
        pressure_request_started_tick = HAL_GetTick();
    }

}

void Pressure_RxByte(uint8_t byte)
{
    if ((pressure_request_active == 0U) || (pressure_frame_ready != 0U)) {
        return;
    }

    if (pressure_rx_count == 0U) {
        if (byte != pressure_active_address) {
            return;
        }
        pressure_rx_frame[0] = byte;
        pressure_rx_count = 1U;
        return;
    }

    if ((pressure_rx_count == 1U) && (byte != 0x03U)) {
        pressure_rx_count = 0U;
        return;
    }
    if ((pressure_rx_count == 2U) && (byte != 0x02U)) {
        pressure_rx_count = 0U;
        return;
    }

    if (pressure_rx_count < PRESSURE_FRAME_LENGTH) {
        pressure_rx_frame[pressure_rx_count++] = byte;
    }
    if (pressure_rx_count >= PRESSURE_FRAME_LENGTH) {
        pressure_rx_count = 0U;
        pressure_frame_ready = 1U;
    }
}

void Pressure_Init(PressureData_t data[PRESSURE_SENSOR_COUNT])
{
    uint8_t sensor_index;

    if (data == NULL) {
        return;
    }

    pressure_rx_count = 0U;
    pressure_frame_ready = 0U;
    pressure_request_active = 0U;
    pressure_waiting = 0U;
    pressure_active_address = 0U;
    for (sensor_index = 0U; sensor_index < PRESSURE_SENSOR_COUNT; sensor_index++) {
        data[sensor_index].value = 0U;
        data[sensor_index].time_info.last_update_ms = 0U;
        data[sensor_index].time_info.age_ms = 0xFFFFFFFFUL;
        data[sensor_index].time_info.frame_count = 0U;
        data[sensor_index].time_info.valid = 0U;
        data[sensor_index].time_info.fresh = 0U;
    }
    pressure_next_sensor = 0U;
    pressure_has_sent = 0U;
    pressure_last_request_tick = 0U;
    pressure_request_started_tick = 0U;
}
