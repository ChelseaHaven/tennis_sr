#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash_ex.h"
#include <stdio.h>

#define FLASH_STORAGE_BASE_ADDRESS      0x08060000U
#define FLASH_STORAGE_SECTOR            FLASH_SECTOR_7
#define FLASH_STORAGE_SIZE_BYTES        0x00020000U
#define FLASH_STORAGE_WORD_SIZE_BYTES   4U
#define FLASH_STORAGE_MAX_WORDS         (FLASH_STORAGE_SIZE_BYTES / FLASH_STORAGE_WORD_SIZE_BYTES)
#define FLASH_DATA_WORD_COUNT           16U
#define FLASH_DATA_MAGIC                0x544E5331U

typedef union
{
    uint32_t u32;
    int32_t i32;
    float f32;
    uint16_t u16;
    int16_t i16;
    uint8_t u8;
    int8_t i8;
} FlashDataWord_t;

typedef char FlashDataWord_must_be_4_bytes[(sizeof(FlashDataWord_t) == 4U) ? 1 : -1];

/**
 * @brief 从预留 Flash 扇区读取应用数据到 flash_data，并校验数据标记。
 * @return 读取且标记有效时返回 HAL_OK，否则返回 HAL_ERROR 或底层 HAL 状态。
 */
HAL_StatusTypeDef flash_read_application_data(void);
/**
 * @brief 将 flash_data 中的应用数据写入预留 Flash 扇区。
 * @return 写入成功返回 HAL_OK，否则返回对应的 HAL 错误状态。
 */
HAL_StatusTypeDef flash_write_application_data(void);

#endif
