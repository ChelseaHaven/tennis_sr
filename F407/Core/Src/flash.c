#include "flash.h"

FlashDataWord_t flash_data[FLASH_DATA_WORD_COUNT] = {0};
const uint32_t flash_data_word_count = (uint32_t)(sizeof(flash_data) / sizeof(flash_data[0]));

/**
 * @brief 从预留 Flash 地址读取指定数量的 32 位数据字。
 * @param[out] data 用于接收读取数据的数组。
 * @param word_count 读取的数据字数量。
 * @return 读取成功返回 HAL_OK；参数无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef flash_load_words(FlashDataWord_t *data, uint32_t word_count)
{
    const volatile uint32_t *flash_word;
    uint32_t index;

    if ((data == NULL) || (word_count == 0U) || (word_count > FLASH_STORAGE_MAX_WORDS))
    {
        return HAL_ERROR;
    }

    flash_word = (const volatile uint32_t *)FLASH_STORAGE_BASE_ADDRESS;
    for (index = 0U; index < word_count; index++)
    {
        data[index].u32 = flash_word[index];
    }

    return HAL_OK;
}

/**
 * @brief 擦除预留 Flash 扇区并写入指定数量的 32 位数据字。
 * @param data 待写入的数据数组。
 * @param word_count 写入的数据字数量。
 * @return 擦除和写入成功返回 HAL_OK，否则返回对应的 HAL 状态。
 */
HAL_StatusTypeDef flash_save_words(const FlashDataWord_t *data, uint32_t word_count)
{
    FLASH_EraseInitTypeDef erase_config = {0};
    HAL_StatusTypeDef status;
    uint32_t sector_error = 0U;
    uint32_t index;

    if ((data == NULL) || (word_count == 0U) || (word_count > FLASH_STORAGE_MAX_WORDS))
    {
        return HAL_ERROR;
    }

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }

    erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_config.Sector = FLASH_STORAGE_SECTOR;
    erase_config.NbSectors = 1U;
    erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&erase_config, &sector_error);
    if (status == HAL_OK)
    {
        for (index = 0U; index < word_count; index++)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_STORAGE_BASE_ADDRESS + (index * FLASH_STORAGE_WORD_SIZE_BYTES),
                                       data[index].u32);
            if (status != HAL_OK)
            {
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    return status;
}

HAL_StatusTypeDef flash_read_application_data(void)
{
    if (flash_data_word_count > FLASH_STORAGE_MAX_WORDS)
    {
        return HAL_ERROR;
    }

    if (flash_load_words(flash_data, flash_data_word_count) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (flash_data[0].u32 != FLASH_DATA_MAGIC)
    {
        return HAL_ERROR;
    }

    uint32_t flash_example_counter;
    float flash_example_gain;
    int16_t flash_example_offset;

    flash_example_counter = flash_data[1].u32;
    flash_example_gain = flash_data[2].f32;
    flash_example_offset = flash_data[3].i16;

    printf("Flash data loaded: counter=%u, gain=%f, offset=%d\r\n", flash_example_counter, flash_example_gain, flash_example_offset);

    return HAL_OK;
}

HAL_StatusTypeDef flash_write_application_data(void)
{
    uint32_t index;

    if (flash_data_word_count > FLASH_STORAGE_MAX_WORDS)
    {
        return HAL_ERROR;
    }

    for (index = 0U; index < flash_data_word_count; index++)
    {
        flash_data[index].u32 = 0U;
    }

    uint32_t flash_example_counter = 123U;
    float flash_example_gain = 1.25f;
    int16_t flash_example_offset = -25;

    flash_data[0].u32 = FLASH_DATA_MAGIC;
    flash_data[1].u32 = flash_example_counter;
    flash_data[2].f32 = flash_example_gain;
    flash_data[3].i16 = flash_example_offset;

    return flash_save_words(flash_data, flash_data_word_count);
}
