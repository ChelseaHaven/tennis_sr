#include "servo.h"

volatile float servo_1_target_angle = SERVO_1_DEFAULT_ANGLE_DEG;
volatile float servo_2_target_angle = SERVO_2_DEFAULT_ANGLE_DEG;

static uint8_t servo_1_initialized = 0U;
static uint8_t servo_2_initialized = 0U;

typedef struct
{
    uint32_t channel;
    volatile float *target_angle;
    float angle_min_deg;
    float angle_max_deg;
    float default_angle_deg;
    uint16_t pulse_min_us;
    uint16_t pulse_max_us;
    uint8_t *initialized;
} servo_config_t;

//要改舵机或者新增舵机只需要改servo_get_config()里的数据，和servo.h的对应数据，其它函数通用
/**
 * @brief 根据舵机编号取得对应的定时器通道和角度配置。
 * @param servo_id 舵机编号。
 * @param[out] config 用于接收舵机配置。
 * @return 编号有效并成功填充配置时返回 1，否则返回 0。
 */
static uint8_t servo_get_config(uint8_t servo_id, servo_config_t *config)
{
    if (servo_id == SERVO_1)
    {
        config->channel = TIM_CHANNEL_3;
        config->target_angle = &servo_1_target_angle;
        config->angle_min_deg = SERVO_1_ANGLE_MIN_DEG;
        config->angle_max_deg = SERVO_1_ANGLE_MAX_DEG;
        config->default_angle_deg = SERVO_1_DEFAULT_ANGLE_DEG;
        config->pulse_min_us = SERVO_1_PULSE_MIN_US;
        config->pulse_max_us = SERVO_1_PULSE_MAX_US;
        config->initialized = &servo_1_initialized;
        return 1U;
    }

    if (servo_id == SERVO_2)
    {
        config->channel = TIM_CHANNEL_4;
        config->target_angle = &servo_2_target_angle;
        config->angle_min_deg = SERVO_2_ANGLE_MIN_DEG;
        config->angle_max_deg = SERVO_2_ANGLE_MAX_DEG;
        config->default_angle_deg = SERVO_2_DEFAULT_ANGLE_DEG;
        config->pulse_min_us = SERVO_2_PULSE_MIN_US;
        config->pulse_max_us = SERVO_2_PULSE_MAX_US;
        config->initialized = &servo_2_initialized;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 将舵机角度限制在允许范围内。
 * @param angle_deg 待限制的角度，单位为度。
 * @param angle_min_deg 允许的最小角度。
 * @param angle_max_deg 允许的最大角度。
 * @return 限幅后的角度。
 */
static float servo_limit_angle(float angle_deg, float angle_min_deg, float angle_max_deg)
{
    if (angle_deg < angle_min_deg) return angle_min_deg;
    if (angle_deg > angle_max_deg) return angle_max_deg;
    return angle_deg;
}

/**
 * @brief 将舵机角度线性映射为 PWM 脉宽。
 * @param angle_deg 舵机角度，单位为度。
 * @param angle_min_deg 角度下限。
 * @param angle_max_deg 角度上限。
 * @param pulse_min_us 角度下限对应的脉宽，单位微秒。
 * @param pulse_max_us 角度上限对应的脉宽，单位微秒。
 * @return 换算并四舍五入后的 PWM 脉宽，单位微秒。
 */
static uint16_t servo_angle_to_pulse(float angle_deg, float angle_min_deg, float angle_max_deg, uint16_t pulse_min_us, uint16_t pulse_max_us)
{
    float pulse_us;

    angle_deg = servo_limit_angle(angle_deg, angle_min_deg, angle_max_deg);
    pulse_us = (float)pulse_min_us + (angle_deg - angle_min_deg) * (float)(pulse_max_us - pulse_min_us) / (angle_max_deg - angle_min_deg);

    return (uint16_t)(pulse_us + 0.5f);
}

HAL_StatusTypeDef servo_set_angle(uint8_t servo_id, float angle_deg)
{
    servo_config_t config;
    uint16_t pulse;

    if (angle_deg != angle_deg)
    {
        return HAL_ERROR;
    }

    if (servo_get_config(servo_id, &config) == 0U)
    {
        return HAL_ERROR;
    }

    *config.target_angle = servo_limit_angle(angle_deg, config.angle_min_deg, config.angle_max_deg);

    if (*config.initialized != 0U)
    {
        pulse = servo_angle_to_pulse(*config.target_angle, config.angle_min_deg, config.angle_max_deg, config.pulse_min_us, config.pulse_max_us);
        __HAL_TIM_SET_COMPARE(&htim3, config.channel, pulse);
    }

    return HAL_OK;
}

void servo_stop(uint8_t servo_id)
{
    servo_config_t config;

    if (servo_get_config(servo_id, &config) == 0U)
    {
        return;
    }

    *config.initialized = 0U;
    __HAL_TIM_SET_COMPARE(&htim3, config.channel, 0U);
    (void)HAL_TIM_PWM_Stop(&htim3, config.channel);
}

HAL_StatusTypeDef servo_init(uint8_t servo_id)
{
    servo_config_t config;
    uint16_t pulse;

    if (servo_get_config(servo_id, &config) == 0U)
    {
        return HAL_ERROR;
    }

    *config.target_angle = config.default_angle_deg;
    pulse = servo_angle_to_pulse(*config.target_angle, config.angle_min_deg, config.angle_max_deg, config.pulse_min_us, config.pulse_max_us);
    __HAL_TIM_SET_COMPARE(&htim3, config.channel, pulse);

    if (HAL_TIM_PWM_Start(&htim3, config.channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *config.initialized = 1U;
    return HAL_OK;
}
