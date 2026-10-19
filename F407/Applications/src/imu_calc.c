#include "imu_calc.h"
#include "imu_REG.h"
#include "usart.h"

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static uint8_t imu_init_in_progress = 1;

IMU_init imu;

float yaw_start;
float yaw_error;

/**
 * @brief 近似计算浮点数的平方根倒数。
 * @param x 待计算的数值。
 * @return 1 / sqrt(x) 的近似值。
 */
static float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/**
 * @brief 通过 UART1 向 IMU 发送字节数据。
 * @param[in] p_data 待发送数据的缓冲区。
 * @param uiSize 待发送的字节数。
 */
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  //理论上485通信，会有控制收发的引脚，得定义一个引脚，连接对应的口，这里随便写的引脚示例，这里硬件上处理过，所以直接发送数据
	//GPIO_SetBits(GPIOA, GPIO_Pin_1);
	Uart1Send(p_data, uiSize);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

/**
 * @brief 初始化期间使用 HAL 延时，之后使用 RTOS 延时。
 * @param ms 延时时长，单位毫秒。
 */
static void imu_delay_ms(uint16_t ms)
{
    if (imu_init_in_progress != 0U)
    {
        HAL_Delay(ms);
    }
    else
    {
        osDelay(ms);
    }
}

/**
 * @brief 为 WIT 驱动提供延时回调。
 * @param ms 延时时长，单位毫秒。
 */
static void SensorDelayMs(uint16_t ms)
{
    imu_delay_ms(ms);
}

/**
 * @brief 根据 WIT 驱动更新的寄存器设置对应的数据更新标志。
 * @param uiReg 首个更新的寄存器地址。
 * @param uiRegNum 连续更新的寄存器数量。
 */
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

/**
 * @brief 最多请求三次加速度寄存器，以探测 IMU 是否响应。
 * @return 加速度数据已更新时返回 1，否则返回 0。
 */
static uint8_t ProbeImu(void)
{
	uint8_t retry;
	for (retry = 0; retry < 3; retry++) {
		s_cDataUpdate = 0;
		if (WitReadReg(AX, 3) == WIT_HAL_OK) {
			imu_delay_ms(120U);
			if (s_cDataUpdate & ACC_UPDATE) return 1;
		}
	}
	return 0;
}

/**
 * @brief 等待启动阶段累计收到 7 次航向角更新，并将当前航向角设为零点。
 * @return 收到足够航向角数据并完成归零时返回 1，超时返回 0。
 */
static uint8_t yaw_initialize_zero(void)
{
	uint8_t num = 0U;
	uint8_t yaw_if_get;
	uint32_t start_ms = HAL_GetTick();
	const uint32_t timeout_ms = 3000U;

	while ((HAL_GetTick() - start_ms) < timeout_ms) {
		yaw_if_get = imu_data_get();
		if (yaw_if_get) {
			num++;
			if (num >= 7U) {
				yaw_init();
				return 1U;
			}
		}
		imu_delay_ms(100U);
	}

	return 0U;
}

int32_t imu_set_algorithm(IMU_Algorithm algorithm)
{
	uint16_t algorithm_value;

	switch (algorithm) {
	case IMU_ALGORITHM_6_AXIS:
		algorithm_value = ALGRITHM6;
		break;
	case IMU_ALGORITHM_9_AXIS:
		algorithm_value = ALGRITHM9;
		break;
	default:
		return WIT_HAL_INVAL;
	}

	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
	imu_delay_ms(20U);
	if (WitWriteReg(AXIS6, algorithm_value) != WIT_HAL_OK) return WIT_HAL_ERROR;
	imu_delay_ms(20U);
	if (WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK) return WIT_HAL_ERROR;
	imu_delay_ms(20U);

	return WIT_HAL_OK;
}

/**
 * @brief 探测 IMU 波特率，并配置为 230400 波特率和六轴模式。
 * @return 探测并配置成功时返回 1，否则返回 0。
 */
static uint8_t AutoScanSensor(void)
{
	uint8_t i;
	uint32_t startup_baud = huart1.Init.BaudRate;
	uint32_t detected_baud = 0U;
	static const struct {
		uint32_t wit_baud;
		uint32_t baud_rate;
	} baud_candidates[] = {
		{WIT_BAUD_4800, 4800U}, {WIT_BAUD_9600, 9600U},
		{WIT_BAUD_19200, 19200U}, {WIT_BAUD_38400, 38400U},
		{WIT_BAUD_57600, 57600U}, {WIT_BAUD_115200, 115200U},
		{WIT_BAUD_230400, 230400U}, {WIT_BAUD_460800, 460800U},
		{WIT_BAUD_921600, 921600U}
	};

	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	if (Uart1SetBaudRate(startup_baud) != HAL_OK) {
		printf("IMU UART init failed at startup baud %lu\r\n",
			(unsigned long)startup_baud);
		return 0U;
	}
	if (ProbeImu()) {
		detected_baud = startup_baud;
	}

	if (detected_baud == 0U) {
		for (i = 0U; i < (uint8_t)(sizeof(baud_candidates) / sizeof(baud_candidates[0])); i++) {
			if (baud_candidates[i].baud_rate == startup_baud) continue;
			if (Uart1SetBaudRate(baud_candidates[i].baud_rate) != HAL_OK) return 0U;
			if (ProbeImu()) {
				detected_baud = baud_candidates[i].baud_rate;
				break;
			}
		}
	}

	if (detected_baud == 0U) {
		if (Uart1SetBaudRate(9600U) != HAL_OK) {
			printf("IMU UART restore to 9600 failed\r\n");
		}
		printf("IMU not found at supported WIT baud rates\r\n");
		return 0U;
	}
	printf("IMU found at %lu\r\n", (unsigned long)detected_baud);

	if (detected_baud != 230400U) {
		if (WitSetUartBaud(WIT_BAUD_230400) != WIT_HAL_OK ||
			Uart1SetBaudRate(230400U) != HAL_OK) {
			printf("IMU baud change to 230400 failed\r\n");
			return 0U;
		}
		imu_delay_ms(50U);
	}
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	if (!ProbeImu()) {
		printf("IMU at 230400 not confirmed\r\n");
		return 0U;
	}
	if (imu_set_algorithm(IMU_ALGORITHM_6_AXIS) != WIT_HAL_OK) {
		printf("IMU 6-axis algorithm setup failed\r\n");
		return 0U;
	}
	printf("IMU ready at 230400\r\n");
	return 1U;
}

/**
 * @brief 处理航向角跨越正负 180 度的跳变，并更新相对航向角。
 */
static void yaw_overall(void)
{
  float yaw_change;
  static float yaw_old;
  static float yaw_total;
  yaw_change = imu.yaw - yaw_old;
  if (yaw_change < -180.0f) {
      yaw_change += 360.0f;
  }
  else if (yaw_change > 180.0f) {
      yaw_change -= 360.0f;
  }
  yaw_total += yaw_change;
  imu.yaw_total = yaw_total - yaw_start;
  yaw_old = imu.yaw;
}

//下面是为了方便在开始或者过程中重建坐标系
void yaw_init(void){
  yaw_start = imu.yaw_total;
}

uint8_t imu_data_get(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (imu.time_info.valid != 0U) {
    imu.time_info.age_ms = now_ms - imu.time_info.last_update_ms;
    imu.time_info.fresh =
        (uint8_t)(imu.time_info.age_ms <= IMU_FRESH_TIMEOUT_MS);
  } else {
    imu.time_info.age_ms = 0xFFFFFFFFUL;
    imu.time_info.fresh = 0U;
  }

  WitReadReg(AX, 12);
	if(s_cDataUpdate)
	{
		imu.acc_x = sReg[AX] / 32768.0f * 16.0f;
		imu.acc_y = sReg[AY] / 32768.0f * 16.0f;
		imu.acc_z = sReg[AZ] / 32768.0f * 16.0f;
		imu.gyro_x = sReg[GX] / 32768.0f * 2000.0f;
		imu.gyro_y = sReg[GY] / 32768.0f * 2000.0f;
		imu.gyro_z = sReg[GZ] / 32768.0f * 2000.0f;
		imu.mag_x = (float)sReg[HX];
		imu.mag_y = (float)sReg[HY];
		imu.mag_z = (float)sReg[HZ];
		imu.roll = sReg[Roll] / 32768.0f * 180.0f;
		imu.pitch = sReg[Pitch] / 32768.0f * 180.0f;
		imu.yaw = sReg[Yaw] / 32768.0f * 180.0f;
		yaw_overall();
		if(s_cDataUpdate & ACC_UPDATE)
		{
			//printf("acc:%.3f %.3f %.3f\r\n", imu.acc_x, imu.acc_y, imu.acc_z);
			s_cDataUpdate &= ~ACC_UPDATE;
		}
		if(s_cDataUpdate & GYRO_UPDATE)
		{
			//printf("gyro:%.3f %.3f %.3f\r\n", imu.gyro_x, imu.gyro_y, imu.gyro_z);
			s_cDataUpdate &= ~GYRO_UPDATE;
		}
		if(s_cDataUpdate & MAG_UPDATE)
		{
			//printf("mag:%.3f %.3f %.3f\r\n", imu.mag_x, imu.mag_y, imu.mag_z);
			s_cDataUpdate &= ~MAG_UPDATE;
		}
		if(s_cDataUpdate & ANGLE_UPDATE)
		{
			// printf("angle:%.3f %.3f %.3f\r\n", imu.roll, imu.pitch, imu.yaw_total);
			s_cDataUpdate &= ~ANGLE_UPDATE;
			imu.time_info.last_update_ms = HAL_GetTick();
			imu.time_info.age_ms = 0U;
			imu.time_info.frame_count++;
			imu.time_info.valid = 1U;
			imu.time_info.fresh = 1U;
			return 1;
		}

	}
  return 0;
}

void imu_init(void)
{
    imu_init_in_progress = 1U;
    WitInit(WIT_PROTOCOL_MODBUS, 0x50);
    WitSerialWriteRegister(SensorUartSend);  //控制收发
    WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(SensorDelayMs);
    imu_delay_ms(300U);
    if (AutoScanSensor()) {
		if (!yaw_initialize_zero()) {
			printf("IMU yaw initialization timed out; continuing without zeroing\r\n");
		}
	}
    imu_init_in_progress = 0U;
}
