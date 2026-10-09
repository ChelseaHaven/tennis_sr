#include "imu_calc.h"
#include "imu_REG.h"
#include "usart.h"

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

IMU_init imu;

float yaw_start;
float yaw_error;

//快速计算平方根的倒数
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  //理论上485通信，会有控制收发的引脚，得定义一个引脚，连接对应的口，这里随便写的引脚示例，这里硬件上应该是处理过，所以直接发送数据
	//GPIO_SetBits(GPIOA, GPIO_Pin_1);
	Uart1Send(p_data, uiSize);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

static uint8_t imu_init_in_progress = 1;

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

static void SensorDelayMs(uint16_t ms)
{
    imu_delay_ms(ms);
}

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

static void AutoScanSensor(void)
{
	uint8_t num = 0;
	uint8_t yaw_if_get;
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
		return;
	}
	if (ProbeImu()) {
		detected_baud = startup_baud;
	}

	if (detected_baud == 0U) {
		for (i = 0U; i < (uint8_t)(sizeof(baud_candidates) / sizeof(baud_candidates[0])); i++) {
			if (baud_candidates[i].baud_rate == startup_baud) continue;
			if (Uart1SetBaudRate(baud_candidates[i].baud_rate) != HAL_OK) return;
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
		return;
	}
	printf("IMU found at %lu\r\n", (unsigned long)detected_baud);

	if (detected_baud != 230400U) {
		if (WitSetUartBaud(WIT_BAUD_230400) != WIT_HAL_OK ||
			Uart1SetBaudRate(230400U) != HAL_OK) {
			printf("IMU baud change to 230400 failed\r\n");
			return;
		}
		imu_delay_ms(50U);
	}
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	if (!ProbeImu()) {
		printf("IMU at 230400 not confirmed\r\n");
		return;
	}
	if (imu_set_algorithm(IMU_ALGORITHM_9_AXIS) != WIT_HAL_OK) {
		printf("IMU 9-axis algorithm setup failed\r\n");
		return;
	}
	printf("IMU ready at 230400\r\n");

	while (1) {
		yaw_if_get = imu_data_get();
		imu_delay_ms(100U);
		if (yaw_if_get && num++ > 5) {
			yaw_init();
			break;
		}
	}
}

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
		imu.acc_raw[0] = sReg[AX];
		imu.acc_raw[1] = sReg[AY];
		imu.acc_raw[2] = sReg[AZ];
		imu.gyro_raw[0] = sReg[GX];
		imu.gyro_raw[1] = sReg[GY];
		imu.gyro_raw[2] = sReg[GZ];
		imu.mag_raw[0] = sReg[HX];
		imu.mag_raw[1] = sReg[HY];
		imu.mag_raw[2] = sReg[HZ];
		imu.angle_raw[0] = sReg[Roll];
		imu.angle_raw[1] = sReg[Pitch];
		imu.angle_raw[2] = sReg[Yaw];
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
    AutoScanSensor();
    imu_init_in_progress = 0U;
}
