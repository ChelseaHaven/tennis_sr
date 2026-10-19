/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system.h"
#include "launcher.h"
#include "oid.h"
#include "rk3588.h"
#include "imu_REG.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId imu_TaskHandle;
osThreadId chassis_TaskHandle;
osThreadId launcher_TaskHandle;
osThreadId can_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_imu_Task(void const * argument);
void Start_chassis_Task(void const * argument);
void Start_launcher_Task(void const * argument);
void Start_can_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of imu_Task */
  osThreadDef(imu_Task, Start_imu_Task, osPriorityNormal, 0, 512);
  imu_TaskHandle = osThreadCreate(osThread(imu_Task), NULL);

  /* definition and creation of chassis_Task */
  osThreadDef(chassis_Task, Start_chassis_Task, osPriorityIdle, 0, 512);
  chassis_TaskHandle = osThreadCreate(osThread(chassis_Task), NULL);

  /* definition and creation of launcher_Task */
  osThreadDef(launcher_Task, Start_launcher_Task, osPriorityIdle, 0, 512);
  launcher_TaskHandle = osThreadCreate(osThread(launcher_Task), NULL);

  /* definition and creation of can_Task */
  osThreadDef(can_Task, Start_can_task, osPriorityIdle, 0, 512);
  can_TaskHandle = osThreadCreate(osThread(can_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_imu_Task */
/**
  * @brief  Function implementing the imu_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_imu_Task */
void Start_imu_Task(void const * argument)
{
  /* USER CODE BEGIN Start_imu_Task */
  /* Infinite loop */
  TickType_t lastWakeTime;
  const TickType_t taskPeriod = pdMS_TO_TICKS(5U);

  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    //HAL_GPIO_TogglePin(GPIOA, LED1_Pin | LED2_Pin);

    // if (imu_data_get() != 0U)
    // {
      (void)rk3588_data_send_int16x3(RK3588_CMD_IMU_GYRO, sReg[GX], sReg[GY], sReg[GZ]);
      (void)rk3588_data_send_int16x3(RK3588_CMD_IMU_ACCEL, sReg[AX], sReg[AY], sReg[AZ]);
      (void)rk3588_data_send_int16x3(RK3588_CMD_IMU_ATTITUDE, sReg[Pitch], sReg[Roll], sReg[Yaw]);
    // }

    // printf("%f,%f,%f\r\n",imu.pitch,imu.roll,imu.yaw_total);
    vTaskDelayUntil(&lastWakeTime, taskPeriod);
  }
  /* USER CODE END Start_imu_Task */
}

/* USER CODE BEGIN Header_Start_chassis_Task */
/**
* @brief Function implementing the chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_chassis_Task */
void Start_chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Start_chassis_Task */
  TickType_t lastWakeTime;
  const TickType_t taskPeriod = pdMS_TO_TICKS(20U);

  vTaskDelay(pdMS_TO_TICKS(2000));

  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {

    Rk3588DataValue_t item;
    int16_t target[3];
    uint8_t chassis_enabled = (uint8_t)(rk3588_is_global_enabled() != 0U &&
        rk3588_data_is_enabled(RK3588_MANUAL_ENABLE_CHASSIS_MASK) != 0U);


    if (chassis_enabled == 0U)
    {
      (void)rk3588_data_clear(RK3588_CMD_TARGET_VELOCITY);
      R_Vel.X_target = 0.0f;
      R_Vel.Y_target = 0.0f;
      R_Vel.W_target = 0.0f;
    }
    else if (rk3588_data_get_value(RK3588_CMD_TARGET_VELOCITY, &item) &&
        item.time_info.valid && item.time_info.fresh &&
        rk3588_data_parse_int16x3(&item, target))
    {
      R_Vel.X_target = (float)target[0];
      R_Vel.Y_target = (float)target[1];
      R_Vel.W_target = (float)target[2] / 1000.0f;
    }

    // PidSet(&motor_speed_pid[0], wireless_test[6], wireless_test[7], wireless_test[8]);
    // PidSet(&motor_speed_pid[1], wireless_test[6], wireless_test[7], wireless_test[8]);
    // PidSet(&motor_speed_pid[2], wireless_test[6], wireless_test[7], wireless_test[8]);
    // PidSet(&motor_speed_pid[3], wireless_test[6], wireless_test[7], wireless_test[8]);
    // PidSet(&car_position_pid[0], wireless_test[9], wireless_test[10], wireless_test[11]);
    // PidSet(&car_position_pid[1], wireless_test[9], wireless_test[10], wireless_test[11]);
    // PidSet(&car_position_pid[2], wireless_test[9], wireless_test[10], wireless_test[11]);

      // R_Vel.X_target = 0.0f;
      // R_Vel.Y_target = 0.0f;
      // R_Vel.W_target = 0.0f;

      // R_Pos.X_global_target = (float)wireless_test[0];
      // R_Pos.Y_global_target = (float)wireless_test[1];
      // R_Pos.W_global_target = DEG_TO_RAD((float)wireless_test[2]);
    encoder_process();
    chassis_control_process(chassis_enabled);

    (void)rk3588_data_send_int16x3(RK3588_CMD_CHASSIS_VELOCITY, telemetry_float_to_int16(R_Vel.X), telemetry_float_to_int16(R_Vel.Y), telemetry_float_to_int16(R_Vel.W * 1000.0f));
    (void)rk3588_data_send_int16x3(RK3588_CMD_CHASSIS_POSITION, telemetry_float_to_int16(R_Pos.X_global / 10.0f), telemetry_float_to_int16(R_Pos.Y_global / 10.0f), telemetry_float_to_int16(R_Pos.W_global * 1000.0f));

      printf("%f,%f,%f,%f,%f,%f,%f,%f\r\n",
             R_Wheel_A.V_target, R_Wheel_A.V_real,
             R_Wheel_B.V_target, R_Wheel_B.V_real,
             R_Wheel_C.V_target, R_Wheel_C.V_real,
             R_Wheel_D.V_target, R_Wheel_D.V_real);

    //printf("%f,%f\r\n",R_Pos.W_global_target,imu.yaw_total);
  // Keep this high-rate telemetry trace disabled.
  // printf("%f,%f,%f,%f,%f,%f\r\n",R_Pos.X_body_error,R_Pos.Y_body_error,R_Pos.W_body_error,wireless_test[6],wireless_test[7],wireless_test[8]);
    vTaskDelayUntil(&lastWakeTime, taskPeriod);
  }
  /* USER CODE END Start_chassis_Task */
}

/* USER CODE BEGIN Header_Start_launcher_Task */
/**
* @brief Function implementing the launcher_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_launcher_Task */
void Start_launcher_Task(void const * argument)
{
  /* USER CODE BEGIN Start_launcher_Task */
  TickType_t lastWakeTime;
  const TickType_t taskPeriod = pdMS_TO_TICKS(20U);

  vTaskDelay(pdMS_TO_TICKS(10));

  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    wireless_read();
    Pressure_Process(pressure_data);


// Rk3588DataValue_t enable_data;

// rk3588_data_get_value(RK3588_CMD_MANUAL_ENABLE, &enable_data);

// printf("chassis_enable=%ld valid=%u fresh=%u age=%lu ms\r\n",
//        (long)enable_data.value,
//        (unsigned)enable_data.time_info.valid,
//        (unsigned)enable_data.time_info.fresh,
//        (unsigned long)enable_data.time_info.age_ms);

    uint8_t launcher_enabled = (uint8_t)(rk3588_is_global_enabled() != 0U &&
        rk3588_data_is_enabled(RK3588_MANUAL_ENABLE_LAUNCHER_MASK) != 0U);
    // printf("chassis_enabled=%u, launcher_enabled=%u\r\n", (unsigned)chassis_enabled, (unsigned)launcher_enabled);



    // printf("%u,%u\r\n", pressure_data[0].value, pressure_data[1].value);

    // printf("%u,%u\r\n", pressure_data[0].value, pressure_data[1].value);
    // servo_set_angle(SERVO_1, wireless_test[4]);
    // servo_set_angle(SERVO_2, wireless_test[5]);
    // printf("%f,%f\r\n",wireless_test[4],wireless_test[5]);
    //printf("%f,%f,%f\r\n",imu.pitch,imu.roll,imu.yaw_total);
    vTaskDelayUntil(&lastWakeTime, taskPeriod);
  }
  /* USER CODE END Start_launcher_Task */
}

/* USER CODE BEGIN Header_Start_can_task */
/**
* @brief Function implementing the can_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_can_task */
void Start_can_task(void const * argument)
{
  /* USER CODE BEGIN Start_can_task */
  TickType_t lastWakeTime;
  CanFifoFrame_t rx_frame;
  uint8_t can_task_5ms_counter = 0U;
  uint8_t can_task_20ms_counter = 0U;
  uint8_t can_task_100ms_counter = 0U;
  const TickType_t taskPeriod = pdMS_TO_TICKS(1);

  lastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    while (can_fifo_rx_pop(&rx_frame) != 0U)
    {
      oid_handle_can_rx_frame(&rx_frame);
      rk3588_handle_can_rx_frame(&rx_frame);
    }

    can_task_5ms_counter++;
    if (can_task_5ms_counter >= 5)
    {
      can_task_5ms_counter = 0U;

      rk3588_refresh_freshness();
      oid_refresh_freshness();
      oid_process_speed_queries();  // �? 5 ms 查询�?个节点的速度�?6 个节点一轮约 30 ms
      oid_process_heartbeats();     // �? 5 ms 查询�?个节点的速度�?6 个节点一轮约 30 ms
    }

    can_task_20ms_counter++;
    if (can_task_20ms_counter >= 20)
    {
      can_task_20ms_counter = 0U;
      oid_process_queries();        //�? 20 ms 查询�?个节点的�?项非速度数据�?72 项轮询一轮约 1440 ms
    }

    can_task_100ms_counter++;
    if (can_task_100ms_counter >= 100)
    {
      can_task_100ms_counter = 0U;
      rk3588_send_heartbeat();
    }

    can_fifo_tx_process();

    vTaskDelayUntil(&lastWakeTime, taskPeriod);
  }
  /* USER CODE END Start_can_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
