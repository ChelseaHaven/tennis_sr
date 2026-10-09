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
#include "shooter.h"
#include "oid.h"
#include "rk3588.h"
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
osThreadId shooter_TaskHandle;
osThreadId can_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_imu_Task(void const * argument);
void Start_chassis_Task(void const * argument);
void Start_shooter_Task(void const * argument);
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

  /* definition and creation of shooter_Task */
  osThreadDef(shooter_Task, Start_shooter_Task, osPriorityIdle, 0, 512);
  shooter_TaskHandle = osThreadCreate(osThread(shooter_Task), NULL);

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
  const TickType_t taskPeriod = pdMS_TO_TICKS(20U);



  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    //HAL_GPIO_TogglePin(GPIOA, LED1_Pin | LED2_Pin);

    imu_data_get();	

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

  // car_position_pid[0].Kp = wireless_test[6];   //电机A速度环比例系�??
  // car_position_pid[0].Ki = wireless_test[7];  //电机A速度环微分系�??
  // car_position_pid[0].Kd = wireless_test[8];
  
  // car_position_pid[1].Kp = wireless_test[6];   //电机B速度环比例系�??
  // car_position_pid[1].Ki = wireless_test[7];  //电机B速度环微分系�??对了
  // car_position_pid[2].Ki = wireless_test[7];  //电机C速度环微分系�??
  // car_position_pid[2].Kd = wireless_test[8];
  

      // R_Vel.X_target = 0.0f;
      // R_Vel.Y_target = 0.0f;
      // R_Vel.W_target = 0.0f;

      // R_Vel.X_target = (float)wireless_test[0];
      // R_Vel.Y_target = (float)wireless_test[1];
      // R_Vel.W_target = (float)wireless_test[2];
    //encoder_process();
    //chassis_control_process((uint8_t)(wireless_test[3] == 0.0f));

      // printf("%f,%f,%f\r\n", R_Vel.X_target, R_Vel.Y_target, R_Vel.W_target);

    //printf("%f,%f\r\n",R_Pos.W_global_target,imu.yaw_total);
  // Keep this high-rate telemetry trace disabled.
  // printf("%f,%f,%f,%f,%f,%f\r\n",R_Pos.X_body_error,R_Pos.Y_body_error,R_Pos.W_body_error,wireless_test[6],wireless_test[7],wireless_test[8]);
    vTaskDelayUntil(&lastWakeTime, taskPeriod);
  }
  /* USER CODE END Start_chassis_Task */
}

/* USER CODE BEGIN Header_Start_shooter_Task */
/**
* @brief Function implementing the shooter_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_shooter_Task */
void Start_shooter_Task(void const * argument)
{
  /* USER CODE BEGIN Start_shooter_Task */
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

// rk3588_get_data_value(RK3588_CMD_CHASSIS_ENABLE, &enable_data);

// printf("chassis_enable=%ld valid=%u fresh=%u age=%lu ms\r\n",
//        (long)enable_data.value,
//        (unsigned)enable_data.time_info.valid,
//        (unsigned)enable_data.time_info.fresh,
//        (unsigned long)enable_data.time_info.age_ms);



    // printf("%u,%u\r\n", pressure_data[0].value, pressure_data[1].value);

    // printf("%u,%u\r\n", pressure_data[0].value, pressure_data[1].value);
    // servo_set_angle(SERVO_1, wireless_test[4]);
    // servo_set_angle(SERVO_2, wireless_test[5]);
    // printf("%f,%f\r\n",wireless_test[4],wireless_test[5]);
    //printf("%f,%f,%f\r\n",imu.pitch,imu.roll,imu.yaw_total);
    vTaskDelayUntil(&lastWakeTime, taskPeriod);
  }
  /* USER CODE END Start_shooter_Task */
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
      //oid_refresh_freshness();
      //oid_process_speed_queries();  // 每 5 ms 查询一个节点的速度，6 个节点一轮约 30 ms
      //oid_process_heartbeats();     // 每 5 ms 查询一个节点的速度，6 个节点一轮约 30 ms
    }

    can_task_20ms_counter++;
    if (can_task_20ms_counter >= 20)
    {
      can_task_20ms_counter = 0U;
      //oid_process_queries();        //每 20 ms 查询一个节点的一项非速度数据；72 项轮询一轮约 1440 ms
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
