/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "INS_task.h"
#include "chassisR_task.h"
#include "chassisL_task.h"
#include "observe_task.h"
#include "ps2_task.h"
#include "xbox_task.h"
#include "power_task.h"
#include "vofa_task.h"
#include "ros2_task.h"
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
osThreadId defaultTaskHandle;
osThreadId INS_TASKHandle;
osThreadId CHASSISR_TASKHandle;
osThreadId CHASSISL_TASKHandle;
osThreadId OBSERVE_TASKHandle;
osThreadId PS2_TASKHandle;
osThreadId XboxTASKHandle;
osThreadId POWERTASKHandle;
osThreadId VOFA_TASKHandle;
osThreadId ROS_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void INS_Task(void const * argument);
void ChassisR_Task(void const * argument);
void ChassisL_Task(void const * argument);
void OBSERVE_Task(void const * argument);
void PS2_Task(void const * argument);
void XboxTask(void const * argument);
void POWERTask(void const * argument);
void VOFATask(void const * argument);
void ROS_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 1024);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of CHASSISR_TASK */
  osThreadDef(CHASSISR_TASK, ChassisR_Task, osPriorityAboveNormal, 0, 1024);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  /* definition and creation of CHASSISL_TASK */
  osThreadDef(CHASSISL_TASK, ChassisL_Task, osPriorityAboveNormal, 0, 1024);
  CHASSISL_TASKHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

  /* definition and creation of OBSERVE_TASK */
  osThreadDef(OBSERVE_TASK, OBSERVE_Task, osPriorityHigh, 0, 1024);
  OBSERVE_TASKHandle = osThreadCreate(osThread(OBSERVE_TASK), NULL);

  /* definition and creation of PS2_TASK */
  osThreadDef(PS2_TASK, PS2_Task, osPriorityAboveNormal, 0, 128);
  PS2_TASKHandle = osThreadCreate(osThread(PS2_TASK), NULL);

  /* definition and creation of XboxTASK */
  osThreadDef(XboxTASK, XboxTask, osPriorityAboveNormal, 0, 512);
  XboxTASKHandle = osThreadCreate(osThread(XboxTASK), NULL);

  /* definition and creation of POWERTASK */
  osThreadDef(POWERTASK, POWERTask, osPriorityNormal, 0, 128);
  POWERTASKHandle = osThreadCreate(osThread(POWERTASK), NULL);

  /* definition and creation of VOFA_TASK */
  osThreadDef(VOFA_TASK, VOFATask, osPriorityAboveNormal, 0, 512);
  VOFA_TASKHandle = osThreadCreate(osThread(VOFA_TASK), NULL);

  /* definition and creation of ROS_TASK */
  osThreadDef(ROS_TASK, ROS_Task, osPriorityAboveNormal, 0, 1024);
  ROS_TASKHandle = osThreadCreate(osThread(ROS_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the ins_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */

  /* Infinite loop */
  osDelay(100);
  for(;;)
  {
    INS_task();		
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_ChassisR_Task */
/**
* @brief Function implementing the CHASSISR_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisR_Task */
void ChassisR_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisR_Task */
  /* Infinite loop */
  osDelay(500);
  for(;;)
  {
    ChassisR_task();
  }
  /* USER CODE END ChassisR_Task */
}

/* USER CODE BEGIN Header_ChassisL_Task */
/**
* @brief Function implementing the CHASSISL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisL_Task */
void ChassisL_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisL_Task */
  /* Infinite loop */
  osDelay(500);
  for(;;)
  {
    ChassisL_task();
  }
  /* USER CODE END ChassisL_Task */
}

/* USER CODE BEGIN Header_OBSERVE_Task */
/**
* @brief Function implementing the OBSERVE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OBSERVE_Task */
void OBSERVE_Task(void const * argument)
{
  /* USER CODE BEGIN OBSERVE_Task */
  /* Infinite loop */
  osDelay(500);
  for(;;)
  {
    Observe_task();
  }
  /* USER CODE END OBSERVE_Task */
}

/* USER CODE BEGIN Header_PS2_Task */
/**
* @brief Function implementing the PS2_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PS2_Task */
void PS2_Task(void const * argument)
{
  /* USER CODE BEGIN PS2_Task */
  /* Infinite loop */
  for(;;)
  {
    //pstwo_task();
  }
  /* USER CODE END PS2_Task */
}

/* USER CODE BEGIN Header_XboxTask */
/**
* @brief Function implementing the XboxTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_XboxTask */
void XboxTask(void const * argument)
{
  /* USER CODE BEGIN XboxTask */
  /* Infinite loop */
  osDelay(1000);
  for(;;)
  {
    Xbox_task();
  }
  /* USER CODE END XboxTask */
}

/* USER CODE BEGIN Header_POWERTask */
/**
* @brief Function implementing the POWERTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_POWERTask */
void POWERTask(void const * argument)
{
  /* USER CODE BEGIN POWERTask */
  /* Infinite loop */
  osDelay(250);
  for(;;)
  {
    StartPowerTask();
  }
  /* USER CODE END POWERTask */
}

/* USER CODE BEGIN Header_VOFATask */
/**
* @brief Function implementing the VOFA_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VOFATask */
void VOFATask(void const * argument)
{
  /* USER CODE BEGIN VOFATask */
  /* Infinite loop */
  for(;;)
  {
    StartVofaTask();
  }
  /* USER CODE END VOFATask */
}

/* USER CODE BEGIN Header_ROS_Task */
/**
* @brief Function implementing the ROS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ROS_Task */
__weak void ROS_Task(void const * argument)
{
  /* USER CODE BEGIN ROS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ROS_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
