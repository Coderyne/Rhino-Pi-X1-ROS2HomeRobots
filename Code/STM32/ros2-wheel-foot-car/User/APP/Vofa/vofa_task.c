//
// Created by RyneXie on 2026/2/15.
//

#include "vofa_task.h"

#include "cmsis_os.h"
#include <string.h>
#include "gim6010_drv.h"
#include "INS_task.h"
#include "chassisR_task.h"
#include "power_task.h"

#define VOFA_UART_DMA_TX_BUF_SIZE 256U
#define CPU_CACHE_LINE_SIZE 32U

extern struct Motor leftWheel;
extern struct Target target;

int motor1_cnt = 0;
int motor2_cnt = 0;
int motor3_cnt = 0;
int motor4_cnt = 0;

void Vofa_SendFloatArray(UART_HandleTypeDef *huart, const float *data, uint16_t count)
{
    static const uint8_t tail[4] = {
        VOFA_FRAME_TAIL_0,
        VOFA_FRAME_TAIL_1,
        VOFA_FRAME_TAIL_2,
        VOFA_FRAME_TAIL_3
    };
    static uint8_t txBuf[VOFA_UART_DMA_TX_BUF_SIZE];
    size_t payloadLen = 0;
    size_t totalLen = 0;

    if (huart == NULL || data == NULL || count == 0)
    {
        return;
    }

    if (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
    {
        return;
    }

    payloadLen = (size_t)count * sizeof(float);
    totalLen = payloadLen + sizeof(tail);
    if (totalLen == 0 || totalLen >= VOFA_UART_DMA_TX_BUF_SIZE)
    {
        return;
    }

    memcpy(txBuf, data, payloadLen);
    memcpy(txBuf + payloadLen, tail, sizeof(tail));

    /* Ensure DMA reads the latest data from memory when D-Cache is enabled. */
    {
        uint32_t start = (uint32_t)txBuf & ~(CPU_CACHE_LINE_SIZE - 1U);
        uint32_t end = (uint32_t)txBuf + (uint32_t)totalLen;
        uint32_t size = (end - start + (CPU_CACHE_LINE_SIZE - 1U)) & ~(CPU_CACHE_LINE_SIZE - 1U);
        SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)size);
    }

    HAL_UART_Transmit_DMA(huart, txBuf, (uint16_t)totalLen);
}

void Vofa_Send1(UART_HandleTypeDef *huart, float ch0)
{
    float data[1] = {ch0};
    Vofa_SendFloatArray(huart, data, 1);
}

void Vofa_Send2(UART_HandleTypeDef *huart, float ch0, float ch1)
{
    float data[2] = {ch0, ch1};
    Vofa_SendFloatArray(huart, data, 2);
}

void Vofa_Send3(UART_HandleTypeDef *huart, float ch0, float ch1, float ch2)
{
    float data[3] = {ch0, ch1, ch2};
    Vofa_SendFloatArray(huart, data, 3);
}

void Vofa_Send4(UART_HandleTypeDef *huart, float ch0, float ch1, float ch2, float ch3)
{
    float data[4] = {ch0, ch1, ch2, ch3};
    Vofa_SendFloatArray(huart, data, 4);
}

void Vofa_PrintString(UART_HandleTypeDef *huart, const char *str)
{
    static uint8_t txBuf[VOFA_UART_DMA_TX_BUF_SIZE];
    size_t len = 0;

    if (huart == NULL || str == NULL)
    {
        return;
    }

    if (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
    {
        return;
    }

    len = strlen(str);
    if (len == 0 || len >= VOFA_UART_DMA_TX_BUF_SIZE)
    {
        return;
    }

    memcpy(txBuf, str, len);
    txBuf[len] = '\0';

    /* Ensure DMA reads the latest data from memory when D-Cache is enabled. */
    {
        uint32_t start = (uint32_t)txBuf & ~(CPU_CACHE_LINE_SIZE - 1U);
        uint32_t end = (uint32_t)txBuf + (uint32_t)len;
        uint32_t size = (end - start + (CPU_CACHE_LINE_SIZE - 1U)) & ~(CPU_CACHE_LINE_SIZE - 1U);
        SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)size);
    }

    HAL_UART_Transmit_DMA(huart, txBuf, (uint16_t)len);
}

void StartVofaTask()
{
    /* USER CODE BEGIN StartVofaTask */
    /* Infinite loop */
    for(;;)
    {
        // 定时发送任务

        //Vofa_SendFloatArray(&huart10, data, 9, 1000);
        //UART_PrintString(&huart10, "Sys OK\r\n", 100);
        UpdateMotorCNT();
        osDelay(10);
    }
    /* USER CODE END StartVofaTask */
}
extern SW_MOTOR_DATA SWMotorList[4];
extern INS_t INS;

extern chassis_t chassis_move;
extern vmc_leg_t left;
extern vmc_leg_t right;

extern uint8_t  ros2_active;

void UpdateMotorCNT()
{
    float wheel_motorR_T = chassis_move.wheel_motor[0].wheel_T;
    float wheel_motorL_T = chassis_move.wheel_motor[1].wheel_T;
    float joint_motorL1_T = chassis_move.SW_joint_motor[0].motorMIT.ffTorque;
    float joint_motorL2_T = chassis_move.SW_joint_motor[3].motorMIT.ffTorque;
    float joint_motorR1_T = chassis_move.SW_joint_motor[1].motorMIT.ffTorque;
    float joint_motorR2_T = chassis_move.SW_joint_motor[2].motorMIT.ffTorque;
    float left_leg_L0 = left.L0;
    float right_leg_L0 = right.L0;
    float left_leg_phi0 = left.phi0;
    float right_leg_phi0 = chassis_move.v_set;
    float leg_set = ros2_active;

    //IMU Data转换为角度制
    float roll = INS.Roll * 180.0f / 3.1415926f;
    float pitch = INS.Pitch * 180.0f / 3.1415926f;
    float yaw = INS.Yaw * 180.0f / 3.1415926f;
    
    Vofa_SendFloatArray(&huart10, (const float[]){wheel_motorR_T, wheel_motorL_T, joint_motorL1_T, joint_motorL2_T, joint_motorR1_T, joint_motorR2_T, left_leg_L0, right_leg_L0, left_leg_phi0, right_leg_phi0, leg_set, roll, pitch, yaw}, 14);

    motor1_cnt = 0;
    motor2_cnt = 0;
    motor3_cnt = 0;
    motor4_cnt = 0;
}