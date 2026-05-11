#include "gim6010_drv.h"

#include "fdcan.h"
#include "can_bsp.h"
#include "usart.h"
#include <string.h>
#include "chassisR_task.h"

//力矩限幅
#define MAX_TORQUE 100

extern chassis_t chassis_move;

//初始化电机对象
SW_MOTOR_DATA SWMotorList[4] = {
    { .motorID = 1 },
    { .motorID = 2 },
    { .motorID = 3 },
    { .motorID = 4 }
};


/**
 * @brief 设置GIM6010电机的控制模式和输入模式
 * @param hcan: 指向CAN_HandleTypeDef结构的指针
 * @param id: 电机ID
 * @param ctrlMode: 电机控制模式
 * @param inputMode: 电机输入模式
 */
void SWSetControllMode(hcan_t* hcan, uint8_t id, MW_CONTROL_MODE ctrlMode, MW_INPUT_MODE inputMode){

    uint8_t txBuff[8];
    memcpy(&txBuff[0], (uint8_t*)&ctrlMode, 4);
    memcpy(&txBuff[4], (uint8_t*)&inputMode, 4);

    uint16_t head = ((uint16_t)id << 5) | MW_SET_CONTROLLER_MODE_CMD;
    canx_send_data(hcan, head, txBuff, 8);
}

/**
 * @brief 设置电机轴状态
 * @param id 节点ID
 * @param state 目标状态
 */
void SWSetAxisState(hcan_t* hcan, uint8_t id, MW_MOTER_STATE state){
    uint8_t txBuff[8];
    memcpy(&txBuff[0], (uint8_t*)&state, 4);

    uint16_t head = ((uint16_t)id << 5) | MW_SET_AXIS_STATE_CMD;
    canx_send_data(hcan, head, txBuff, 8);

}

/**
 * @brief MIT模式控制指令发送
 * @param id 节点ID
 * @param mit MIT控制输入参数
 */
void SWMitControl(hcan_t* hcan, uint8_t id, double targetPos, double ffVel, double kp, double kd, double ffTorque) {

    uint8_t txBuff[8] = { 0 };  
    int16_t pos_int = (int16_t)((targetPos + 12.5f) * 65535.0f / 25.0f);
    int16_t vel_int = (int16_t)((ffVel + 65.0f) * 4095.0f / 130.0f);
    int16_t kp_int = (int16_t)(kp * 4095.0f / 500.0f);
    int16_t kd_int = (int16_t)(kd * 4095.0f / 5.0f);
    
    ffTorque = ffTorque * 8.0f; // 根据实际情况调整力矩放大倍数

    // 力矩补偿
    if (id == 0x01 || id == 0x02){
        ffTorque = ffTorque * 1.0f;
    }

    // 力矩限幅
    if (ffTorque > MAX_TORQUE) {
        ffTorque = MAX_TORQUE;
    } else if (ffTorque < -MAX_TORQUE) {
        ffTorque = -MAX_TORQUE;
    }
    int16_t t_int = (int16_t)((ffTorque + 50.0f) * 4095.0f / 100.0f);
    txBuff[0] = pos_int >> 8;
    txBuff[1] = pos_int & 0xFF;
    txBuff[2] = vel_int >> 4;
    txBuff[3] = ((vel_int & 0xF) << 4) + (kp_int >> 8);
    txBuff[4] = kp_int & 0xFF;
    txBuff[5] = kd_int >> 4;
    txBuff[6] = ((kd_int & 0xF) << 4) + (t_int >> 8);
    txBuff[7] = t_int & 0xFF;
    
    uint16_t head = ((uint16_t)id << 5) | MW_MIT_CONTROL_CMD;
    canx_send_data(hcan, head, txBuff, sizeof(txBuff));
}

/**
 * @brief 请求电机编码器计数
 * @param id 节点ID
 */
void SWGetEncoderCount(hcan_t* hcan, uint8_t id) {
    uint8_t txBuff[8] = {0};

    uint16_t head = ((uint16_t)id << 5) | MW_GET_ENCODER_COUNT_CMD;
    canx_send_data(hcan, head, txBuff, sizeof(txBuff));
}
/**
 * @brief 急停电机
 * @param id 节点ID
 */
void SWEstop(hcan_t* hcan, uint8_t id) {

    uint8_t txBuff[8] = {0};

    uint16_t head = ((uint16_t)id << 5) | MW_ESTOP_CMD;
    canx_send_data(hcan, head, txBuff, sizeof(txBuff));
}

/**
 * @brief 重启电机控制器
 * @param id 节点ID
 */
void SWReboot(hcan_t* hcan, uint8_t id) {

    uint8_t txBuff[8] = {0};

    uint16_t head = ((uint16_t)id << 5) | MW_REBOOT_CMD;
    canx_send_data(hcan, head, txBuff, sizeof(txBuff));
}

/**
 * @brief 清除电机错误状态
 * @param id 节点ID
 */
void SWClearErrors(hcan_t* hcan, uint8_t id) {

    uint8_t txBuff[8] = {0};

    uint16_t head = ((uint16_t)id << 5) | MW_CLEAR_ERRORS_CMD;
    canx_send_data(hcan, head, txBuff, sizeof(txBuff));
}

/**
 * @brief 禁用电机CAN通信(<=3.7版本切换到USB模式)
 * @param id 节点ID
 */
void SWDisableCAN(hcan_t* hcan, uint8_t id) {

    uint8_t txBuff[8] = {0};

    uint16_t head = ((uint16_t)id << 5) | MW_DISABLE_CAN_CMD;
    canx_send_data(hcan, head, txBuff, sizeof(txBuff));
}


void SWGetAllEncoderCount(hcan_t* hcan) {
    for (uint8_t id = 1; id <= 4; id++) {
        SWGetEncoderCount(hcan, id);
        osDelay(1);
    }
}
/**
 * @brief 电机MIT控制模式接收
 * @param Dst MW电机数据结构体
 * @param data CAN接收数据
 * @note cmd_id: 0x008
 */
uint8_t MWMitControlRec(uint8_t motorID, uint8_t *data) {
    chassis_move.SW_joint_motor[motorID - 1].motorMIT.targetPos = -(((float)(data[1] << 8 | data[2]) * 25.0f / 65535) - 12.5f);
    chassis_move.SW_joint_motor[motorID - 1].motorMIT.ffVel = ((float)(data[3] << 4 | data[4] >> 4) * 130.0f / 4095.0f) - 65.0f;
    chassis_move.SW_joint_motor[motorID - 1].motorMIT.ffTorque = ((float)(((data[4] & 0xF) << 8) | data[5]) * 100.0f / 4095.0f) - 50.0f;
    return data[0];
}

/**
 * @brief 电机编码器CPR数据接收
 * @param Dst MW电机数据结构体
 * @param data CAN接收数据
 * @note cmd_id: 0x00A
 */
void MWGetEncoderCountRec(uint8_t motorID, uint8_t *data) {
    memcpy(&(SWMotorList[motorID - 1].encoderData.shadowCount), &data[0], sizeof(SWMotorList[motorID - 1].encoderData.shadowCount));
    memcpy(&(SWMotorList[motorID - 1].encoderData.countInCPR), &data[4], sizeof(SWMotorList[motorID - 1].encoderData.countInCPR));
}

extern int motor1_cnt;
extern int motor2_cnt;
extern int motor3_cnt;
extern int motor4_cnt;


void GIM6010_fbdata(uint32_t id, uint8_t *data){
    uint8_t motorID = id >> 5;
    MW_CMD_ID cmdID = (MW_CMD_ID)(id & 0x1F);

    switch (cmdID)
    {
    case MW_MIT_CONTROL_CMD:
        MWMitControlRec(motorID, data);
        break;

    case MW_GET_ENCODER_COUNT_CMD:
        MWGetEncoderCountRec(motorID, data);
        break;
    
    default:
        break;
    }
}