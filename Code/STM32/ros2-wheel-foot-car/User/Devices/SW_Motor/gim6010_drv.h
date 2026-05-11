#ifndef __GIM6010_DRV_H__
#define __GIM6010_DRV_H__

#include "fdcan.h"
#include "can_bsp.h"



/**
 * @brief MW电机MIT模式数据结构体
 */
typedef struct {
    double targetPos;
    double ffVel;
    double kp;
    double kd;
    double ffTorque;
} MW_MIT_CTRL_INPUT;

typedef struct {
    int32_t shadowCount;
    int32_t countInCPR;
} MW_ENCODER_DATA;

typedef struct 
{
    uint8_t motorID;

    MW_MIT_CTRL_INPUT motorMIT;          //!<@brief MIT模式下控制返回参数
    MW_ENCODER_DATA encoderData;         //!<@brief 编码器数据
} SW_MOTOR_DATA;

/**
 * @brief MW_CMD_ID指令
 */
typedef enum {
    MW_HEARTBEAT_CMD                  = 0x001,
    MW_ESTOP_CMD                      = 0x002,
    MW_GET_ERROR_CMD                  = 0x003,
    MW_RXSDO_CMD                      = 0x004,
    MW_TXSDO_CMD                      = 0x005,
    MW_SET_AXIS_NODE_ID_CMD           = 0x006,
    MW_SET_AXIS_STATE_CMD             = 0x007,
    MW_MIT_CONTROL_CMD                = 0x008,
    MW_GET_ENCODER_ESTIMATES_CMD      = 0x009,
    MW_GET_ENCODER_COUNT_CMD          = 0x00A,
    MW_SET_CONTROLLER_MODE_CMD        = 0x00B,
    MW_SET_INPUT_POS_CMD              = 0x00C,
    MW_SET_INPUT_VEL_CMD              = 0x00D,
    MW_SET_INPUT_TORQUE_CMD           = 0x00E,
    MW_SET_LIMITS_CMD                 = 0x00F,
    MW_START_ANTICOGGING_CMD          = 0x010,
    MW_SET_TRAJ_VEL_LIMIT_CMD         = 0x011,
    MW_SET_TRAJ_ACCEL_LIMIT_CMD       = 0x012,
    MW_SET_TRAJ_INERTIA_CMD           = 0x013,
    MW_GET_IQ_CMD                     = 0x014,
    MW_GET_SENSORLESS_ESTIMATES_CMD   = 0x015,
    MW_REBOOT_CMD                     = 0x016,
    MW_GET_BUS_VOLTAGE_CURRENT_CMD    = 0x017,
    MW_CLEAR_ERRORS_CMD               = 0x018,
    MW_SET_LINEAR_COUNT_CMD           = 0x019,
    MW_SET_POS_GAIN_CMD               = 0x01A,
    MW_SET_VEL_GAIN_CMD               = 0x01B,
    MW_GET_TORQUES_CMD                = 0x01C,
    MW_GET_POWERS_CMD                 = 0x01D,
    MW_DISABLE_CAN_CMD                = 0x01E,
    MW_SAVE_CONFIGURATION_CMD         = 0x01F
} MW_CMD_ID;

/**
 * @brief 电机控制模式
 */
typedef enum { 
    MW_VOLTAGE_CONTROL          = 0,
    MW_TORQUE_CONTROL           = 1,
    MW_VELOCITY_CONTROL         = 2,
    MW_POSITION_CONTROL         = 3
} MW_CONTROL_MODE;

/**
 * @brief 电机输入模式
 */
typedef enum { 
    MW_IDLE_INPUT               = 0,
    MW_DIRECT_CONTROL_INPUT     = 1,
    MW_RAMP_RATE_INPUT          = 2,
    MW_POSITION_FILTERING_INPUT = 3,
    MW_TRAPEZOIDAL_CURVE_INPUT  = 5,
    MW_TORQUE_RAMP_INPUT        = 6,
    MW_MIT_INPUT                = 9
} MW_INPUT_MODE;

/**
 * @brief 电机状态
 */
typedef enum { 
    MW_AXIS_STATE_UNDEFINED                            = 0x0,
    MW_AXIS_STATE_IDLE                                 = 0x1,
    MW_AXIS_STATE_STARTUP_SEQUENCE                     = 0x2,
    MW_AXIS_STATE_FULL_CALIBRATION_SEQUENCE            = 0x3,
    MW_AXIS_STATE_MOTOR_CALIBRATION                    = 0x4,
    MW_AXIS_STATE_ENCODER_INDEX_SEARCH                 = 0x6,
    MW_AXIS_STATE_ENCODER_OFFSET_CALIBRATION           = 0x7,
    MW_AXIS_STATE_CLOSED_LOOP_CONTROL                  = 0x8,
    MW_AXIS_STATE_LOCKIN_SPIN                          = 0x9,
    MW_AXIS_STATE_ENCODER_DIR_FIND                     = 0xA,
    MW_AXIS_STATE_HOMING                               = 0xB,
    MW_AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION    = 0xC,
    MW_AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION       = 0XD,
    MW_AXIS_STATE_ANTICOGGING_CALIBRATION              = 0XE
} MW_MOTER_STATE;

void SWSetControllMode(hcan_t* hcan, uint8_t id, MW_CONTROL_MODE ctrlMode, MW_INPUT_MODE inputMode);

void SWSetAxisState(hcan_t* hcan, uint8_t id, MW_MOTER_STATE state);

void SWMitControl(hcan_t* hcan, uint8_t id, double targetPos, double ffVel, double kp, double kd, double ffTorque);

void SWGetEncoderCount(hcan_t* hcan, uint8_t id);

void SWEstop(hcan_t* hcan, uint8_t id);

void SWReboot(hcan_t* hcan, uint8_t id);

void SWClearErrors(hcan_t* hcan, uint8_t id);

void SWDisableCAN(hcan_t* hcan, uint8_t id);

void SWGetAllEncoderCount(hcan_t* hcan);


// 反馈数据解析函数

void GIM6010_fbdata(uint32_t id, uint8_t *data);

uint8_t MWMitControlRec(uint8_t motorID, uint8_t *data);

#endif /* __GIM6010_DRV_H__ */