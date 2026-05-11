#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"
#include "gim6010_drv.h"

#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f
#define ROLL_PID_KD 0.2f
#define ROLL_PID_MAX_OUT  200.0f
#define ROLL_PID_MAX_IOUT 0.0f

// IMU重映射后roll轴符号校正：若补偿方向反了，只需改这两个宏
#define BODY_ROLL_SIGN (-1.0f)
#define BODY_ROLL_GYRO_SIGN (1.0f)

#define TP_PID_KP 30.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 1.0f
#define TP_PID_MAX_OUT  15.0f
#define TP_PID_MAX_IOUT 0.0f

// 防劈叉补偿方向校正：若方向反了，仅需翻转该符号
#define BODY_TP_SIGN (1.0f)

#define TURN_PID_KP 1.6f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.45f
#define TURN_PID_MAX_OUT  0.6f//轮毂电机yaw补偿最大扭矩
#define TURN_PID_MAX_IOUT 0.0f

#define TURN_ERR_DEADBAND 0.015f
#define TURN_GYRO_DEADBAND 0.08f
#define TURN_GYRO_LPF_ALPHA 0.2f

typedef struct
{
  //Joint_Motor_t joint_motor[4];
  SW_MOTOR_DATA SW_joint_motor[4]; //ph4: ID2 ph1: ID3
  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
	float target_v;
	
	float turn_set;//期望yaw轴弧度
	float roll_set;	//期望roll轴弧度
	float roll_x;
	float phi_set;
	float theta_set;
	
	float leg_set;//期望腿长，单位是m
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
		
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
		
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志

	uint8_t jump_flag;//右腿跳跃标志
	uint8_t jump_flag2;//左腿跳跃标志
	
	uint8_t prejump_flag;//预跳跃标志
	uint8_t recover_flag;//一种情况下的倒地自起标志
	
} chassis_t;


extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr);
extern void ChassisR_task(void);
extern void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg);

#endif




