/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can2总线上
	*						 从底盘上往下看，左上角的DM4310发送id为8、接收id为4，
	*						 左下角的DM4310发送id为6、接收id为3，
	*						 左边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisL_task.h"
#include "fdcan.h"
#include "VMC_calc.h"

#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "usart.h"

vmc_leg_t left;

float LQR_K_L[12]={  
  -6.7217  , -0.6637,   -0.6191,   -1.0619 ,  -0.8307,   -0.0776,
    2.2565 ,   0.4553 ,   0.5788 ,   0.9624 ,  15.9639 ,   0.9901

};

extern float Poly_Coefficient[12][4];

extern chassis_t chassis_move;

float jump_time2;
extern float jump_time;	

PidTypeDef LegL_Pid;	

extern INS_t INS;
uint32_t CHASSL_TIME=1;	

void ChassisL_task(void)
{
	uint8_t left_motor_started = 0;

  while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
  ChassisL_init(&chassis_move,&left,&LegL_Pid);//初始化左边两个关节电机和左边轮毂电机的id和控制模式、初始化腿部
		
	while(1)
	{	
		chassisL_feedback_update(&chassis_move,&left,&INS);//更新数据
		
		chassisL_control_loop(&chassis_move,&left,&INS,LQR_K_L,&LegL_Pid);//控制计算

		if (chassis_move.start_flag==1 && left_motor_started==0)
		{
			LeftMotorStart();
			left_motor_started = 1;
		}
		else if (chassis_move.start_flag==0 && left_motor_started==1)
		{
			LeftMotorStop();
			left_motor_started = 0;
		}

    if(chassis_move.start_flag==1)	
		{
			// mit_ctrl(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[1]);//left.torque_set[1]
			// osDelay(CHASSL_TIME);
			// mit_ctrl(&hfdcan1,0x04, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[0]);
			// osDelay(CHASSL_TIME);
			SWMitControl(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, -left.torque_set[1]);//左边前髋关节电机
			osDelay(CHASSL_TIME);
			SWMitControl(&hfdcan1, 0x04, 0.0f, 0.0f, 0.0f, 0.0f, -left.torque_set[0]);//左边后髋关节电机
			osDelay(CHASSL_TIME);
			mit_ctrl2(&hfdcan2,0x06, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[1].wheel_T);//左边边轮毂电机
			osDelay(CHASSL_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			osDelay(CHASSL_TIME);
		}
	}
}

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legl)
{
  const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	// joint_motor_init(&chassis->joint_motor[2],6,MIT_MODE);//发送id为6
	// joint_motor_init(&chassis->joint_motor[3],8,MIT_MODE);//发送id为8
	chassis->SW_joint_motor[0].motorID=1;//左边前髋关节电机
	chassis->SW_joint_motor[3].motorID=4;//左边后髋关节电机
	
	wheel_motor_init(&chassis->wheel_motor[1],6,MIT_MODE);//发送id为6
	
	VMC_init(vmc);//给杆长赋值
	
	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
}

//左侧电机使能函数，启动时调用一次
void LeftMotorStart(void)
{
	//轮毂电机
	enable_motor_mode(&hfdcan2,0x06,MIT_MODE);
	osDelay(1);

	//关节电机
	SWClearErrors(&hfdcan1, 0x01);
	osDelay(1);
	SWClearErrors(&hfdcan1, 0x04);
	osDelay(10);
	SWSetControllMode(&hfdcan1, 0x01, MW_TORQUE_CONTROL, MW_MIT_INPUT);
    osDelay(1);
    SWSetControllMode(&hfdcan1, 0x04, MW_TORQUE_CONTROL, MW_MIT_INPUT);
    osDelay(1);
	SWSetAxisState(&hfdcan1, 0x01, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    osDelay(1);
    SWSetAxisState(&hfdcan1, 0x04, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    osDelay(1);

	Vofa_PrintString(&huart10,"Left Motor Started\r\n");
}

//急停左侧电机
void LeftMotorStop(void)
{
	// disable_motor_mode(&hfdcan2,0x06,MIT_MODE);
	// osDelay(CHASSL_TIME);

	SWEstop(&hfdcan1, 0x01);
	osDelay(CHASSL_TIME);
	SWEstop(&hfdcan1, 0x04);
	osDelay(CHASSL_TIME);
	mit_ctrl2(&hfdcan2,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//左边边轮毂电机
	osDelay(CHASSL_TIME);
	mit_ctrl2(&hfdcan2,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//左边边轮毂电机
	osDelay(CHASSL_TIME);
}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
  vmc->phi1=pi/2.0f+chassis->SW_joint_motor[3].motorMIT.targetPos;
	vmc->phi4=pi/2.0f+chassis->SW_joint_motor[0].motorMIT.targetPos;
		
	chassis->myPithL=-BODY_PITCH_SIGN*ins->Pitch;
	chassis->myPithGyroL=-BODY_PITCH_GYRO_SIGN*ins->Gyro[1];
	
}


extern uint8_t right_flag;
uint8_t left_flag;
extern float mg;
extern float jump_time;
void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
	}
		
	  chassis->wheel_motor[1].wheel_T=(LQR_K[0]*(vmcl->theta-0.0f)
																		+LQR_K[1]*(vmcl->d_theta-0.0f)
																		+LQR_K[2]*(chassis->x_set-chassis->x_filter)
																		+LQR_K[3]*(0.4f*chassis->v_set-chassis->v_filter)
																		+LQR_K[4]*(chassis->myPithL-(-0.04f))
																		+LQR_K[5]*(chassis->myPithGyroL-0.0f));
		
		//右边髋关节输出力矩				
		vmcl->Tp=(LQR_K[6]*(vmcl->theta-0.0f+chassis->theta_set)
						+LQR_K[7]*(vmcl->d_theta-0.0f)
						+LQR_K[8]*(chassis->x_set-chassis->x_filter)
						+LQR_K[9]*(0.4f*chassis->v_set-chassis->v_filter)
						+LQR_K[10]*(chassis->myPithL-(-0.04f))
						+LQR_K[11]*(chassis->myPithGyroL-0.0f));
	
		vmcl->Tp=vmcl->Tp+chassis->leg_tp;//髋关节输出力矩
		
		//TODO
		chassis->wheel_motor[1].wheel_T= chassis->wheel_motor[1].wheel_T-chassis->turn_T;	//轮毂电机输出力矩

	mySaturate(&chassis->wheel_motor[1].wheel_T,-2.0f,2.0f);

	if(chassis->jump_flag2==1||chassis->jump_flag2==2||chassis->jump_flag2==3)
	{
    if(chassis->jump_flag2==1)
		{//压缩阶段
		 vmcl->F0=mg/arm_cos_f32(vmcl->theta)+PID_Calc(leg,vmcl->L0,0.08f);//前馈+pd

		 if(vmcl->L0<0.10f)
		 {
		  jump_time2++;
		 }
		 if(jump_time2>=10&&jump_time>=10)
		 {  
			 jump_time2=0;
			 jump_time=0;
			 chassis->jump_flag2=2;
			 chassis->jump_flag=2;//压缩完毕进入上升加速阶段
		 }			 
		}
		else if(chassis->jump_flag2==2)
		{//上升加速阶段
		   vmcl->F0=mg/arm_cos_f32(vmcl->theta)+PID_Calc(leg,vmcl->L0,0.40f);//前馈+pd
			
			 if(vmcl->L0>0.18f)
			 {
				jump_time2++;
			 }
			 if(jump_time2>=2&&jump_time>=2)
			 {  
				 jump_time2=0;
				  jump_time=0;
				 chassis->jump_flag2=3;
				 chassis->jump_flag=3;//上升完毕进入缩腿阶段
			 }
		}
		else if(chassis->jump_flag2==3)
		{//缩腿阶段
		  vmcl->F0=PID_Calc(leg,vmcl->L0,0.10f);//pd
			chassis->theta_set=0.0f;
		  if(vmcl->L0<0.15f)
		  {
			 jump_time2++;
		  }
		  if(jump_time2>=3&&jump_time>=3)
		  { 
				 jump_time2=0;
				 jump_time=0;
				 chassis->leg_set=0.10f;
				 chassis->last_leg_set=0.10f;
				 chassis->jump_flag2=0;
				 chassis->jump_flag=0;
		  }
		}
	}	
	else
	{
		vmcl->F0=mg/arm_cos_f32(vmcl->theta)+PID_Calc(leg,vmcl->L0,chassis->leg_set);//前馈+pd
	}
	
	 left_flag=ground_detectionL(vmcl,ins);//右腿离地检测
	 
	 if(chassis->recover_flag==0)		
	 {//倒地自起不需要检测是否离地	 
		if((right_flag==1&&left_flag==1&&vmcl->leg_flag==0&&chassis->jump_flag2!=1&&chassis->jump_flag!=1&&chassis->jump_flag2!=2&&chassis->jump_flag!=2)
			||chassis->jump_flag2==3)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			//排除跳跃的压缩阶段和跳跃的缩腿阶段
				chassis->wheel_motor[1].wheel_T=0.0f;
				vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);

				chassis->x_filter=0.0f;
				//chassis->x_set=chassis->x_filter;

				vmcl->Tp=vmcl->Tp+chassis->leg_tp;	
		}
		else
		{//没有离地
			vmcl->leg_flag=0;//置为0
	
			if(chassis->jump_flag2==0)
			{//不跳跃的时候需要roll轴补偿
				
				vmcl->F0=vmcl->F0-chassis->roll_f0;//roll轴补偿取反然后加上去	
				
			}
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcl->Tp=0.0f;
		 vmcl->F0=0.0f;
	 }

	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅 

	VMC_calc_2(vmcl);//计算期望的关节输出力矩
	
	if(chassis->jump_flag2==1||chassis->jump_flag2==2||chassis->jump_flag2==3)
	{//跳跃的时候需要更大扭矩
		mySaturate(&vmcl->torque_set[1],-6.0f,6.0f);	
		mySaturate(&vmcl->torque_set[0],-6.0f,6.0f);	
	}	
	else
	{//不跳跃的时候最大为额定扭矩
    mySaturate(&vmcl->torque_set[1],-3.0f,3.0f);	
		mySaturate(&vmcl->torque_set[0],-3.0f,3.0f);	
	}		
	
}


