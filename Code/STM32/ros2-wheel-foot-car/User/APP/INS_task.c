/**
  *********************************************************************
  * @file      ins_task.c/h
  * @brief     该任务是用mahony方法获取机体姿态
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
	
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "mahony_filter.h"
#include "MahonyAHRS.h"

INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;

uint8_t attitude_flag=1;
#define correct_Time_define 1000    //上电去0飘 1000次取平均
uint32_t correct_times=0;

float gyro_correct[3]={0};

#define cheat TRUE  //作弊模式 去掉较小的gyro值
const int MahonyMethod = 0;//0是原始mahony，1是改进的mahony

void INS_Init(void)
{ 
	 mahony_init(&mahony,1.0f,0.0001f,0.001f);
   INS.AccelLPF = 0.0089f;
}

void INS_task(void)
{
	if (MahonyMethod == 0){
		INS_Init();
	}else if (MahonyMethod == 1){
		//改进的mahony算法的初始化
    	BMI088_Read(&BMI088);
		osDelay(10);
		Mahony_Init(500);  //mahony姿态解算初始化
		MahonyAHRSinit(-BMI088.Accel[1],BMI088.Accel[0],BMI088.Accel[2],0,0,0);
	}
	 
	while(1)
	{  
    	BMI088_Read(&BMI088);

		if (attitude_flag==1)
		{
			gyro_correct[0] += BMI088.Gyro[0];
			gyro_correct[1] += BMI088.Gyro[1];
			gyro_correct[2] += BMI088.Gyro[2];
			correct_times++;
			if (correct_times >= correct_Time_define)
			{
				gyro_correct[0] /= correct_Time_define;
				gyro_correct[1] /= correct_Time_define;
				gyro_correct[2] /= correct_Time_define;
				attitude_flag=0;
			}
		}
		else
		{

			BMI088.Gyro[0] -= gyro_correct[0];
			BMI088.Gyro[1] -= gyro_correct[1];
			BMI088.Gyro[2] -= gyro_correct[2];
		
			//作弊 可以让yaw很稳定 去掉比较小的值0.003f); 0.02->yaw:0°/h;0.01->yaw:5°/h
			#if cheat              
					if(fabsf(BMI088.Gyro[Z])<0.01f)
						BMI088.Gyro[Z]=0;
			#endif
			
			//如果修改了IMU摆放方向，需要修改下面的代码，保证加速度计和陀螺仪数据正确对应到机体坐标系的x、y、z轴
	    	INS.Accel[X] = BMI088.Accel[Y];
	    	INS.Accel[Y] = -BMI088.Accel[X];
	    	INS.Accel[Z] = BMI088.Accel[Z];
			Accel.x=BMI088.Accel[1];
			Accel.y=-BMI088.Accel[0];
			Accel.z=BMI088.Accel[2];
	    	INS.Gyro[X] = BMI088.Gyro[Y];
	    	INS.Gyro[Y] = -BMI088.Gyro[X];
	    	INS.Gyro[Z] = BMI088.Gyro[Z];
	  		Gyro.x=BMI088.Gyro[1];
			Gyro.y=-BMI088.Gyro[0];
			Gyro.z=BMI088.Gyro[2];

			if (MahonyMethod == 0){
				ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
				mahony.dt = ins_dt;

				mahony_input(&mahony,Gyro,Accel);
				mahony_update(&mahony);
				mahony_output(&mahony);
			}else if (MahonyMethod == 1){
				Mahony_update(&mahony,BMI088.Gyro[1],-BMI088.Gyro[0],BMI088.Gyro[2],BMI088.Accel[1],-BMI088.Accel[0],BMI088.Accel[2],0,0,0);
				Mahony_computeAngles(&mahony);
			}


		  	RotationMatrix_update(&mahony);

			INS.q[0]=mahony.q0;
			INS.q[1]=mahony.q1;
			INS.q[2]=mahony.q2;
			INS.q[3]=mahony.q3;
		
	      	// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
			float gravity_b[3];
	    	EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
	    	for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
	    	{
	    	  INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) 
																+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt); 
	//				INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt) 
	//															+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);			
				}
			BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

			//死区处理
			if(fabsf(INS.MotionAccel_n[0])<0.02f)
			{
			  INS.MotionAccel_n[0]=0.0f;	//x轴
			}
			if(fabsf(INS.MotionAccel_n[1])<0.02f)
			{
			  INS.MotionAccel_n[1]=0.0f;	//y轴
			}
			if(fabsf(INS.MotionAccel_n[2])<0.04f)
			{
			  INS.MotionAccel_n[2]=0.0f;//z轴
				stop_time++;
			}
	//		if(stop_time>10)
	//		{//静止10ms
	//		  stop_time=0;
	//			INS.v_n=0.0f;
	//		}
		
			if(ins_time>3000.0f)
			{
				INS.v_n=INS.v_n+INS.MotionAccel_n[1]*0.001f;
			  	INS.x_n=INS.x_n+INS.v_n*0.001f;
				INS.ins_flag=1;//四元数基本收敛，加速度也基本收敛，可以开始底盘任务
				// 获取最终数据
	      		INS.Roll=mahony.roll;
			  INS.Pitch=mahony.pitch;
			  INS.Yaw=mahony.yaw;
			
			//INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;

				if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
				{
						INS.YawRoundCount--;
				}
				else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
				{
						INS.YawRoundCount++;
				}
				INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
				INS.YawAngleLast = INS.Yaw;
			}
			else
			{
			 ins_time++;
			}

	    osDelay(1);
		}
	}
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}




