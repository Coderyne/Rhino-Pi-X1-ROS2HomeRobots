#include "xbox_task.h"
#include "chassisR_task.h"
#include <stdint.h>
#include <math.h>
#include "tim.h"
#include "vofa_task.h"
#include "gim6010_drv.h"
#include "ros2_task.h"

// 结构体声明
struct XboxController xboxController = {0};
uint8_t xbox_buff[36] = {0};
volatile uint8_t buff_len = 0;
volatile uint8_t rx_data_byte = 0;

unsigned long last_time = 0;
unsigned long cur_time = 0;

extern chassis_t chassis_move;
extern vmc_leg_t right;
extern vmc_leg_t left;
extern uint8_t  ros2_active;

volatile bool receiveSuccess = false; //接收完一帧数据包

#define XBOX_PACKET_LEN 11
#define XBOX_PACKET_HEADER0 0x07
#define XBOX_PACKET_HEADER1 0x21
#define XBOX_AXIS_DEADZONE 5

static void Xbox_data_process(chassis_t *chassis, float dt)
{
    static bool last_btn_a = false;
    static bool last_jump_combo = false;
    static bool last_btn_start = false;
    static bool last_leftStickY_zero = true;

    bool btn_a = xboxController.btnA;
    bool btn_b = xboxController.btnB;
    bool btn_start = xboxController.btnStart;
    bool jump_combo = xboxController.btnLeftBumper && xboxController.btnRightBumper;

    /* ROS2 authority guard — when ROS2 is active, Xbox yields all control
       writes to chassis_move to prevent racing. Comm timeout still runs. */
    if (!ros2_active) {

    // Start 按键边沿切换启动/关闭
    if (!last_btn_start && btn_start)
    {
        Vofa_PrintString(&huart10, "Start button pressed\r\n");
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
        
        if (chassis->start_flag == 0)
        {
            // 启动
            for (int i = 0; i < 3; i++) 
            {
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 3000);
			osDelay(100);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
			osDelay(100);
		    }

            //6010电机切换到USB模式 老版本的驱动板需要手动禁用CAN

            // SWDisableCAN(&hfdcan1,0x01);
            // osDelay(1);
            // SWDisableCAN(&hfdcan1,0x02);
            // osDelay(1);
            

            // 启动瞬间把偏航目标对齐到当前角度，避免历史偏差导致自旋
            chassis->turn_set = chassis->total_yaw;
            chassis->start_flag = 1;
        }
        else
        {
            chassis->start_flag = 0;
            chassis->recover_flag = 0;
            //StopAllMotors();
        }
    }
    last_btn_start = btn_start;

    // 倒地自起判定
    if (chassis->recover_flag == 0
        && ((chassis->myPithR < ((-3.1415926f) / 4.0f) && chassis->myPithR > ((-3.1415926f) / 2.0f))
            || (chassis->myPithR > (3.1415926f / 4.0f) && chassis->myPithR < (3.1415926f / 2.0f))))
    {
        chassis->recover_flag = 1;
        chassis->leg_set = 0.13f;
    }

    // 左右肩键同时按下触发跳跃（组合键上升沿）
    if (!last_jump_combo && jump_combo && chassis->jump_flag == 0 && chassis->jump_flag2 == 0)
    {
        //TODO: 跳跃逻辑
        // chassis->jump_flag = 1;
        // chassis->jump_flag2 = 1;
    }
    last_jump_combo = jump_combo;

    if (chassis->start_flag == 1)
    {
        // 左摇杆：前后速度 + 转向
        chassis->v_set = (xboxController.leftStickY) * 0.010f;
        
        chassis->x_set = chassis->x_set + chassis->v_set * dt;
        // if (xboxController.leftStickY == 0 && !last_leftStickY_zero) {
        //     chassis->x_set = chassis->x_filter; // 小于死区则不更新位置目标，保持在滤波后的位置，避免抖动
        //     last_leftStickY_zero = true;
        // } else {
        //     chassis->x_set = chassis->x_set + chassis->v_set * dt;
        //     last_leftStickY_zero = false;
        // }
        chassis->turn_set = chassis->turn_set + (-xboxController.leftStickX) * 0.00025f;

        // 右摇杆：横滚 + 腿长
        chassis->roll_set = chassis->roll_set + (-xboxController.rightStickX) * 0.00010f;
        mySaturate(&chassis->roll_set, -0.40f, 0.40f);

        chassis->leg_set = chassis->leg_set + (-xboxController.rightStickY) * 0.00003f;
        mySaturate(&chassis->leg_set, 0.011f, 0.22f);

        if (fabsf(chassis->last_leg_set - chassis->leg_set) > 0.0001f)
        {
            right.leg_flag = 1;
            left.leg_flag = 1;
        }
        chassis->last_leg_set = chassis->leg_set;
    }
    else
    {
        chassis->v_set = 0.0f;
        //chassis->x_set = chassis->x_filter;
        chassis->turn_set = chassis->total_yaw;
        chassis->leg_set = 0.13f;
        chassis->roll_set = -0.00f;
    }

    // B 按键复位 roll
    if (btn_a && !last_btn_a)
    {
        chassis->roll_set = -0.00f;
    }
    
    last_btn_a = btn_a;
    } /* !ros2_active — Xbox control writes blocked above */
}

void Xbox_task(void)
{		
    //osDelay(1000);
    Xbox_Init();
    for(;;)
    {
        // 通信丢失保护
        if (cur_time - last_time > 150){
            if (chassis_move.start_flag == 1) {
                memset(&xboxController, 0, sizeof(xboxController));
                //chassis_move.start_flag = 0;
                //StopAllMotors();
            }
        }
        
        if (receiveSuccess)
        {
            last_time = HAL_GetTick();
            Xbox_ProcessData();
            receiveSuccess = false;
        }
        cur_time = HAL_GetTick();

        if (chassis_move.start_flag == 1)
        {
            // //mit_ctrl2(&hfdcan2,0x5, 0.0f, 0.0f,0.0f, 0.0f,0.2f);
            // osDelay(1);
            // //mit_ctrl2(&hfdcan2,0x6, 0.0f, 0.0f,0.0f, 0.0f,0.2f);//左边边轮毂电机
            // osDelay(1);
            // SWMitControl(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // osDelay(1);
            // SWMitControl(&hfdcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // osDelay(1);
            // SWMitControl(&hfdcan1, 0x03, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // osDelay(1);
            // SWMitControl(&hfdcan1, 0x04, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // osDelay(1);
        }

        //SWGetAllEncoderCount(&hfdcan1);
        
        Xbox_data_process(&chassis_move, 0.01f);
        //HAL_UART_Transmit(&huart1, xbox_buff, 128, 100);
        osDelay(10);
    }
}

// 将0~255的输入转换为-100~100的输出，并进行简单的死区处理
static int8_t decode_axis(uint8_t v)
{
    int16_t raw = (int16_t)v - 128;
    if (raw < XBOX_AXIS_DEADZONE && raw > -XBOX_AXIS_DEADZONE)
        raw = 0;
    if (raw > 100)
        raw = 100;
    else if (raw < -100)
        raw = -100;
    return (int8_t)raw;
}

// 简单的校验和，所有字节相加取低8位
static uint8_t calc_checksum(const uint8_t *data)
{
    uint16_t sum = 0;
    for (int i = 0; i < 10; ++i)
    {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void StopAllMotors()
{
    // 停止
    disable_motor_mode(&hfdcan2,0x05,MIT_MODE);
    osDelay(1);
    disable_motor_mode(&hfdcan2,0x06,MIT_MODE);
    osDelay(1);
    SWMitControl(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    osDelay(1);
    SWMitControl(&hfdcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    osDelay(1);
    SWMitControl(&hfdcan1, 0x03, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    osDelay(1);
    SWMitControl(&hfdcan1, 0x04, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    osDelay(1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 3000);
	osDelay(500);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
}

void Xbox_Init(void)
{
    HAL_UART_Receive_IT(&huart7, (uint8_t *)&rx_data_byte, 1);
    Vofa_PrintString(&huart10, "Xbox controller initialized\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7)
    {
        HAL_UART_Receive_IT(&huart7, (uint8_t *)&rx_data_byte, 1);

        if (buff_len == 0) {
            if (rx_data_byte == XBOX_PACKET_HEADER0) {
                xbox_buff[buff_len++] = rx_data_byte;
            }else {
                buff_len = 0;
            }
        }else {
            xbox_buff[buff_len++] = rx_data_byte;
        }
        if (buff_len >= XBOX_PACKET_LEN) {
            buff_len = 0;
            receiveSuccess = true;
            //HAL_UART_Transmit(&huart1, xbox_buff, 11, 100);
        }
    }
}
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//     UART_PrintString(&huart10, "UART Rx Complete\r\n");
//     HAL_UART_Receive_IT(&huart7, &rx_data_buff, sizeof(rx_data_buff));
// }

// 解析接收到的数据包，更新xboxController状态
void Xbox_ProcessData()
{
    uint8_t *p = &xbox_buff[0];
    if (p[0] != XBOX_PACKET_HEADER0 || p[1] != XBOX_PACKET_HEADER1)
    {
        return;
    }

    // 校验和
    if (calc_checksum(p) != p[10])
    {
        return;
    }

    // 解析轴与扳机，范围 -100~100 / 0~100
    xboxController.leftStickX = (float)decode_axis(p[2]);
    xboxController.leftStickY = (float)decode_axis(p[3]);
    xboxController.rightStickX = (float)decode_axis(p[4]);
    xboxController.rightStickY = (float)decode_axis(p[5]);
    xboxController.leftTrigger = p[6] > 100 ? 100.0f : (float)p[6];
    xboxController.rightTrigger = p[7] > 100 ? 100.0f : (float)p[7];

    uint8_t btns = p[8];
    xboxController.btnA = (btns & 0x01u) != 0;
    xboxController.btnLeftBumper = (btns & 0x02u) != 0;
    xboxController.btnRightBumper = (btns & 0x04u) != 0;
    xboxController.btnStart = (btns & 0x08u) != 0; // 将Xbox键映射为Start

    // 发送端未提供其余按键，统一清零
    xboxController.btnB = false;
    xboxController.btnX = false;
    xboxController.btnY = false;
    xboxController.btnBack = false;
    xboxController.btnUp = false;
    xboxController.btnDown = false;
    xboxController.btnLeft = false;
    xboxController.btnRight = false;

}

// Getter 实现
float Xbox_GetLeftStickX(void) { return xboxController.leftStickX; }
float Xbox_GetLeftStickY(void) { return xboxController.leftStickY; }
float Xbox_GetRightStickX(void) { return xboxController.rightStickX; }
float Xbox_GetRightStickY(void) { return xboxController.rightStickY; }
float Xbox_GetLeftTrigger(void) { return xboxController.leftTrigger; }
float Xbox_GetRightTrigger(void) { return xboxController.rightTrigger; }

bool Xbox_GetBtnA(void) { return xboxController.btnA; }
bool Xbox_GetBtnB(void) { return xboxController.btnB; }
bool Xbox_GetBtnX(void) { return xboxController.btnX; }
bool Xbox_GetBtnY(void) { return xboxController.btnY; }
bool Xbox_GetBtnBack(void) { return xboxController.btnBack; }
bool Xbox_GetBtnStart(void) { return xboxController.btnStart; }
bool Xbox_GetBtnLeftBumper(void) { return xboxController.btnLeftBumper; }
bool Xbox_GetBtnRightBumper(void) { return xboxController.btnRightBumper; }
bool Xbox_GetBtnUp(void) { return xboxController.btnUp; }
bool Xbox_GetBtnDown(void) { return xboxController.btnDown; }
bool Xbox_GetBtnLeft(void) { return xboxController.btnLeft; }
bool Xbox_GetBtnRight(void) { return xboxController.btnRight; }
