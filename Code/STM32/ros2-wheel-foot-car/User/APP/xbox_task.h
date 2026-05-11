#ifndef XBOX_TASK_H
#define XBOX_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"

typedef struct
{
  bool buttonA; //按键A
  bool buttonB; //按键B
  bool buttonLeftBumper; //左肩键
  bool buttonRightBumper; //右肩键
	
	int16_t lx;  //左边遥感X轴方向的模拟量
	int16_t ly;//左边遥感Y轴方向的模拟量 
	int16_t rx;//右边遥感X轴方向的模拟量 
	int16_t ry;//右边遥感Y轴方向的模拟量  
	
}xboxData_t;

struct XboxController {
    float leftStickX;
    float leftStickY;
    float rightStickX;
    float rightStickY;
    float leftTrigger;
    float rightTrigger;
    bool btnA;
    bool btnB;
    bool btnX;
    bool btnY;
    bool btnBack;
    bool btnStart;
    bool btnLeftBumper;
    bool btnRightBumper;
    bool btnUp;
    bool btnDown;
    bool btnLeft;
    bool btnRight;
};

extern struct XboxController xboxController;

void Xbox_task(void);
void Xbox_Init(void);
void Xbox_ProcessData();
    // 摇杆与扳机
    float Xbox_GetLeftStickX(void);
    float Xbox_GetLeftStickY(void);
    float Xbox_GetRightStickX(void);
    float Xbox_GetRightStickY(void);
    float Xbox_GetLeftTrigger(void);
    float Xbox_GetRightTrigger(void);

    // 按键
    bool Xbox_GetBtnA(void);
    bool Xbox_GetBtnB(void);
    bool Xbox_GetBtnX(void);
    bool Xbox_GetBtnY(void);
    bool Xbox_GetBtnBack(void);
    bool Xbox_GetBtnStart(void);
    bool Xbox_GetBtnLeftBumper(void);
    bool Xbox_GetBtnRightBumper(void);
    bool Xbox_GetBtnUp(void);
    bool Xbox_GetBtnDown(void);
    bool Xbox_GetBtnLeft(void);
    bool Xbox_GetBtnRight(void);
#endif /* XBOX_TASK_H */

