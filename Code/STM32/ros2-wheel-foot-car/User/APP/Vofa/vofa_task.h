#ifndef VOFA_TASK_H
#define VOFA_TASK_H

#include <stdint.h>

#include "usart.h"

#define VOFA_FRAME_TAIL_0 (0x00)
#define VOFA_FRAME_TAIL_1 (0x00)
#define VOFA_FRAME_TAIL_2 (0x80)
#define VOFA_FRAME_TAIL_3 (0x7F)

extern void Vofa_SendFloatArray(UART_HandleTypeDef *huart, const float *data, uint16_t count);
extern void Vofa_Send1(UART_HandleTypeDef *huart, float ch0);
extern void Vofa_Send2(UART_HandleTypeDef *huart, float ch0, float ch1);
extern void Vofa_Send3(UART_HandleTypeDef *huart, float ch0, float ch1, float ch2);
extern void Vofa_Send4(UART_HandleTypeDef *huart, float ch0, float ch1, float ch2, float ch3);
extern void Vofa_PrintString(UART_HandleTypeDef *huart, const char *str);

void StartVofaTask();
#endif /* VOFA_TASK_H */
