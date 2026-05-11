//
// Created by RyneXie on 2026/2/14.
//
#include "stdbool.h"

#ifndef POWER_TASK_H
#define POWER_TASK_H

void Power_Init(const bool DC24_0_Status, const bool DC24_1_Status, const bool DC5_Status);

float GetBatteryVoltage(void);

void StartPowerTask();

#endif /* POWER_TASK_H */
