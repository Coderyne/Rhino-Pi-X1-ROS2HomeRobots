//
// Created by RyneXie on 2026/2/14.
//

#include "power_task.h"

#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "stdbool.h"

bool enable_low_power = 1;
const float Battery_Low_Threshold = 24.0f; // 低电压阈值
float offset = 1.0f;

/* DMA buffer for ADC1 continuous circular transfers (cache-line aligned) */
static uint32_t adc_dma_buf __attribute__((aligned(32)));

void Power_Init(const bool DC24_0_Status, const bool DC24_1_Status, const bool DC5_Status)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

	HAL_GPIO_WritePin(DC24_0__OUTPUT_GPIO_Port, DC24_0__OUTPUT_Pin, DC24_0_Status ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC24_1__OUTPUT_GPIO_Port, DC24_1__OUTPUT_Pin, DC24_1_Status ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC5__OUTPUT_GPIO_Port, DC5__OUTPUT_Pin, DC5_Status ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

float GetBatteryVoltage(void)
{
	/* Invalidate D-Cache before reading DMA buffer (STM32H7 D-Cache enabled) */
	SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_dma_buf, sizeof(adc_dma_buf));
	uint32_t adc_value = adc_dma_buf;
	float voltage = ((float) (adc_value) / (float) (1 << 16)) * 3.3f * 11.0f;
	//printf("Battery Voltage: %.2fV\n", voltage);
	return voltage + offset;
}
void CheckLowPower(void)
{
	float voltage = GetBatteryVoltage();
	if(voltage < Battery_Low_Threshold)
	{
		for (int i = 0; i < 3; i++) {
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 30000);
			osDelay(100);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
			osDelay(100);
		}
	}
	else {
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	}
}

void StartPowerTask()
{
	HAL_ADC_Start_DMA(&hadc1, &adc_dma_buf, 1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	Power_Init(false,false, true);
	for(;;)
	{
		if(enable_low_power)
			CheckLowPower();
		osDelay(2000);
	}
}
