#ifndef __FRA_H
#define __FRA_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"

void FRA_Init(ADC_HandleTypeDef *, TIM_HandleTypeDef *, TIM_HandleTypeDef *, UART_HandleTypeDef *);
void InjectTimerTriggeredHandler();

#endif