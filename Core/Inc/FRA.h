#ifndef __FRA_H
#define __FRA_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_BEGIN 0xFF
#define UART_END 0xEE

#define BOOL uint8_t
#define TRUE 1
#define FALSE 0

struct FRAParameterBlock{
    uint8_t FclkMHz;
    float32_t SwitchingFrequencyKHz;     // Fsw of the SMPS
    float32_t NorminalDutyCycle;         // D at steady state
    float32_t InjectAmplitude;           // Delta D of injected sinusoidal wave
};

void FRA_InjectTimerTriggeredHandler(TIM_HandleTypeDef*);
void FRA_Init(ADC_HandleTypeDef *, TIM_HandleTypeDef *, TIM_HandleTypeDef *, UART_HandleTypeDef *, struct FRAParameterBlock);
void FRA_Sweep(float32_t fstart, float32_t fend, uint8_t numDecade);
void FRA_Inject(float32_t sinefreq);
void FRA_GainShift(float32_t *pgain, float32_t *pshift);

void UartTransmitFrame(uint8_t*, uint16_t);

#endif