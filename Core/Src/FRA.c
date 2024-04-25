#include "FRA.h"
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "arm_const_structs.h"

float32_t NorminalDutyCycle;         // D at steady state
int16_t NorminalCRR;                 // CRR1 responding to D
float32_t InjectAmplitude;           // Delta D of injected sinusoidal wave
float32_t SwitchingFrequencyKHz;     // Fsw of the SMPS
int16_t PwmARR;                      // ARR (timer period) responding to fsw
float32_t CurrentInjectFrequencyKHz; // Current injection frequency (injection timer frequency) during a sweep

float32_t SineRef[8] = {1.0000, 0.7071, 0.0000, -0.7071, -1.0000, -0.7071, -0.0000, 0.7071}; // Norminal cos wave
int16_t SineInt[8];                                                                          // Actual value injected into the CRR of the PWM timer
uint16_t ResponseBuf[512];                                                                   // Buffer for ADC, stores the response of SMPS
int16_t InjectBuf[512];                                                                      // Buffer for injection !! TODO: the size can be reduced
float32_t FFTComplexBuf[1024];                                                               // Odd indexes stores the real part, even store the imag part
float32_t FFTMagBuf[512];

ADC_HandleTypeDef *pAdc;
TIM_HandleTypeDef *pPwmTimer;
TIM_HandleTypeDef *pInjectTimer;
UART_HandleTypeDef *pUart;

uint8_t injectSineIndex = 0;
uint16_t adcReadIndex = 0;

void FRA_Init(ADC_HandleTypeDef *padc, TIM_HandleTypeDef *ppwm, TIM_HandleTypeDef *pinj, UART_HandleTypeDef *puart)
{
    // Accept handles
    pAdc = padc;
    pPwmTimer = ppwm;
    pInjectTimer = pinj;
    pUart = puart;

    // Temporarily used to initialize
    NorminalDutyCycle = 0.3;
    InjectAmplitude = 0.05;
    SwitchingFrequencyKHz = 100.0;
    PwmARR = 168000 / SwitchingFrequencyKHz - 1;
    CurrentInjectFrequencyKHz = 10.0;
    pInjectTimer->Instance->ARR = (84000 / CurrentInjectFrequencyKHz) - 1; // TIM3's clock source is APB1, 84MHz

    for (int i = 0; i < 8; ++i)
        SineInt[i] = SineRef[i] * PwmARR * InjectAmplitude;
    NorminalCRR = 1680 * NorminalDutyCycle;
}

float32_t max_mag = 0.0;
float32_t max_pha = 0.0;
float32_t max_freq = 0;
uint32_t max_pos = 0;
void FRA_InjectTimerTriggeredHandler()
{
    // Inject sinusoidal wave into duty cycle
    injectSineIndex++;
    if (injectSineIndex >= 8)
        injectSineIndex = 0;
    pPwmTimer->Instance->CCR1 = NorminalCRR + SineInt[injectSineIndex];

    // Read response and excitation simutaneously to calculate phase difference
    adcReadIndex++;
    if (adcReadIndex >= 512)
        adcReadIndex = 0;
    HAL_ADC_Start(pAdc);
    if (HAL_OK == HAL_ADC_PollForConversion(pAdc, 50))
    {
        ResponseBuf[adcReadIndex] = HAL_ADC_GetValue(pAdc);
        InjectBuf[adcReadIndex] = SineInt[injectSineIndex];
    }

    // Read 512 pts, then do an FFT
    if (adcReadIndex == 511)
    {
        HAL_TIM_Base_Stop_IT(pInjectTimer);
        // HAL_UART_Transmit_DMA(pUart, (uint8_t*)ResponseBuf, (uint16_t)512);

        // Laod FFT buffer. Clear the imagine part.
        for (int i = 0; i < 512; ++i)
        {
            FFTComplexBuf[i * 2] = 3.3 * (float32_t)ResponseBuf[i] / 4095;
            FFTComplexBuf[i * 2 + 1] = 0;
        }

        arm_cfft_f32(&arm_cfft_sR_f32_len512, FFTComplexBuf, 0, 1);     // FFT
        arm_cmplx_mag_f32(FFTComplexBuf, FFTMagBuf, 256);               // Nyquist theorm, only half of the spectrum is effective
        FFTMagBuf[0] = 0;

        arm_max_f32(FFTMagBuf, 256, &max_mag, &max_pos);                // Nyquist theorm, only half of the spectrum is effective
        arm_atan2_f32(FFTComplexBuf[max_pos * 2 + 1], FFTComplexBuf[max_pos * 2], &max_pha);

        // Adjust magnitude and phase.
        max_mag = (max_mag) * 2 / 512;
        max_freq = (float32_t)(max_pos + 1) * (CurrentInjectFrequencyKHz / 512);
        max_pha = max_pha * 180 / 3.14;

        printf("Resp Max mag = %fV at %f kHz, %fdeg\n", max_mag, max_freq, max_pha);

        // HAL_UART_Transmit_DMA(pUart, (uint8_t *)ResponseBuf, 8192);
        // HAL_Delay(100);

        for (int i = 0; i < 512; ++i)
        {
            FFTComplexBuf[i * 2] = (float32_t)InjectBuf[i] / PwmARR;
            FFTComplexBuf[i * 2 + 1] = 0;
        }
        arm_cfft_f32(&arm_cfft_sR_f32_len512, FFTComplexBuf, 0, 1);
        arm_cmplx_mag_f32(FFTComplexBuf, FFTMagBuf, 256);
        FFTMagBuf[0] = 0;

        arm_max_f32(FFTMagBuf, 256, &max_mag, &max_pos);
        arm_atan2_f32(FFTComplexBuf[max_pos * 2 + 1], FFTComplexBuf[max_pos * 2], &max_pha);

        max_mag = max_mag * 2 / 512;
        max_freq = (float32_t)(max_pos + 1) * (CurrentInjectFrequencyKHz / 512);
        max_pha = (max_pha * 180 / 3.14);

        printf("Inject Max mag = %f at %f kHz, %fdeg\n", max_mag, max_freq, max_pha);

        HAL_TIM_Base_Start_IT(pInjectTimer);
    }
}

#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(pUart, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}