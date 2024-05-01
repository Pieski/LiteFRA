#include "FRA.h"
#include "arm_math.h"
#include "arm_const_structs.h"

ADC_HandleTypeDef *pAdc;
TIM_HandleTypeDef *pPwmTimer;
TIM_HandleTypeDef *pInjectTimer;
UART_HandleTypeDef *pUart;

uint8_t FclkMHz;
float32_t SwitchingFrequencyKHz; // Fsw of the SMPS
float32_t NorminalDutyCycle;     // D at steady state
float32_t InjectAmplitude;       // Delta D of injected sinusoidal wave
uint16_t NorminalCRR;
uint16_t PwmARR;

float32_t SineRef[8] = {1.0000, 0.7071, 0.0000, -0.7071, -1.0000, -0.7071, -0.0000, 0.7071}; // Norminal cos wave
int16_t SineInt[8];                                                                          // Actual value injected into the CRR of the PWM timer
uint16_t ResponseBuf[512];                                                                   // Buffer for ADC, stores the response of SMPS
int16_t InjectBuf[512];                                                                      // Buffer for injection !! TODO: the size can be reduced
float32_t FFTComplexBuf[1024];                                                               // Odd indexes stores the real part, even store the imag part
float32_t FFTMagBuf[256];

volatile uint8_t preInjectCount = 0;
volatile uint8_t injectSineIndex = 0;
volatile uint16_t adcReadIndex = 0;
volatile BOOL startReadFlag = FALSE;
volatile BOOL preInjectFlag = FALSE;
volatile BOOL adcFinishFlag = FALSE;

uint8_t uartBuffer[1029];

void FRA_Init(ADC_HandleTypeDef *padc, TIM_HandleTypeDef *ppwm, TIM_HandleTypeDef *pinj, UART_HandleTypeDef *puart, struct FRAParameterBlock parblock)
{
    // Accept handles
    pAdc = padc;
    pPwmTimer = ppwm;
    pInjectTimer = pinj;
    pUart = puart;

    FclkMHz = parblock.FclkMHz;
    NorminalDutyCycle = parblock.NorminalDutyCycle;
    InjectAmplitude = parblock.InjectAmplitude;
    SwitchingFrequencyKHz = parblock.SwitchingFrequencyKHz;

    PwmARR = ((FclkMHz * 1000) / SwitchingFrequencyKHz) - 1;
    for (int i = 0; i < 8; ++i)
        SineInt[i] = SineRef[i] * PwmARR * InjectAmplitude;
    NorminalCRR = PwmARR * NorminalDutyCycle;

    HAL_TIM_RegisterCallback(pInjectTimer, HAL_TIM_PERIOD_ELAPSED_CB_ID, FRA_InjectTimerTriggeredHandler);

    HAL_TIM_Base_Start_IT(pInjectTimer);
    while (TRUE)
    {
        FRA_Sweep(0.01, 20, 10);
    }
}

void FRA_GainShift(float32_t *pgain, float32_t *pshift)
{
    float32_t peak_mag_input = 0.0, peak_mag_output = 0.0;
    float32_t peak_pha_input = 0.0, peak_pha_output = 0.0;
    int peak_freq_pos_input = 0, peak_freq_pos_output = 0;

    // 1. Do an FFT on injected sine wave
    // Use injection record to load FFT input buffer, clear the imag part
    for (int i = 0; i < 512; ++i)
    {
        FFTComplexBuf[i * 2] = (float32_t)InjectBuf[i] / PwmARR;
        FFTComplexBuf[i * 2 + 1] = 0;
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len512, FFTComplexBuf, 0, 1); // FFT on 512 pts
    arm_cmplx_mag_f32(FFTComplexBuf, FFTMagBuf, 256);           // Nyquist theorm, only half of the spectrum is effective
    FFTMagBuf[0] = 0;                                           // Ignore the DC part

    // Find the peak in the injection spectrum, calculate the corresponding phase angle
    arm_max_f32(FFTMagBuf, 256, &peak_mag_input, &peak_freq_pos_input);
    arm_atan2_f32(FFTComplexBuf[peak_freq_pos_input * 2 + 1], FFTComplexBuf[peak_freq_pos_input * 2], &peak_pha_input);

    peak_mag_input = peak_mag_input * 2 / 512;
    peak_pha_input = (peak_pha_input * 180 / 3.14);

    // 2. Do another FFT on ADC result
    // Reload FFT buffer from ADC record, clear the imag part
    for (int i = 0; i < 512; ++i)
    {
        FFTComplexBuf[i * 2] = 3.3 * (float32_t)ResponseBuf[i] / 4095;
        FFTComplexBuf[i * 2 + 1] = 0;
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len512, FFTComplexBuf, 0, 1); // FFT
    arm_cmplx_mag_f32(FFTComplexBuf, FFTMagBuf, 256);           // Nyquist theorm, only half of the spectrum is effective
    FFTMagBuf[0] = 0;                                           // Ignore the DC part

    // Calculate the magnitude and phase of the ADC spectrum at the peak frequency of injection spectrum
    // peak_mag_output = arm_euclidean_distance_f32(FFTComplexBuf + peak_freq_pos_input * 2, FFTComplexBuf + peak_freq_pos_input * 2 + 1, 1);
    peak_mag_output = FFTMagBuf[peak_freq_pos_input];
    arm_atan2_f32(FFTComplexBuf[peak_freq_pos_input * 2 + 1], FFTComplexBuf[peak_freq_pos_input * 2], &peak_pha_output);

    // UartTransmitFrame((uint8_t*)FFTMagBuf, 256*4);

    peak_mag_output = (peak_mag_output) * 2 / 512;
    peak_pha_output = peak_pha_output * 180 / 3.14;

    *pgain = peak_mag_output / peak_mag_input;
    *pshift = peak_pha_output - peak_pha_input;
}

void FRA_Inject(float32_t sinefreq)
{
    // Calculate injection timer ARR
    float injectfreq = sinefreq * 8;
    uint32_t injectARR = ((FclkMHz * 1000 / 2) / (injectfreq)) - 1;
    pInjectTimer->Instance->ARR = injectARR;

    HAL_TIM_Base_Start_IT(pInjectTimer);
}

float current_freq_tmp = 0.0;
void FRA_Sweep(float32_t fstart, float32_t fend, uint8_t numDecade)
{
    float logfstart = log10f(fstart);
    float logfend = log10f(fend);
    float logfrange = logfend - logfstart;
    uint8_t num = (uint8_t)(logfrange * numDecade);

    float logsinefreq = logfstart;
    uint8_t sweepcount = 0;
    while (sweepcount < num)
    {
        logsinefreq += logfrange / num;
        sweepcount++;

        // Calculate injection timer ARR
        float sinefreq = powf(10, logsinefreq);
        current_freq_tmp = sinefreq; // used for debug
        float injectfreq = sinefreq * 8;
        uint32_t injectARR = ((FclkMHz * 1000 / 2) / (injectfreq)) - 1;
        pInjectTimer->Instance->ARR = injectARR;

        // Do a 10 cycle pre-injection
        preInjectFlag = TRUE;
        preInjectCount = 0;
        HAL_TIM_Base_Start_IT(pInjectTimer);
        while (preInjectCount <= 8 * 10)
            continue;
        preInjectFlag = FALSE;

        // Read 512 pts from ADC
        startReadFlag = TRUE;
        adcReadIndex = 0;
        while (adcReadIndex < 512)
            continue;
        startReadFlag = FALSE;

        // After reading 512 pts, stop injection then calculate gain and phase shift
        HAL_TIM_Base_Stop_IT(pInjectTimer);
        float32_t gain, shift;
        FRA_GainShift(&gain, &shift);
        printf("Freq = %.6fkHz, Gain = %.2f, Shift = %.2f\n", current_freq_tmp, gain, shift);
    }
}

void FRA_ReadPoint()
{
    if (adcReadIndex >= 512)
        return;
    HAL_ADC_Start(pAdc);
    if (HAL_OK == HAL_ADC_PollForConversion(pAdc, 50))
    {
        // Read response and excitation simutaneously to calculate phase difference
        ResponseBuf[adcReadIndex] = HAL_ADC_GetValue(pAdc);
        InjectBuf[adcReadIndex] = SineInt[injectSineIndex];
    }

    adcReadIndex++;
}

void FRA_InjectTimerTriggeredHandler(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
    // Inject sinusoidal wave into duty cycle
    injectSineIndex++;
    if (injectSineIndex >= 8)
        injectSineIndex = 0;
    pPwmTimer->Instance->CCR1 = NorminalCRR + SineInt[injectSineIndex];

    if (preInjectFlag)
        preInjectCount++;

    if (startReadFlag)
        FRA_ReadPoint();
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

void UartTransmitFrame(uint8_t *buf, uint16_t size)
{
    uartBuffer[0] = UART_BEGIN;
    uartBuffer[1] = UART_BEGIN;
    memcpy(uartBuffer + 2, &size, 2);
    memcpy(uartBuffer + 4, buf, size);

    uartBuffer[size + 4] = 0;
    for (uint16_t i = 0; i < size; ++i)
        uartBuffer[size + 4] += buf[i];

    HAL_UART_Transmit(pUart, uartBuffer, size + 5, 0xFFFF);
}