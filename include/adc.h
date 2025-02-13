#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "core_drivers.h"

#define CHANNEL_1 1
#define ADC_SEQUENCE_LENGTH  1
#define ADC_SAMPLES_144_CYCLES 6

void ADC_DMA_Start(uint16_t *adc_buffer, uint16_t buffer_size);
void ADC_Start();
void ADC_StartConversion();
uint32_t ADC_Read();
void ADC_Stop();
void ADC_GPIO_Init();

#ifdef __cplusplus
}
#endif

#endif