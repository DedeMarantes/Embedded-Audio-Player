#ifndef CORE_DRIVERS_H
#define CORE_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "stm32f4xx.h"

#define CTRL_ENABLE (1 << 0)
#define CTRL_TICKINT (1 << 1)
#define CTRL_CLKSOURCE (1 << 2)
#define CTRL_COUNTFLAG (1 << 16)

#define BUTTON_PIN 13
#define BUTTON_PORT GPIOC
#define LED_PIN 5
#define LED_PORT GPIOA


void fpu_enable();
void debug_uart_init();
void delay_ms(uint32_t delay);
void SysTick_Init();
void BSP_GPIO_Config();
void write_gpio(GPIO_TypeDef* gpio, uint8_t state, uint8_t pin_number);
uint8_t read_gpio(GPIO_TypeDef* gpio, uint8_t pin_number);

#ifdef __cplusplus
}
#endif

#endif