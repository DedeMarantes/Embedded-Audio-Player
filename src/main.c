#include "stm32f4xx.h"
#include "core_drivers.h"
#include <stdio.h>

#define LED_PIN 5

int main() {
    SysTick_Init();
    BSP_GPIO_Config();
    fpu_enable();
    debug_uart_init();
    uint8_t led_state = 0;
    while(1) {

    }
}
