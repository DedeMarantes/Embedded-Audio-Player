#include "stm32f4xx.h"
#include "core_drivers.h"
#include <stdio.h>
#include "adc.h"
#include "uart.h"
#include "df_player.h"

#define LED_PIN 5
#define BUFFER_SIZE 10

uint16_t adc_value[BUFFER_SIZE];
uint8_t data_ready = 0;

//Handler interrupt do DMA
void DMA2_Stream0_IRQHandler() 
{
    //Verifique se bit de transferência completa foi setado
    if(DMA2->LISR & DMA_LISR_TCIF0){
        //Limpa interrupt
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
        //data_ready = 1; //Flag para indicar que DMA terminou de escrever no buffer
    }
}

int main() {
    SysTick_Init();
    BSP_GPIO_Config();
    fpu_enable();
    debug_uart_init();
    ADC_GPIO_Init();
    ADC_DMA_Start(adc_value, BUFFER_SIZE);
    UART_DFPlayer_Init();
    circular_buffer_init();
    df_player_init(20);//Volume 20 de 0-30
    //df_player_play_first_track();
    df_player_repeat_track();
    while(1) {
        //Quando encerrar de ler todos os valores do buffer printa na tela
        /*if(data_ready){
            data_ready = 0;
            for(uint8_t i = 0; i < BUFFER_SIZE; i++) {
                printf("ADC value: %d\n", adc_value[i]);
            }
        }*/
        buffer_send_str("tes\n", DEBUG_UART);
        delay_ms(1000);
/*         ADC_Start();
        ADC_StartConversion();
        adc_value = ADC_Read();
        ADC_Stop();
        printf("Valor lido ADC: %d\n", adc_value);
        delay_ms(500); */
    }
}
