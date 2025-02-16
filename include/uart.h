#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "core_drivers.h"
#include "string.h"

typedef enum {
    DF_PLAYER_UART = 0,
    DEBUG_UART
} UartType;

void UART_DFPlayer_Init();
void debug_uart_init();
void buffer_send_str(char* str, UartType uart_type);
void circular_buffer_init();

#define UART_BUFFER_SIZE 100
#define INIT_VAL 0

//Tipo para criar buffer circular
/*
quando valor é lido tail é incrementado
quando valor no buffer é inserido head é incrementado
*/
typedef struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} circular_buffer;




#ifdef __cplusplus
}
#endif

#endif