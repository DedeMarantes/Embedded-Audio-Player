#include "stm32f4xx.h"
#include "core_drivers.h"

#define UART_BAUD_RATE 9600
#define SYS_CLK 16000000
#define APB1_CLK SYS_CLK
#define MAX_DELAY 0xFFFFFFFF

volatile uint32_t ticks, current_tick;


static void uart_baud_rate_conf(uint32_t periph_clk, uint32_t baud_rate);
static void uart_write_char(char ch);
char uart_read_char();

uint32_t get_tick()
{
    __disable_irq(); //Desabilita interrupções para poder pegar valor de tick
    current_tick = ticks;
    __enable_irq();
    return current_tick;
}

void delay_ms(uint32_t delay)
{
    //Pega valor atual do tick que acontece a cada 1ms
    uint32_t tick_start = get_tick();
    while((get_tick() - tick_start) < delay);
}

void SysTick_Handler()
{
    ticks++;
}

void SysTick_Init()
{
    //Desabilitar global interrupt
    __disable_irq();
    //Carregar o timer com o número de ciclos de clock por segundo
    //sistema conta do 0 por isso subtrai por 1 
    //nesse caso quero que o timer seja feito a cada ms então divide por 1000 a frequência
    //A cada 16000 ciclos de clock gera uma interrupção e incrementa o tick
    SysTick->LOAD = (SYS_CLK / 1000)  - 1;
    //Limpar valor atual do registrador systick
    SysTick->VAL = 0;
    //Selecionar timer interno do clock
    SET_BIT(SysTick->CTRL, CTRL_CLKSOURCE);
    //Habilitar interrupt
    SET_BIT(SysTick->CTRL, CTRL_TICKINT);
    //Habilitar systick
    SET_BIT(SysTick->CTRL, CTRL_ENABLE);
    //Habilitar global interrupt
    __enable_irq();
}


void fpu_enable() 
{
    /*Cortex M4 na documentação para habilitar FPU tem que dar Full Access para CP10 e CP11 no registrador CPACR
    Registrador fica no System Control Block (SCB) são configurações do processador */
    SCB->CPACR |= (3 << 22) | (3 << 20); 
}

/*UART pins conectados no usb do st-link
    PA2 (Tx) e PA3 (Rx) com ambos AF07 alternate function
    Habilitar clocks para GPIOA e USART2
Configurações do USART como baud rate e direção e habilitar usart*/
void debug_uart_init()
{

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    MODIFY_REG(GPIOA->MODER,
        GPIO_MODER_MODER2 | GPIO_MODER_MODER3,
        _VAL2FLD(GPIO_MODER_MODER2, 2) | _VAL2FLD(GPIO_MODER_MODER3, 2)
    );
    MODIFY_REG(GPIOA->AFR[0], 
        GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3,
        _VAL2FLD(GPIO_AFRL_AFSEL2, 7) | _VAL2FLD(GPIO_AFRL_AFSEL3, 7)
    );
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    uart_baud_rate_conf(APB1_CLK, UART_BAUD_RATE);
    //usando = para zerar tudo no registrador e apenas setar os valores desejados
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_baud_rate_conf(uint32_t periph_clk, uint32_t baud_rate) 
{
    //Divide baud_rate por 2 para arrendodamento correto
    uint32_t uart_div = (periph_clk + (baud_rate / 2)) / baud_rate;
    USART2->BRR = uart_div;
}

//Enviar dados serial
static void uart_write_char(char ch) 
{
    //Esperar até buffer transmissão fique vazio
    while(!(READ_BIT(USART2->SR, USART_SR_TXE)));
    USART2->DR = ch & 0xFF; //registrador DR onde ficam os dados a serem enviados
}

char uart_read_char()
{
    //esperar até que dado seja recebido
    while(!(READ_BIT(USART2->SR, USART_SR_RXNE)));
    return USART2->DR & 0xFF;
}

int _write(int file, char* ptr, int len) {
    for (int i = 0; i < len; i++) {
        uart_write_char(ptr[i]);  // Envia cada caractere do buffer
    }
    return len;  // Retorna o número de caracteres enviados
}

//Configura GPIOs para LED e Botão embutidos na placa
void BSP_GPIO_Config()
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    /*
    Define GPIO:
    Como Output 
    velocidade do pino medium
    habilita pull-up
    */
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER5, _VAL2FLD(GPIO_MODER_MODER5, 1));
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED5, _VAL2FLD(GPIO_OSPEEDR_OSPEED5, 1));
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD5, _VAL2FLD(GPIO_PUPDR_PUPD5, 1));

    /*Habilita clock e configura PC13 como input para o botão user*/
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
    MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER13, _VAL2FLD(GPIO_MODER_MODER13, 0));
    MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD13, _VAL2FLD(GPIO_PUPDR_PUPD13, 1));
}

void write_gpio(GPIO_TypeDef* gpio, uint8_t pin_number, uint8_t state)
{
    if(state) {
        gpio->ODR |= (1 << pin_number);
    } else {
        gpio->ODR &= ~ (1 << pin_number);
    }
}

uint8_t read_gpio(GPIO_TypeDef* gpio, uint8_t pin_number)
{
    return gpio->IDR & (1 << pin_number) ? 1 : 0;
}

