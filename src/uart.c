#include "uart.h"

#define UART_BAUD_RATE 9600
#define SYS_CLK 16000000
#define APB1_CLK SYS_CLK
#define APB2_CLK SYS_CLK

#include "stdio.h"

static void uart_baud_rate_conf(USART_TypeDef* uart, uint32_t periph_clk, uint32_t baud_rate) 
{
    //Divide baud_rate por 2 para arrendodamento correto
    uint32_t uart_div = (periph_clk + (baud_rate / 2)) / baud_rate;
    uart->BRR = uart_div;
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
    uart_baud_rate_conf(USART2, APB1_CLK, UART_BAUD_RATE);
    //usando = para zerar tudo no registrador e apenas setar os valores desejados
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}



//Enviar dados serial
static void uart_write_char(USART_TypeDef* uart, char ch) 
{
    //Esperar até buffer transmissão fique vazio
    while(!(READ_BIT(uart->SR, USART_SR_TXE)));
    uart->DR = ch & 0xFF; //registrador DR onde ficam os dados a serem enviados
}

char uart_read_char(USART_TypeDef* uart)
{
    //esperar até que dado seja recebido
    while(!(READ_BIT(uart->SR, USART_SR_RXNE)));
    return uart->DR & 0xFF;
}

int _write(int file, char* ptr, int len) {
    for (int i = 0; i < len; i++) {
        uart_write_char(USART2, ptr[i]);  // Envia cada caractere do buffer
    }
    return len;  // Retorna o número de caracteres enviados
}

void UART_DFPlayer_Init()
{
    //Habilita clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    //GPIOA PA9 e PA10 para alterante function mode
    MODIFY_REG(GPIOA->MODER, 
        GPIO_MODER_MODER9 | GPIO_MODER_MODER10, 
        _VAL2FLD(GPIO_MODER_MODER9, 2) | _VAL2FLD(GPIO_MODER_MODER10, 2)
    );
    //Alternate Function 7 para USART_TX (PA9) e USART_RX(PA10)
    MODIFY_REG(GPIOA->AFR[1],
        GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10,
        _VAL2FLD(GPIO_AFRH_AFSEL9, 7) | _VAL2FLD(GPIO_AFRH_AFSEL10, 7)
    );
    //Habilitar clock para UART1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    uart_baud_rate_conf(USART1, APB2_CLK, UART_BAUD_RATE);
    //Setando valores para habilitar RX e TX
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;

    //Habilitar Interrupção para USART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 2);

    //Habilitar UART
    USART1->CR1 |= USART_CR1_UE;
}

circular_buffer rx_buffer = {{INIT_VAL}, INIT_VAL, INIT_VAL}; //Buffer de recepção para o dispositivo escravo
circular_buffer tx_buffer = {{INIT_VAL}, INIT_VAL, INIT_VAL}; //Buffer de transmissão para o dispositivo escravo

circular_buffer rx_buffer_debug = {{INIT_VAL}, INIT_VAL, INIT_VAL}; //Buffer de recepção para debug
circular_buffer tx_buffer_debug = {{INIT_VAL}, INIT_VAL, INIT_VAL}; //Buffer de transmissão para debug

//Definindo ponteiros para os buffers

circular_buffer* ptr_rx_buffer;
circular_buffer* ptr_tx_buffer;
circular_buffer* ptr_rx_buffer_debug;
circular_buffer* ptr_tx_buffer_debug;

void circular_buffer_init()
{
    //Inicializando ponteiros dos buffers
    ptr_rx_buffer = &rx_buffer;
    ptr_tx_buffer = &tx_buffer;
    ptr_rx_buffer_debug = &rx_buffer_debug;
    ptr_tx_buffer_debug = &tx_buffer_debug;
    //Iniciando interrupção para o RX buffer
    USART1->CR1 |= USART_CR1_RXNEIE;
    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 1);
}

static void buff_store_char(uint8_t ch, circular_buffer* circ_buffer)
{
    //Indice do header, incrementado toda vez que character é inserido no buffer 
    //com a operação de resto quando o indice chegar no tamanho do buffer contagem indice é reiniciado 
    uint8_t head_location = (circ_buffer->head + 1) % UART_BUFFER_SIZE; 
    //Checar se não está cheio para não causar overflow
    if(head_location != circ_buffer->tail) {
        //Inserir caractere no buffer na posição head
        circ_buffer->buffer[circ_buffer->head] = ch;
        circ_buffer->head = head_location; //Atualiza posição de head
    }
}

void clear_buffer(UartType uart_type)
{
    if(uart_type == DF_PLAYER_UART) {
        //Seta conteudo do buffer de recepção para 0 e reinicia head
        memset(ptr_rx_buffer->buffer, 0, UART_BUFFER_SIZE);
        ptr_rx_buffer->head = 0;
    }
    else if(uart_type == DEBUG_UART) {
        memset(ptr_rx_buffer_debug->buffer, 0, UART_BUFFER_SIZE);
        ptr_rx_buffer_debug->head = 0;
    }
}

//Checa próximo valor do buffer sem remove-lo
int8_t buffer_peek(UartType uart_type)
{
    if(uart_type == DF_PLAYER_UART) {
        //Se buffer estiver vazio retorna código de erro
        if(ptr_rx_buffer->head == ptr_rx_buffer->tail) {
            return -1;
        } else {
            //Retorna valor sem incrementar tail
            return ptr_rx_buffer->buffer[ptr_rx_buffer->tail];
        }
    }
    else if(uart_type == DEBUG_UART) {
        //Se buffer estiver vazio retorna código de erro
        if(ptr_rx_buffer_debug->head == ptr_rx_buffer_debug->tail) {
            return -1;
        } else {
            //Retorna valor sem incrementar tail
            return ptr_rx_buffer_debug->buffer[ptr_rx_buffer_debug->tail];
        }
    } else {
        return -1;
    }
}

//Função para ler valor e depois incrementar tail para remover valor
int8_t buffer_read(UartType uart_type)
{
    if(uart_type == DF_PLAYER_UART) {
        //Se buffer estiver vazio retorna código de erro
        if(ptr_rx_buffer->head == ptr_rx_buffer->tail) {
            return -1;
        } else {
            uint8_t ch = ptr_rx_buffer->buffer[ptr_rx_buffer->tail];
            //Incrementa tail por já ter lido valor de forma circular
            ptr_rx_buffer->tail = (ptr_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
            return ch;
        }
    }
    else if(uart_type == DEBUG_UART) {
        //Se buffer estiver vazio retorna código de erro
        if(ptr_rx_buffer_debug->head == ptr_rx_buffer_debug->tail) {
            return -1;
        } else {
            uint8_t ch =  ptr_rx_buffer_debug->buffer[ptr_rx_buffer_debug->tail];
            ptr_rx_buffer_debug->tail = (ptr_rx_buffer_debug->tail + 1) % UART_BUFFER_SIZE;
            return ch;
        }
    } else { return -1; }
}

void buffer_write(uint8_t ch, UartType uart_type)
{
    if(uart_type == DF_PLAYER_UART) {
        buff_store_char(ch, ptr_tx_buffer);

    } 
    else if(uart_type == DEBUG_UART) {
        buff_store_char(ch, ptr_tx_buffer_debug);

    }
}

//Função para enviar bytes
void buffer_send_byte(uint8_t* data, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) {
        buffer_write(data[i], DF_PLAYER_UART);
    }
}

//Função para verificar se existe dados no buffer
uint8_t buffer_empty(UartType uart_type)
{
    switch (uart_type)
    {
    case DF_PLAYER_UART:
        return ptr_rx_buffer->head == ptr_rx_buffer->tail;
    case DEBUG_UART:
        return ptr_rx_buffer_debug->head == ptr_rx_buffer_debug->tail;
    default:
        return 1; // Se o tipo for inválido, consideramos o buffer vazio
    }
}

//Pega 1 caracter de uma string no buffer
static uint8_t get_first_char(uint8_t *str, uint8_t *result) {
    // Verifica se o buffer está vazio
    if (buffer_empty(DF_PLAYER_UART)) {
        return 0;  // Buffer vazio, nenhum caractere encontrado
    }

    // Procura o primeiro caractere da string no buffer
    char first_char = *str;  // Primeiro caractere da string
    while (!buffer_empty(DF_PLAYER_UART)) {
        uint8_t current_char = buffer_peek(DF_PLAYER_UART);  // Lê o próximo caractere do buffer
        if (current_char == first_char) {
            *result = current_char;  // Retorna o caractere encontrado
            return 1;  // Caractere encontrado
        }
        // Move para o próximo caractere no buffer
        ptr_rx_buffer->tail = (ptr_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
    }
    return 0;  // Caractere não encontrado
}

//Função para checar se resposta está no buffer
int8_t resp_present(uint8_t* str)
{
    uint8_t pos = 0;
    uint8_t len = strlen((char*)str);
    int8_t ch;
    
    // Percorre o buffer até encontrar a string ou chegar ao final
    while (!buffer_empty(DF_PLAYER_UART)) {
        // Lê o próximo caractere do buffer
        ch = buffer_read(DF_PLAYER_UART);
    
        // Verifica se o caractere corresponde ao esperado
        if (ch == str[pos]) {
            pos++;  // Avança para o próximo caractere da string
            if (pos == len) {
                return 1;  // String encontrada
            }
        } else {
            pos = 0; 
        }
    }
    return -1;  
}

void get_string(char* dest_buff, uint8_t size)
{
    for(uint8_t idx = 0; idx < size; idx++) {
        delay_ms(500);
        dest_buff[idx] = (char) buffer_read(DF_PLAYER_UART);
    }
}

void buffer_send_str(char* str, UartType uart_type)
{
    while(*str != '\0') {
        buffer_write((uint8_t) *str, uart_type);
        str++;
    }
    //printf("%s\n", ptr_tx_buffer_debug->buffer);
    if (uart_type == DEBUG_UART) {
        USART2->CR1 |= USART_CR1_TXEIE;
    } 
    else if (uart_type == DF_PLAYER_UART) {
        USART1->CR1 |= USART_CR1_TXEIE;
    }
}

void uart_callback(USART_TypeDef* uart, circular_buffer* rx_buff, circular_buffer* tx_buff)
{
    //Checar se interrupção RXNE está habilitada
    if((uart->SR & USART_SR_RXNE) && (uart->CR1 & USART_CR1_RXNEIE)) {
        uint8_t ch = uart->DR;
        buff_store_char(ch, rx_buff);
    }

    if((uart->SR & USART_SR_TXE) && (uart->CR1 & USART_CR1_TXEIE)) {
        if(tx_buff->head == tx_buff->tail) {
            uart->CR1 &= ~(USART_CR1_TXEIE);
        } else {
            uint8_t ch = tx_buff->buffer[tx_buff->tail];
            tx_buff->tail = (tx_buff->tail + 1) % UART_BUFFER_SIZE;
            uart->DR = ch;
        }
    }
}

void USART1_IRQHandler()
{
    uart_callback(USART1, ptr_rx_buffer, ptr_tx_buffer);
}

void USART2_IRQHandler()
{
    uart_callback(USART2, ptr_rx_buffer_debug, ptr_tx_buffer_debug);
}


