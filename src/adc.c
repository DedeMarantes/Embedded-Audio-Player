#include "adc.h"
#include "core_drivers.h"

void ADC_GPIO_Init()
{
    //Configurar pino GPIO para ADC e configurar como modo analog
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Habilita clock para GPIOA
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER1, _VAL2FLD(GPIO_MODER_MODER1, 3));
}

void ADC_Start()
{
    /**Configurar Módulo ADC**/
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    /*Configurar sequência de conversão e tamanho para definir canais adc a serem usados*/
    //Somente 1 canal vai ser usado então só vai ter uma sequencia
    //Definir tamanho de sequência que é 1. Campo L de 4 bits fica na posição 20 do reg SQR1 
    //onde 0b0 = 1 conversão por isso subtrai valor por 1
    ADC1->SQR1 = ((ADC_SEQUENCE_LENGTH - 1) << 20);
    ADC1->SQR3 = (CHANNEL_1 << 0); //Habilitar canal 1 como SQ1 (primeira e única sequência)

    //Alterar número de ciclos na amostra para 144 ciclos
    MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP1, _VAL2FLD(ADC_SMPR2_SMP1, ADC_SAMPLES_144_CYCLES));

    //Habilitar módulo ADC 
    ADC1->CR2 |= ADC_CR2_ADON;
}

void ADC_DMA_Start(uint16_t *adc_buffer, uint16_t buffer_size)
{
    //Habilitar clocks para ADC1 e DMA2 (ADC1 usa o DMA2 (Canal 0))
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    //Configurar Modulo ADC para usar DMA e conversão contínua
    ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    delay_ms(500);
    DMA2_Stream0->CR &= ~(DMA_SxCR_EN);
    while(DMA2_Stream0->CR & DMA_SxCR_EN);
    //Configurar DMA para ele saber que é o ADC o periférica onde irá ser transferido os dados para um buffer na memória
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR; //Endereço do periférico onde irá ser pego os dados
    DMA2_Stream0->M0AR = (uint32_t) adc_buffer; //Endereço de memória do buffer onde dma vai ler/escrever dados
    //Número de transferências (tamanho do buffer)
    DMA2_Stream0->NDTR = buffer_size;
    /*Configurações do DMA
    Canal 0 (ADC fica no canal 0)
    Definir tamanho da memória e periférico (16 bits para os dois)
    Definir direção como periférico para memória
    habilitar incremento de memória e modo circular
    */
    MODIFY_REG(DMA2_Stream0->CR, 
        DMA_SxCR_CHSEL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_DIR,
        _VAL2FLD(DMA_SxCR_CHSEL, 0) | _VAL2FLD(DMA_SxCR_MSIZE, 1) | _VAL2FLD(DMA_SxCR_PSIZE, 1) |
        _VAL2FLD(DMA_SxCR_DIR, 0) | DMA_SxCR_CIRC | DMA_SxCR_MINC
    );
    //Ativa DMA e a interrupção quando transferência estiver completa 
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;
    //Ativar interrupção correspondente no NVIC
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
    ADC1->SQR1 = ((ADC_SEQUENCE_LENGTH - 1) << 20);
    ADC1->SQR3 = (CHANNEL_1 << 0); //Habilitar canal 1 como SQ1 (primeira e única sequência)
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    //Alterar número de ciclos na amostra para 144 ciclos
    MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP1, _VAL2FLD(ADC_SMPR2_SMP1, ADC_SAMPLES_144_CYCLES));
    //Inicia conversão 
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC_StartConversion()
{
    //Habilitar modo de conversão continua
    ADC1->CR2 |= ADC_CR2_CONT;
    //Começar conversão
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint32_t ADC_Read()
{
    //Espera conversão estar completa
    while(!(ADC1->SR & ADC_SR_EOC));
    //Retorna valor lido pelo adc
    return ADC1->DR;
}

void ADC_Stop()
{
    //desabilitar ADC
    CLEAR_BIT(ADC1->CR2, ADC_CR2_ADON);

    //desabilitar DMA
    CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
    while (DMA2_Stream0->CR & DMA_SxCR_EN);
    //Limpa interrupções para limpar tem que setar como 1 nos campos correspondentes
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
    //Desabilita clock para ADC e para DMA
    CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
    CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);

}