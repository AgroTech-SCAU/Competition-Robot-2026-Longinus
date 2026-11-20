#include "assemble.h"

#include "stp23l.h"
#include "stm32_hal_uart.h"

static UART_HandleTypeDef s_uart9;
static Stp23lParser s_parser;
static volatile uint16_t s_distance_mm;
static volatile uint32_t s_stamp_ms;
static volatile bool s_has_frame;

SystemStatus assemble_work_laser(void) {
    /* STP-23L streams on power-up. Connect its TX to PD14 (UART9_RX, AF11). */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART9_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {
        .Pin = GPIO_PIN_14,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF11_UART9,
    };
    HAL_GPIO_Init(GPIOD, &gpio);

    s_uart9.Instance = UART9;
    s_uart9.Init = (UART_InitTypeDef){
        .BaudRate = 230400u,
        .WordLength = UART_WORDLENGTH_8B,
        .StopBits = UART_STOPBITS_1,
        .Parity = UART_PARITY_NONE,
        .Mode = UART_MODE_RX,
        .HwFlowCtl = UART_HWCONTROL_NONE,
        .OverSampling = UART_OVERSAMPLING_16,
        .ClockPrescaler = UART_PRESCALER_DIV1,
    };
    if(HAL_UART_Init(&s_uart9) != HAL_OK || HAL_UARTEx_EnableFifoMode(&s_uart9) != HAL_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    UART9->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    UART9->RQR = USART_RQR_RXFRQ;
    SET_BIT(UART9->CR1, USART_CR1_RXNEIE_RXFNEIE);
    SET_BIT(UART9->CR3, USART_CR3_EIE);
    HAL_NVIC_SetPriority(UART9_IRQn, 2u, 0u);
    HAL_NVIC_EnableIRQ(UART9_IRQn);
    return SYSTEM_STATUS_OK;
}

void UART9_IRQHandler(void) {
    while((UART9->ISR & (USART_ISR_RXNE_RXFNE | USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0u) {
        if((UART9->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0u) {
            UART9->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
            UART9->RQR = USART_RQR_RXFRQ;
            s_parser.used = 0u;
            s_has_frame = false;
            break;
        }
        uint16_t distance_mm;
        if(stp23l_receive_byte(&s_parser, (uint8_t)UART9->RDR, &distance_mm)) {
            s_distance_mm = distance_mm;
            s_stamp_ms = HAL_GetTick();
            s_has_frame = true;
        }
    }
}

bool assemble_work_laser_get_range(uint16_t* distance_mm, uint32_t* stamp_ms) {
    const uint32_t irq = uart_critical_enter(NULL);
    const bool received = s_has_frame;
    *distance_mm = s_distance_mm;
    *stamp_ms = s_stamp_ms;
    uart_critical_exit(NULL, irq);
    return received;
}
