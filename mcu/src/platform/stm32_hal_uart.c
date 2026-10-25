#include "stm32_hal_uart.h"

static void (*uart5_rx_complete_callback)(void) = NULL;
static void (*uart5_error_callback)(void) = NULL;
static volatile UartRxDiagnostics s_uart5_rx_diagnostics;

bool uart1_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, 10) == HAL_OK;
}

bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    if(huart == NULL || data == NULL || len == 0u) {
        return false;
    }

    return HAL_UART_Receive_IT(huart, data, len) == HAL_OK;
}

bool uart_abort_receive(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive(huart) == HAL_OK;
}

void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart5) {
        uart5_rx_complete_callback = callback;
    }
}

void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart5) {
        uart5_error_callback = callback;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart5) {
        ++s_uart5_rx_diagnostics.received_bytes;
        s_uart5_rx_diagnostics.last_rx_ms = HAL_GetTick();
        if(uart5_rx_complete_callback != NULL) uart5_rx_complete_callback();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart5) {
        ++s_uart5_rx_diagnostics.error_interrupts;
        s_uart5_rx_diagnostics.last_error = huart->ErrorCode;
        if(uart5_error_callback != NULL) uart5_error_callback();
    }
}

uint32_t uart_critical_enter(void* context) {
    (void)context;
    const uint32_t state = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return state;
}

void uart_critical_exit(void* context, uint32_t state) {
    (void)context;
    __DMB();
    __set_PRIMASK(state);
}

bool uart_get_rx_diagnostics(UART_HandleTypeDef* huart, UartRxDiagnostics* out) {
    if(huart != &huart5 || out == NULL) return false;
    const uint32_t irq_state = uart_critical_enter(NULL);
    *out = s_uart5_rx_diagnostics;
    uart_critical_exit(NULL, irq_state);
    return true;
}
