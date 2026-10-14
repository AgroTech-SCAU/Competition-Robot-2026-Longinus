#include "stm32_hal_uart.h"

static void (*uart5_rx_complete_callback)(void) = NULL;
static void (*uart5_error_callback)(void) = NULL;

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

bool uart_abort_receive_it(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive_IT(huart) == HAL_OK;
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
    if(huart == &huart5 && uart5_rx_complete_callback != NULL) {
        uart5_rx_complete_callback();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart5 && uart5_error_callback != NULL) {
        uart5_error_callback();
    }
}
