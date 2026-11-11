#ifndef _stm32_hal_uart_h_
#define _stm32_hal_uart_h_

#include "main.h" // IWYU pragma: keep

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t received_bytes;
    uint32_t error_interrupts;
    uint32_t last_rx_ms;
    uint32_t last_error;
} UartRxDiagnostics;

bool uart_get_rx_diagnostics(UART_HandleTypeDef* huart, UartRxDiagnostics* out);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

bool uart1_write_blocking(const char* data, uint32_t len);
bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len);
bool uart_abort_receive(UART_HandleTypeDef* huart);
void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void));
void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void));

uint32_t uart_critical_enter(void* context);
void uart_critical_exit(void* context, uint32_t state);

#endif
