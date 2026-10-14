#ifndef _stm32_hal_uart_h_
#define _stm32_hal_uart_h_

#include "main.h" // IWYU pragma: keep

#include <stdbool.h>
#include <stdint.h>

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
