#ifndef TEST_USART_H
#define TEST_USART_H
#include "main.h"
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart5;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
#endif
