#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef struct { volatile uint32_t State, disabled_it; } DMA_HandleTypeDef;
typedef struct {
    volatile uint32_t gState, RxState, ErrorCode;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
#define HAL_UART_STATE_BUSY_RX 0x22U
#define HAL_DMA_STATE_READY 1U
#define HAL_DMA_STATE_BUSY 2U
#define DMA_IT_HT 4U
#define __HAL_DMA_DISABLE_IT(handle, flag) ((handle)->disabled_it |= (flag))
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
uint32_t HAL_DMA_GetState(DMA_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
typedef struct { int unused; } FDCAN_HandleTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2;
#define POWER_24V_1_GPIO_Port 0
#define POWER_24V_2_GPIO_Port 0
#define POWER_5V_GPIO_Port 0
#define POWER_24V_1_Pin 0
#define POWER_24V_2_Pin 0
#define POWER_5V_Pin 0
#define GPIO_PIN_SET 1
static inline void HAL_GPIO_WritePin(int port, int pin, int state) { (void)port; (void)pin; (void)state; }
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
static inline void __DMB(void) {}
#endif
