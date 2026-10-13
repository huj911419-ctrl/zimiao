/* Exercise the real BSP: retry a stopped receiver without aborting active DMA. */
#include "bsp_usart.h"
#include <assert.h>
#include <stdio.h>

extern uint8_t (*recv_buffs)[USART_RXBUFF_LIMIT];
static uint8_t buffers[DEVICE_USART_CNT][USART_RXBUFF_LIMIT];
static unsigned starts, callbacks;
static HAL_StatusTypeDef next_start=HAL_OK;
static USARTInstance *instance;
uint32_t HAL_GetTick(void) { return 0; }
uint32_t HAL_DMA_GetState(DMA_HandleTypeDef *dma) { return dma->State; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    assert(data==buffers[0] && length==18);
    ++starts;
    if (next_start==HAL_OK)
    {
        uart->RxState=HAL_UART_STATE_BUSY_RX;
        uart->hdmarx->State=HAL_DMA_STATE_BUSY;
    }
    return next_start;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *b,uint16_t n,uint32_t t)
{ (void)u; (void)b; (void)n; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n)
{ (void)u; (void)b; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *b,uint16_t n)
{ (void)u; (void)b; (void)n; return HAL_OK; }
static void OnFrame(void) { assert(instance->recv_len==18); ++callbacks; }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart,uint16_t size);

int main(void)
{
    DMA_HandleTypeDef dma={.State=HAL_DMA_STATE_READY};
    UART_HandleTypeDef uart={.RxState=HAL_UART_STATE_READY,.hdmarx=&dma};
    recv_buffs=buffers;
    USART_Init_Config_s cfg={.recv_buff_size=18,.usart_handle=&uart,.module_callback=OnFrame};
    instance=USARTRegister(&cfg);
    assert(starts==1 && instance->start_failures==0 && (dma.disabled_it&DMA_IT_HT));
    USARTEnsureReceive(instance); assert(starts==1); /* active receive is untouched */
    uart.RxState=HAL_UART_STATE_READY;
    USARTEnsureReceive(instance); assert(starts==1); /* DMA abort still pending */
    dma.State=HAL_DMA_STATE_READY; next_start=HAL_ERROR;
    USARTEnsureReceive(instance);
    assert(starts==2 && instance->receive_retries==1 && instance->start_failures==1);
    next_start=HAL_OK; USARTEnsureReceive(instance);
    assert(starts==3 && instance->receive_retries==2 && uart.RxState==HAL_UART_STATE_BUSY_RX);
    USARTEnsureReceive(instance); assert(starts==3);
    uart.RxState=HAL_UART_STATE_READY; dma.State=HAL_DMA_STATE_READY;
    uart.ErrorCode=8; next_start=HAL_BUSY; HAL_UART_ErrorCallback(&uart);
    assert(instance->error_count==1 && instance->last_error==8 && instance->start_failures==2);
    next_start=HAL_OK; USARTEnsureReceive(instance);
    assert(starts==5 && instance->receive_retries==3);
    uart.RxState=HAL_UART_STATE_READY; dma.State=HAL_DMA_STATE_READY;
    HAL_UARTEx_RxEventCallback(&uart,18);
    assert(callbacks==1 && starts==6 && instance->start_failures==2);
    USARTEnsureReceive(NULL);
    puts("PASS: UART failed-start retry, active/aborting DMA exclusion, error diagnostics and callback rearm");
    return 0;
}
