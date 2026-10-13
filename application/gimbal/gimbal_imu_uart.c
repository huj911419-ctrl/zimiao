/** @file gimbal_imu_uart.c @brief Separate IMU logging over USART10. */
#include "gimbal_imu_uart.h"
#include "gimbal_imu_text.h"
#include "gimbal_manual_config.h"
#include "usart.h"

#if GIMBAL_IMU_UART_ENABLED
/* HAL retains this buffer until TX complete. DMA1 cannot access the default DTCM. */
static char tx_buffer[GIMBAL_IMU_TEXT_CAPACITY];
static uint32_t last_attempt_ms, last_header_ms, skipped_count;
static uint8_t header_sent;
#endif

void GimbalImuUartInit(void)
{
#if GIMBAL_IMU_UART_ENABLED
    last_attempt_ms = HAL_GetTick() - GIMBAL_IMU_UART_PERIOD_MS;
    last_header_ms = 0U;
    skipped_count = 0U;
    header_sent = 0U;
#endif
}

void GimbalImuUartTask(void)
{
#if GIMBAL_IMU_UART_ENABLED
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_attempt_ms) < GIMBAL_IMU_UART_PERIOD_MS) return;
    last_attempt_ms = now;
    /* Never rewrite a buffer still owned by an interrupt-driven transmission. */
    if (huart10.gState != HAL_UART_STATE_READY)
    {
        ++skipped_count;
        return;
    }
    uint8_t send_header = !header_sent ||
        (uint32_t)(now - last_header_ms) >= GIMBAL_IMU_TEXT_HEADER_PERIOD_MS;
    size_t length;
    if (send_header)
        length = GimbalImuTextHeader("UART10", tx_buffer, sizeof(tx_buffer));
    else
    {
        Gimbal_Debug_Data_s data;
        GimbalGetDebugData(&data);
        length = GimbalImuTextFormat(&data, skipped_count, tx_buffer, sizeof(tx_buffer));
    }
    if (!length || HAL_UART_Transmit_IT(&huart10, (uint8_t *)tx_buffer, (uint16_t)length) != HAL_OK)
    {
        ++skipped_count;
        return;
    }
    if (send_header)
    {
        header_sent = 1U;
        last_header_ms = now;
    }
#endif
}
