/** @file gimbal_imu_uart.h @brief Passive USART10 IMU text capture. */
#ifndef GIMBAL_IMU_UART_H
#define GIMBAL_IMU_UART_H

/** @brief Initialize capture timing before starting the RTOS tasks. */
void GimbalImuUartInit(void);
/** @brief Run in the communication task; interrupt TX, no waiting or queueing. */
void GimbalImuUartTask(void);
#endif
