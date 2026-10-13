/** @file gimbal_imu_text.h @brief IMU CSV formatting shared by USB and USART10. */
#ifndef GIMBAL_IMU_TEXT_H
#define GIMBAL_IMU_TEXT_H
#include <stddef.h>
#include "2yaw_gimbal.h"

#define GIMBAL_IMU_TEXT_CAPACITY 512U
#define GIMBAL_IMU_TEXT_PERIOD_MS 50U
#define GIMBAL_IMU_TEXT_HEADER_PERIOD_MS 1000U

/** @brief Fixed three decimals; nonfinite/out-of-range values become nan. */
void GimbalImuTextFormatValue(float value, char buffer[20]);
/** @brief Format column names and units; transport is USB or UART10. */
size_t GimbalImuTextHeader(const char *transport, char *buffer, size_t capacity);
/** @brief Format one CSV sample; return 0 if the destination is too small. */
size_t GimbalImuTextFormat(const Gimbal_Debug_Data_s *data, uint32_t skipped,
                         char *buffer, size_t capacity);
#endif
