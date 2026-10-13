/** @file gimbal_imu_text.c @brief HAL-independent IMU text formatting. */
#include "gimbal_imu_text.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define IMU_RAD_TO_DEG 57.29577951308232f

/* Integer formatting avoids pulling nano printf floating-point support in. */
void GimbalImuTextFormatValue(float value, char buffer[20])
{
    if (!isfinite(value) || fabsf(value) > 1000000.0f)
    {
        strcpy(buffer, "nan");
        return;
    }
    uint32_t scaled = (uint32_t)(fabsf(value) * 1000.0f + 0.5f);
    snprintf(buffer, 20U, "%s%lu.%03lu", value < 0.0f && scaled ? "-" : "",
             (unsigned long)(scaled / 1000U), (unsigned long)(scaled % 1000U));
}

static size_t CheckedLength(int length, char *buffer, size_t capacity)
{
    if (length < 0 || (size_t)length >= capacity)
    {
        buffer[0] = '\0';
        return 0U;
    }
    return (size_t)length;
}

size_t GimbalImuTextHeader(const char *transport, char *buffer, size_t capacity)
{
    if (!transport || !buffer || !capacity) return 0U;
    int length = snprintf(buffer, capacity,
        "# %s_IMU v2; angles=INS deg; gyro/accel=sensor axes before mounting rotation\r\n"
        "#type,t_ms,imu_seq,valid,cal,imu_age_ms,yaw_deg,pitch_deg,roll_deg,"
        "gx_dps,gy_dps,gz_dps,ax_ms2,ay_ms2,az_ms2,yaw_ecd,pitch_ecd,"
        "yaw_age_ms,pitch_age_ms,stage,state,fault,yaw_out,pitch_out,tx_skip,"
        "rc_online,sw_left,sw_right,stick_yaw,stick_pitch,centered,request,axis_mask\r\n", transport);
    return CheckedLength(length, buffer, capacity);
}

size_t GimbalImuTextFormat(const Gimbal_Debug_Data_s *data, uint32_t skipped,
                         char *buffer, size_t capacity)
{
    if (!data || !buffer || !capacity) return 0U;
    char number[9][20];
    GimbalImuTextFormatValue(data->raw_yaw, number[0]);
    GimbalImuTextFormatValue(data->raw_pitch, number[1]);
    GimbalImuTextFormatValue(data->raw_roll, number[2]);
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        GimbalImuTextFormatValue(data->sensor_gyro[i] * IMU_RAD_TO_DEG, number[3U + i]);
        GimbalImuTextFormatValue(data->sensor_accel[i], number[6U + i]);
    }
    int length = snprintf(buffer, capacity,
        "IMU,%lu,%lu,%u,%u,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%u,%lu,%lu,%u,%u,%lu,%d,%d,%lu,%u,%u,%u,%d,%d,%u,%u,%u\r\n",
        (unsigned long)data->tick_ms, (unsigned long)data->imu_seq,
        (unsigned)data->imu_valid, (unsigned)data->imu_calibrated,
        (unsigned long)data->imu_age_ms, number[0], number[1], number[2],
        number[3], number[4], number[5], number[6], number[7], number[8],
        (unsigned)data->ecd[0], (unsigned)data->ecd[1],
        (unsigned long)data->motor_age_ms[0], (unsigned long)data->motor_age_ms[1],
        (unsigned)data->stage, (unsigned)data->control.state,
        (unsigned long)data->control.faults, (int)data->output[0], (int)data->output[1],
        (unsigned long)skipped, (unsigned)data->command.online,
        (unsigned)data->command.switch_left, (unsigned)data->command.switch_right,
        (int)data->command.stick_yaw, (int)data->command.stick_pitch,
        (unsigned)data->command.request.sticks_centered,
        (unsigned)data->command.request.mode, (unsigned)data->command.request.axis_mask);
    return CheckedLength(length, buffer, capacity);
}
