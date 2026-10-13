/** @file 2yaw_gimbal.h @brief GM6020 gimbal application and diagnostic interface. */
#ifndef _2YAW_GIMBAL_H
#define _2YAW_GIMBAL_H
#include <stdint.h>
#include "gimbal_control.h"
#include "vofa_protocol.h"

typedef struct {
    Gimbal_Request_s request;
    uint32_t timestamp_ms;
    uint8_t online;
    uint8_t switch_left, switch_right;
    int16_t stick_yaw, stick_pitch;
} Gimbal_Manual_Cmd_s;

typedef struct {
    uint8_t lower_valid, upper_valid, current_safe;
    uint16_t current_ecd, lower_ecd, upper_ecd, safe_lower_ecd, safe_upper_ecd;
    int32_t lower_to_upper_counts, span_counts, span_deg_x10;
} Pitch_Limit_Calibration_s;

typedef struct {
    Gimbal_Control_s control;
    Gimbal_Axis_Feedback_s feedback[2];
    Gimbal_Manual_Cmd_s command;
    uint32_t tick_ms, imu_age_ms, imu_seq, motor_age_ms[2], rx_count[2];
    uint16_t ecd[2];
    int16_t output[2];
    int16_t motor_current_raw[2], motor_rpm_raw[2];
    uint32_t tx_enqueue_ok[2], tx_enqueue_fail[2];
    uint8_t temperature[2];
    uint8_t imu_calibrated;
    uint8_t stage;
    Gimbal_Axis_Config_s config[2];
    float raw_yaw, raw_pitch, raw_roll, gyro[3], dt;
    float sensor_gyro[3], sensor_accel[3];
    uint8_t imu_valid;
} Gimbal_Debug_Data_s;

/** @brief Initialize INS, motors and command subscriber before RTOS start. */
void GimbalInit(void);
/** @brief Consume command and feedback; run in the control task. */
void GimbalTask(void);
void GimbalDataTask(void);
/** @brief Coherent task-context debug copy. */
void GimbalGetDebugData(Gimbal_Debug_Data_s *debug_data);
/** @brief Execute a validated tuning command in the CONTROL task only. */
Vofa_Result_e GimbalApplyVofaCommand(const Vofa_Command_s *command, float *applied_value);
/* Call calibration APIs only in the control task; capture requires healthy RC OFF. */
void GimbalPitchLimitReset(void);
void GimbalPitchLimitLoadConfigured(void);
void GimbalPitchLimitCaptureLower(void);
void GimbalPitchLimitCaptureUpper(void);
void GimbalPitchLimitGet(Pitch_Limit_Calibration_s *calibration);
#endif
