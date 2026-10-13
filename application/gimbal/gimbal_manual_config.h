/** @file gimbal_manual_config.h
 * @brief Bench configuration. Verify signs and tune gains on the actual mount.
 */
#ifndef GIMBAL_MANUAL_CONFIG_H
#define GIMBAL_MANUAL_CONFIG_H
#define GIMBAL_YAW_CAN_HANDLE (&hfdcan1)
#define GIMBAL_PITCH_CAN_HANDLE (&hfdcan2)
#define GIMBAL_YAW_MOTOR_ID 1U
#define GIMBAL_PITCH_MOTOR_ID 2U
#define GIMBAL_BRINGUP_ENABLE_IMU 1U
#define GIMBAL_MOTOR_OUTPUT_ENABLED 1U
/* 0: observe only, 1: rate PI tuning, 2: attitude P + rate PI. */
/* Position hold + manual velocity feedforward; explicit centered RC OFF precedes enable. */
#define GIMBAL_CONTROL_STAGE 2U
/* Stick direction is independent of feedback/output polarity. */
/* Bench check: right stick is positive; physical right turn decreases INS Yaw. */
#define GIMBAL_YAW_COMMAND_SIGN (-1.0f)
#define GIMBAL_PITCH_COMMAND_SIGN (-1.0f)
/* Map INS-coordinate output to CAN voltage. Yaw verified on the bench.
 * Pitch: positive CAN raised ECD and decreased INS Pitch in the loaded trial. */
#define GIMBAL_YAW_OUTPUT_SIGN 1.0f
#define GIMBAL_PITCH_OUTPUT_SIGN (-1.0f)
/* Increasing ECD -> increasing INS angle; assumes 1:1 direct drive. */
#define GIMBAL_YAW_ECD_TO_ANGLE_SIGN 1.0f
#define GIMBAL_PITCH_ECD_TO_ANGLE_SIGN (-1.0f)
/* Pitch=X rotation, Roll=Y rotation in this repository. Mount offsets in degrees. */
#define GIMBAL_IMU_MOUNT_YAW_DEG 0.0f
#define GIMBAL_IMU_MOUNT_PITCH_DEG 0.0f
#define GIMBAL_IMU_MOUNT_ROLL_DEG 0.0f
/* Measured pitch calibration retained from 2026-09-29. */
#define GIMBAL_PITCH_LIMIT_LOWER_ECD 384U
#define GIMBAL_PITCH_LIMIT_UPPER_ECD 1394U
#define GIMBAL_PITCH_LIMIT_MARGIN_ECD 100
/* User-confirmed unrestricted Yaw mechanism/cabling: no travel bounds. */
#define GIMBAL_YAW_CONTINUOUS_ROTATION 1U
#if GIMBAL_YAW_CONTINUOUS_ROTATION > 1U
#error "Yaw continuous rotation must be 0 or 1"
#endif
/* Used only when continuous rotation is disabled, relative to first encoder pose. */
#define GIMBAL_YAW_SOFT_TRAVEL_DEG 90.0f
#define GIMBAL_YAW_HARD_TRAVEL_DEG 100.0f
#define GIMBAL_YAW_MAX_SPEED_DPS 240.0f
#define GIMBAL_PITCH_MAX_SPEED_DPS 120.0f
#define GIMBAL_YAW_ACCEL_DPS2 720.0f
#define GIMBAL_PITCH_ACCEL_DPS2 360.0f
/* Endpoint recovery starts with an inward stick; neutral then retains inward PI support. */
#define GIMBAL_PITCH_SOFT_RECOVERY_DPS 30.0f
#define GIMBAL_MAX_FOLLOW_ERROR_DEG 10.0f
/* Bench trial gains: rad, rad/s, and GM6020 voltage-command units.
 * Dual-axis capture showed Pitch near-full-scale alternating output at neutral.
 * Lower Pitch bandwidth and soften acceleration; keep speed/output authority. */
#define GIMBAL_YAW_ANGLE_KP 8.0f
#define GIMBAL_PITCH_ANGLE_KP 4.0f
#define GIMBAL_YAW_SPEED_KP 24000.0f
#define GIMBAL_PITCH_SPEED_KP 10000.0f
#define GIMBAL_YAW_SPEED_KI 15000.0f
#define GIMBAL_PITCH_SPEED_KI 9000.0f
#define GIMBAL_YAW_RATE_FILTER_HZ 50.0f
#define GIMBAL_PITCH_RATE_FILTER_HZ 30.0f
#define GIMBAL_INTEGRAL_LIMIT 1000.0f
/* Integral ceiling and voltage ceilings are command units, not measured torque. */
#define GIMBAL_VOLTAGE_LIMIT 8000.0f
#define GIMBAL_YAW_VOLTAGE_LIMIT 15000.0f
#define GIMBAL_PITCH_VOLTAGE_LIMIT 15000.0f
/* Per-axis boot values; replace an alias with the value measured in VOFA+. */
#define GIMBAL_YAW_INTEGRAL_LIMIT 5000.0f
#define GIMBAL_PITCH_INTEGRAL_LIMIT 8000.0f
#define GIMBAL_YAW_OUTPUT_LIMIT GIMBAL_YAW_VOLTAGE_LIMIT
#define GIMBAL_PITCH_OUTPUT_LIMIT GIMBAL_PITCH_VOLTAGE_LIMIT
#define GIMBAL_YAW_FOLLOW_ERROR_DEG GIMBAL_MAX_FOLLOW_ERROR_DEG
#define GIMBAL_PITCH_FOLLOW_ERROR_DEG GIMBAL_MAX_FOLLOW_ERROR_DEG
#define GIMBAL_RC_FULL_SCALE 660.0f
#define GIMBAL_RC_DEADBAND 80
#define GIMBAL_MOTOR_TIMEOUT_MS 30U
#define GIMBAL_IMU_TIMEOUT_US 10000U
#define GIMBAL_COMMAND_TIMEOUT_MS 30U
#define GIMBAL_OUTPUT_TIMEOUT_MS 20U
#define GIMBAL_IMU_SETTLE_MS 1000U
#define GIMBAL_MOTOR_MAX_TEMP_C 70U
#define GIMBAL_MAX_MEASURED_RATE_RPS 8.0f
/* VOFA+: stream JustFloat over USB CDC at boot, even in observe-only stage. */
#define GIMBAL_VOFA_DEFAULT_RATE_HZ 100U
#define GIMBAL_VOFA_STREAM_ENABLED 1U
/* Temporary USB text capture at boot; imu 0 restores JustFloat, imu 1 selects text. */
#define GIMBAL_USB_IMU_TEXT_DEFAULT 1U
#if GIMBAL_USB_IMU_TEXT_DEFAULT > 1U
#error "USB IMU text default must be 0 or 1"
#endif
/* Independent USART10 ASCII capture; existing USB JustFloat stays unchanged. */
#define GIMBAL_IMU_UART_ENABLED 1U
#define GIMBAL_IMU_UART_PERIOD_MS 50U
#if GIMBAL_IMU_UART_PERIOD_MS < 50U
#error "USART10 IMU text requires at least 50 ms per transmission at 115200 baud"
#endif
#if GIMBAL_VOFA_DEFAULT_RATE_HZ != 10U && GIMBAL_VOFA_DEFAULT_RATE_HZ != 20U && \
    GIMBAL_VOFA_DEFAULT_RATE_HZ != 25U && GIMBAL_VOFA_DEFAULT_RATE_HZ != 50U && \
    GIMBAL_VOFA_DEFAULT_RATE_HZ != 100U
#error "VOFA+ telemetry rate must be 10, 20, 25, 50 or 100 Hz"
#endif
#endif
