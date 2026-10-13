/** @file 2yaw_gimbal.c
 * @brief Feedback/actuator adapter for the HAL-independent gimbal controller.
 */
#include "2yaw_gimbal.h"
#include "gimbal_manual_config.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "task.h"
#include <math.h>
#include <string.h>

static DJIMotorInstance *motors[2];
static Subscriber_t *command_sub;
static Gimbal_Manual_Cmd_s command;
static Gimbal_Control_s control;
static Gimbal_Axis_Feedback_s feedback[2];
static INS_Vision_Snapshot_t imu;
static Pitch_Limit_Calibration_s pitch_cal;
static Gimbal_Debug_Data_s debug;
static uint32_t start_ms, last_step_ms;
static uint8_t imu_seen, yaw_seen;
static float previous_angle[2], yaw_origin;

static const Gimbal_Axis_Config_s default_configs[2] = {
    {GIMBAL_YAW_ANGLE_KP, GIMBAL_YAW_SPEED_KP, GIMBAL_YAW_SPEED_KI,
     GIMBAL_YAW_INTEGRAL_LIMIT, GIMBAL_YAW_OUTPUT_LIMIT, GIMBAL_YAW_MAX_SPEED_DPS * GIMBAL_RAD_PER_DEG,
     GIMBAL_YAW_ACCEL_DPS2 * GIMBAL_RAD_PER_DEG, GIMBAL_YAW_FOLLOW_ERROR_DEG * GIMBAL_RAD_PER_DEG,
     GIMBAL_YAW_ECD_TO_ANGLE_SIGN, 0.0f, GIMBAL_YAW_RATE_FILTER_HZ},
    {GIMBAL_PITCH_ANGLE_KP, GIMBAL_PITCH_SPEED_KP, GIMBAL_PITCH_SPEED_KI,
     GIMBAL_PITCH_INTEGRAL_LIMIT, GIMBAL_PITCH_OUTPUT_LIMIT, GIMBAL_PITCH_MAX_SPEED_DPS * GIMBAL_RAD_PER_DEG,
     GIMBAL_PITCH_ACCEL_DPS2 * GIMBAL_RAD_PER_DEG, GIMBAL_PITCH_FOLLOW_ERROR_DEG * GIMBAL_RAD_PER_DEG,
     GIMBAL_PITCH_ECD_TO_ANGLE_SIGN, GIMBAL_PITCH_SOFT_RECOVERY_DPS * GIMBAL_RAD_PER_DEG,
     GIMBAL_PITCH_RATE_FILTER_HZ},
};
static Gimbal_Axis_Config_s configs[2];
static uint8_t active_stage = GIMBAL_CONTROL_STAGE;
static const float output_sign[2] = {GIMBAL_YAW_OUTPUT_SIGN, GIMBAL_PITCH_OUTPUT_SIGN};

static Motor_Init_Config_s MotorConfig(FDCAN_HandleTypeDef *can, uint8_t id)
{
    Motor_Init_Config_s cfg = {
        .can_init_config = {.can_handle = can, .tx_id = id},
        .controller_setting_init_config = {.outer_loop_type = OPEN_LOOP, .close_loop_type = OPEN_LOOP},
        .motor_type = GM6020,
    };
    return cfg;
}

void GimbalInit(void)
{
    memcpy(configs, default_configs, sizeof(configs));
#if GIMBAL_BRINGUP_ENABLE_IMU
    INS_SetMounting(GIMBAL_IMU_MOUNT_YAW_DEG, GIMBAL_IMU_MOUNT_PITCH_DEG, GIMBAL_IMU_MOUNT_ROLL_DEG);
    (void)INS_Init();
#endif
    Motor_Init_Config_s yaw = MotorConfig(GIMBAL_YAW_CAN_HANDLE, GIMBAL_YAW_MOTOR_ID);
    Motor_Init_Config_s pitch = MotorConfig(GIMBAL_PITCH_CAN_HANDLE, GIMBAL_PITCH_MOTOR_ID);
    motors[0] = DJIMotorInit(&yaw);
    motors[1] = DJIMotorInit(&pitch);
    for (uint8_t i = 0; i < 2U; ++i)
    {
        DJIMotorStop(motors[i]);
        DJIMotorSetRef(motors[i], 0.0f);
        motors[i]->feedback_timeout_ms = GIMBAL_MOTOR_TIMEOUT_MS;
        motors[i]->command_timeout_ms = GIMBAL_OUTPUT_TIMEOUT_MS;
    }
    command_sub = SubRegister("gimbal_manual", sizeof(Gimbal_Manual_Cmd_s));
    command.request.axis_mask = 1U;
    GimbalControlInit(&control);
    GimbalPitchLimitLoadConfigured();
    start_ms = last_step_ms = HAL_GetTick();
}

void GimbalTask(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t faults = 0U;
    float dt = (float)(uint32_t)(now - last_step_ms) * 0.001f;
    last_step_ms = now;
    (void)SubGetMessage(command_sub, &command);
    if (!command.online || (uint32_t)(now - command.timestamp_ms) > GIMBAL_COMMAND_TIMEOUT_MS)
        faults |= GIMBAL_FAULT_RC;

    /* A preempted snapshot publication may fail once; retain the last coherent sample. */
    INS_Vision_Snapshot_t sample;
    if (INS_GetVisionSnapshot(&sample)) imu = sample;
    now = HAL_GetTick(); /* sample may have been published while this task was preempted */
    uint32_t imu_age = (uint32_t)(now - imu.timestamp_ms);
    float rate[2] = {0};
    float wrapped[2] = {imu.yaw * GIMBAL_RAD_PER_DEG, imu.pitch * GIMBAL_RAD_PER_DEG};
    float norm = 0.0f;
    for (uint8_t i = 0; i < 4U; ++i) norm += imu.q[i] * imu.q[i];
    if (!imu.valid || !imu.calibrated || imu_age > GIMBAL_IMU_TIMEOUT_US / 1000U ||
        (uint32_t)(now - start_ms) < GIMBAL_IMU_SETTLE_MS ||
        !isfinite(norm) || norm < 0.8f || norm > 1.2f || !isfinite(imu.yaw) ||
        !GimbalEulerRates(wrapped[1], imu.roll * GIMBAL_RAD_PER_DEG, imu.gyro, rate))
        faults |= GIMBAL_FAULT_IMU;
    if (!(faults & GIMBAL_FAULT_IMU))
    {
        for (uint8_t i = 0; i < 2U; ++i)
        {
            feedback[i].angle = imu_seen ? feedback[i].angle + GimbalAngleDelta(wrapped[i], previous_angle[i]) : wrapped[i];
            previous_angle[i] = wrapped[i];
            feedback[i].rate = rate[i];
            if (fabsf(rate[i]) > GIMBAL_MAX_MEASURED_RATE_RPS) faults |= GIMBAL_FAULT_IMU;
        }
        imu_seen = 1U;
    }

    DJI_Motor_Measure_s measure[2];
    uint32_t rx_count[2], motor_age[2];
    uint8_t watchdog[2];
    uint32_t primask = __get_PRIMASK();
    __disable_irq(); /* CAN callbacks own these fields. Keep the copy short. */
    for (uint8_t i = 0; i < 2U; ++i)
    {
        measure[i] = motors[i]->measure;
        rx_count[i] = motors[i]->rx_count;
        motor_age[i] = (uint32_t)(HAL_GetTick() - motors[i]->last_rx_ms);
        watchdog[i] = motors[i]->watchdog_fault;
    }
    __set_PRIMASK(primask);
    for (uint8_t i = 0; i < 2U; ++i)
    {
        if (!rx_count[i] || motor_age[i] > GIMBAL_MOTOR_TIMEOUT_MS) faults |= GIMBAL_FAULT_MOTOR;
        if (watchdog[i]) faults |= GIMBAL_FAULT_TIMING;
        if (measure[i].temperature >= GIMBAL_MOTOR_MAX_TEMP_C) faults |= GIMBAL_FAULT_TEMPERATURE;
        if (fabsf(output_sign[i]) != 1.0f) faults |= GIMBAL_FAULT_CONFIG;
    }
    if (rx_count[0] && !yaw_seen)
    {
        yaw_origin = measure[0].total_angle;
        yaw_seen = 1U;
    }
    feedback[0].joint = (measure[0].total_angle - yaw_origin) * GIMBAL_RAD_PER_DEG;
    feedback[0].soft_min = -GIMBAL_YAW_SOFT_TRAVEL_DEG * GIMBAL_RAD_PER_DEG;
    feedback[0].soft_max = -feedback[0].soft_min;
    feedback[0].hard_min = -GIMBAL_YAW_HARD_TRAVEL_DEG * GIMBAL_RAD_PER_DEG;
    feedback[0].hard_max = -feedback[0].hard_min;
    feedback[0].continuous_rotation = GIMBAL_YAW_CONTINUOUS_ROTATION;

    int32_t span = GimbalEncoderDelta(pitch_cal.upper_ecd, pitch_cal.lower_ecd);
    int32_t direction = span >= 0 ? 1 : -1;
    int32_t span_abs = span * direction;
    float count_rad = 6.283185307179586f / 8192.0f;
    /* Keep joint coordinates increasing with ECD even if endpoints were captured in reverse. */
    feedback[1].joint = (float)GimbalEncoderDelta(measure[1].ecd, pitch_cal.lower_ecd) * count_rad;
    feedback[1].hard_min = fminf(0.0f, (float)span * count_rad);
    feedback[1].hard_max = fmaxf(0.0f, (float)span * count_rad);
    feedback[1].soft_min = feedback[1].hard_min + GIMBAL_PITCH_LIMIT_MARGIN_ECD * count_rad;
    feedback[1].soft_max = feedback[1].hard_max - GIMBAL_PITCH_LIMIT_MARGIN_ECD * count_rad;
    if ((!pitch_cal.lower_valid || !pitch_cal.upper_valid || span_abs <= 2 * GIMBAL_PITCH_LIMIT_MARGIN_ECD) &&
        (command.request.axis_mask & 2U)) faults |= GIMBAL_FAULT_CONFIG;
    uint8_t stage = GIMBAL_MOTOR_OUTPUT_ENABLED ? active_stage : 0U;
    Gimbal_Request_s request = command.request;
    for (uint8_t i = 0U; i < 2U; ++i)
        request.rate[i] *= configs[i].speed_limit / default_configs[i].speed_limit;
    GimbalControlStep(&control, configs, feedback, &request, dt, faults, stage);

    /* One short critical section prevents the motor task seeing a half-updated command pair. */
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < 2U; ++i)
    {
        uint8_t enabled = (control.state == GIMBAL_STATE_HOLD || control.state == GIMBAL_STATE_MANUAL) &&
                          (control.axis_mask & (1U << i));
        float command_output = enabled ? output_sign[i] * control.axis[i].output : 0.0f;
        DJIMotorSetRef(motors[i], enabled ? command_output : 0.0f);
        if (enabled) DJIMotorEnable(motors[i]);
        else DJIMotorStop(motors[i]);
        /* Clearing a driver timeout is permitted only on an explicit healthy-RC OFF request.
         * The core still requires healthy feedback and centered sticks on a subsequent tick.
         */
        if (command.online && command.request.mode == GIMBAL_REQUEST_OFF)
            motors[i]->watchdog_fault = 0U;
    }
    debug.control = control;
    memcpy(debug.feedback, feedback, sizeof(feedback));
    debug.command = command;
    debug.tick_ms = now;
    debug.imu_age_ms = imu_age;
    debug.imu_seq = imu.sample_seq;
    debug.imu_calibrated = imu.calibrated;
    debug.stage = stage;
    memcpy(debug.config, configs, sizeof(configs));
    debug.raw_yaw = imu.yaw;
    debug.raw_pitch = imu.pitch;
    debug.raw_roll = imu.roll;
    memcpy(debug.gyro, imu.gyro, sizeof(debug.gyro));
    memcpy(debug.sensor_gyro, imu.sensor_gyro, sizeof(debug.sensor_gyro));
    memcpy(debug.sensor_accel, imu.sensor_accel, sizeof(debug.sensor_accel));
    debug.imu_valid = imu.valid;
    debug.dt = dt;
    for (uint8_t i = 0; i < 2U; ++i)
    {
        debug.motor_age_ms[i] = motor_age[i];
        debug.rx_count[i] = rx_count[i];
        debug.ecd[i] = measure[i].ecd;
        debug.output[i] = motors[i]->last_set;
        debug.temperature[i] = measure[i].temperature;
        debug.motor_current_raw[i] = measure[i].torque_current_raw;
        debug.motor_rpm_raw[i] = measure[i].speed_rpm_raw;
        debug.tx_enqueue_ok[i] = motors[i]->tx_enqueue_ok;
        debug.tx_enqueue_fail[i] = motors[i]->tx_enqueue_fail;
    }
    taskEXIT_CRITICAL();
}

void GimbalDataTask(void) {}

void GimbalGetDebugData(Gimbal_Debug_Data_s *data)
{
    if (!data) return;
    taskENTER_CRITICAL();
    *data = debug;
    taskEXIT_CRITICAL();
}

static uint8_t CanCalibrate(void)
{
    return command.online && command.request.mode == GIMBAL_REQUEST_OFF &&
           (uint32_t)(HAL_GetTick() - command.timestamp_ms) <= GIMBAL_COMMAND_TIMEOUT_MS &&
           control.state != GIMBAL_STATE_HOLD && control.state != GIMBAL_STATE_MANUAL &&
           motors[1]->rx_count && (uint32_t)(HAL_GetTick() - motors[1]->last_rx_ms) <= GIMBAL_MOTOR_TIMEOUT_MS;
}

static void StopForTuning(void)
{
    GimbalControlInit(&control);
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        DJIMotorStop(motors[i]);
        DJIMotorSetRef(motors[i], 0.0f);
    }
    debug.control = control;
    debug.stage = GIMBAL_MOTOR_OUTPUT_ENABLED ? active_stage : 0U;
    memcpy(debug.config, configs, sizeof(configs));
    taskEXIT_CRITICAL();
}

Vofa_Result_e GimbalApplyVofaCommand(const Vofa_Command_s *cmd, float *value)
{
    *value = 0.0f;
    uint8_t off = command.online && command.request.mode == GIMBAL_REQUEST_OFF &&
                  command.request.sticks_centered &&
                  (uint32_t)(HAL_GetTick() - command.timestamp_ms) <= GIMBAL_COMMAND_TIMEOUT_MS &&
                  control.state != GIMBAL_STATE_HOLD && control.state != GIMBAL_STATE_MANUAL;
    switch (cmd->type)
    {
    case VOFA_CMD_STAGE:
        if (!isfinite(cmd->value) || cmd->value < 0.0f || cmd->value > 2.0f || floorf(cmd->value) != cmd->value)
            return VOFA_RESULT_RANGE;
        if (cmd->value > 0.0f && !GIMBAL_MOTOR_OUTPUT_ENABLED) return VOFA_RESULT_OUTPUT_DISABLED;
        if (cmd->value > 0.0f && !off) return VOFA_RESULT_REQUIRE_OFF;
        active_stage = (uint8_t)cmd->value;
        StopForTuning();
        *value = active_stage;
        return VOFA_RESULT_OK;
    case VOFA_CMD_SET:
    case VOFA_CMD_GET:
        if (cmd->axis > 1U || cmd->parameter < VOFA_PARAM_ANGLE_KP || cmd->parameter > VOFA_PARAM_FOLLOW_ERROR_DEG)
            return VOFA_RESULT_RANGE;
        if (cmd->type == VOFA_CMD_GET)
        {
            *value = VofaGetParameter(&configs[cmd->axis], cmd->parameter);
            return VOFA_RESULT_OK;
        }
        if (!off) return VOFA_RESULT_REQUIRE_OFF;
        float speed_ceiling = cmd->axis == 0U ? GIMBAL_YAW_MAX_SPEED_DPS : GIMBAL_PITCH_MAX_SPEED_DPS;
        float output_ceiling = cmd->parameter == VOFA_PARAM_OUTPUT_LIMIT ?
            (cmd->axis == 0U ? GIMBAL_YAW_VOLTAGE_LIMIT : GIMBAL_PITCH_VOLTAGE_LIMIT) :
            GIMBAL_VOLTAGE_LIMIT;
        float acceleration_ceiling = cmd->axis == 0U ? GIMBAL_YAW_ACCEL_DPS2 : GIMBAL_PITCH_ACCEL_DPS2;
        Vofa_Result_e result = VofaValidateParameter(cmd, output_ceiling, speed_ceiling, acceleration_ceiling);
        if (result != VOFA_RESULT_OK) return result;
        *value = VofaSetParameter(&configs[cmd->axis], cmd);
        StopForTuning();
        return VOFA_RESULT_OK;
    case VOFA_CMD_DEFAULTS:
        if (!off) return VOFA_RESULT_REQUIRE_OFF;
        memcpy(configs, default_configs, sizeof(configs));
        active_stage = GIMBAL_CONTROL_STAGE;
        StopForTuning();
        *value = active_stage;
        return VOFA_RESULT_OK;
    case VOFA_CMD_CAL_LOWER:
    case VOFA_CMD_CAL_UPPER:
    case VOFA_CMD_CAL_RESET:
        if (!off || !CanCalibrate()) return VOFA_RESULT_REQUIRE_OFF;
        if (cmd->type == VOFA_CMD_CAL_LOWER) GimbalPitchLimitCaptureLower();
        if (cmd->type == VOFA_CMD_CAL_UPPER) GimbalPitchLimitCaptureUpper();
        if (cmd->type == VOFA_CMD_CAL_RESET) GimbalPitchLimitReset();
        StopForTuning();
        *value = cmd->type == VOFA_CMD_CAL_RESET ? 0.0f :
                 (float)(cmd->type == VOFA_CMD_CAL_LOWER ? pitch_cal.lower_ecd : pitch_cal.upper_ecd);
        return VOFA_RESULT_OK;
    default: return VOFA_RESULT_SYNTAX;
    }
}

void GimbalPitchLimitReset(void)
{
    if (!CanCalibrate()) return;
    memset(&pitch_cal, 0, sizeof(pitch_cal));
    GimbalControlInit(&control);
}

void GimbalPitchLimitLoadConfigured(void)
{
    pitch_cal.lower_ecd = GIMBAL_PITCH_LIMIT_LOWER_ECD;
    pitch_cal.upper_ecd = GIMBAL_PITCH_LIMIT_UPPER_ECD;
    pitch_cal.lower_valid = pitch_cal.upper_valid = 1U;
}

void GimbalPitchLimitCaptureLower(void)
{
    if (!CanCalibrate()) return;
    pitch_cal.lower_ecd = motors[1]->measure.ecd;
    pitch_cal.lower_valid = 1U;
    GimbalControlInit(&control);
}

void GimbalPitchLimitCaptureUpper(void)
{
    if (!CanCalibrate()) return;
    pitch_cal.upper_ecd = motors[1]->measure.ecd;
    pitch_cal.upper_valid = 1U;
    GimbalControlInit(&control);
}

void GimbalPitchLimitGet(Pitch_Limit_Calibration_s *calibration)
{
    if (!calibration) return;
    taskENTER_CRITICAL();
    *calibration = pitch_cal;
    taskEXIT_CRITICAL();
    int32_t span = GimbalEncoderDelta(calibration->upper_ecd, calibration->lower_ecd);
    int32_t direction = span >= 0 ? 1 : -1;
    calibration->current_ecd = motors[1]->measure.ecd;
    calibration->lower_to_upper_counts = span;
    calibration->span_counts = span * direction;
    calibration->span_deg_x10 = calibration->span_counts * 3600 / 8192;
    calibration->safe_lower_ecd = (calibration->lower_ecd + direction * GIMBAL_PITCH_LIMIT_MARGIN_ECD + 8192) % 8192;
    calibration->safe_upper_ecd = (calibration->upper_ecd - direction * GIMBAL_PITCH_LIMIT_MARGIN_ECD + 8192) % 8192;
    int32_t position = GimbalEncoderDelta(calibration->current_ecd, calibration->lower_ecd) * direction;
    calibration->current_safe = calibration->lower_valid && calibration->upper_valid &&
        position >= GIMBAL_PITCH_LIMIT_MARGIN_ECD && position <= calibration->span_counts - GIMBAL_PITCH_LIMIT_MARGIN_ECD;
}
