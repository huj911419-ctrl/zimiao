/** @file gimbal_control.h
 * @brief HAL-independent cascade and explicit rearming state machine.
 * Angles/rates use rad/rad/s. Output units belong to the actuator adapter.
 */
#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include <stdint.h>
#define GIMBAL_RAD_PER_DEG 0.017453292519943295f
typedef enum { GIMBAL_REQUEST_OFF, GIMBAL_REQUEST_HOLD, GIMBAL_REQUEST_MANUAL, GIMBAL_REQUEST_AUTO } Gimbal_Request_e;
typedef enum { GIMBAL_STATE_OFF, GIMBAL_STATE_READY, GIMBAL_STATE_HOLD,
               GIMBAL_STATE_MANUAL, GIMBAL_STATE_FAULT, GIMBAL_STATE_AUTO } Gimbal_State_e;
typedef enum {
    GIMBAL_FAULT_NONE = 0, GIMBAL_FAULT_RC = 1U, GIMBAL_FAULT_IMU = 2U,
    GIMBAL_FAULT_MOTOR = 4U, GIMBAL_FAULT_LIMIT = 8U,
    GIMBAL_FAULT_TIMING = 16U, GIMBAL_FAULT_VALUE = 32U,
    GIMBAL_FAULT_TEMPERATURE = 64U, GIMBAL_FAULT_CONFIG = 128U
} Gimbal_Fault_e;
typedef struct {
    float angle_kp, speed_kp, speed_ki;
    float integral_limit, output_limit, speed_limit, acceleration_limit, max_follow_error;
    float encoder_to_angle_sign;
    float soft_recovery_speed; /* inward recovery/hold outside soft bounds; 0 disables */
    float rate_filter_hz; /* one-pole rate feedback cutoff, Hz; 0 bypasses */
} Gimbal_Axis_Config_s;
typedef struct {
    float angle, rate, joint; /* unwrapped encoder joint position, rad */
    float soft_min, soft_max, hard_min, hard_max;
    uint8_t continuous_rotation; /* 1 bypasses travel bounds only; other guards remain */
} Gimbal_Axis_Feedback_s;
typedef struct {
    Gimbal_Request_e mode;
    uint8_t axis_mask; /* bit 0 yaw, bit 1 pitch */
    uint8_t sticks_centered;
    float rate[2];
    float target[2]; /* AUTO only: absolute wrapped radians, never velocity/integrated input */
} Gimbal_Request_s;
typedef struct {
    float target, manual_rate, rate_target, integral, output;
    uint8_t rate_control_started; /* preserve neutral PI after deliberate recovery or safe-region enable */
    float filtered_rate;
    uint8_t rate_filter_initialized;
    float last_manual_request; /* detect a commanded-motion -> neutral transition */
} Gimbal_Axis_State_s;
typedef struct {
    Gimbal_State_e state;
    uint32_t faults;
    uint8_t armed, axis_mask;
    Gimbal_Request_e last_mode;
    Gimbal_Axis_State_s axis[2];
} Gimbal_Control_s;
/** @brief Reset to OFF; healthy feedback and centered sticks in OFF must precede enable. */
void GimbalControlInit(Gimbal_Control_s *control);
/** @brief Tick; stage 0=observe, 1=rate PI, 2=angle P + velocity feedforward + rate PI.
 * Stage 2 manual release generates a stop from measured pose/rate once, then holds.
 */
void GimbalControlStep(Gimbal_Control_s *control, const Gimbal_Axis_Config_s config[2],
                       const Gimbal_Axis_Feedback_s feedback[2], const Gimbal_Request_s *request,
                       float dt, uint32_t faults, uint8_t stage);
/** @brief Shortest signed radian difference; finite inputs required. */
float GimbalAngleDelta(float to, float from);
/** @brief Shortest signed difference of 13-bit encoder positions. */
int32_t GimbalEncoderDelta(uint16_t to, uint16_t from);
/** @brief Repository convention: Pitch=X rotation, Roll=Y rotation. */
uint8_t GimbalEulerRates(float pitch, float roll, const float gyro[3], float rate[2]);
#endif
