#include "auto_aim.h"
#include "main.h"
#include "auto_aim_config.h"
#include "auto_aim_coordinates.h"
#include "cboard_protocol.h"
#include "gimbal_manual_config.h"
#include "ins_task.h"
#include "vision_usb_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

typedef struct {
    CBoard_Command_s command;
    uint32_t sequence, received_ms;
    uint8_t valid;
} Target_Mailbox_s;
static CBoard_Parser_s parser;
static Target_Mailbox_s latest;
static AutoAim_Diagnostics_s diagnostics;
static uint32_t session_watermark, last_imu_ms, last_status_ms, last_sample_seq;
static uint8_t session_open, sample_sent;
static const float world[4] = AUTO_AIM_WORLD_QUATERNION;
static const float body[4] = AUTO_AIM_BODY_QUATERNION;
static const float solver[4] = AUTO_AIM_SOLVER_QUATERNION;
static float target_world[4], target_body[4];
static uint8_t frames_valid;

void AutoAimInit(void)
{
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float inverse_solver[4] = {solver[0], -solver[1], -solver[2], -solver[3]};
    /* Hero Solver uses C^-1 * q_wire * C, not q_wire directly.
     * Inbound targets must invert the effective transform, including YAML extrinsics. */
    frames_valid = AutoAimOrientation(world, inverse_solver, identity, target_world) &&
        AutoAimOrientation(body, identity, solver, target_body);
    CBoardParserInit(&parser);
    memset(&latest, 0, sizeof(latest));
    memset(&diagnostics, 0, sizeof(diagnostics));
    session_open = sample_sent = 0U;
    session_watermark = last_sample_seq = 0U;
    last_imu_ms = HAL_GetTick() - AUTO_AIM_IMU_PERIOD_MS;
    last_status_ms = HAL_GetTick() - AUTO_AIM_STATUS_PERIOD_MS;
}

void AutoAimReceiveByte(uint8_t byte, uint32_t received_ms)
{
    CBoard_Command_s cmd;
    uint8_t accepted = CBoardParseByteAt(&parser, byte, received_ms, &cmd);
    taskENTER_CRITICAL();
    diagnostics.good_frames = parser.accepted;
    diagnostics.bad_frames = parser.rejected;
    if (accepted)
    {
        latest.command = cmd;
        latest.received_ms = received_ms;
        latest.valid = 1U;
        ++latest.sequence;
        diagnostics.last_rx_ms = received_ms;
        if (cmd.shoot) ++diagnostics.ignored_shoot;
    }
    taskEXIT_CRITICAL();
}

void AutoAimInvalidate(void)
{
    CBoardParserReset(&parser);
    taskENTER_CRITICAL();
    latest.valid = 0U;
    ++latest.sequence;
    taskEXIT_CRITICAL();
}

void AutoAimSelect(Gimbal_Request_s *request, const Gimbal_Control_s *control,
                   uint32_t faults, uint8_t stage, uint32_t now)
{
    Target_Mailbox_s target;
    taskENTER_CRITICAL();
    target = latest;
    taskEXIT_CRITICAL();
    uint8_t eligible = AUTO_AIM_USB_MODE && AUTO_AIM_COORDINATES_CONFIRMED &&
        VisionUSBPortConnected() && request->mode == GIMBAL_REQUEST_AUTO && request->axis_mask == 3U &&
        !faults && !control->faults && control->armed && request->axis_mask == control->axis_mask &&
        (control->last_mode != GIMBAL_REQUEST_OFF || request->sticks_centered) && stage == 2U &&
        isfinite(AUTO_AIM_TRACKING_SPEED_MPS) && AUTO_AIM_TRACKING_SPEED_MPS >= 10.0f &&
        AUTO_AIM_TRACKING_SPEED_MPS <= 20.0f &&
        frames_valid;
    if (!eligible)
    {
        session_open = 0U;
        session_watermark = target.sequence;
    }
    else if (!session_open)
    {
        /* Never reuse a target accepted during OFF/manual/fault or a previous session. */
        session_open = 1U;
        session_watermark = target.sequence;
    }
    uint8_t active = eligible && target.valid && target.sequence != session_watermark &&
        target.command.control && (uint32_t)(now - target.received_ms) <= AUTO_AIM_TIMEOUT_MS;
    if (active)
    {
        active = AutoAimAngles(target.command.yaw, target.command.pitch, target_world, target_body, request->target);
    }
    if (request->mode == GIMBAL_REQUEST_AUTO && !active)
    {
        request->mode = GIMBAL_REQUEST_HOLD;
        request->rate[0] = request->rate[1] = 0.0f;
    }
    taskENTER_CRITICAL();
    diagnostics.mode = eligible;
    diagnostics.target_active = active;
    taskEXIT_CRITICAL();
}

void AutoAimControlResult(const Gimbal_Control_s *control)
{
    if (!control->armed || control->faults ||
        (control->state != GIMBAL_STATE_HOLD && control->state != GIMBAL_STATE_AUTO))
    {
        taskENTER_CRITICAL();
        diagnostics.mode = diagnostics.target_active = 0U;
        taskEXIT_CRITICAL();
    }
}

void AutoAimGetDiagnostics(AutoAim_Diagnostics_s *result)
{
    if (!result) return;
    taskENTER_CRITICAL();
    *result = diagnostics;
    taskEXIT_CRITICAL();
}

void AutoAimComm(void)
{
    if (!VisionUSBPortConnected())
    {
        AutoAimInvalidate();
        return;
    }
    uint32_t now = HAL_GetTick();
    uint8_t frame[CBOARD_ATTITUDE_SIZE + CBOARD_STATUS_SIZE];
    uint16_t length = 0U;
    uint8_t imu_added = 0U, status_added = 0U;
    INS_Vision_Snapshot_t imu;
    float q[4];
    uint8_t available = INS_GetVisionSnapshot(&imu);
    now = HAL_GetTick(); /* A higher-priority sensor may publish while the copy is taken. */
    uint8_t fresh = available && imu.valid && imu.calibrated &&
        (uint32_t)(now - imu.timestamp_ms) <= GIMBAL_IMU_TIMEOUT_US / 1000U &&
        AutoAimOrientation(imu.q, world, body, q);
    if (fresh && (uint32_t)(now - last_imu_ms) >= AUTO_AIM_IMU_PERIOD_MS &&
        (!sample_sent || imu.sample_seq != last_sample_seq) && CBoardPackAttitude(q, frame))
    {
        length = CBOARD_ATTITUDE_SIZE;
        imu_added = 1U;
    }
    if ((uint32_t)(now - last_status_ms) >= AUTO_AIM_STATUS_PERIOD_MS)
    {
        AutoAim_Diagnostics_s d;
        AutoAimGetDiagnostics(&d);
        float speed = AUTO_AIM_TRACKING_SPEED_MPS;
        uint8_t speed_valid = isfinite(speed) && speed >= 10.0f && speed <= 20.0f;
        status_added = CBoardPackStatus(speed_valid ? speed : 0.0f,
            fresh && speed_valid ? d.mode : 0U, 0.0f, frame + length);
        if (status_added) length += CBOARD_STATUS_SIZE;
    }
    if (length && VisionUSBPortTransmit(frame, length) == VISION_USB_PORT_OK)
    {
        if (imu_added)
        {
            last_imu_ms = now;
            last_sample_seq = imu.sample_seq;
            sample_sent = 1U;
        }
        if (status_added) last_status_ms = now;
    }
}
