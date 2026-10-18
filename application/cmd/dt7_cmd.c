/** @file dt7_cmd.c @brief Validated DT7 input -> rad/s gimbal command mailbox. */
#include "dt7_cmd.h"
#include "2yaw_gimbal.h"
#include "gimbal_manual_config.h"
#include "auto_aim_config.h"
#include "message_center.h"
#include "remote_control.h"
#include "gimbal_key_mouse_config.h"
#if GIMBAL_KEY_MOUSE_ENABLED
#include "gimbal_key_mouse.h"
static Gimbal_Key_Mouse_s key_mouse;
#endif
#include <math.h>
#include <string.h>

static Publisher_t *gimbal_cmd_pub;

#if !GIMBAL_KEY_MOUSE_ENABLED
static float StickRate(int16_t value, float maximum)
{
    float magnitude = fabsf((float)value);
    if (magnitude <= GIMBAL_RC_DEADBAND) return 0.0f;
    float fraction = (magnitude - GIMBAL_RC_DEADBAND) /
                     (GIMBAL_RC_FULL_SCALE - GIMBAL_RC_DEADBAND);
    return copysignf(fminf(fraction, 1.0f) * maximum * GIMBAL_RAD_PER_DEG, (float)value);
}
#endif

void RobotCMDInit(void)
{
    (void)RemoteControlInit(&huart5);
    gimbal_cmd_pub = PubRegister("gimbal_manual", sizeof(Gimbal_Manual_Cmd_s));
#if GIMBAL_KEY_MOUSE_ENABLED
    GimbalKeyMouseInit(&key_mouse);
#endif
}

void RobotCMDTask(void)
{
    RC_ctrl_t rc = {0};
    Gimbal_Manual_Cmd_s cmd = {0};
#if GIMBAL_KEY_MOUSE_ENABLED
    uint32_t age_ms;
    cmd.online = RemoteControlGetSnapshotWithAge(&rc, &age_ms);
#else
    cmd.online = RemoteControlGetSnapshot(&rc);
#endif
    cmd.timestamp_ms = HAL_GetTick();
    cmd.switch_left = rc.rc.switch_left;
    cmd.switch_right = rc.rc.switch_right;
    cmd.stick_yaw = rc.rc.rocker_r_;
    cmd.stick_pitch = rc.rc.rocker_r1;
    /* Down: everyday dual-axis control; mid/up retain single-axis diagnostics. */
    cmd.request.axis_mask = switch_is_down(rc.rc.switch_left) ? 3U :
                            (switch_is_mid(rc.rc.switch_left) ? 2U : 1U);
#if GIMBAL_KEY_MOUSE_ENABLED
    GimbalKeyMouseApply(&key_mouse, &rc, age_ms, &cmd);
#else
    cmd.request.sticks_centered = fabsf((float)cmd.stick_yaw) <= GIMBAL_RC_DEADBAND &&
                                  fabsf((float)cmd.stick_pitch) <= GIMBAL_RC_DEADBAND;
    if (cmd.online)
    {
        if (switch_is_mid(rc.rc.switch_right)) cmd.request.mode = AUTO_AIM_USB_MODE ? GIMBAL_REQUEST_AUTO : GIMBAL_REQUEST_HOLD;
        if (switch_is_up(rc.rc.switch_right)) cmd.request.mode = GIMBAL_REQUEST_MANUAL;
        cmd.request.rate[0] = GIMBAL_YAW_COMMAND_SIGN * StickRate(cmd.stick_yaw, GIMBAL_YAW_MAX_SPEED_DPS);
        cmd.request.rate[1] = GIMBAL_PITCH_COMMAND_SIGN * StickRate(cmd.stick_pitch, GIMBAL_PITCH_MAX_SPEED_DPS);
    }
#endif
    if (AUTO_AIM_USB_MODE && cmd.online && cmd.request.mode == GIMBAL_REQUEST_HOLD &&
        switch_is_mid(rc.rc.switch_right)) cmd.request.mode = GIMBAL_REQUEST_AUTO;
    PubPushMessage(gimbal_cmd_pub, &cmd);
}
