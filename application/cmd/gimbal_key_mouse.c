/** @file gimbal_key_mouse.c
 * @brief DBUS mouse rate + keyboard jog; no motor/PID or USB receive ownership.
 * Reference patterns and differences are documented in gimbal_key_mouse.md.
 */
#include "gimbal_key_mouse.h"
#include "gimbal_key_mouse_config.h"
#include "gimbal_manual_config.h"
#include <math.h>

static float BoundedRate(float dps, float limit)
{
    return fminf(fmaxf(dps, -limit), limit) * GIMBAL_RAD_PER_DEG;
}

void GimbalKeyMouseInit(Gimbal_Key_Mouse_s *state)
{
    state->stop_latched = 1U;
}

void GimbalKeyMouseApply(Gimbal_Key_Mouse_s *state, const RC_ctrl_t *rc,
                        uint32_t age_ms, Gimbal_Manual_Cmd_s *cmd)
{
    cmd->request.mode = GIMBAL_REQUEST_OFF;
    cmd->request.rate[0] = cmd->request.rate[1] = 0.0f;
    cmd->request.sticks_centered = 0U;
    if (!cmd->online || age_ms > GIMBAL_KEY_MOUSE_TIMEOUT_MS)
    {
        cmd->online = 0U;
        state->stop_latched = 1U;
        return;
    }
    uint16_t keys = rc->key[KEY_PRESS].keys;
    uint8_t neutral = rc->mouse.x == 0 && rc->mouse.y == 0 && keys == 0U &&
        !rc->mouse.press_l && !rc->mouse.press_r &&
        fabsf((float)rc->rc.rocker_r_) <= GIMBAL_RC_DEADBAND &&
        fabsf((float)rc->rc.rocker_r1) <= GIMBAL_RC_DEADBAND;
    if (keys & (1U << Key_X)) state->stop_latched = 1U;
    /* X or stale input cannot restart merely because keys are released. */
    if (switch_is_down(rc->rc.switch_right))
    {
        if (neutral) state->stop_latched = 0U;
        cmd->request.sticks_centered = neutral;
        return;
    }
    if (state->stop_latched) return;
    cmd->request.sticks_centered = neutral;
    if (switch_is_mid(rc->rc.switch_right) || (keys & (1U << Key_B)))
    {
        cmd->request.mode = GIMBAL_REQUEST_HOLD;
        return;
    }
    if (!switch_is_up(rc->rc.switch_right)) return;
    cmd->request.mode = GIMBAL_REQUEST_MANUAL;
    int yaw_key = !!(keys & (1U << Key_E)) - !!(keys & (1U << Key_Q));
    int pitch_key = !!(keys & (1U << Key_R)) - !!(keys & (1U << Key_F));
    float yaw = GIMBAL_MOUSE_YAW_SIGN * rc->mouse.x * GIMBAL_MOUSE_YAW_DPS_PER_UNIT +
                GIMBAL_YAW_COMMAND_SIGN * yaw_key * GIMBAL_KEY_YAW_DPS;
    float pitch = GIMBAL_MOUSE_PITCH_SIGN * rc->mouse.y * GIMBAL_MOUSE_PITCH_DPS_PER_UNIT +
                  GIMBAL_PITCH_COMMAND_SIGN * pitch_key * GIMBAL_KEY_PITCH_DPS;
    float scale = (keys & (1U << Key_Ctrl)) ? GIMBAL_KEY_MOUSE_FINE_SCALE : 1.0f;
    cmd->request.rate[0] = scale * BoundedRate(yaw,
        fminf(GIMBAL_KEY_MOUSE_YAW_MAX_DPS, GIMBAL_YAW_MAX_SPEED_DPS));
    cmd->request.rate[1] = scale * BoundedRate(pitch,
        fminf(GIMBAL_KEY_MOUSE_PITCH_MAX_DPS, GIMBAL_PITCH_MAX_SPEED_DPS));
}
