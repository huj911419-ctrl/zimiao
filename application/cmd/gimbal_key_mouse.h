/** @file gimbal_key_mouse.h @brief Optional DBUS input adapter for manual gimbal control. */
#ifndef GIMBAL_KEY_MOUSE_H
#define GIMBAL_KEY_MOUSE_H
#include "remote_control.h"
#include "2yaw_gimbal.h"

typedef struct {
    uint8_t stop_latched;
} Gimbal_Key_Mouse_s;

/** @brief Require a neutral, right-switch-OFF preparation before first use. */
void GimbalKeyMouseInit(Gimbal_Key_Mouse_s *state);
/** @brief Override only the command input; keep the caller's switch/axis metadata.
 * age_ms must belong to the same coherent RC sample. Rates are rad/s.
 * Call only when the compile-time input selection enables keyboard/mouse.
 */
void GimbalKeyMouseApply(Gimbal_Key_Mouse_s *state, const RC_ctrl_t *rc,
                        uint32_t age_ms, Gimbal_Manual_Cmd_s *cmd);
#endif
