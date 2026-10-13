/** @file gimbal_key_mouse_config.h @brief Reserved DBUS keyboard/mouse input. */
#ifndef GIMBAL_KEY_MOUSE_CONFIG_H
#define GIMBAL_KEY_MOUSE_CONFIG_H
/* Keep 0 for the current stick-controlled firmware. No runtime auto takeover. */
#ifndef GIMBAL_KEY_MOUSE_ENABLED
#define GIMBAL_KEY_MOUSE_ENABLED 0
#endif
#if GIMBAL_KEY_MOUSE_ENABLED != 0 && GIMBAL_KEY_MOUSE_ENABLED != 1
#error "GIMBAL_KEY_MOUSE_ENABLED must be 0 or 1"
#endif

/* Raw DBUS mouse values map to angular RATE, not a per-task angle increment.
 * These are initial bench values; verify mouse DPI, upstream semantics and signs. */
#define GIMBAL_MOUSE_YAW_DPS_PER_UNIT 0.25f
#define GIMBAL_MOUSE_PITCH_DPS_PER_UNIT 0.20f
#define GIMBAL_MOUSE_YAW_SIGN (-1.0f)   /* positive X = right, negative INS Yaw */
#define GIMBAL_MOUSE_PITCH_SIGN 1.0f    /* positive Y = down, positive INS Pitch */
#define GIMBAL_KEY_YAW_DPS 45.0f
#define GIMBAL_KEY_PITCH_DPS 30.0f
#define GIMBAL_KEY_MOUSE_YAW_MAX_DPS 120.0f
#define GIMBAL_KEY_MOUSE_PITCH_MAX_DPS 60.0f
#define GIMBAL_KEY_MOUSE_FINE_SCALE 0.25f
#define GIMBAL_KEY_MOUSE_TIMEOUT_MS 50U
#endif
