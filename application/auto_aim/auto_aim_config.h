/** @file auto_aim_config.h @brief Explicit tracking-only integration configuration. */
#ifndef AUTO_AIM_CONFIG_H
#define AUTO_AIM_CONFIG_H
/* 1: CBoard binary USB, 0: existing USB text/JustFloat/ASCII tuning. */
#ifndef AUTO_AIM_USB_MODE
#define AUTO_AIM_USB_MODE 1
#endif
/* Must be confirmed on the real mount; never infer calibration from a field name.
 * See README.md. Both quaternions below are proper rotations, w,x,y,z.
 */
#ifndef AUTO_AIM_COORDINATES_CONFIRMED
#define AUTO_AIM_COORDINATES_CONFIRMED 0
#endif
/* Candidate basis: hero forward=-INS Y, left=INS X, up=INS Z.
 * q_hero = world * q_INS * body. Z+90 conjugation maps INS pitch-X to Hero pitch-Y.
 * Include the measured vision Solver R_gimbal2imubody in this configuration.
 */
#define AUTO_AIM_WORLD_QUATERNION {0.7071067812f, 0.0f, 0.0f, 0.7071067812f}
/* Approximate held-neutral reference, 2026-10-13: INS Pitch=-0.130816 deg,
 * Roll=4.316776 deg. Remove this fixed tilt ONLY in communication coordinates.
 * Raw INS/manual feedback/output polarity are unchanged. Yaw is still the INS
 * initialization reference; captured yaw is not a permanent mounting offset.
 * This bench estimate remains gated by COORDINATES_CONFIRMED=0.
 */
#define AUTO_AIM_NEUTRAL_INS_PITCH_DEG (-0.1308157895f)
#define AUTO_AIM_NEUTRAL_INS_ROLL_DEG 4.3167763158f
#define AUTO_AIM_BODY_QUATERNION {0.7065742499f, 0.0274377365f, -0.0258244399f, -0.7066350531f}
/* Must match vision YAML R_gimbal2imubody; current hero.yaml uses identity. */
#define AUTO_AIM_SOLVER_QUATERNION {1.0f, 0.0f, 0.0f, 0.0f}
/* Explicit nominal value for non-firing bench tracking; NOT measured muzzle speed. */
#ifndef AUTO_AIM_TRACKING_SPEED_MPS
#define AUTO_AIM_TRACKING_SPEED_MPS 15.0f
#endif
#define AUTO_AIM_TIMEOUT_MS 150U
#define AUTO_AIM_IMU_PERIOD_MS 10U /* 100 Hz */
#define AUTO_AIM_STATUS_PERIOD_MS 20U /* 50 Hz */
#if AUTO_AIM_USB_MODE != 0 && AUTO_AIM_USB_MODE != 1
#error "AUTO_AIM_USB_MODE must be 0 or 1"
#endif
#if AUTO_AIM_COORDINATES_CONFIRMED != 0 && AUTO_AIM_COORDINATES_CONFIRMED != 1
#error "AUTO_AIM_COORDINATES_CONFIRMED must be 0 or 1"
#endif
#endif
