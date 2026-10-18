/** @file auto_aim_coordinates.h @brief Explicit frame transforms, radian targets. */
#ifndef AUTO_AIM_COORDINATES_H
#define AUTO_AIM_COORDINATES_H
#include <stdint.h>
/** Transform body-to-world orientation: q_hero = world * q_INS * body.
 * Reject nonfinite/non-unit inputs. Output uses w,x,y,z and retains quaternion sign.
 */
uint8_t AutoAimOrientation(const float ins[4], const float world[4],
                           const float body[4], float hero[4]);
/** Hero intrinsic Z-Y-X yaw/pitch (roll=0) -> repository yaw-Z / Pitch-X (Roll-Y) angles.
 * Invert the SAME world/body transform used to report orientation.
 */
uint8_t AutoAimAngles(float yaw, float pitch, const float world[4],
                     const float body[4], float target[2]);
#endif
