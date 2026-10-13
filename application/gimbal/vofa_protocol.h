/** @file vofa_protocol.h
 * @brief HAL-independent VOFA+ JustFloat framing and ASCII tuning commands.
 */
#ifndef VOFA_PROTOCOL_H
#define VOFA_PROTOCOL_H
#include "gimbal_control.h"
#include <stddef.h>
#include <stdint.h>

#define VOFA_CHANNEL_COUNT 48U
#define VOFA_FRAME_SIZE (VOFA_CHANNEL_COUNT * 4U + 4U)
#define VOFA_LINE_SIZE 96U

typedef enum {
    VOFA_CMD_INVALID, VOFA_CMD_STREAM, VOFA_CMD_RATE, VOFA_CMD_STAGE,
    VOFA_CMD_SET, VOFA_CMD_GET, VOFA_CMD_DEFAULTS,
    VOFA_CMD_CAL_LOWER, VOFA_CMD_CAL_UPPER, VOFA_CMD_CAL_RESET,
    VOFA_CMD_IMU_TEXT
} Vofa_Command_Type_e;
typedef enum {
    VOFA_PARAM_ANGLE_KP, VOFA_PARAM_SPEED_KP, VOFA_PARAM_SPEED_KI,
    VOFA_PARAM_INTEGRAL_LIMIT, VOFA_PARAM_OUTPUT_LIMIT, VOFA_PARAM_SPEED_DPS,
    VOFA_PARAM_ACCEL_DPS2, VOFA_PARAM_FOLLOW_ERROR_DEG
} Vofa_Parameter_e;
typedef enum {
    VOFA_RESULT_NONE, VOFA_RESULT_OK, VOFA_RESULT_SYNTAX,
    VOFA_RESULT_RANGE, VOFA_RESULT_REQUIRE_OFF, VOFA_RESULT_OUTPUT_DISABLED
} Vofa_Result_e;
typedef struct {
    Vofa_Command_Type_e type;
    Vofa_Parameter_e parameter;
    uint8_t axis;
    float value;
} Vofa_Command_s;
typedef struct {
    char text[VOFA_LINE_SIZE];
    uint16_t length;
    uint8_t discard;
} Vofa_Line_s;

/** @brief Append one byte; 1=complete nonempty line, 0=waiting, -1=rejected line.
 * Parse text immediately after a 1 result; the next byte starts a new line.
 */
int VofaLineFeed(Vofa_Line_s *line, uint8_t byte);
/** @brief Discard a partial/corrupted line until its next newline. */
void VofaLineDiscard(Vofa_Line_s *line);
/** @brief Parse an entire line; unknown/trailing tokens and nonfinite values are rejected. */
uint8_t VofaParseCommand(const char *text, Vofa_Command_s *command);
/** @brief Validate a requested value without changing configuration. */
Vofa_Result_e VofaValidateParameter(const Vofa_Command_s *command, float output_ceiling, float speed_ceiling,
                                  float acceleration_ceiling);
/** @brief Apply/get one axis parameter. User-facing speed/acceleration/error use degrees. */
float VofaSetParameter(Gimbal_Axis_Config_s *config, const Vofa_Command_s *command);
float VofaGetParameter(const Gimbal_Axis_Config_s *config, Vofa_Parameter_e parameter);
/** @brief Pack fixed-count little-endian float32 channels + 00 00 80 7F. Nonfinite samples become 0. */
size_t VofaPackFrame(const float channels[VOFA_CHANNEL_COUNT], uint8_t *frame, size_t capacity);
#endif
