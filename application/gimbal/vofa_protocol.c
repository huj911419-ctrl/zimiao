#include "vofa_protocol.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

void VofaLineDiscard(Vofa_Line_s *line)
{
    line->length = 0U;
    line->discard = 1U;
}

int VofaLineFeed(Vofa_Line_s *line, uint8_t byte)
{
    if (byte == '\r') return 0;
    if (byte == '\n')
    {
        if (line->discard)
        {
            line->discard = 0U;
            line->length = 0U;
            return -1;
        }
        if (!line->length) return 0;
        line->text[line->length] = '\0';
        line->length = 0U;
        return 1;
    }
    if (line->discard) return 0;
    if ((byte < 32U && byte != '\t') || byte > 126U || line->length >= VOFA_LINE_SIZE - 1U)
    {
        VofaLineDiscard(line);
        return 0;
    }
    line->text[line->length++] = (char)byte;
    return 0;
}

static uint8_t ReadNumber(const char *text, float *value)
{
    char *end;
    errno = 0;
    float result = strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !isfinite(result)) return 0U;
    *value = result;
    return 1U;
}

uint8_t VofaParseCommand(const char *text, Vofa_Command_s *cmd)
{
    static const char *const names[] = {
        "angle_kp", "speed_kp", "speed_ki", "integral_limit",
        "output_limit", "speed_dps", "accel_dps2", "follow_error_deg"
    };
    char local[VOFA_LINE_SIZE];
    const char *token[5];
    size_t length = strlen(text);
    memset(cmd, 0, sizeof(*cmd));
    if (!length || length >= sizeof(local)) return 0U;
    for (size_t i = 0; i <= length; ++i) local[i] = (char)tolower((unsigned char)text[i]);
    uint8_t count = 0U;
    char *cursor = local;
    while (*cursor)
    {
        while (isspace((unsigned char)*cursor)) ++cursor;
        if (!*cursor) break;
        if (count == 5U) return 0U;
        token[count++] = cursor;
        while (*cursor && !isspace((unsigned char)*cursor)) ++cursor;
        if (*cursor) *cursor++ = '\0';
    }
    if (!count) return 0U;
    if (count == 1U)
    {
        if (!strcmp(token[0], "t")) { cmd->type = VOFA_CMD_STREAM; cmd->value = 1.0f; }
        else if (!strcmp(token[0], "s")) cmd->type = VOFA_CMD_STREAM;
        else if (!strcmp(token[0], "stop")) cmd->type = VOFA_CMD_STAGE;
        else if (!strcmp(token[0], "defaults")) cmd->type = VOFA_CMD_DEFAULTS;
        else if (!strcmp(token[0], "l")) cmd->type = VOFA_CMD_CAL_LOWER;
        else if (!strcmp(token[0], "u")) cmd->type = VOFA_CMD_CAL_UPPER;
        else if (!strcmp(token[0], "r")) cmd->type = VOFA_CMD_CAL_RESET;
        else return 0U;
        return 1U;
    }
    if (count == 2U)
    {
        if (!ReadNumber(token[1], &cmd->value)) return 0U;
        if (!strcmp(token[0], "stage")) cmd->type = VOFA_CMD_STAGE;
        else if (!strcmp(token[0], "stream")) cmd->type = VOFA_CMD_STREAM;
        else if (!strcmp(token[0], "rate")) cmd->type = VOFA_CMD_RATE;
        else if (!strcmp(token[0], "imu")) cmd->type = VOFA_CMD_IMU_TEXT;
        else return 0U;
        return 1U;
    }
    if ((count == 4U && !strcmp(token[0], "set")) || (count == 3U && !strcmp(token[0], "get")))
    {
        cmd->type = count == 4U ? VOFA_CMD_SET : VOFA_CMD_GET;
        if (!strcmp(token[1], "yaw")) cmd->axis = 0U;
        else if (!strcmp(token[1], "pitch")) cmd->axis = 1U;
        else return 0U;
        uint8_t found = 0U;
        for (uint8_t i = 0U; i < sizeof(names) / sizeof(names[0]); ++i)
            if (!strcmp(token[2], names[i])) { cmd->parameter = (Vofa_Parameter_e)i; found = 1U; break; }
        if (!found || (count == 4U && !ReadNumber(token[3], &cmd->value))) return 0U;
        return 1U;
    }
    return 0U;
}

Vofa_Result_e VofaValidateParameter(const Vofa_Command_s *cmd, float output_ceiling, float speed_ceiling,
                                  float acceleration_ceiling)
{
    if (cmd->axis > 1U || !isfinite(cmd->value)) return VOFA_RESULT_RANGE;
    float low = 0.0f, high;
    switch (cmd->parameter)
    {
    case VOFA_PARAM_ANGLE_KP: high = 50.0f; break;
    case VOFA_PARAM_SPEED_KP:
    case VOFA_PARAM_SPEED_KI: high = 50000.0f; break;
    case VOFA_PARAM_INTEGRAL_LIMIT: high = output_ceiling; break;
    case VOFA_PARAM_OUTPUT_LIMIT: low = 1.0f; high = output_ceiling; break;
    case VOFA_PARAM_SPEED_DPS: low = 0.1f; high = speed_ceiling; break;
    case VOFA_PARAM_ACCEL_DPS2: low = 0.1f; high = acceleration_ceiling; break;
    case VOFA_PARAM_FOLLOW_ERROR_DEG: low = 0.1f; high = 20.0f; break;
    default: return VOFA_RESULT_RANGE;
    }
    return cmd->value >= low && cmd->value <= high ? VOFA_RESULT_OK : VOFA_RESULT_RANGE;
}

float VofaGetParameter(const Gimbal_Axis_Config_s *cfg, Vofa_Parameter_e param)
{
    switch (param)
    {
    case VOFA_PARAM_ANGLE_KP: return cfg->angle_kp;
    case VOFA_PARAM_SPEED_KP: return cfg->speed_kp;
    case VOFA_PARAM_SPEED_KI: return cfg->speed_ki;
    case VOFA_PARAM_INTEGRAL_LIMIT: return cfg->integral_limit;
    case VOFA_PARAM_OUTPUT_LIMIT: return cfg->output_limit;
    case VOFA_PARAM_SPEED_DPS: return cfg->speed_limit / GIMBAL_RAD_PER_DEG;
    case VOFA_PARAM_ACCEL_DPS2: return cfg->acceleration_limit / GIMBAL_RAD_PER_DEG;
    case VOFA_PARAM_FOLLOW_ERROR_DEG: return cfg->max_follow_error / GIMBAL_RAD_PER_DEG;
    default: return 0.0f;
    }
}

float VofaSetParameter(Gimbal_Axis_Config_s *cfg, const Vofa_Command_s *cmd)
{
    float v = cmd->value;
    switch (cmd->parameter)
    {
    case VOFA_PARAM_ANGLE_KP: cfg->angle_kp = v; break;
    case VOFA_PARAM_SPEED_KP: cfg->speed_kp = v; break;
    case VOFA_PARAM_SPEED_KI: cfg->speed_ki = v; break;
    case VOFA_PARAM_INTEGRAL_LIMIT: cfg->integral_limit = v; break;
    case VOFA_PARAM_OUTPUT_LIMIT: cfg->output_limit = v; break;
    case VOFA_PARAM_SPEED_DPS: cfg->speed_limit = v * GIMBAL_RAD_PER_DEG; break;
    case VOFA_PARAM_ACCEL_DPS2: cfg->acceleration_limit = v * GIMBAL_RAD_PER_DEG; break;
    case VOFA_PARAM_FOLLOW_ERROR_DEG: cfg->max_follow_error = v * GIMBAL_RAD_PER_DEG; break;
    default: break;
    }
    return VofaGetParameter(cfg, cmd->parameter);
}

size_t VofaPackFrame(const float channels[VOFA_CHANNEL_COUNT], uint8_t *frame, size_t capacity)
{
    _Static_assert(sizeof(float) == 4U, "JustFloat requires IEEE-754 float32");
    if (!channels || !frame || capacity < VOFA_FRAME_SIZE) return 0U;
    for (size_t i = 0U; i < VOFA_CHANNEL_COUNT; ++i)
    {
        float value = isfinite(channels[i]) ? channels[i] : 0.0f;
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        frame[i * 4U] = (uint8_t)bits;
        frame[i * 4U + 1U] = (uint8_t)(bits >> 8);
        frame[i * 4U + 2U] = (uint8_t)(bits >> 16);
        frame[i * 4U + 3U] = (uint8_t)(bits >> 24);
    }
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7fU};
    memcpy(frame + VOFA_CHANNEL_COUNT * 4U, tail, sizeof(tail));
    return VOFA_FRAME_SIZE;
}
