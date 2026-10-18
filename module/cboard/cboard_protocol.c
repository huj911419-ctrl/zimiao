#include "cboard_protocol.h"
#include <math.h>
#include <string.h>

static void WriteLE(uint8_t *p, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    p[0] = (uint8_t)bits;
    p[1] = (uint8_t)(bits >> 8);
}

void CBoardParserInit(CBoard_Parser_s *p)
{
    if (!p) return;
    memset(p, 0, sizeof(*p));
    (void)VisionCommInit(&p->stream, 20U);
}

void CBoardParserReset(CBoard_Parser_s *p)
{
    if (p) VisionCommResetRx(&p->stream);
}

uint8_t CBoardParseByteAt(CBoard_Parser_s *p, uint8_t byte, uint32_t received_ms,
                         CBoard_Command_s *command)
{
    if (!p || !command) return 0U;
    if (!p->stream.partial_timeout_ms) CBoardParserInit(p);
    uint32_t before = p->stream.invalid_frames;
    uint8_t complete = VisionCommFeedByte(&p->stream, byte, received_ms);
    p->rejected += p->stream.invalid_frames - before;
    if (!complete) return 0U;
    VisionCommand_s raw;
    if (!VisionCommGetCommand(&p->stream, received_ms, 1U, &raw)) return 0U;
    CBoard_Command_s next = {raw.control, raw.shoot,
        raw.yaw_raw / 10000.0f, raw.pitch_raw / 10000.0f};
    if (!next.control || (fabsf(next.yaw) <= 3.1416f && fabsf(next.pitch) < 1.5708f))
    {
        *command = next;
        ++p->accepted;
        return 1U;
    }
    ++p->rejected;
    /* Retain an embedded header even when syntax passed but angle semantics failed. */
    uint8_t suffix[CBOARD_COMMAND_SIZE - 1U];
    memcpy(suffix, p->stream.frame + 1U, sizeof(suffix));
    VisionCommResetRx(&p->stream);
    for (uint8_t i = 0U; i < sizeof(suffix); ++i)
        (void)VisionCommFeedByte(&p->stream, suffix[i], received_ms);
    return 0U;
}

uint8_t CBoardParseByte(CBoard_Parser_s *p, uint8_t byte, CBoard_Command_s *command)
{
    return CBoardParseByteAt(p, byte, 0U, command);
}

uint8_t CBoardPackAttitude(const float q[4], uint8_t frame[CBOARD_ATTITUDE_SIZE])
{
    if (!q || !frame) return 0U;
    float norm = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (!isfinite(q[i])) return 0U;
        norm += q[i] * q[i];
    }
    if (fabsf(norm - 1.0f) > 0.1f) return 0U;
    float scale = 10000.0f / sqrtf(norm);
    frame[0] = 0xAAU;
    for (uint8_t i = 0U; i < 4U; ++i) WriteLE(frame + 1U + 2U * i, (int16_t)(q[i] * scale));
    frame[9] = 0xDDU;
    return 1U;
}

uint8_t CBoardPackStatus(float speed, uint8_t mode, float ft_angle,
                        uint8_t frame[CBOARD_STATUS_SIZE])
{
    if (!frame || !isfinite(speed) || speed < 0.0f || speed > 327.67f || mode > 1U ||
        !isfinite(ft_angle) || fabsf(ft_angle) > 3.2767f) return 0U;
    frame[0] = 0xBBU;
    WriteLE(frame + 1, (int16_t)(speed * 100.0f));
    WriteLE(frame + 3, mode);
    WriteLE(frame + 5, 0); /* Reserved shoot side; this is never an actuation request. */
    WriteLE(frame + 7, (int16_t)(ft_angle * 10000.0f));
    return 1U;
}
