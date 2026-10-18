/** @file vision_comm.c
 * @brief Explicit little-endian codec and instance-based stream parser.
 */
#include "vision_comm.h"
#include <string.h>

static int16_t ReadI16LE(const uint8_t *buffer)
{
    uint16_t raw = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    return (int16_t)((raw & 0x8000u) ? (int32_t)raw - 65536 : (int32_t)raw);
}

static void WriteI16LE(uint8_t *buffer, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    buffer[0] = (uint8_t)raw;
    buffer[1] = (uint8_t)(raw >> 8);
}

uint8_t VisionCommInit(VisionComm_s *comm, uint32_t partial_timeout_ms)
{
    if (comm == NULL) return 0u;
    memset(comm, 0, sizeof(*comm));
    if (partial_timeout_ms == 0u || partial_timeout_ms > INT32_MAX) return 0u;
    comm->partial_timeout_ms = partial_timeout_ms;
    return 1u;
}

void VisionCommResetRx(VisionComm_s *comm)
{
    if (comm == NULL) return;
    comm->frame_index = 0u;
    comm->has_command = 0u;
    memset(&comm->latest, 0, sizeof(comm->latest));
}

uint8_t VisionCommDecodeCommand(const uint8_t *frame, size_t len, VisionCommand_s *command)
{
    if (command == NULL) return 0u;
    memset(command, 0, sizeof(*command));
    if (frame == NULL || len != VISION_COMM_COMMAND_LEN || frame[0] != 0xCCu ||
        frame[7] != 0xEEu || frame[1] > 1u || frame[2] > 1u) return 0u;
    command->control = frame[1];
    command->shoot = frame[2];
    command->yaw_raw = ReadI16LE(frame + 3);
    command->pitch_raw = ReadI16LE(frame + 5);
    return 1u;
}

uint8_t VisionCommFeedByte(VisionComm_s *comm, uint8_t byte, uint32_t received_ms)
{
    if (comm == NULL || comm->partial_timeout_ms == 0u) return 0u;
    if (comm->frame_index != 0u &&
        (uint32_t)(received_ms - comm->last_byte_ms) >= comm->partial_timeout_ms)
    {
        comm->frame_index = 0u;
        comm->partial_timeouts++;
    }
    if (comm->frame_index == 0u && byte != 0xCCu) return 0u;
    comm->last_byte_ms = received_ms;
    comm->frame[comm->frame_index++] = byte;
    if (comm->frame_index != sizeof(comm->frame)) return 0u;

    VisionCommand_s decoded;
    if (VisionCommDecodeCommand(comm->frame, sizeof(comm->frame), &decoded))
    {
        comm->latest = decoded;
        comm->last_command_ms = received_ms;
        comm->has_command = 1u;
        comm->accepted_frames++;
        comm->frame_index = 0u;
        return 1u;
    }
    comm->invalid_frames++;
    uint8_t start = 1u;
    while (start < sizeof(comm->frame) && comm->frame[start] != 0xCCu) start++;
    comm->frame_index = (uint8_t)(sizeof(comm->frame) - start);
    memmove(comm->frame, comm->frame + start, comm->frame_index);
    return 0u;
}

uint8_t VisionCommGetCommand(const VisionComm_s *comm, uint32_t now_ms,
                            uint32_t timeout_ms, VisionCommand_s *command)
{
    if (command == NULL) return 0u;
    memset(command, 0, sizeof(*command));
    if (comm == NULL || !comm->has_command || timeout_ms == 0u || timeout_ms > INT32_MAX ||
        (uint32_t)(now_ms - comm->last_command_ms) >= timeout_ms) return 0u;
    *command = comm->latest;
    return 1u;
}

size_t VisionCommPackTelemetry(const VisionTelemetry_s *telemetry, uint8_t *buffer, size_t capacity)
{
    if (telemetry == NULL || buffer == NULL || capacity < VISION_COMM_TELEMETRY_LEN) return 0u;
    buffer[0] = 0xAAu;
    WriteI16LE(buffer + 1, telemetry->x_raw);
    WriteI16LE(buffer + 3, telemetry->y_raw);
    WriteI16LE(buffer + 5, telemetry->z_raw);
    WriteI16LE(buffer + 7, telemetry->w_raw);
    buffer[9] = 0xDDu;
    buffer[10] = 0xBBu;
    WriteI16LE(buffer + 11, telemetry->bullet_speed_raw);
    WriteI16LE(buffer + 13, telemetry->mode);
    buffer[15] = telemetry->shoot_mode;
    WriteI16LE(buffer + 16, telemetry->ft_angle_raw);
    buffer[18] = 0xDDu;
    return VISION_COMM_TELEMETRY_LEN;
}
