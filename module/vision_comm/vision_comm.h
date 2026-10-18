/**
 * @file vision_comm.h
 * @brief Portable streaming codec for the provisional AA/BB/CC bench profile.
 * @warning No CRC/remote timestamp. This profile is NOT production-qualified.
 * No HAL, RTOS, logging, watchdog, allocation or actuator dependencies.
 * Serialize calls per instance; caller supplies monotonic uint32 millisecond time.
 */
#ifndef VISION_COMM_H
#define VISION_COMM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_COMM_COMMAND_LEN 8u
#define VISION_COMM_TELEMETRY_LEN 19u

typedef struct
{
    uint8_t control;
    uint8_t shoot;
    int16_t yaw_raw;
    int16_t pitch_raw;
} VisionCommand_s;

/** Application-supplied wire units; obtain a coherent sensor/state snapshot first. */
typedef struct
{
    int16_t x_raw, y_raw, z_raw, w_raw; /* quaternion * 10000; wire order XYZW */
    int16_t bullet_speed_raw;          /* m/s * 100 */
    int16_t mode;
    uint8_t shoot_mode;
    int16_t ft_angle_raw;              /* meaning must be agreed with vision team */
} VisionTelemetry_s;

typedef struct
{
    uint8_t frame[VISION_COMM_COMMAND_LEN];
    uint8_t frame_index;
    uint8_t has_command;
    VisionCommand_s latest;
    uint32_t last_byte_ms;
    uint32_t last_command_ms;
    uint32_t partial_timeout_ms;
    uint32_t accepted_frames;
    uint32_t invalid_frames;
    uint32_t partial_timeouts;
} VisionComm_s;

/** Initialize an instance. partial_timeout_ms must be 1..INT32_MAX. */
uint8_t VisionCommInit(VisionComm_s *comm, uint32_t partial_timeout_ms);
/** Invalidate pending and latest commands on disconnect, overrun or stream loss. */
void VisionCommResetRx(VisionComm_s *comm);
/** Feed one byte with its transport RECEIVE time. Return 1 for a new command.
 * A success means syntax accepted, not checksum verified or permission to fire. */
uint8_t VisionCommFeedByte(VisionComm_s *comm, uint8_t byte, uint32_t received_ms);
/** Copy latest command only when age < timeout_ms; clear output on failure.
 * timeout_ms must be 1..INT32_MAX. This is local RX age, NOT camera/target age. */
uint8_t VisionCommGetCommand(const VisionComm_s *comm, uint32_t now_ms,
                            uint32_t timeout_ms, VisionCommand_s *command);
/** Decode exactly eight bytes, independent of native endian/alignment. */
uint8_t VisionCommDecodeCommand(const uint8_t *frame, size_t len, VisionCommand_s *command);
/** Pack supplied raw telemetry. Returns bytes written, or 0 for invalid arguments.
 * Caller owns semantic validation (quaternion norm, physical limits, mode map). */
size_t VisionCommPackTelemetry(const VisionTelemetry_s *telemetry, uint8_t *buffer, size_t capacity);

#ifdef __cplusplus
}
#endif
#endif
