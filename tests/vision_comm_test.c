/* Portable tests: this binary links NO HAL/RTOS/USB/UART stubs. */
#include "vision_comm.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void Feed(VisionComm_s *comm, const uint8_t *bytes, size_t len, uint32_t received_ms)
{
    for (size_t i = 0; i < len; i++) VisionCommFeedByte(comm, bytes[i], received_ms);
}

int main(void)
{
    VisionComm_s first, second;
    VisionCommand_s result;
    const uint8_t frame[] = {0xCC, 1, 1, 0x2E, 0xFB, 0x2E, 0x16, 0xEE};
    assert(!VisionCommInit(NULL, 20));
    assert(!VisionCommInit(&first, 0));
    assert(!VisionCommFeedByte(&first, 0xCC, 0));
    assert(VisionCommInit(&first, 20));
    assert(VisionCommInit(&second, 20));
    assert(!VisionCommGetCommand(&first, 10, 100, &result));
    assert(result.control == 0 && result.shoot == 0);

    Feed(&first, frame, 3, 10);
    Feed(&second, frame, sizeof(frame), 12);
    assert(first.frame_index == 3 && first.accepted_frames == 0);
    assert(second.accepted_frames == 1);
    Feed(&first, frame + 3, sizeof(frame) - 3, 15);
    assert(VisionCommGetCommand(&first, 114, 100, &result));
    assert(result.yaw_raw == -1234 && result.pitch_raw == 5678 && result.control == 1);
    assert(!VisionCommGetCommand(&first, 115, 100, &result));
    assert(result.control == 0 && result.shoot == 0 && result.yaw_raw == 0);
    assert(!VisionCommGetCommand(&first, 16, 0, &result));
    assert(!VisionCommGetCommand(&first, 16, UINT32_MAX, &result));

    Feed(&first, frame, 3, 200);
    Feed(&first, frame + 3, sizeof(frame) - 3, 220);
    assert(first.partial_timeouts == 1 && first.accepted_frames == 1);
    Feed(&first, frame, sizeof(frame), 221);
    assert(first.accepted_frames == 2);
    VisionCommResetRx(&first);
    assert(!VisionCommGetCommand(&first, 222, 100, &result));
    assert(first.accepted_frames == 2); /* Counters survive link/session reset. */

    Feed(&first, frame, 3, UINT32_MAX - 4u);
    Feed(&first, frame + 3, sizeof(frame) - 3, 2u);
    assert(VisionCommGetCommand(&first, 11u, 10u, &result));
    assert(!VisionCommGetCommand(&first, 12u, 10u, &result));
    Feed(&first, frame, sizeof(frame), UINT32_MAX - 4u);
    assert(VisionCommGetCommand(&first, 3u, 10u, &result));
    assert(!VisionCommGetCommand(&first, 5u, 10u, &result));

    const VisionTelemetry_s actual = {
        .x_raw = 100, .y_raw = -200, .z_raw = 300, .w_raw = 9993,
        .bullet_speed_raw = 1800, .mode = 2, .shoot_mode = 1, .ft_angle_raw = -500,
    };
    const uint8_t expected[] = {0xAA, 0x64, 0, 0x38, 0xFF, 0x2C, 1, 0x09, 0x27, 0xDD,
                                0xBB, 0x08, 7, 2, 0, 1, 0x0C, 0xFE, 0xDD};
    uint8_t buffer[20];
    memset(buffer, 0x55, sizeof(buffer));
    assert(!VisionCommPackTelemetry(&actual, buffer, 18));
    assert(buffer[0] == 0x55 && buffer[19] == 0x55);
    assert(VisionCommPackTelemetry(&actual, buffer, sizeof(buffer)) == 19);
    assert(memcmp(buffer, expected, sizeof(expected)) == 0 && buffer[19] == 0x55);
    assert(!VisionCommPackTelemetry(NULL, buffer, sizeof(buffer)));
    assert(!VisionCommPackTelemetry(&actual, NULL, sizeof(buffer)));
    assert(!VisionCommDecodeCommand(frame, 7, &result));
    assert(!VisionCommDecodeCommand(NULL, 8, &result));
    assert(!VisionCommDecodeCommand(frame, sizeof(frame), NULL));

    /* Explicitly demonstrate the provisional profile's unresolved CRC gap. */
    uint8_t corrupted[8];
    memcpy(corrupted, frame, sizeof(frame));
    corrupted[3] ^= 1u;
    assert(VisionCommDecodeCommand(corrupted, sizeof(corrupted), &result));
    assert(result.yaw_raw != -1234); /* Syntax success cannot authorize actuation. */
    puts("PASS: independent instances, freshness, partial timeout, clock wrap, supplied telemetry, bounds; no-CRC limitation reproduced");
    return 0;
}
