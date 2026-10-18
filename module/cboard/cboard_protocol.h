/** @file cboard_protocol.h @brief DGE CBoard wire format, independent of HAL/RTOS.
 * All int16 fields are little endian. No CRC exists in this upstream protocol.
 */
#ifndef CBOARD_PROTOCOL_H
#define CBOARD_PROTOCOL_H
#include <stdint.h>
#include "vision_comm.h"
#define CBOARD_ATTITUDE_SIZE 10U
#define CBOARD_STATUS_SIZE 9U
#define CBOARD_COMMAND_SIZE 8U
typedef struct {
    uint8_t control, shoot;
    float yaw, pitch; /* absolute wrapped angles, rad; control=0 ignores these */
} CBoard_Command_s;
typedef struct {
    VisionComm_s stream;
    uint32_t accepted, rejected;
} CBoard_Parser_s;
/** Reuse carm_linkcoure's stream parser with a 20 ms inter-fragment timeout. */
void CBoardParserInit(CBoard_Parser_s *parser);
void CBoardParserReset(CBoard_Parser_s *parser);
uint8_t CBoardParseByteAt(CBoard_Parser_s *parser, uint8_t byte,
                         uint32_t received_ms, CBoard_Command_s *command);
/** Append a byte. Returns 1 only for a full validated command. Caller timestamps it.
 * Handles fragments, concatenated frames and malformed-frame resynchronization.
 */
uint8_t CBoardParseByte(CBoard_Parser_s *parser, uint8_t byte, CBoard_Command_s *command);
/** Reject invalid/non-unit quaternion. w,x,y,z are scaled by 10000, truncated. */
uint8_t CBoardPackAttitude(const float q[4], uint8_t frame[CBOARD_ATTITUDE_SIZE]);
/** speed is m/s, scaled by 100; ft_angle is rad, scaled by 10000.
 * mode is 0 idle or 1 auto aim. ShootMode is always 0 in tracking-only firmware.
 */
uint8_t CBoardPackStatus(float speed, uint8_t mode, float ft_angle,
                        uint8_t frame[CBOARD_STATUS_SIZE]);
#endif
