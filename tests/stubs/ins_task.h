#ifndef TEST_INS_TASK_H
#define TEST_INS_TASK_H
#include <stdint.h>
typedef struct { int unused; } attitude_t;
typedef struct {
    float q[4], gyro[3], yaw, pitch, roll;
    uint32_t timestamp_us, timestamp_ms, sample_seq;
    uint8_t valid, calibrated;
    float sensor_gyro[3], sensor_accel[3];
} INS_Vision_Snapshot_t;
void INS_SetMounting(float yaw, float pitch, float roll);
attitude_t *INS_Init(void);
uint8_t INS_GetVisionSnapshot(INS_Vision_Snapshot_t *sample);
#endif
