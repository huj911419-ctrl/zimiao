#ifndef TEST_DJI_MOTOR_H
#define TEST_DJI_MOTOR_H
#include "main.h"
enum { OPEN_LOOP, GM6020, MOTOR_STOP, MOTOR_ENALBED };
typedef struct {
    uint16_t ecd;
    float total_angle;
    uint8_t temperature;
    int16_t torque_current_raw, speed_rpm_raw;
} DJI_Motor_Measure_s;
typedef struct {
    struct { FDCAN_HandleTypeDef *can_handle; uint8_t tx_id; } can_init_config;
    struct { int outer_loop_type, close_loop_type; } controller_setting_init_config;
    int motor_type;
} Motor_Init_Config_s;
typedef struct {
    DJI_Motor_Measure_s measure;
    uint32_t rx_count, last_rx_ms, feedback_timeout_ms, command_timeout_ms;
    uint8_t watchdog_fault, stop_flag;
    int16_t last_set;
    uint32_t tx_enqueue_ok, tx_enqueue_fail;
    float ref;
} DJIMotorInstance;
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorSetRef(DJIMotorInstance *motor, float value);
void DJIMotorEnable(DJIMotorInstance *motor);
#endif
