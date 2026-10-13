/** @file robot.c @brief Manual GM6020 gimbal bring-up entry point. */
#include "robot.h"
#include "robot_task.h"
#include "gimbal_runtime.h"
#include "gimbal_imu_uart.h"
#include "gimbal_manual_config.h"
#include "ins_task.h"
#include "iwdg.h"

#include "can_scan.h"

void RobotInit(void)
{
    GimbalRuntimeInit();
    GimbalImuUartInit();
    CanScanInit();
    RobotOSTaskInit();
}
void RobotControlTask(void) { GimbalRuntimeControl(); }
void RobotSensorTask(void)
{
#if GIMBAL_BRINGUP_ENABLE_IMU
    INS_Task();
#endif
}
void RobotCommTask(void)
{
    /* A pending one-shot text report gets the USB slot before periodic telemetry. */
    CanScanTask();
    GimbalRuntimeComm();
    GimbalImuUartTask();
}
void RobotWDTTask(void) { HAL_IWDG_Refresh(&hiwdg1); }
