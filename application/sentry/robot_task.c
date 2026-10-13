#include "robot_task.h"

/* Skip missed releases rather than starving lower-priority tasks with catch-up loops. */
static void DelayPeriodic(TickType_t *wake, TickType_t period)
{
    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - *wake) >= period) *wake = now;
    vTaskDelayUntil(wake, period);
}

osThreadId controlTaskHandle;
osThreadId motorTaskHandle;
osThreadId sensorTaskHandle;
osThreadId commTaskHandle;
osThreadId WDTTaskHandle;

void RobotOSTaskInit(void)
{
    osThreadDef(controltask, StartControlTask, osPriorityAboveNormal, 0, 768);
    controlTaskHandle = osThreadCreate(osThread(controltask), NULL);

    osThreadDef(motortask, StartMotorTask, osPriorityAboveNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(sensortask, StartSensorTask, osPriorityHigh, 0, 512);
    sensorTaskHandle = osThreadCreate(osThread(sensortask), NULL);

    /* USB CDC formatting/transmit path needs more stack than the original 256-word task. */
    osThreadDef(commtask, StartCommTask, osPriorityBelowNormal, 0, 768);
    commTaskHandle = osThreadCreate(osThread(commtask), NULL);

    osThreadDef(wdttask, StartWDTTask, osPriorityNormal, 0, 256);
    WDTTaskHandle = osThreadCreate(osThread(wdttask), NULL);
}

__attribute__((noreturn)) void StartControlTask(void const *argument)
{
    static float control_dt;
    static float control_start;
    (void)argument;
    LOGINFO("[freeRTOS] CONTROL Task Start");
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        control_start = DWT_GetTimeline_ms();
        RobotControlTask();
        control_dt = DWT_GetTimeline_ms() - control_start;
        if (control_dt > 1)
            LOGERROR("[freeRTOS] CONTROL Task is being DELAY! dt = [%f]", &control_dt);
        DelayPeriodic(&wake, pdMS_TO_TICKS(1));
    }
}

__attribute__((noreturn)) void StartMotorTask(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    (void)argument;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        DelayPeriodic(&wake, pdMS_TO_TICKS(1));
    }
}

__attribute__((noreturn)) void StartSensorTask(void const *argument)
{
    static float sensor_dt;
    static float sensor_start;
    (void)argument;
    LOGINFO("[freeRTOS] SENSOR Task Start");
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        sensor_start = DWT_GetTimeline_ms();
        RobotSensorTask();
        sensor_dt = DWT_GetTimeline_ms() - sensor_start;
        if (sensor_dt > 1)
            LOGERROR("[freeRTOS] SENSOR Task is being DELAY! dt = [%f]", &sensor_dt);
        DelayPeriodic(&wake, pdMS_TO_TICKS(1));
    }
}

__attribute__((noreturn)) void StartCommTask(void const *argument)
{
    static float comm_dt;
    static float comm_start;
    (void)argument;
    LOGINFO("[freeRTOS] COMM Task Start");
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        comm_start = DWT_GetTimeline_ms();
        RobotCommTask();
        comm_dt = DWT_GetTimeline_ms() - comm_start;
        if (comm_dt > 1)
            LOGERROR("[freeRTOS] COMM Task is being DELAY! dt = [%f]", &comm_dt);
        DelayPeriodic(&wake, pdMS_TO_TICKS(10));
    }
}

__attribute__((noreturn)) void StartWDTTask(void const *argument)
{
    static float wdt_dt;
    static float wdt_start;
    (void)argument;
    LOGINFO("[freeRTOS] WDT Task Start");
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        wdt_start = DWT_GetTimeline_ms();
        RobotWDTTask();
        wdt_dt = DWT_GetTimeline_ms() - wdt_start;
        if (wdt_dt > 1)
            LOGERROR("[freeRTOS] WDT Task is being DELAY! dt = [%f]", &wdt_dt);
        DelayPeriodic(&wake, pdMS_TO_TICKS(10));
    }
}
