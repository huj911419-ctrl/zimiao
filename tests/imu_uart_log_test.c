/* Verify text units and asynchronous UART buffer ownership; no hardware needed. */
#include "gimbal_imu_uart.h"
#include "gimbal_imu_text.h"
#include "usart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart10 = {.gState = HAL_UART_STATE_READY};
static uint32_t tick;
static Gimbal_Debug_Data_s sample;
static uint8_t *pending;
static uint16_t pending_size;
static unsigned sends;
static HAL_StatusTypeDef status = HAL_OK;

uint32_t HAL_GetTick(void) { return tick; }
void GimbalGetDebugData(Gimbal_Debug_Data_s *data) { *data = sample; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart10 && uart->gState == HAL_UART_STATE_READY);
    if (status != HAL_OK) return status;
    pending = data;
    pending_size = size;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    ++sends;
    return HAL_OK;
}

int main(void)
{
    sample.tick_ms = 1234U;
    sample.imu_seq = 1200U;
    sample.imu_valid = sample.imu_calibrated = 1U;
    sample.imu_age_ms = 2U;
    sample.raw_yaw = 36.75f;
    sample.raw_pitch = -52.335f;
    sample.raw_roll = -59.0f;
    sample.sensor_gyro[0] = 0.01745329252f;
    sample.sensor_gyro[1] = -0.03490658504f;
    sample.sensor_accel[2] = 9.81f;
    sample.ecd[0] = 5683U;
    sample.ecd[1] = 391U;
    sample.motor_age_ms[0] = 1U;
    sample.motor_age_ms[1] = 2U;
    sample.control.faults = 8U;
    sample.output[0] = -120;
    sample.output[1] = 30;
    sample.command = (Gimbal_Manual_Cmd_s){.online=1,.switch_left=2,.switch_right=1,
        .stick_yaw=125,.stick_pitch=-240,.request={.mode=GIMBAL_REQUEST_MANUAL,.axis_mask=1}};
    char buffer[512], copy[512];
    const char *expected = "IMU,1234,1200,1,1,2,36.750,-52.335,-59.000,1.000,-2.000,0.000,0.000,0.000,9.810,5683,391,1,2,0,0,8,-120,30,7,1,2,1,125,-240,0,2,1\r\n";
    assert(GimbalImuTextFormat(&sample, 7U, buffer, sizeof(buffer)) == strlen(expected));
    assert(strcmp(buffer, expected) == 0);
    assert(!GimbalImuTextFormat(&sample, 0U, buffer, 10U) && buffer[0] == '\0');
    sample.sensor_accel[0] = NAN;
    sample.sensor_accel[1] = INFINITY;
    assert(GimbalImuTextFormat(&sample, 0U, buffer, sizeof(buffer)));
    assert(strstr(buffer, ",nan,nan,9.810,"));
    assert(!GimbalImuTextHeader("USB", buffer, 10U) && buffer[0] == '\0');
    assert(GimbalImuTextHeader("USB", buffer, sizeof(buffer)));
    assert(strncmp(buffer, "# USB_IMU v2;", 13U) == 0);
    assert(strstr(buffer,"rc_online,sw_left,sw_right,stick_yaw,stick_pitch,centered,request,axis_mask"));

    GimbalImuUartInit();
    GimbalImuUartTask();
    assert(sends == 1U && strstr((char *)pending, "#type,t_ms,imu_seq,valid,cal,"));
    memcpy(copy, pending, pending_size + 1U);
    tick = 50U;
    sample.raw_pitch = 12.0f;
    GimbalImuUartTask();
    assert(sends == 1U && memcmp(copy, pending, pending_size + 1U) == 0);
    huart10.gState = HAL_UART_STATE_READY;
    tick = 100U;
    GimbalImuUartTask();
    assert(sends == 2U && strncmp((char *)pending, "IMU,", 4U) == 0);
    assert(strstr((char *)pending, "36.750,12.000,-59.000"));
    assert(strstr((char *)pending, ",-120,30,1,1,2,1,125,-240,0,2,1\r\n"));
    huart10.gState = HAL_UART_STATE_READY;
    tick = 110U;
    GimbalImuUartTask();
    assert(sends == 2U); /* no bursts between sample slots */
    tick = 1000U;
    status = HAL_ERROR;
    GimbalImuUartTask();
    assert(sends == 2U);
    tick = 1050U;
    status = HAL_OK;
    GimbalImuUartTask();
    assert(sends == 3U && pending[0] == '#'); /* retry header after failed TX */
    huart10.gState = HAL_UART_STATE_READY;
    tick = 1100U;
    GimbalImuUartTask();
    assert(sends == 4U && strstr((char *)pending, ",-120,30,2,1,2,1,125,-240,0,2,1\r\n"));

    /* HAL tick wrapping must not stall the capture schedule. */
    huart10.gState = HAL_UART_STATE_READY;
    tick = UINT32_MAX - 20U;
    GimbalImuUartInit();
    GimbalImuUartTask();
    assert(sends == 5U && pending[0] == '#');
    huart10.gState = HAL_UART_STATE_READY;
    tick = 29U;
    GimbalImuUartTask();
    assert(sends == 6U && pending[0] == 'I');
    puts("USART10 IMU logger tests passed");
    return 0;
}
