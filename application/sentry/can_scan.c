/** @file can_scan.c @brief Passive CAN1 scan; one bounded report in USB text mode. */
#include "can_scan.h"
#include "bsp_can.h"
#include "bsp_usb.h"
#include "gimbal_runtime.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define CAN_SCAN_SLOTS 8U
#define CAN_SCAN_WAIT_MS 2000U
#define CAN_SCAN_REPORT_CAPACITY 768U

typedef struct {
    uint32_t rx_count, last_rx_ms;
    uint16_t ecd;
} Can_Scan_Sample_s;

static CANInstance *scan_instances[CAN_SCAN_SLOTS];
static Can_Scan_Sample_s samples[CAN_SCAN_SLOTS];
static uint32_t start_ms;
static uint8_t scan_done;
static char report[CAN_SCAN_REPORT_CAPACITY];
static uint16_t report_length;

static void ScanCallback(CANInstance *instance)
{
    if (instance->rx_len != 8U) return; /* reject short/invalid motor feedback */
    for (uint8_t i = 0U; i < CAN_SCAN_SLOTS; ++i)
    {
        if (scan_instances[i] != instance) continue;
        if (samples[i].rx_count != UINT32_MAX) ++samples[i].rx_count;
        samples[i].ecd = ((uint16_t)instance->rx_buff[0] << 8) | instance->rx_buff[1];
        samples[i].last_rx_ms = HAL_GetTick();
        return;
    }
}

void CanScanInit(void)
{
    memset(samples, 0, sizeof(samples));
    report_length = 0U;
    scan_done = 0U;
    start_ms = HAL_GetTick();
    /* Passive listeners share existing filters; they never transmit CAN commands. */
    for (uint8_t i = 0U; i < CAN_SCAN_SLOTS; ++i)
    {
        CAN_Init_Config_s config = {
            .can_handle = &hfdcan1,
            .tx_id = 0x201U + i,
            .rx_id = 0x205U + i,
            .id_type = CAN_ID_STD,
            .can_module_callback = ScanCallback,
        };
        scan_instances[i] = CANRegister(&config);
    }
}

static uint16_t BuildReport(uint32_t now)
{
    Can_Scan_Sample_s snapshot[CAN_SCAN_SLOTS];
    taskENTER_CRITICAL();
    memcpy(snapshot, samples, sizeof(snapshot));
    taskEXIT_CRITICAL();
    now = HAL_GetTick(); /* the callback may have published a newer tick before the copy */
    int count = snprintf(report, sizeof(report), "# CAN1_SCAN t_ms=%lu\r\n", (unsigned long)now);
    if (count < 0 || (size_t)count >= sizeof(report)) return 0U;
    size_t used = (size_t)count;
    for (uint8_t i = 0U; i < CAN_SCAN_SLOTS; ++i)
    {
        if (!snapshot[i].rx_count) continue;
        count = snprintf(report + used, sizeof(report) - used,
            "# CAN1 id=%u rx=%lu ecd=%u age_ms=%lu\r\n", (unsigned)i + 1U,
            (unsigned long)snapshot[i].rx_count, (unsigned)snapshot[i].ecd,
            (unsigned long)(uint32_t)(now - snapshot[i].last_rx_ms));
        if (count < 0 || (size_t)count >= sizeof(report) - used) return 0U;
        used += (size_t)count;
    }
    count = snprintf(report + used, sizeof(report) - used, "# CAN1_SCAN done\r\n");
    if (count < 0 || (size_t)count >= sizeof(report) - used) return 0U;
    return (uint16_t)(used + (size_t)count);
}

void CanScanTask(void)
{
    uint32_t now = HAL_GetTick();
    if (scan_done || (uint32_t)(now - start_ms) < CAN_SCAN_WAIT_MS ||
        !USBIsReady() || !GimbalRuntimeTextIsStreaming()) return;
    if (!report_length) report_length = BuildReport(now);
    /* One complete packet: BSP copies into its protected staging buffer.
     * Keep the report until USB accepts it; never inject text into JustFloat.
     */
    if (report_length && USBTransmit((const uint8_t *)report, report_length))
        scan_done = 1U;
}
