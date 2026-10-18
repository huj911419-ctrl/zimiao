/** @file vision_usb_port.h
 * @brief DM-MC02/Cube CDC adapter. Owns the single CDC interface exclusively.
 * Init once before USB enumeration; call all public operations from one task.
 * No task creation, logs, watchdog or test payloads. Do not also call VisionInit/USBInit.
 */
#ifndef VISION_USB_PORT_H
#define VISION_USB_PORT_H
#include <stdint.h>

#define VISION_USB_PORT_RX_SIZE 2048u
#define VISION_USB_PORT_TX_MAX 64u

typedef enum
{
    VISION_USB_PORT_OK = 0,
    VISION_USB_PORT_BUSY,
    VISION_USB_PORT_OFFLINE,
    VISION_USB_PORT_ERROR
} VisionUSBPortResult_e;

typedef struct
{
    uint32_t rx_dropped_bytes;
    uint32_t tx_submitted;
    uint32_t tx_completed;
} VisionUSBPortStats_s;

void VisionUSBPortInit(void);
uint8_t VisionUSBPortConnected(void);
/** Drain bytes sharing one ISR receive timestamp. reset_stream=1 requires caller
 * to invalidate its parser/latest command (disconnect or RX overflow). */
uint16_t VisionUSBPortRead(uint8_t *buffer, uint16_t capacity,
                           uint32_t *received_ms, uint8_t *reset_stream);
/** Copies accepted payload into adapter storage; caller may reuse buffer on OK.
 * OK means submitted, not delivered. BUSY retains nothing; caller retries later. */
VisionUSBPortResult_e VisionUSBPortTransmit(const uint8_t *buffer, uint16_t len);
/** Return a coherent counter snapshot, including ISR-owned counters. */
VisionUSBPortStats_s VisionUSBPortGetStats(void);
#endif
