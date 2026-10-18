/** @file vision_usb_port.c
 * @brief Board-specific critical sections, CDC lifetime and receive queue.
 */
#include "vision_usb_port.h"
#include <bsp_usb.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;
static uint8_t *port_rx_buffer;
static uint8_t port_rx_ring[VISION_USB_PORT_RX_SIZE];
static uint32_t port_rx_time[VISION_USB_PORT_RX_SIZE];
static volatile uint16_t port_rx_head;
static volatile uint16_t port_rx_tail;
static volatile uint8_t port_rx_overflow;
static uint8_t port_tx_buffer[VISION_USB_PORT_TX_MAX];
static volatile VisionUSBPortStats_s port_stats;

static uint8_t PortConnectedLocked(void)
{
    return hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceHS.pClassData != NULL;
}

static void PortOnReceive(uint16_t len)
{
    uint16_t free_bytes = (uint16_t)((port_rx_tail + VISION_USB_PORT_RX_SIZE - port_rx_head - 1u) % VISION_USB_PORT_RX_SIZE);
    if (len > free_bytes)
    {
        port_stats.rx_dropped_bytes += len;
        port_rx_overflow = 1u;
        return;
    }
    uint16_t head = port_rx_head;
    uint32_t now = HAL_GetTick();
    for (uint16_t i = 0u; i < len; i++)
    {
        port_rx_ring[head] = port_rx_buffer[i];
        port_rx_time[head] = now;
        head = (uint16_t)((head + 1u) % VISION_USB_PORT_RX_SIZE);
    }
    __DMB();
    port_rx_head = head;
}

static void PortOnTransmit(uint16_t len)
{
    (void)len;
    port_stats.tx_completed++;
}

void VisionUSBPortInit(void)
{
    /* This application starts USB before RobotInit(). Register callbacks and
     * publish their RX pointer atomically even if the host already enumerated. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    USB_Init_Config_s config = { .tx_cbk = PortOnTransmit, .rx_cbk = PortOnReceive };
    port_rx_buffer = USBInit(config);
    __set_PRIMASK(primask);
}

uint8_t VisionUSBPortConnected(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t connected = PortConnectedLocked();
    __set_PRIMASK(primask);
    return connected;
}

uint16_t VisionUSBPortRead(uint8_t *buffer, uint16_t capacity,
                           uint32_t *received_ms, uint8_t *reset_stream)
{
    if (buffer == NULL || received_ms == NULL || reset_stream == NULL || capacity == 0u) return 0u;
    *reset_stream = 0u;
    *received_ms = 0u;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!PortConnectedLocked() || port_rx_overflow)
    {
        port_stats.rx_dropped_bytes += (port_rx_head + VISION_USB_PORT_RX_SIZE - port_rx_tail) % VISION_USB_PORT_RX_SIZE;
        port_rx_tail = port_rx_head;
        port_rx_overflow = 0u;
        *reset_stream = 1u;
        __set_PRIMASK(primask);
        return 0u;
    }
    uint16_t len = 0u;
    if (port_rx_tail != port_rx_head) *received_ms = port_rx_time[port_rx_tail];
    /* Bound interrupt masking even if a caller supplies an oversized buffer. */
    if (capacity > 64u) capacity = 64u;
    while (port_rx_tail != port_rx_head && len < capacity && port_rx_time[port_rx_tail] == *received_ms)
    {
        buffer[len++] = port_rx_ring[port_rx_tail];
        port_rx_tail = (uint16_t)((port_rx_tail + 1u) % VISION_USB_PORT_RX_SIZE);
    }
    __set_PRIMASK(primask);
    return len;
}

VisionUSBPortResult_e VisionUSBPortTransmit(const uint8_t *buffer, uint16_t len)
{
    if (buffer == NULL || len == 0u || len > sizeof(port_tx_buffer)) return VISION_USB_PORT_ERROR;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    VisionUSBPortResult_e result = VISION_USB_PORT_OFFLINE;
    if (PortConnectedLocked())
    {
        USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
        result = VISION_USB_PORT_BUSY;
        if (cdc->TxState == 0u)
        {
            memcpy(port_tx_buffer, buffer, len);
            uint8_t status = CDC_Transmit_HS(port_tx_buffer, len);
            result = status == USBD_OK ? VISION_USB_PORT_OK : VISION_USB_PORT_ERROR;
            if (status == USBD_BUSY) result = VISION_USB_PORT_BUSY;
            if (result == VISION_USB_PORT_OK) port_stats.tx_submitted++;
        }
    }
    __set_PRIMASK(primask);
    return result;
}

VisionUSBPortStats_s VisionUSBPortGetStats(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    VisionUSBPortStats_s snapshot = port_stats;
    __set_PRIMASK(primask);
    return snapshot;
}
