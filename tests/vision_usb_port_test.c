/* Exercise imported production USB adapter; fake only Cube and the board clock. */
#include "vision_usb_port.h"
#include "bsp_usb.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

USBD_HandleTypeDef hUsbDeviceHS;
static USBD_CDC_HandleTypeDef cdc;
static USB_Init_Config_s callbacks;
static uint8_t rx[VISION_USB_PORT_RX_SIZE];
static uint8_t *active_tx;
static uint16_t active_length;
static uint32_t tick;

uint32_t HAL_GetTick(void) { return tick; }
uint8_t *USBInit(USB_Init_Config_s config) { callbacks = config; return rx; }
uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length)
{
    assert(hUsbDeviceHS.pClassData == &cdc && cdc.TxState == 0U);
    active_tx = buffer;
    active_length = length;
    cdc.TxState = 1U;
    return USBD_OK;
}
static void Receive(const uint8_t *data, uint16_t length)
{
    memcpy(rx, data, length);
    callbacks.rx_cbk(length);
}
static void Complete(void)
{
    cdc.TxState = 0U;
    callbacks.tx_cbk(active_length);
}
int main(void)
{
    VisionUSBPortInit();
    uint8_t data[256], reset;
    uint32_t received_ms;
    assert(!VisionUSBPortConnected());
    assert(!VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) && reset);
    assert(VisionUSBPortTransmit(data, 1U) == VISION_USB_PORT_OFFLINE);
    hUsbDeviceHS.pClassData = &cdc;
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    assert(VisionUSBPortConnected());
    uint8_t payload[] = {1U, 2U, 3U};
    assert(VisionUSBPortTransmit(payload, sizeof(payload)) == VISION_USB_PORT_OK);
    payload[0] = 99U;
    assert(active_tx[0] == 1U);
    assert(VisionUSBPortTransmit(payload, sizeof(payload)) == VISION_USB_PORT_BUSY);
    assert(active_tx[0] == 1U);
    Complete();
    VisionUSBPortStats_s stats = VisionUSBPortGetStats();
    assert(stats.tx_submitted == 1U && stats.tx_completed == 1U);

    const uint8_t frame[] = {0xCC, 1, 0, 0x10, 0x27, 0, 0, 0xEE};
    tick = 10U; Receive(frame, 3U);
    tick = 15U; Receive(frame + 3U, 5U);
    tick = 500U; /* Processing delay must not turn the old frame into new data. */
    assert(VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) == 3U);
    assert(!reset && received_ms == 10U && !memcmp(data, frame, 3U));
    assert(VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) == 5U);
    assert(!reset && received_ms == 15U && !memcmp(data, frame + 3U, 5U));
    memset(data, 0x55, 128U); Receive(data, 128U);
    assert(VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) == 64U);
    assert(VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) == 64U);
    assert(!VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) && !reset);

    Receive(frame, 3U);
    memset(rx, 0x55, sizeof(rx)); callbacks.rx_cbk(sizeof(rx));
    assert(!VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) && reset);
    assert(VisionUSBPortGetStats().rx_dropped_bytes == sizeof(rx) + 3U);
    Receive(frame, sizeof(frame));
    assert(VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) == sizeof(frame));
    assert(!reset && !memcmp(data, frame, sizeof(frame)));

    Receive(frame, 3U);
    hUsbDeviceHS.pClassData = NULL;
    hUsbDeviceHS.dev_state = 0U;
    assert(!VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) && reset);
    assert(VisionUSBPortTransmit(payload, sizeof(payload)) == VISION_USB_PORT_OFFLINE);
    hUsbDeviceHS.pClassData = &cdc;
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    Receive(frame, sizeof(frame));
    assert(VisionUSBPortRead(data, sizeof(data), &received_ms, &reset) == sizeof(frame));
    assert(!reset && !memcmp(data, frame, sizeof(frame)));
    assert(VisionUSBPortTransmit(NULL, 1U) == VISION_USB_PORT_ERROR);
    assert(VisionUSBPortTransmit(data, 65U) == VISION_USB_PORT_ERROR);
    puts("PASS: actual imported USB adapter, arrival timestamps, bounded reads, overflow/disconnect reset, async TX ownership");
    return 0;
}
