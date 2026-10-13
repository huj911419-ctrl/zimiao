/**
 * @file bsp_usb.c
 * @brief USB CDC(VCP) BSP.
 */

#include "bsp_usb.h"
#include "bsp_log.h"

#include <string.h>

#define BSP_USB_TX_STAGING_SIZE 1024U

static uint8_t *bsp_usb_rx_buffer;
static uint8_t bsp_usb_tx_staging[BSP_USB_TX_STAGING_SIZE];

extern USBD_HandleTypeDef hUsbDeviceHS;

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk);
    LOGINFO("USB init success");
    return bsp_usb_rx_buffer;
}

uint8_t *USBGetRxBuffer(void)
{
    return bsp_usb_rx_buffer;
}

uint8_t USBIsReady(void)
{
    return (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
            hUsbDeviceHS.pClassData != NULL) ? 1U : 0U;
}

uint8_t USBTransmit(const uint8_t *buffer, uint16_t len)
{
    if (!USBIsReady() || buffer == NULL || len == 0U || len > BSP_USB_TX_STAGING_SIZE)
        return 0U;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;

    /*
     * USBD_CDC_SetTxBuffer() does not copy user data. The caller's memory must
     * stay valid until the IN transfer completes, so never overwrite our
     * staging buffer while TxState is busy.
     */
    if (hcdc->TxState != 0U)
        return 0U;

    memcpy(bsp_usb_tx_staging, buffer, len);

    return (CDC_Transmit_HS(bsp_usb_tx_staging, len) == USBD_OK) ? 1U : 0U;
}
