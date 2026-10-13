/**
 * @file bsp_usb.h
 * @brief USB CDC(VCP) BSP.
 */
#pragma once

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

typedef struct
{
    USBCallback tx_cbk;
    USBCallback rx_cbk;
} USB_Init_Config_s;

/* @note 虚拟串口的波特率/校验位/数据位等由上位机设置,固件侧无需固定这些参数。 */
uint8_t *USBInit(USB_Init_Config_s usb_conf);
uint8_t *USBGetRxBuffer(void);

/* USB CDC 已完成枚举时返回1。 */
uint8_t USBIsReady(void);

/*
 * 发送数据时内部会先复制到持久缓冲区，避免把任务栈上的临时buffer
 * 直接交给异步USB发送。成功提交返回1，忙/未枚举/参数错误返回0。
 */
uint8_t USBTransmit(const uint8_t *buffer, uint16_t len);
