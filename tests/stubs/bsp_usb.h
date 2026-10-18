#ifndef TEST_BSP_USB_H
#define TEST_BSP_USB_H
#include <stdint.h>
#include "main.h"
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { void *pClassData; uint8_t dev_state; } USBD_HandleTypeDef;
extern USBD_HandleTypeDef hUsbDeviceHS;
uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length);
typedef struct { void (*tx_cbk)(uint16_t); void (*rx_cbk)(uint16_t); } USB_Init_Config_s;
uint8_t *USBInit(USB_Init_Config_s config);
uint8_t *USBGetRxBuffer(void);
uint8_t USBIsReady(void);
uint8_t USBTransmit(const uint8_t *data, uint16_t length);
#endif
