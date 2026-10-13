#ifndef TEST_BSP_USB_H
#define TEST_BSP_USB_H
#include <stdint.h>
typedef struct { void (*tx_cbk)(uint16_t); void (*rx_cbk)(uint16_t); } USB_Init_Config_s;
uint8_t *USBInit(USB_Init_Config_s config);
uint8_t *USBGetRxBuffer(void);
uint8_t USBIsReady(void);
uint8_t USBTransmit(const uint8_t *data, uint16_t length);
#endif
