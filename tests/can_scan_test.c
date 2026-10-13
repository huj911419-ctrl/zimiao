/* Real CAN scan with fake receive/USB: complete packet, busy retry, format isolation. */
#include "can_scan.h"
#include "bsp_can.h"
#include "bsp_usb.h"
#include "gimbal_runtime.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1,hfdcan2;
static uint32_t tick;
static CANInstance instances[8];
static unsigned registered,sends;
static uint8_t usb_ready,text_stream=1,usb_busy;
static char last_packet[1025];
static uint8_t *pending_pointer;
static uint16_t pending_length;

uint32_t HAL_GetTick(void) { return tick; }
uint8_t USBIsReady(void) { return usb_ready; }
uint8_t GimbalRuntimeTextIsStreaming(void) { return text_stream; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    unsigned slot=registered++;
    assert(slot<8 && config->can_handle==&hfdcan1 && config->rx_id==0x205U+slot);
    instances[slot]=(CANInstance){.rx_id=config->rx_id,.can_module_callback=config->can_module_callback};
    return &instances[slot];
}
uint8_t USBTransmit(const uint8_t *buffer,uint16_t size)
{
    assert(size>0 && size<=1024);
    if (usb_busy)
    {
        pending_pointer=(uint8_t *)buffer; pending_length=size;
        memcpy(last_packet,buffer,size); last_packet[size]='\0';
        return 0;
    }
    if (pending_pointer)
        assert(size==pending_length && memcmp(last_packet,buffer,size)==0);
    memcpy(last_packet,buffer,size); last_packet[size]='\0'; ++sends;
    return 1;
}
static void Receive(unsigned slot,uint8_t length)
{
    CANInstance *instance=&instances[slot];
    instance->rx_len=length; instance->rx_buff[0]=0x12; instance->rx_buff[1]=0x34;
    instance->can_module_callback(instance);
}
int main(void)
{
    CanScanInit(); assert(registered==8);
    Receive(1,2); /* rejected malformed feedback */
    for(unsigned i=0;i<300;i++) Receive(0,8);
    for(unsigned i=2;i<8;i++) Receive(i,8);
    tick=2500; CanScanTask(); assert(sends==0); /* no USB enumeration */
    usb_ready=1; text_stream=0; CanScanTask(); assert(sends==0); /* JustFloat/paused */
    text_stream=1; usb_busy=1; CanScanTask(); assert(sends==0 && pending_pointer);
    assert(strstr(last_packet,"# CAN1 id=1 rx=300 ecd=4660 age_ms=2500\r\n"));
    assert(!strstr(last_packet,"# CAN1 id=2 "));
    assert(strstr(last_packet,"# CAN1 id=8 rx=1 ") && strstr(last_packet,"# CAN1_SCAN done\r\n"));
    tick=2600; Receive(0,8); CanScanTask(); assert(sends==0);
    usb_busy=0; CanScanTask(); assert(sends==1);
    tick=5000; CanScanTask(); assert(sends==1); /* report only once */
    puts("PASS: CAN scan USB readiness, text isolation, one complete packet, busy retry, frame length and 32-bit count");
    return 0;
}
