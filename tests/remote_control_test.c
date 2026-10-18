/* Exercise the actual RC decoder and command producer with a fake UART clock. */
#include "remote_control.h"
#include "bsp_usart.h"
#include "message_center.h"
#include "dt7_cmd.h"
#include "2yaw_gimbal.h"
#include "gimbal_manual_config.h"
#include "auto_aim_config.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

UART_HandleTypeDef huart5;
static uint32_t tick;
static uint8_t buffer[18];
static USARTInstance uart;
static Publisher_t publisher;
static Gimbal_Manual_Cmd_s command;
uint32_t HAL_GetTick(void) { return tick; }
USARTInstance *USARTRegister(USART_Init_Config_s *cfg)
{
    uart=(USARTInstance){.recv_buff=buffer,.recv_buff_size=18,.module_callback=cfg->module_callback};
    return &uart;
}
void USARTServiceInit(USARTInstance *instance) { (void)instance; }
void USARTEnsureReceive(USARTInstance *instance) { assert(instance==&uart); }
Publisher_t *PubRegister(char *name,uint8_t size)
{
    assert(strcmp(name,"gimbal_manual")==0 && size==sizeof(command));
    publisher.size=size;
    return &publisher;
}
uint8_t PubPushMessage(Publisher_t *pub,void *data)
{
    memcpy(&command,data,pub->size);
    return 1;
}
static void Frame(int yaw,int pitch,uint8_t left,uint8_t right)
{
    unsigned ch[4]={(unsigned)(1024+yaw),(unsigned)(1024+pitch),1024,1024};
    memset(buffer,0,sizeof(buffer));
    for (unsigned i=0;i<4;i++)
        for (unsigned bit=0;bit<11;bit++)
            if (ch[i]&(1U<<bit)) buffer[(11*i+bit)/8]|=1U<<((11*i+bit)%8);
    buffer[5]|=(right<<4)|(left<<6);
    buffer[17]=4; /* dial center 1024 */
    uart.recv_len=18;
}
static void Receive(void) { uart.module_callback(); RobotCMDTask(); }
int main(void)
{
    RobotCMDInit(); RobotCMDTask(); assert(!command.online);
    Frame(0,0,RC_SW_DOWN,RC_SW_DOWN); Receive();
    assert(command.online && command.request.mode==GIMBAL_REQUEST_OFF && command.request.sticks_centered);
    assert(command.switch_left==RC_SW_DOWN && command.switch_right==RC_SW_DOWN &&
           command.stick_yaw==0 && command.stick_pitch==0);
    assert(command.request.axis_mask==3);
    Frame(660,-660,RC_SW_MID,RC_SW_UP); Receive();
    assert(command.online && command.request.mode==GIMBAL_REQUEST_MANUAL && command.request.axis_mask==2);
    /* Physical right turn decreases INS Yaw, while rightward stick input is positive. */
    assert(fabsf(command.request.rate[0] - GIMBAL_YAW_COMMAND_SIGN *
                 GIMBAL_YAW_MAX_SPEED_DPS * GIMBAL_RAD_PER_DEG) < 0.00001f);
    assert(command.request.rate[0]<0 && command.request.rate[1]>0.25f);
    assert(fabsf(command.request.rate[1]/GIMBAL_RAD_PER_DEG-GIMBAL_PITCH_MAX_SPEED_DPS)<0.001f);
    Frame(-660,660,RC_SW_DOWN,RC_SW_UP); Receive();
    assert(command.stick_yaw==-660 && command.stick_pitch==660);
    assert(fabsf(command.request.rate[0] + GIMBAL_YAW_COMMAND_SIGN *
                 GIMBAL_YAW_MAX_SPEED_DPS * GIMBAL_RAD_PER_DEG) < 0.00001f);
    assert(command.request.rate[0]>0 && command.request.rate[1]<-0.25f);
    assert(command.request.axis_mask==3);
    assert(fabsf(command.request.rate[1]/GIMBAL_RAD_PER_DEG+GIMBAL_PITCH_MAX_SPEED_DPS)<0.001f);
    Frame(20,-20,RC_SW_UP,RC_SW_MID); Receive();
    assert(command.request.mode==(AUTO_AIM_USB_MODE ? GIMBAL_REQUEST_AUTO : GIMBAL_REQUEST_HOLD) && command.request.axis_mask==1);
    assert(command.request.sticks_centered && command.request.rate[0]==0 && command.request.rate[1]==0);
    Frame(GIMBAL_RC_DEADBAND,-GIMBAL_RC_DEADBAND,RC_SW_DOWN,RC_SW_UP); Receive();
    assert(command.request.sticks_centered && command.request.rate[0]==0 && command.request.rate[1]==0);
    Frame(GIMBAL_RC_DEADBAND+1,-GIMBAL_RC_DEADBAND-1,RC_SW_DOWN,RC_SW_UP); Receive();
    assert(!command.request.sticks_centered && command.request.rate[0]<0 && command.request.rate[1]>0);
    tick=101; RobotCMDTask(); assert(!command.online && command.request.mode==GIMBAL_REQUEST_OFF);
    Frame(0,0,RC_SW_UP,0); Receive(); assert(!command.online);
    Frame(0,0,0,RC_SW_UP); Receive(); assert(!command.online);
    Frame(700,0,RC_SW_UP,RC_SW_UP); Receive(); assert(!command.online);
    Frame(0,0,RC_SW_UP,RC_SW_UP); uart.recv_len=9; Receive(); assert(!command.online);
    Frame(0,0,RC_SW_UP,RC_SW_UP); buffer[12]=2; Receive(); assert(!command.online);
    Remote_Control_Diagnostics_s diagnostics;
    RemoteControlGetDiagnostics(&diagnostics);
    assert(diagnostics.good_frames>0 && diagnostics.bad_frames==5 && diagnostics.bad_reason==3);
    assert(diagnostics.last_length==18 && !diagnostics.online);
    Frame(0,0,RC_SW_UP,RC_SW_UP); tick=0xfffffff0U; Receive(); assert(command.online);
    tick=20; RobotCMDTask(); assert(command.online); /* unsigned wrap-around */
    tick=100; RobotCMDTask(); assert(!command.online);
    RemoteControlGetDiagnostics(&diagnostics);
    assert(diagnostics.age_ms==116 && !diagnostics.online);
    /* A fragment after a fresh valid frame holds only the last validated input.
     * Repeated fragments cannot extend its watchdog or revive invalid data. */
    tick=200; Frame(220,-180,RC_SW_DOWN,RC_SW_UP); Receive();
    tick=215; Frame(-660,660,RC_SW_UP,RC_SW_DOWN); uart.recv_len=9; Receive();
    assert(command.online && command.stick_yaw==220 && command.stick_pitch==-180);
    assert(command.request.axis_mask==3 && command.request.mode==GIMBAL_REQUEST_MANUAL);
    tick=300; Receive(); assert(command.online);
    tick=301; Receive(); assert(!command.online);
    RemoteControlGetDiagnostics(&diagnostics); assert(diagnostics.age_ms==101);
    Frame(0,0,RC_SW_DOWN,RC_SW_UP); Receive(); assert(command.online);
    Frame(0,0,0,RC_SW_UP); Receive(); assert(!command.online);
    Frame(0,0,RC_SW_DOWN,RC_SW_UP); uart.recv_len=9; Receive(); assert(!command.online);
    /* Disabled keyboard/mouse input must not take over the current stick path. */
    Frame(200,-200,RC_SW_DOWN,RC_SW_UP);
    buffer[6]=0xff; buffer[7]=0x7f; buffer[8]=0; buffer[9]=0x80;
    uint16_t keys=(1U<<Key_X)|(1U<<Key_R);
    buffer[14]=(uint8_t)keys; buffer[15]=(uint8_t)(keys>>8);
    Receive();
    assert(command.online && command.request.mode==GIMBAL_REQUEST_MANUAL);
    assert(command.request.rate[0]<0 && command.request.rate[1]>0);
    assert(command.stick_yaw==200 && command.stick_pitch==-200);
    puts("PASS: real DT7 decoder/command producer, switches, signs, deadband, short/malformed frames, timeout and disabled keyboard/mouse");
    return 0;
}
