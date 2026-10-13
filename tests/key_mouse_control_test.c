/* Real DBUS decoder -> optional command adapter -> actual arming controller. */
#include "remote_control.h"
#include "bsp_usart.h"
#include "message_center.h"
#include "dt7_cmd.h"
#include "2yaw_gimbal.h"
#include "gimbal_key_mouse_config.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#if !GIMBAL_KEY_MOUSE_ENABLED
#error "This test must compile the enabled input path"
#endif

UART_HandleTypeDef huart5;
static uint32_t tick;
static uint8_t buffer[18];
static USARTInstance uart;
static Publisher_t publisher;
static Gimbal_Manual_Cmd_s command;
static Gimbal_Control_s control;
static Gimbal_Axis_Config_s cfg[2];
static Gimbal_Axis_Feedback_s fb[2];
uint32_t HAL_GetTick(void) { return tick; }
USARTInstance *USARTRegister(USART_Init_Config_s *c)
{
    uart=(USARTInstance){.recv_buff=buffer,.recv_buff_size=18,.module_callback=c->module_callback};
    return &uart;
}
void USARTServiceInit(USARTInstance *instance) { (void)instance; }
void USARTEnsureReceive(USARTInstance *instance) { assert(instance==&uart); }
Publisher_t *PubRegister(char *name,uint8_t size)
{
    assert(strcmp(name,"gimbal_manual")==0 && size==sizeof(command));
    publisher.size=size; return &publisher;
}
uint8_t PubPushMessage(Publisher_t *pub,void *data)
{
    memcpy(&command,data,pub->size); return 1;
}
static void Task(void)
{
    RobotCMDTask();
    GimbalControlStep(&control,cfg,fb,&command.request,0.001f,
                     command.online ? 0 : GIMBAL_FAULT_RC,2);
}
static void Frame(int16_t mx,int16_t my,uint16_t keys,uint8_t left,uint8_t right)
{
    memset(buffer,0,sizeof(buffer));
    for (unsigned i=0;i<4;i++)
        buffer[(11*i+10)/8]|=1U<<((11*i+10)%8); /* all sticks centered at 1024 */
    buffer[5]|=(right<<4)|(left<<6); buffer[17]=4;
    buffer[6]=(uint8_t)mx; buffer[7]=(uint16_t)mx>>8;
    buffer[8]=(uint8_t)my; buffer[9]=(uint16_t)my>>8;
    buffer[14]=(uint8_t)keys; buffer[15]=keys>>8;
    uart.recv_len=18;
}
static void Receive(void) { uart.module_callback(); Task(); }
static void Input(int16_t mx,int16_t my,uint16_t keys,uint8_t right)
{
    Frame(mx,my,keys,RC_SW_DOWN,right); Receive();
}
static void Enable(void)
{
    Input(0,0,0,RC_SW_DOWN); assert(control.state==GIMBAL_STATE_READY);
    Input(0,0,0,RC_SW_UP); assert(control.state==GIMBAL_STATE_MANUAL);
}
static float Rate(unsigned axis) { return command.request.rate[axis]/GIMBAL_RAD_PER_DEG; }

int main(void)
{
    for (unsigned i=0;i<2;i++)
    {
        cfg[i]=(Gimbal_Axis_Config_s){4,1200,0,1000,5000,5,10,0.2f,1,0,0};
        fb[i]=(Gimbal_Axis_Feedback_s){.soft_min=-2,.soft_max=2,.hard_min=-3,.hard_max=3};
    }
    RobotCMDInit(); GimbalControlInit(&control);
    Input(100,-100,0,RC_SW_UP); assert(!control.armed && control.axis[0].output==0);
    Input(0,0,0,RC_SW_DOWN);
    Input(100,-100,0,RC_SW_UP); assert(!control.armed); /* moving mouse at enable */
    Enable(); Input(100,-100,0,RC_SW_UP);
    assert(fabsf(Rate(0)+25)<1e-4f && fabsf(Rate(1)+20)<1e-4f);
    for (unsigned i=0;i<20;i++) Task();
    assert(fabsf(Rate(0)+25)<1e-4f); /* rate does not depend on task invocation count */
    Input(-100,100,0,RC_SW_UP); assert(Rate(0)>0 && Rate(1)>0);
    Input(0,0,(1U<<Key_Q)|(1U<<Key_R),RC_SW_UP);
    assert(fabsf(Rate(0)-45)<1e-4f && fabsf(Rate(1)+30)<1e-4f);
    Input(0,0,(1U<<Key_E)|(1U<<Key_F),RC_SW_UP);
    assert(Rate(0)<0 && Rate(1)>0);
    Input(0,0,(1U<<Key_Q)|(1U<<Key_E)|(1U<<Key_R)|(1U<<Key_F),RC_SW_UP);
    assert(Rate(0)==0 && Rate(1)==0);
    Input(32767,-32768,0,RC_SW_UP);
    assert(fabsf(Rate(0)+120)<1e-4f && fabsf(Rate(1)+60)<1e-4f);
    Input(32767,-32768,1U<<Key_Ctrl,RC_SW_UP);
    assert(fabsf(Rate(0)+30)<1e-4f && fabsf(Rate(1)+15)<1e-4f);
    Input(100,-100,1U<<Key_B,RC_SW_UP);
    assert(control.state==GIMBAL_STATE_HOLD && Rate(0)==0 && Rate(1)==0);
    Input(100,-100,0,RC_SW_MID);
    assert(control.state==GIMBAL_STATE_HOLD && Rate(0)==0 && Rate(1)==0);
    Input(0,0,1U<<Key_X,RC_SW_UP);
    assert(!control.armed && control.axis[0].output==0 && control.axis[1].output==0);
    Input(0,0,0,RC_SW_UP); assert(!control.armed);
    Input(0,0,1U<<Key_Q,RC_SW_DOWN);
    Input(0,0,0,RC_SW_UP); assert(!control.armed); /* non-neutral OFF cannot clear X */
    Enable();
    Frame(0,0,0,RC_SW_DOWN,RC_SW_DOWN); buffer[12]=1; Receive();
    assert(!control.armed); /* pressed mouse button also blocks preparation */
    Enable();
    Input(100,100,0,RC_SW_UP); tick+=50; Task(); assert(command.online);
    tick++; Task();
    assert(!command.online && (control.faults&GIMBAL_FAULT_RC));
    assert(control.axis[0].output==0 && control.axis[1].output==0);
    Input(0,0,0,RC_SW_UP); assert(!control.armed && control.faults);
    Enable();
    Input(100,100,0,RC_SW_UP); tick+=20;
    Frame(0,0,0,RC_SW_DOWN,RC_SW_UP); uart.recv_len=9; Receive();
    assert(command.online && Rate(0)<0);
    tick+=31; Receive(); assert(!command.online); /* fragments cannot refresh deadline */
    Enable();
    Frame(0,0,0,RC_SW_DOWN,RC_SW_UP); buffer[12]=2; Receive();
    assert(!command.online && !control.armed); /* malformed DBUS stops the controller */
    Enable();
    Frame(0,0,0,RC_SW_MID,RC_SW_DOWN); Receive();
    Frame(0,0,0,RC_SW_MID,RC_SW_UP); Receive();
    Frame(100,100,0,RC_SW_MID,RC_SW_UP); Receive();
    assert(control.axis_mask==2 && control.axis[0].output==0 && control.axis[1].output>0);
    tick=0xfffffff0U; Input(0,0,0,RC_SW_DOWN);
    tick=20; Task(); assert(command.online); /* 36 ms across tick wrap */
    tick=35; Task(); assert(!command.online); /* 51 ms across tick wrap */
    puts("PASS: enabled keyboard/mouse DBUS mapping, signs, caps, precision, holds, latched stop, neutral arming, freshness and faults");
    return 0;
}
