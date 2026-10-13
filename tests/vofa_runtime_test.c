/* Exercise real runtime + real gimbal adapter; fake only HAL, transport and RTOS. */
#include "gimbal_runtime.h"
#include "2yaw_gimbal.h"
#include "bsp_usb.h"
#include "message_center.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "queue.h"
#include "fdcan.h"
#include "remote_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2;
static uint32_t tick;
static DJIMotorInstance motor[2];
static Gimbal_Manual_Cmd_s rc;
static uint8_t usb_rx[1024], last_frame[VOFA_FRAME_SIZE];
static char last_packet[1025];
static uint16_t last_length;
static void (*rx_callback)(uint16_t);
static unsigned frames_sent;
static uint8_t usb_busy;
static float imu_roll;
static float imu_yaw, imu_pitch;
static float imu_pitch_rate;
static float imu_yaw_rate;
static uint8_t can_diag_error;
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *bus, FDCAN_ProtocolStatusTypeDef *status)
{
    status->BusOff = bus == &hfdcan1;
    return can_diag_error ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *bus, FDCAN_ErrorCountersTypeDef *errors)
{
    errors->TxErrorCnt = bus == &hfdcan1 ? 128U : 0U;
    errors->RxErrorCnt = bus == &hfdcan1 ? 4U : 0U;
    return can_diag_error ? HAL_ERROR : HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *bus) { return bus == &hfdcan1 ? 0U : 3U; }
static struct { unsigned size, count; uint8_t data[8][64]; } queue;

uint32_t HAL_GetTick(void) { return tick; }
void INS_SetMounting(float yaw, float pitch, float roll) { (void)yaw; (void)pitch; (void)roll; }
attitude_t *INS_Init(void) { static attitude_t state; return &state; }
uint8_t INS_GetVisionSnapshot(INS_Vision_Snapshot_t *sample)
{
    *sample = (INS_Vision_Snapshot_t){.q={1,0,0,0},.roll=imu_roll,.yaw=imu_yaw,.pitch=imu_pitch,
        .timestamp_ms=tick,.sample_seq=tick,.valid=1,.calibrated=1,.gyro={imu_pitch_rate,0,imu_yaw_rate},
        .sensor_gyro={0.1f,-0.2f,0.3f},.sensor_accel={1.0f,2.0f,9.5f}};
    return 1;
}
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *m = &motor[config->can_init_config.tx_id - 1];
    m->measure.ecd = config->can_init_config.tx_id == 1 ? 4000 : 889;
    m->measure.total_angle = m->measure.ecd * 360.0f / 8192.0f;
    m->rx_count = 1;
    return m;
}
void DJIMotorStop(DJIMotorInstance *m) { m->stop_flag=MOTOR_STOP; m->last_set=0; }
void DJIMotorSetRef(DJIMotorInstance *m,float value) { m->ref=value; }
void DJIMotorEnable(DJIMotorInstance *m) { m->stop_flag=MOTOR_ENALBED; m->last_set=(int16_t)m->ref; }
Subscriber_t *SubRegister(char *name,uint8_t size) { static Subscriber_t sub; (void)name; sub.size=size; return &sub; }
uint8_t SubGetMessage(Subscriber_t *sub,void *data) { memcpy(data,&rc,sub->size); return 1; }
void RobotCMDInit(void) {}
void RobotCMDTask(void) { rc.timestamp_ms=tick; }
void RemoteControlGetDiagnostics(Remote_Control_Diagnostics_s *sample)
{
    *sample=(Remote_Control_Diagnostics_s){.age_ms=14,.good_frames=123,.bad_frames=2,
        .last_length=9,.bad_reason=1,.uart_errors=3,.last_uart_error=8,.receive_retries=4,.start_failures=5};
}
uint8_t *USBInit(USB_Init_Config_s cfg) { rx_callback=cfg.rx_cbk; return usb_rx; }
uint8_t *USBGetRxBuffer(void) { return usb_rx; }
uint8_t USBIsReady(void) { return 1; }
uint8_t USBTransmit(const uint8_t *data,uint16_t length)
{
    assert(length>0 && length<sizeof(last_packet));
    if (usb_busy) return 0;
    memcpy(last_packet,data,length); last_packet[length]='\0'; last_length=length;
    if (length==VOFA_FRAME_SIZE) memcpy(last_frame,data,length);
    ++frames_sent; return 1;
}
QueueHandle_t xQueueCreate(unsigned length,unsigned size)
{
    assert(length==8 && size<=64); queue.size=size; return &queue;
}
BaseType_t xQueueReceive(QueueHandle_t q,void *item,TickType_t wait)
{
    (void)q; (void)wait;
    if (!queue.count) return 0;
    memcpy(item,queue.data[0],queue.size);
    --queue.count;
    memmove(queue.data[0],queue.data[1],queue.count*64);
    return pdPASS;
}
BaseType_t xQueueSend(QueueHandle_t q,const void *item,TickType_t wait)
{
    (void)q; (void)wait;
    if (queue.count==8) return 0;
    memcpy(queue.data[queue.count++],item,queue.size); return pdPASS;
}
BaseType_t xQueueSendToFront(QueueHandle_t q,const void *item,TickType_t wait)
{
    (void)q; (void)wait;
    if (queue.count==8) return 0;
    memmove(queue.data[1],queue.data[0],queue.count*64);
    memcpy(queue.data[0],item,queue.size); ++queue.count; return pdPASS;
}
BaseType_t xQueueReset(QueueHandle_t q) { (void)q; queue.count=0; return pdPASS; }
static float Channel(unsigned index)
{
    assert(last_length==VOFA_FRAME_SIZE);
    const uint8_t tail[]={0,0,0x80,0x7f};
    assert(memcmp(last_frame+VOFA_CHANNEL_COUNT*4,tail,4)==0);
    float v; memcpy(&v,last_frame+index*4,4); return v;
}
static void Step(void)
{
    ++tick;
    motor[0].last_rx_ms=motor[1].last_rx_ms=tick;
    GimbalRuntimeControl();
}
static void Send(const char *text)
{
    size_t length=strlen(text); assert(length<=sizeof(usb_rx));
    memcpy(usb_rx,text,length); rx_callback((uint16_t)length); GimbalRuntimeComm();
}
static void Command(const char *text,Vofa_Result_e expected)
{
    Send(text); Step(); GimbalRuntimeComm();
    if (Channel(23)!=expected)
        fprintf(stderr,"Command %s: result %.0f expected %u\n",text,Channel(23),(unsigned)expected);
    assert(Channel(23)==expected);
}
static void TextCommand(const char *text,Vofa_Result_e expected)
{
    Send(text); Step(); GimbalRuntimeComm();
    char result[30];
    snprintf(result,sizeof(result)," result=%u ",(unsigned)expected);
    assert(strncmp(last_packet,"# ACK seq=",10)==0 && strstr(last_packet,result));
}

int main(void)
{
    rc=(Gimbal_Manual_Cmd_s){.online=1,.request={.mode=GIMBAL_REQUEST_MANUAL,.axis_mask=1,.sticks_centered=1,.rate={0.1f,0}}};
    GimbalRuntimeInit(); tick=1001; Step(); GimbalRuntimeComm();
    assert(strncmp(last_packet,"# USB_IMU v2;",13)==0);
    assert(GimbalRuntimeTextIsStreaming());
    Gimbal_Debug_Data_s boot_debug;
    for (unsigned i=0;i<20;i++)
    {
        Step(); GimbalGetDebugData(&boot_debug);
        assert(boot_debug.stage==2 && !boot_debug.control.armed);
        assert(boot_debug.output[0]==0 && boot_debug.output[1]==0);
    }
    rc.request.mode=GIMBAL_REQUEST_OFF; rc.request.rate[0]=0; Step();
    motor[0].measure.torque_current_raw=-123;
    motor[0].measure.speed_rpm_raw=-7;
    motor[0].measure.temperature=41;
    motor[0].tx_enqueue_ok=12345;
    motor[0].tx_enqueue_fail=6;
    tick+=50; Step(); GimbalRuntimeComm();
    assert(strncmp(last_packet,"IMU,",4)==0);
    assert(strstr(last_packet,"0.000,0.000,0.000,5.730,-11.459,17.189,1.000,2.000,9.500,4000,889,"));
    assert(strstr(last_packet,"# MOTOR axis=yaw ") && strstr(last_packet,"i_raw=-123 rpm=-7 q_ok=12345 q_fail=6 can_ok=1 bo=1 tec=128 rec=4 tx_free=0 temp_c=41"));
    assert(strstr(last_packet,"# MOTOR axis=pitch ") && strstr(last_packet,"can_ok=1 bo=0 tec=0 rec=0 tx_free=3"));
    assert(strstr(last_packet,"# CTRL axis=yaw "));
    assert(strstr(last_packet,"# RC age_ms=14 ok=123 bad=2 len=9 why=1 uart_err=3 err=0x8 retry=4 start_fail=5"));
    assert(strstr(last_packet,"target_dps=0.000 actual_dps=0.000 i_term=0.000 speed_max_dps=240.000 accel_dps2=720.000 out_max=15000"));
    tick+=50; Step(); GimbalRuntimeComm();
    assert(strncmp(last_packet,"IMU,",4)==0 && !strstr(last_packet,"# MOTOR") && !strstr(last_packet,"# CTRL"));
    can_diag_error=1;
    tick+=250; Step(); GimbalRuntimeComm();
    assert(strncmp(last_packet,"IMU,",4)==0 && strstr(last_packet,"can_ok=0"));
    can_diag_error=0;
    Command("imu 0\n",VOFA_RESULT_OK);
    assert(!GimbalRuntimeTextIsStreaming());
    assert(Channel(14)==2 && Channel(27)==8 && Channel(28)==24000);
    assert(Channel(29)==15000 && Channel(33)==15000 && Channel(34)==15000);
    assert(Channel(31)==10000 && Channel(32)==9000);
    assert(Channel(46)==GIMBAL_REQUEST_OFF && Channel(47)==1);
    Gimbal_Debug_Data_s debug;
    GimbalGetDebugData(&debug);
    assert(debug.config[0].integral_limit==5000);
    assert(debug.config[1].integral_limit==8000);
    assert(debug.config[0].rate_filter_hz==50 && debug.config[1].rate_filter_hz==30);
    assert(debug.config[1].angle_kp==4);
    assert(fabsf(debug.config[0].speed_limit/GIMBAL_RAD_PER_DEG-240)<0.001f);
    assert(fabsf(debug.config[0].acceleration_limit/GIMBAL_RAD_PER_DEG-720)<0.001f);
    assert(debug.imu_valid && debug.imu_calibrated && debug.imu_seq==tick);
    assert(debug.sensor_gyro[0]==0.1f && debug.sensor_accel[2]==9.5f);
    assert(debug.motor_current_raw[0]==-123 && debug.motor_rpm_raw[0]==-7 && debug.tx_enqueue_fail[0]==6);
    assert(debug.gyro[0]==0.0f); /* sensor frame is independent of corrected INS gyro */
    assert(fabsf(debug.config[1].speed_limit/GIMBAL_RAD_PER_DEG-120)<0.001f);
    assert(fabsf(debug.config[1].acceleration_limit/GIMBAL_RAD_PER_DEG-360)<0.001f);
    assert(debug.config[1].soft_recovery_speed==30.0f*GIMBAL_RAD_PER_DEG);
    /* Observe-only must stay passive long after the old automatic-test delay. */
    Command("stage 0\n",VOFA_RESULT_OK);
    tick=4500; rc.request.mode=GIMBAL_REQUEST_MANUAL; rc.request.rate[0]=0.1f;
    for (unsigned i=0;i<20;i++)
    {
        Step(); GimbalGetDebugData(&debug);
        assert(debug.stage==0 && debug.output[0]==0 && debug.output[1]==0);
    }
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    Send("set yaw speed_"); Step(); Send("kp 1500\r\n"); Step(); GimbalRuntimeComm();
    assert(Channel(23)==VOFA_RESULT_OK && Channel(24)==1500 && Channel(28)==1500);
    Command("stage 1\n",VOFA_RESULT_OK); assert(Channel(14)==1);
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    Command("set yaw speed_kp 2000\n",VOFA_RESULT_REQUIRE_OFF); assert(Channel(28)==1500);
    Command("stop\n",VOFA_RESULT_OK); assert(Channel(14)==0 && motor[0].stop_flag==MOTOR_STOP);
    for (unsigned i=0;i<20;i++)
    {
        Step(); GimbalGetDebugData(&debug);
        assert(debug.stage==0 && debug.output[0]==0 && debug.output[1]==0);
    }
    Command("stage 2\n",VOFA_RESULT_REQUIRE_OFF);
    assert(Channel(46)==GIMBAL_REQUEST_MANUAL);
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    rc.request.sticks_centered=0;
    Command("set yaw speed_kp 2000\n",VOFA_RESULT_REQUIRE_OFF); assert(Channel(47)==0);
    rc.request.sticks_centered=1; Step();
    Command("set yaw output_limit 15001\n",VOFA_RESULT_RANGE);
    Command("set yaw output_limit 14000\n",VOFA_RESULT_OK); assert(Channel(33)==14000);
    Command("set yaw output_limit 15000\n",VOFA_RESULT_OK); assert(Channel(33)==15000);
    Command("set pitch output_limit 15001\n",VOFA_RESULT_RANGE); assert(Channel(34)==15000);
    Command("set pitch output_limit 9000\n",VOFA_RESULT_OK); assert(Channel(34)==9000);
    Command("set pitch output_limit 10000\n",VOFA_RESULT_OK); assert(Channel(34)==10000);
    Command("set pitch integral_limit 8001\n",VOFA_RESULT_RANGE);
    Command("set pitch speed_dps 120\n",VOFA_RESULT_OK);
    Command("set pitch speed_dps 121\n",VOFA_RESULT_RANGE);
    Command("set yaw integral_limit 8001\n",VOFA_RESULT_RANGE);
    Command("set yaw accel_dps2 720\n",VOFA_RESULT_OK);
    Command("set yaw accel_dps2 721\n",VOFA_RESULT_RANGE);
    Command("set pitch accel_dps2 361\n",VOFA_RESULT_RANGE);
    Command("set yaw speed_kp nan\n",VOFA_RESULT_SYNTAX);
    Command("get yaw speed_kp\n",VOFA_RESULT_OK); assert(Channel(24)==1500);
    Command("stage 0.5\n",VOFA_RESULT_RANGE);
    Command("rate 200\n",VOFA_RESULT_RANGE);
    Command("stream 0\n",VOFA_RESULT_OK);
    unsigned before=frames_sent;
    for (int i=0;i<20;i++) { Step(); GimbalRuntimeComm(); }
    assert(frames_sent==before);
    Command("get yaw angle_kp\n",VOFA_RESULT_OK); assert(frames_sent==before+1);
    Command("stream 1\n",VOFA_RESULT_OK);
    usb_busy=1; before=frames_sent; tick+=20; Step(); GimbalRuntimeComm();
    assert(frames_sent==before);
    usb_busy=0; GimbalRuntimeComm(); assert(Channel(26)>=1);
    Send("set yaw speed_kp 1"); Step(); tick+=300; Send("999\n"); Step(); GimbalRuntimeComm();
    assert(Channel(23)==VOFA_RESULT_SYNTAX && Channel(28)==1500);
    /* Overflow drops the incomplete command and recovers only after a newline. */
    memset(usb_rx,'x',sizeof(usb_rx)); rx_callback(sizeof(usb_rx)); GimbalRuntimeComm();
    Send("\n"); Step(); GimbalRuntimeComm(); assert(Channel(25)>=1);
    Command("get yaw speed_kp\n",VOFA_RESULT_OK); assert(Channel(24)==1500);
    /* Batched commands apply separately. STOP cancels queued stage changes. */
    Send("set yaw angle_kp 5\nset pitch angle_kp 6\n");
    Step(); Step(); GimbalRuntimeComm(); assert(Channel(27)==5 && Channel(30)==6);
    Send("stage 1\nstage 2\nstop\n"); Step(); GimbalRuntimeComm();
    assert(Channel(14)==0 && queue.count==0);
    Command("defaults\n",VOFA_RESULT_OK); assert(Channel(27)==8 && Channel(28)==24000 && Channel(14)==2);
    assert(Channel(29)==15000 && Channel(33)==15000);
    imu_roll=80.0f; Step();
    GimbalGetDebugData(&debug);
    assert(debug.control.faults & GIMBAL_FAULT_IMU);
    assert(debug.imu_seq==tick && debug.raw_roll==80.0f && debug.sensor_gyro[1]==-0.2f);
    assert(debug.output[0]==0 && debug.output[1]==0);
    /* Text mode is reversible, resumes streaming and keeps command replies readable. */
    TextCommand("imu 1\n",VOFA_RESULT_OK);
    assert(strstr(last_packet,"value=1.000") && strstr(last_packet,"stage=2 stream=1"));
    GimbalRuntimeComm();
    assert(strncmp(last_packet,"# USB_IMU v2;",13)==0);
    assert(GimbalRuntimeTextIsStreaming());
    tick+=50; Step(); GimbalRuntimeComm();
    assert(strncmp(last_packet,"IMU,",4)==0 && strstr(last_packet,",80.000,5.730,-11.459,"));
    usb_busy=1; before=frames_sent;
    tick+=50; Step(); GimbalRuntimeComm();
    assert(frames_sent==before);
    usb_busy=0; GimbalRuntimeComm();
    assert(frames_sent==before+1 && strncmp(last_packet,"IMU,",4)==0);
    TextCommand("imu 0.5\n",VOFA_RESULT_RANGE);
    TextCommand("stop\n",VOFA_RESULT_OK);
    TextCommand("stream 0\n",VOFA_RESULT_OK);
    assert(!GimbalRuntimeTextIsStreaming());
    before=frames_sent; tick+=1100; Step(); GimbalRuntimeComm();
    assert(frames_sent==before); /* paused text must not emit repeated headers */
    TextCommand("get yaw angle_kp\n",VOFA_RESULT_OK);
    assert(strstr(last_packet,"value=8.000"));
    Command("imu 0\n",VOFA_RESULT_OK);
    assert(Channel(14)==0 && Channel(27)==8 && Channel(28)==24000);
    before=frames_sent; tick+=20; Step(); GimbalRuntimeComm();
    assert(frames_sent==before+1); /* switching format also resumes the stream */
    /* Exercise the real adapter and runtime while actual control output is active. */
    imu_roll=0; rc.online=1; rc.request.mode=GIMBAL_REQUEST_OFF;
    rc.request.sticks_centered=1; rc.request.axis_mask=1; Step();
    Command("stage 1\n",VOFA_RESULT_OK); Step();
    GimbalGetDebugData(&debug); assert(debug.control.state==GIMBAL_STATE_READY);
    rc.request.mode=GIMBAL_REQUEST_MANUAL; rc.request.rate[0]=0.1f;
    for (unsigned i=0;i<30;i++) Step();
    GimbalGetDebugData(&debug); assert(debug.output[0]>0 && debug.output[1]==0);
    /* Active text diagnostics must expose the slew-limited target, feedback and accumulated PI term. */
    TextCommand("imu 1\n",VOFA_RESULT_OK); GimbalRuntimeComm();
    for (unsigned i=0;i<250;i++) Step();
    GimbalRuntimeComm();
    assert(strstr(last_packet,"target_dps=5.730 actual_dps=0.000"));
    const char *integral_text = strstr(last_packet,"i_term=");
    float integral_value=0;
    assert(integral_text && sscanf(integral_text,"i_term=%f",&integral_value)==1 && integral_value>0);
    Command("imu 0\n",VOFA_RESULT_OK);
    rc.online=0; Step(); GimbalGetDebugData(&debug);
    assert((debug.control.faults & GIMBAL_FAULT_RC) && debug.output[0]==0 && debug.output[1]==0);
    rc.online=1; Step(); GimbalGetDebugData(&debug);
    assert(debug.output[0]==0); /* restored RC alone cannot resume motion */
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    GimbalGetDebugData(&debug); assert(debug.output[0]>0);
    Command("stop\n",VOFA_RESULT_OK);
    for (unsigned i=0;i<30;i++)
    {
        Step(); GimbalGetDebugData(&debug);
        assert(debug.output[0]==0 && debug.output[1]==0);
    }
    /* Pitch recovery starts at ECD393 without a USB stage command or manual lift. */
    rc.request.mode=GIMBAL_REQUEST_OFF; rc.request.sticks_centered=1;
    rc.request.axis_mask=2; rc.request.rate[0]=0; rc.request.rate[1]=0;
    motor[1].measure.ecd=393; Step();
    Command("defaults\n",VOFA_RESULT_OK); Step();
    /* Keep the legacy rate-only recovery regression independently testable. */
    Command("stage 1\n",VOFA_RESULT_OK); Step();
    GimbalGetDebugData(&debug);
    assert(debug.stage==1 && debug.control.state==GIMBAL_STATE_READY);
    assert(debug.output[0]==0 && debug.output[1]==0);
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    GimbalGetDebugData(&debug); assert(debug.output[0]==0 && debug.output[1]==0);
    rc.request.rate[1]=0.2f;
    for (unsigned i=0;i<250;i++) Step();
    GimbalGetDebugData(&debug); assert(debug.output[0]==0 && debug.output[1]==0);
    rc.request.rate[1]=-0.8f;
    /* The logged gyro spikes used to restart the ramp every time braking
     * was blocked at ECD393. The target must still reach the recovery speed. */
    for (unsigned i=0;i<250;i++)
    {
        imu_pitch_rate=(i%40<20 ? -6.0f : 3.0f)*GIMBAL_RAD_PER_DEG;
        Step(); GimbalGetDebugData(&debug);
        assert(debug.output[0]==0 && debug.output[1]>=0 && debug.output[1]<=15000);
        assert(debug.control.faults==0);
    }
    imu_pitch_rate=0;
    Step();
    GimbalGetDebugData(&debug);
    assert(fabsf(debug.control.axis[1].rate_target/GIMBAL_RAD_PER_DEG+30.0f)<0.001f);
    assert(debug.output[0]==0 && debug.output[1]>0 && debug.output[1]<=15000);
    TextCommand("imu 1\n",VOFA_RESULT_OK); GimbalRuntimeComm();
    for (unsigned i=0;i<250;i++) Step();
    GimbalRuntimeComm();
    assert(strstr(last_packet,"# CTRL axis=pitch ") && strstr(last_packet,"recover=1"));
    for (unsigned i=0;i<2000;i++) Step();
    GimbalGetDebugData(&debug);
    assert(debug.output[0]==0 && debug.output[1]>=13000 && debug.output[1]<=15000);
    assert(fabsf(debug.control.axis[1].integral)<=8000);
    Command("imu 0\n",VOFA_RESULT_OK);
    motor[1].rx_count=0; Step(); GimbalGetDebugData(&debug);
    assert((debug.control.faults&GIMBAL_FAULT_MOTOR) && debug.output[0]==0 && debug.output[1]==0);
    motor[1].rx_count=1; Step(); GimbalGetDebugData(&debug); assert(debug.output[1]==0);
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step(); GimbalGetDebugData(&debug);
    assert(debug.output[1]>0);
    rc.request.rate[1]=0;
    /* Gravity starts lowering the load after the stick is centered. */
    imu_pitch_rate=0.05f;
    for (unsigned i=0;i<300;i++) Step();
    GimbalGetDebugData(&debug);
    assert(debug.output[0]==0 && debug.output[1]>0 && debug.control.axis[1].integral<0);
    assert(debug.control.axis[1].rate_target==0);
    imu_pitch_rate=0;
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    GimbalGetDebugData(&debug); assert(debug.output[0]==0 && debug.output[1]==0);
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    /* Inside the soft interval, the ordinary bidirectional PI is available again. */
    motor[1].measure.ecd=889; Step();
    GimbalGetDebugData(&debug); assert(debug.output[1]==0);
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    GimbalGetDebugData(&debug); assert(debug.control.state==GIMBAL_STATE_READY);
    rc.request.mode=GIMBAL_REQUEST_MANUAL; rc.request.rate[1]=-0.1f;
    for (unsigned i=0;i<120;i++) Step();
    GimbalGetDebugData(&debug);
    assert(debug.control.axis[1].rate_target<0 && debug.control.axis[1].output<0);
    assert(debug.output[0]==0 && debug.output[1]>0 && debug.output[1]<=15000);
    int16_t pitch_stationary_output=debug.output[1];
    imu_pitch_rate=-0.05f; Step(); GimbalGetDebugData(&debug);
    assert(debug.output[1]>0 && debug.output[1]<pitch_stationary_output);
    TextCommand("imu 1\n",VOFA_RESULT_OK); GimbalRuntimeComm();
    for (unsigned i=0;i<250;i++) Step();
    GimbalRuntimeComm();
    assert(strstr(last_packet,"# CTRL axis=pitch ") && !strstr(last_packet,"# CTRL axis=yaw "));
    assert(strstr(last_packet,"target_dps=-5.730 actual_dps=-2.865"));
    assert(strstr(last_packet,"speed_max_dps=120.000 accel_dps2=360.000 out_max=15000"));
    imu_pitch_rate=0; rc.request.rate[1]=0.1f;
    for (unsigned i=0;i<250;i++) Step();
    GimbalGetDebugData(&debug);
    assert(debug.output[0]==0 && debug.output[1]<0 && debug.output[1]>=-15000);
    /* Normal travel must reach the new full-stick speed while retaining the
     * actuator ceiling; endpoint recovery remains separately capped at 30. */
    rc.request.rate[1]=-3.0f;
    for (unsigned i=0;i<450;i++)
    {
        Step(); GimbalGetDebugData(&debug);
        assert(debug.output[0]==0 && fabsf((float)debug.output[1])<=15000);
        assert(debug.control.faults==0);
    }
    /* A short Pitch travel can brake-limit before reaching the stick ceiling. */
    float pitch_braking=sqrtf(2.0f*debug.config[1].acceleration_limit*
        (debug.feedback[1].soft_max-debug.feedback[1].joint));
    assert(fabsf(debug.control.axis[1].rate_target+
        fminf(debug.config[1].speed_limit,pitch_braking))<0.0001f);
    rc.online=0; Step(); GimbalGetDebugData(&debug);
    assert((debug.control.faults & GIMBAL_FAULT_RC) && debug.output[0]==0 && debug.output[1]==0);
    rc.online=1; Step(); GimbalGetDebugData(&debug); assert(debug.output[1]==0);
    rc.request.mode=GIMBAL_REQUEST_OFF; rc.request.rate[1]=0; Step();
    rc.request.mode=GIMBAL_REQUEST_MANUAL; rc.request.rate[1]=-0.1f;
    for (unsigned i=0;i<120;i++) Step();
    GimbalGetDebugData(&debug); assert(debug.output[1]>0);
    TextCommand("stop\n",VOFA_RESULT_OK);
    for (unsigned i=0;i<30;i++)
    {
        Step(); GimbalGetDebugData(&debug);
        assert(debug.stage==0 && debug.output[0]==0 && debug.output[1]==0);
    }
    /* Requested everyday workflow: OFF centered -> left down/mask3, right up.
     * The real adapter holds both angles and preserves correct CAN polarity. */
    rc.request.mode=GIMBAL_REQUEST_OFF; rc.request.sticks_centered=1;
    rc.request.axis_mask=3; rc.request.rate[0]=rc.request.rate[1]=0;
    motor[1].measure.ecd=393; imu_pitch_rate=0; Step();
    TextCommand("defaults\n",VOFA_RESULT_OK); Step(); GimbalGetDebugData(&debug);
    assert(debug.stage==2 && debug.control.state==GIMBAL_STATE_READY && debug.control.axis_mask==3);
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    GimbalGetDebugData(&debug); assert(debug.output[0]==0 && debug.output[1]==0);
    rc.request.rate[1]=-1.0f;
    for (unsigned i=0;i<300;i++) Step();
    GimbalGetDebugData(&debug);
    assert(debug.output[1]>0 && debug.output[1]<=15000);
    assert(fabsf(debug.control.axis[1].rate_target/GIMBAL_RAD_PER_DEG+30)<0.001f);
    rc.request.mode=GIMBAL_REQUEST_OFF; rc.request.rate[1]=0;
    motor[1].measure.ecd=889; Step();
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    imu_yaw=5.0f; imu_pitch=-5.0f;
    for (unsigned i=0;i<300;i++) Step();
    GimbalGetDebugData(&debug);
    assert(fabsf(debug.control.axis[0].target)<1e-6f && fabsf(debug.control.axis[1].target)<1e-6f);
    assert(debug.output[0]<0 && debug.output[1]<0); /* restore both INS angles */
    unsigned axes_seen=0;
    for (unsigned i=0;i<5;i++)
    {
        for (unsigned j=0;j<250;j++) Step();
        GimbalRuntimeComm();
        if (strstr(last_packet,"# CTRL axis=yaw ")) axes_seen|=1;
        if (strstr(last_packet,"# CTRL axis=pitch ")) axes_seen|=2;
        if (strstr(last_packet,"# CTRL")) assert(strstr(last_packet,"angle_err_deg=") &&
            strstr(last_packet,"filtered_dps=") && strstr(last_packet,"manual_dps=") &&
            strstr(last_packet,"# RC age_ms="));
    }
    assert(axes_seen==3);
    rc.online=0; Step(); GimbalGetDebugData(&debug);
    assert(debug.control.faults&GIMBAL_FAULT_RC);
    assert(debug.output[0]==0 && debug.output[1]==0);
    rc.online=1; Step(); GimbalGetDebugData(&debug); assert(!debug.control.armed);
    /* Exercise the actual adapter through three turns in each direction:
     * wrapped INS yaw and modulo ECD must produce continuous feedback. */
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    float continuous_deg=imu_yaw;
    float encoder_origin=motor[0].measure.total_angle;
    float previous_yaw=debug.feedback[0].angle;
    for (int direction=1; direction>=-1; direction-=2)
    {
        rc.request.rate[0]=direction*180.0f*GIMBAL_RAD_PER_DEG;
        imu_yaw_rate=rc.request.rate[0]; imu_pitch=0;
        for (unsigned i=0;i<6000;i++)
        {
            continuous_deg+=direction*0.18f;
            imu_yaw=remainderf(continuous_deg,360.0f);
            motor[0].measure.total_angle=encoder_origin+continuous_deg;
            float single=fmodf(motor[0].measure.total_angle,360.0f);
            if (single<0) single+=360.0f;
            motor[0].measure.ecd=(uint16_t)(single*8192.0f/360.0f);
            Step(); GimbalGetDebugData(&debug);
            assert(debug.control.faults==0 && debug.control.state==GIMBAL_STATE_MANUAL);
            assert(debug.feedback[0].continuous_rotation && !debug.feedback[1].continuous_rotation);
            float delta=debug.feedback[0].angle-previous_yaw;
            assert(delta*direction>0 && fabsf(delta)<0.004f);
            previous_yaw=debug.feedback[0].angle;
        }
        assert(fabsf(debug.control.axis[0].manual_rate-rc.request.rate[0])<0.0001f);
    }
    imu_yaw_rate=0; rc.request.rate[0]=0;
    for (unsigned i=0;i<1000;i++) Step();
    GimbalGetDebugData(&debug);
    float held_yaw=debug.control.axis[0].target;
    imu_yaw+=10;
    for (unsigned i=0;i<1000;i++) Step();
    GimbalGetDebugData(&debug);
    assert(debug.control.axis[0].target==held_yaw && debug.output[0]<0);
    /* Pitch hard stop still stops both axes after unrestricted Yaw rotation. */
    motor[1].measure.ecd=1394; Step(); GimbalGetDebugData(&debug);
    assert(debug.control.faults&GIMBAL_FAULT_LIMIT);
    assert(debug.output[0]==0 && debug.output[1]==0);
    motor[1].measure.ecd=889;
    /* Raw overspeed must fault even with a smoothed rate used by the PI. */
    rc.request.mode=GIMBAL_REQUEST_OFF; Step();
    rc.request.mode=GIMBAL_REQUEST_MANUAL; Step();
    imu_pitch_rate=9.0f; Step(); GimbalGetDebugData(&debug);
    assert(debug.control.faults&GIMBAL_FAULT_IMU);
    assert(debug.output[0]==0 && debug.output[1]==0);
    puts("PASS: USB/JustFloat, adapter, dual-axis position hold, alternating diagnostics, tuning, recovery and RC/STOP interlocks");
    return 0;
}
