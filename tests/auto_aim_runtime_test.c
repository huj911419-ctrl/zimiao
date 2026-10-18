/* Same real runtime/adapter fixture as debug regression, now in binary CBoard mode. */
#define main DebugRegressionMain
#include "vofa_runtime_test.c"
#undef main
#include "auto_aim.h"
#include "auto_aim_config.h"
#include "auto_aim_coordinates.h"

static void Target(uint8_t enable, uint8_t shoot, int16_t yaw, int16_t pitch)
{
    uint8_t f[8]={0xCC,enable,shoot,(uint8_t)yaw,(uint8_t)((uint16_t)yaw>>8),
        (uint8_t)pitch,(uint8_t)((uint16_t)pitch>>8),0xEE};
    /* USB fragmentation and concatenation, processed through actual ISR ring. */
    memcpy(usb_rx,f,3);rx_callback(3);GimbalRuntimeComm();
    memcpy(usb_rx,f+3,5);rx_callback(5);GimbalRuntimeComm();
}
static uint8_t Mode(void)
{
    AutoAim_Diagnostics_s d;AutoAimGetDiagnostics(&d);return d.mode;
}
int main(void)
{
    GimbalRuntimeInit();
    rc.online=1;rc.request=(Gimbal_Request_s){.mode=GIMBAL_REQUEST_OFF,.axis_mask=3,.sticks_centered=1};
    for (unsigned i=0;i<1500;i++) Step();
    assert(!GimbalRuntimeTextIsStreaming());
    GimbalRuntimeComm();
    assert(last_length==19 && (uint8_t)last_packet[0]==0xAA && (uint8_t)last_packet[9]==0xDD &&
        (uint8_t)last_packet[10]==0xBB && last_packet[13]==0);
    assert((uint8_t)last_packet[11]==0xDC && (uint8_t)last_packet[12]==5); /* explicit 15 m/s */
    unsigned sent=frames_sent;
    GimbalRuntimeComm();assert(frames_sent==sent); /* same sample cannot acquire a new host timestamp */
    Target(1,1,1000,-1000); /* accepted before authorization must not take over */
    rc.request.mode=GIMBAL_REQUEST_AUTO;Step();
    Gimbal_Debug_Data_s d;GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD);
#if AUTO_AIM_COORDINATES_CONFIRMED
    assert(Mode()==1);
    Target(1,1,1000,-1000);Step();GimbalGetDebugData(&d);
    assert(d.control.state==GIMBAL_STATE_AUTO);
    const float world[4]=AUTO_AIM_WORLD_QUATERNION, body[4]=AUTO_AIM_BODY_QUATERNION;
    float native[2];assert(AutoAimAngles(0.1f,-0.1f,world,body,native));
    assert(fabsf(d.control.axis[0].target-native[0])<1e-5f && fabsf(d.control.axis[1].target-native[1])<1e-5f);
    AutoAim_Diagnostics_s ad;AutoAimGetDiagnostics(&ad);assert(ad.ignored_shoot>=2);
    Target(0,0,0,0);Step();GimbalGetDebugData(&d);
    assert(d.control.state==GIMBAL_STATE_HOLD && fabsf(d.control.axis[1].target-imu_pitch*GIMBAL_RAD_PER_DEG)<1e-5f);
    /* Actual imported USB queue must preserve age across delayed consumption. */
    const uint8_t queued_target[8]={0xCC,1,0,0xE8,3,0x18,0xFC,0xEE};
    memcpy(usb_rx,queued_target,8);rx_callback(8);
    for(unsigned i=0;i<=AUTO_AIM_TIMEOUT_MS;i++) Step();
    GimbalRuntimeComm();Step();GimbalGetDebugData(&d);
    assert(d.control.state==GIMBAL_STATE_HOLD);
    Target(1,0,2000,-500);Step();
    for(unsigned i=0;i<=AUTO_AIM_TIMEOUT_MS;i++) Step();
    GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD);
    /* Invalid bytes cannot refresh the good-command deadline. */
    memset(usb_rx,0x55,32);rx_callback(32);GimbalRuntimeComm();Step();
    GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD);
    Target(1,0,1000,-1000);Step();
    rc.online=0;Step();GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_FAULT && Mode()==0);
    rc.online=1;Target(1,0,1000,-1000);Step();assert(Mode()==0);
    rc.request.mode=GIMBAL_REQUEST_OFF;Step();assert(Mode()==0);
    rc.request.mode=GIMBAL_REQUEST_AUTO;Step();GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD);
    Target(1,0,1000,-1000);Step();assert(Mode()==1);
    usb_ready=0;Step();GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD && Mode()==0);
    usb_ready=1;Step();GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD);
    rc.request.mode=GIMBAL_REQUEST_OFF;Step();
    rc.request.sticks_centered=0;rc.request.mode=GIMBAL_REQUEST_AUTO;Step();
    assert(Mode()==0); /* displaced stick cannot advertise a permitted takeover */
    rc.request.sticks_centered=1;rc.request.mode=GIMBAL_REQUEST_OFF;Step();
    rc.request.mode=GIMBAL_REQUEST_AUTO;Step();
    Target(1,0,1000,-1000);Step();
    /* Broken stream overflow revokes the cached target. */
    memset(usb_rx,0x55,sizeof(usb_rx));
    rx_callback(sizeof(usb_rx));rx_callback(sizeof(usb_rx));GimbalRuntimeComm();Step();
    GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD);
    rc.request.mode=GIMBAL_REQUEST_MANUAL;Step();assert(Mode()==0);
    Target(1,0,1000,-1000);Step();GimbalGetDebugData(&d);
    assert(d.control.state==GIMBAL_STATE_MANUAL);
    rc.request.mode=GIMBAL_REQUEST_OFF;Step();
    rc.request.mode=GIMBAL_REQUEST_AUTO;Step();
    Target(1,0,1000,-1000);Step();
    motor[1].measure.temperature=70;Step();GimbalGetDebugData(&d);
    assert((d.control.faults&GIMBAL_FAULT_TEMPERATURE) && Mode()==0 && motor[0].last_set==0);
    motor[1].measure.temperature=30;
    rc.request.mode=GIMBAL_REQUEST_OFF;Step();
    rc.request.mode=GIMBAL_REQUEST_AUTO;Step();
    Target(1,0,1000,-1000);Step();
    tick+=25;Step();GimbalGetDebugData(&d);
    assert((d.control.faults&GIMBAL_FAULT_TIMING) && Mode()==0);
#else
    assert(Mode()==0);
    Target(1,1,1000,-1000);Step();GimbalGetDebugData(&d);assert(d.control.state==GIMBAL_STATE_HOLD);
#endif
    sent=frames_sent;imu_stale_ms=50;tick+=10;GimbalRuntimeComm();
    assert(frames_sent==sent || ((uint8_t)last_packet[0]==0xBB && last_length==9 && last_packet[3]==0));
    imu_calibrated=0;tick+=20;GimbalRuntimeComm();assert(last_length==9 && (uint8_t)last_packet[0]==0xBB);
    puts("PASS: exclusive binary USB, telemetry freshness, fixed tracking speed, calibration gate, arbitration, no shooting");
}
