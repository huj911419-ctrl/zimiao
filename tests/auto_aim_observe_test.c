/* Real runtime and adapter: motor-disabled binary firmware must remain passive. */
#define main DebugRegressionMain
#include "vofa_runtime_test.c"
#undef main
#include "auto_aim.h"
int main(void)
{
    GimbalRuntimeInit();
    rc.online=1;
    rc.request=(Gimbal_Request_s){.mode=GIMBAL_REQUEST_OFF,.axis_mask=3,.sticks_centered=1};
    for(unsigned i=0;i<1500;i++) Step();
    for(unsigned mode=GIMBAL_REQUEST_HOLD;mode<=GIMBAL_REQUEST_AUTO;mode++)
    {
        rc.request.mode=(Gimbal_Request_e)mode;
        rc.request.rate[0]=1;rc.request.rate[1]=1;
        const uint8_t target[8]={0xCC,1,1,0x10,0x27,0x10,0x27,0xEE};
        memcpy(usb_rx,target,8);rx_callback(8);GimbalRuntimeComm();
        for(unsigned i=0;i<100;i++) Step();
        Gimbal_Debug_Data_s d;GimbalGetDebugData(&d);
        assert(d.stage==0 && d.output[0]==0 && d.output[1]==0);
        assert(motor[0].stop_flag==MOTOR_STOP && motor[1].stop_flag==MOTOR_STOP);
        AutoAim_Diagnostics_s ad;AutoAimGetDiagnostics(&ad);assert(ad.mode==0 && !ad.target_active);
    }
    puts("PASS: binary observation build cannot drive either motor from manual or visual requests");
}
