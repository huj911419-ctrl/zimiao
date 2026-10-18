#include "cboard_protocol.h"
#include "auto_aim_coordinates.h"
#include "auto_aim_config.h"
#include "gimbal_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void Protocol(void)
{
    uint8_t f[10];
    float q[4] = {1,0,0,0};
    const uint8_t attitude[] = {0xAA,0x10,0x27,0,0,0,0,0,0,0xDD};
    assert(CBoardPackAttitude(q,f) && !memcmp(f,attitude,10));
    q[0] = -1;
    assert(CBoardPackAttitude(q,f) && f[1]==0xF0 && f[2]==0xD8);
    q[0] = NAN;
    assert(!CBoardPackAttitude(q,f));
    const uint8_t status[] = {0xBB,0xDC,0x05,1,0,0,0,0xF0,0xD8};
    assert(CBoardPackStatus(15,1,-1,f) && !memcmp(f,status,9));
    assert(!CBoardPackStatus(INFINITY,1,0,f));
    assert(!CBoardPackStatus(15,2,0,f));
    CBoard_Parser_s p = {0}; CBoard_Command_s c;
    const uint8_t control[] = {0xCC,1,1,0x10,0x27,0xF0,0xD8,0xEE};
    assert(!CBoardParseByte(&p,0x55,&c));
    for (unsigned i=0;i<7;i++) assert(!CBoardParseByte(&p,control[i],&c));
    assert(CBoardParseByte(&p,control[7],&c));
    assert(c.control==1 && c.shoot==1 && fabsf(c.yaw-1)<1e-6f && fabsf(c.pitch+1)<1e-6f);
    /* A bad prefix ending inside the next good frame must retain that header. */
    const uint8_t prefix[] = {0xCC,2,0,0};
    for (unsigned i=0;i<sizeof(prefix);i++) CBoardParseByte(&p,prefix[i],&c);
    unsigned n=0;
    for (unsigned k=0;k<2;k++) for (unsigned i=0;i<8;i++) n += CBoardParseByte(&p,control[i],&c);
    assert(n==2 && p.rejected>0);
    const uint8_t off[] = {0xCC,0,0,0xFF,0x7F,0x00,0x80,0xEE};
    for (unsigned i=0;i<8;i++) n=CBoardParseByte(&p,off[i],&c);
    assert(n==1 && !c.control); /* control=0 angles are irrelevant */
}

static void Coordinates(void)
{
    const float w[4]={0.7071067812f,0,0,0.7071067812f}, b[4]={0.7071067812f,0,0,-0.7071067812f};
    for (int yi=-3;yi<=3;yi++) for (int pi=-4;pi<=4;pi++)
    {
        float yaw=yi*0.9f,pitch=pi*0.15f,target[2];
        assert(AutoAimAngles(yaw,pitch,w,b,target));
        assert(fabsf(GimbalAngleDelta(target[0],yaw))<2e-6f);
        assert(fabsf(target[1]-pitch)<2e-6f); /* basis converts X pitch to Y pitch */
        float cy=cosf(yaw/2),sy=sinf(yaw/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
        float ins[4]={cy*cp,cy*sp,sy*sp,sy*cp},hero[4];
        assert(AutoAimOrientation(ins,w,b,hero));
        float expected[4]={cy*cp,-sy*sp,cy*sp,sy*cp};
        for (int i=0;i<4;i++) assert(fabsf(hero[i]-expected[i])<2e-6f);
    }
    /* Nonidentity Hero Solver extrinsics must be part of the inverse target transform. */
    float solver[4]={cosf(0.15f),0,sinf(0.15f),0};
    float inverse[4]={solver[0],0,-solver[2],0},identity[4]={1,0,0,0};
    float tw[4],tb[4],wire[4],effective[4],target[2];
    assert(AutoAimOrientation(w,inverse,identity,tw));
    assert(AutoAimOrientation(b,identity,solver,tb));
    assert(AutoAimAngles(0.4f,-0.2f,tw,tb,target));
    /* Generic extrinsics produce repository Roll-Y too. Compare full quaternion inversion,
     * not a two-motor ability to realize an arbitrary 3D mount transformation. */
    float hero[4]={cosf(0.2f)*cosf(-0.1f),-sinf(0.2f)*sinf(-0.1f),
        cosf(0.2f)*sinf(-0.1f),sinf(0.2f)*cosf(-0.1f)};
    float ti[4]={tw[0],-tw[1],-tw[2],-tw[3]},bi[4]={tb[0],-tb[1],-tb[2],-tb[3]},ins[4];
    assert(AutoAimOrientation(hero,ti,bi,ins));
    assert(AutoAimOrientation(ins,w,b,wire));
    assert(AutoAimOrientation(wire,inverse,solver,effective));
    for(int i=0;i<4;i++) assert(fabsf(effective[i]-hero[i])<2e-6f);
    float t[2];
    assert(!AutoAimAngles(NAN,0,w,b,t));
    const float invalid[4]={0,0,0,0};
    assert(!AutoAimAngles(0,0,invalid,b,t));
}

static void ImportedStreamTiming(void)
{
    CBoard_Parser_s parser = {0};
    CBoard_Command_s command;
    const uint8_t frame[] = {0xCC,1,0,0x10,0x27,0,0,0xEE};
    for (unsigned i=0;i<3;i++) assert(!CBoardParseByteAt(&parser,frame[i],100,&command));
    for (unsigned i=3;i<8;i++) assert(!CBoardParseByteAt(&parser,frame[i],120,&command));
    assert(parser.stream.partial_timeouts==1 && parser.accepted==0);
    for (unsigned i=0;i<8;i++) CBoardParseByteAt(&parser,frame[i],121,&command);
    assert(parser.accepted==1 && command.control && fabsf(command.yaw-1)<1e-6f);
    CBoardParserReset(&parser);
    for (unsigned i=0;i<3;i++) CBoardParseByteAt(&parser,frame[i],UINT32_MAX-4U,&command);
    for (unsigned i=3;i<8;i++) CBoardParseByteAt(&parser,frame[i],2U,&command);
    assert(parser.accepted==2 && parser.stream.partial_timeouts==1);
    uint8_t outside[8]={0xCC,1,0,0,0x80,0,0,0xEE};
    for (unsigned i=0;i<8;i++) CBoardParseByteAt(&parser,outside[i],3U,&command);
    assert(parser.accepted==2 && parser.rejected>0); /* Raw i16 syntax is not an angle authorization. */
}

static void HeldNeutralCoordinates(void)
{
    const float w[4]=AUTO_AIM_WORLD_QUATERNION, b[4]=AUTO_AIM_BODY_QUATERNION;
    float pitch=AUTO_AIM_NEUTRAL_INS_PITCH_DEG*GIMBAL_RAD_PER_DEG;
    float roll=AUTO_AIM_NEUTRAL_INS_ROLL_DEG*GIMBAL_RAD_PER_DEG;
    for (int k=-3;k<=3;k++)
    {
        float yaw=k*0.8f;
        float cy=cosf(yaw/2),sy=sinf(yaw/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
        float cr=cosf(roll/2),sr=sinf(roll/2);
        /* Independently construct repository Rz(yaw)*Ry(Roll)*Rx(Pitch). */
        float ins[4]={cy*cr*cp+sy*sr*sp,cy*cr*sp-sy*sr*cp,
            cy*sr*cp+sy*cr*sp,sy*cr*cp-cy*sr*sp};
        float wire[4], target[2];
        assert(AutoAimOrientation(ins,w,b,wire));
        assert(fabsf(wire[0]-cy)<2e-6f && fabsf(wire[3]-sy)<2e-6f);
        assert(fabsf(wire[1])<2e-6f && fabsf(wire[2])<2e-6f);
        /* Reported neutral target must map back to native manual feedback coordinates. */
        assert(AutoAimAngles(yaw,0,w,b,target));
        assert(fabsf(GimbalAngleDelta(target[0],yaw))<2e-6f);
        assert(fabsf(target[1]-pitch)<2e-6f);
    }
}

static void AbsoluteControl(void)
{
    Gimbal_Control_s c; GimbalControlInit(&c);
    Gimbal_Axis_Config_s cfg[2]; Gimbal_Axis_Feedback_s fb[2];
    for(int i=0;i<2;i++)
    {
        cfg[i]=(Gimbal_Axis_Config_s){4,1200,0,1000,5000,0.5f,1,0.2f,1,0,0};
        fb[i]=(Gimbal_Axis_Feedback_s){.soft_min=-1,.soft_max=1,.hard_min=-1.1f,.hard_max=1.1f};
    }
    fb[0].continuous_rotation=1; fb[0].angle=6.2831853f+3.13f;
    Gimbal_Request_s req={.mode=GIMBAL_REQUEST_OFF,.axis_mask=3,.sticks_centered=1};
    GimbalControlStep(&c,cfg,fb,&req,0.001f,0,2);
    req.mode=GIMBAL_REQUEST_AUTO;req.target[0]=-3.13f;req.target[1]=0.8f;
    for(int i=0;i<100;i++) GimbalControlStep(&c,cfg,fb,&req,0.001f,0,2);
    assert(c.state==GIMBAL_STATE_AUTO && c.axis[0].target>fb[0].angle);
    assert(c.axis[0].target-fb[0].angle<0.03f);
    assert(fabsf(c.axis[1].target-0.8f)<1e-6f); /* never clipped by manual .2 rad backlog */
    assert(c.axis[1].manual_rate==0 && c.axis[1].rate_target==0.5f);
    req.target[1]=2;GimbalControlStep(&c,cfg,fb,&req,0.001f,0,2);
    assert(c.axis[1].target<=1); /* Pitch soft travel clamp */
    GimbalControlStep(&c,cfg,fb,&req,0.001f,GIMBAL_FAULT_RC,2);
    assert(c.state==GIMBAL_STATE_FAULT && c.axis[0].output==0);
    GimbalControlStep(&c,cfg,fb,&req,0.001f,0,2);assert(!c.armed);
}
int main(void)
{
    Protocol(); ImportedStreamTiming(); Coordinates(); HeldNeutralCoordinates(); AbsoluteControl();
    puts("PASS: CBoard byte fixtures, resync, coordinate roundtrip, absolute targets, wrap and guards");
}
