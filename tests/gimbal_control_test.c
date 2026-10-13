/* Host checks for state transitions, units, boundaries and a simple simulated plant. */
#include "gimbal_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static Gimbal_Control_s control;
static Gimbal_Axis_Config_s cfg[2];
static Gimbal_Axis_Feedback_s fb[2];
static Gimbal_Request_s req;
static void Setup(void)
{
    GimbalControlInit(&control);
    req = (Gimbal_Request_s){.mode=GIMBAL_REQUEST_OFF,.axis_mask=3,.sticks_centered=1};
    for (unsigned i=0;i<2;i++)
    {
        cfg[i]=(Gimbal_Axis_Config_s){4,1200,0,1000,5000,0.5f,1.0f,0.2f,1,0,0};
        fb[i]=(Gimbal_Axis_Feedback_s){.soft_min=-1,.soft_max=1,.hard_min=-1.1f,.hard_max=1.1f};
    }
}
static void Step(void) { GimbalControlStep(&control,cfg,fb,&req,0.001f,0,2); }
static void Enable(void) { Step(); req.mode=GIMBAL_REQUEST_MANUAL; Step(); }

static void TestArming(void)
{
    Setup(); req.mode=GIMBAL_REQUEST_MANUAL; Step();
    assert(control.state==GIMBAL_STATE_OFF && control.axis[0].output==0);
    req.mode=GIMBAL_REQUEST_OFF; req.sticks_centered=0; Step();
    req.mode=GIMBAL_REQUEST_HOLD; Step(); assert(!control.armed);
    req.mode=GIMBAL_REQUEST_OFF; req.sticks_centered=1; Step();
    assert(control.state==GIMBAL_STATE_READY);
    fb[0].angle=0.4f; fb[1].angle=-0.3f;
    req.mode=GIMBAL_REQUEST_HOLD; Step();
    assert(fabsf(control.axis[0].target-0.4f)<1e-6f && control.axis[0].output==0);
    req.mode=GIMBAL_REQUEST_MANUAL; Step();
    req.axis_mask=1; Step(); assert(!control.armed && control.axis[0].output==0);
    req.mode=GIMBAL_REQUEST_OFF; Step(); req.mode=GIMBAL_REQUEST_MANUAL; Step();
    assert(control.state==GIMBAL_STATE_MANUAL && control.axis[1].output==0);
    req.mode=GIMBAL_REQUEST_OFF; Step(); req.mode=GIMBAL_REQUEST_MANUAL; req.sticks_centered=0; Step();
    assert(!control.armed);
}

static void TestFaults(void)
{
    for (uint32_t fault=1;fault<=128;fault<<=1)
    {
        Setup(); Enable(); req.rate[0]=0.5f; Step();
        GimbalControlStep(&control,cfg,fb,&req,0.001f,fault,2);
        assert(control.state==GIMBAL_STATE_FAULT && control.axis[0].output==0);
        Step(); assert(control.state==GIMBAL_STATE_FAULT && !control.armed);
        req.mode=GIMBAL_REQUEST_OFF; Step();
        assert(control.state==GIMBAL_STATE_READY && control.faults==0);
    }
    Setup(); Enable(); fb[1].rate=NAN; Step(); assert(control.faults&GIMBAL_FAULT_VALUE);
    Setup(); Enable(); req.rate[0]=INFINITY; Step(); assert(control.faults&GIMBAL_FAULT_VALUE);
    Setup(); Enable(); GimbalControlStep(&control,cfg,fb,&req,0.1f,0,2);
    assert(control.faults&GIMBAL_FAULT_TIMING);
    Setup(); cfg[0].output_limit=NAN; Step(); assert(control.faults&GIMBAL_FAULT_CONFIG);
}

static void TestLimits(void)
{
    for (int sign=-1; sign<=1; sign+=2)
    {
        Setup(); cfg[1].encoder_to_angle_sign=(float)sign; Enable();
        fb[1].joint=0.99f; req.rate[1]=0.5f*sign;
        for (int i=0;i<2000;i++) Step();
        assert(fabsf(control.axis[1].target)<=0.01001f);
        assert(fabsf(control.axis[1].rate_target)<=sqrtf(0.02f)+1e-5f);
        fb[1].joint=1.0f; fb[1].rate=-0.5f*sign; Step();
        assert(control.axis[1].output*sign<=0.0f);
        req.rate[1]=-0.5f*sign; fb[1].rate=0;
        for (int i=0;i<2000;i++) Step();
        assert(control.axis[1].output*sign<0); /* inward motion remains available */
        fb[1].joint=1.1f; Step(); assert(control.faults&GIMBAL_FAULT_LIMIT);
    }
    Setup(); fb[1].joint=1.05f; Step(); assert(!control.armed);
    Setup(); Enable(); req.rate[0]=100;
    for (int i=0;i<5000;i++) Step();
    assert(fabsf(control.axis[0].target-fb[0].angle)<=cfg[0].max_follow_error+1e-6f);
    assert(fabsf(control.axis[0].output)<=cfg[0].output_limit);
    Setup(); cfg[0].speed_ki=100; cfg[0].output_limit=1; Enable(); req.rate[0]=0.5f;
    for (int i=0;i<5000;i++) Step();
    assert(fabsf(control.axis[0].integral)<0.1f); /* saturation cannot wind up */
}

static void TestContinuousRotation(void)
{
    for (int direction=-1; direction<=1; direction+=2)
    {
        Setup(); fb[0].continuous_rotation=1;
        /* Start multiple turns outside the old range; retain bounded Pitch. */
        fb[0].angle=fb[0].joint=direction*20.0f; Enable();
        assert(control.armed && control.faults==0);
        req.rate[0]=direction*0.5f;
        for (unsigned i=0;i<5000;i++)
        {
            Step();
            fb[0].angle=control.axis[0].target;
            fb[0].joint=fb[0].angle;
            assert(control.state==GIMBAL_STATE_MANUAL && control.faults==0);
        }
        assert(fabsf(control.axis[0].manual_rate-direction*0.5f)<1e-6f);
        assert(fabsf(control.axis[0].rate_target-direction*0.5f)<1e-4f);
        assert(control.axis[0].output*direction>0);
        req.rate[0]=0;
        for (unsigned i=0;i<1000;i++) Step();
        float target=control.axis[0].target;
        fb[0].angle+=direction*0.3f; fb[0].joint=fb[0].angle;
        for (unsigned i=0;i<1000;i++) Step();
        assert(control.axis[0].target==target); /* hold does not follow a push */
        assert(control.axis[0].output*direction<0);
        fb[1].joint=fb[1].hard_max; Step();
        assert(control.faults&GIMBAL_FAULT_LIMIT);
        assert(control.axis[0].output==0 && control.axis[1].output==0);
    }
    Setup(); fb[0].continuous_rotation=1; Enable(); fb[0].joint=NAN; Step();
    assert(control.faults&GIMBAL_FAULT_VALUE);
    Setup(); fb[0].continuous_rotation=2; Step();
    assert(control.faults&GIMBAL_FAULT_CONFIG);
}

static void TestGeometryAndStages(void)
{
    assert(fabsf(GimbalAngleDelta(-179*GIMBAL_RAD_PER_DEG,179*GIMBAL_RAD_PER_DEG)-2*GIMBAL_RAD_PER_DEG)<1e-5f);
    assert(GimbalEncoderDelta(10,8190)==12 && GimbalEncoderDelta(8190,10)==-12);
    float rate[2], gyro[3]={0.2f,0.5f,0.8660254f};
    assert(GimbalEulerRates(30*GIMBAL_RAD_PER_DEG,0,gyro,rate));
    assert(fabsf(rate[0]-1.0f)<1e-5f && fabsf(rate[1]-0.2f)<1e-5f);
    assert(!GimbalEulerRates(0,90*GIMBAL_RAD_PER_DEG,gyro,rate));
    Setup(); Enable(); req.rate[0]=0.5f;
    GimbalControlStep(&control,cfg,fb,&req,0.001f,0,0);
    assert(!control.armed && control.axis[0].output==0);
    Setup(); Enable(); req.rate[0]=0.5f;
    GimbalControlStep(&control,cfg,fb,&req,0.001f,0,1);
    assert(fabsf(control.axis[0].rate_target-0.001f)<1e-6f);
}

static void RecoveryStep(void) { GimbalControlStep(&control,cfg,fb,&req,0.001f,0,1); }

static void TestSoftRecovery(void)
{
    for (int sign=-1; sign<=1; sign+=2)
    for (int edge=-1; edge<=1; edge+=2)
    {
        Setup(); req.axis_mask=2;
        cfg[1].encoder_to_angle_sign=(float)sign;
        cfg[1].soft_recovery_speed=0.1f;
        fb[1].joint=1.05f*edge;
        req.sticks_centered=0; Step(); assert(!control.armed);
        req.mode=GIMBAL_REQUEST_MANUAL; RecoveryStep();
        assert(!control.armed && control.axis[1].output==0);
        req.mode=GIMBAL_REQUEST_OFF; req.sticks_centered=0; RecoveryStep();
        assert(!control.armed);
        req.sticks_centered=1; RecoveryStep(); assert(control.state==GIMBAL_STATE_READY);
        req.mode=GIMBAL_REQUEST_MANUAL; fb[1].rate=0.2f*edge*sign;
        RecoveryStep(); assert(control.axis[1].output==0); /* neutral cannot auto-lift */
        req.rate[1]=0.5f*edge*sign;
        for (unsigned i=0;i<200;i++) RecoveryStep();
        assert(control.axis[1].output==0 && control.axis[1].integral==0); /* outward stick */
        req.rate[1]=-0.5f*edge*sign; fb[1].rate=0;
        for (unsigned i=0;i<200;i++) RecoveryStep();
        assert(control.axis[1].output*sign*edge<0);
        assert(fabsf(control.axis[1].rate_target-(-0.1f*edge*sign))<1e-6f);
        assert(control.axis[0].output==0 && fabsf(control.axis[1].output)<=cfg[1].output_limit);
        req.rate[1]=0; fb[1].rate=0.2f*edge*sign;
        for (unsigned i=0;i<200;i++) RecoveryStep();
        assert(control.axis[1].rate_target==0 && control.axis[1].output*sign*edge<0);
        req.rate[1]=0.5f*edge*sign; RecoveryStep();
        assert(control.axis[1].output==0 && !control.axis[1].rate_control_started);
        req.rate[1]=0; RecoveryStep(); assert(control.axis[1].output==0);
        req.mode=GIMBAL_REQUEST_HOLD; fb[1].rate=0.2f*edge*sign; RecoveryStep();
        assert(control.axis[1].output==0);
        fb[1].joint=1.1f*edge; RecoveryStep();
        assert((control.faults&GIMBAL_FAULT_LIMIT) && !control.armed);
        fb[1].joint=1.05f*edge; RecoveryStep(); assert(!control.armed);
        req.mode=GIMBAL_REQUEST_OFF; RecoveryStep(); assert(control.armed && control.faults==0);
    }
    Setup(); cfg[1].soft_recovery_speed=NAN; RecoveryStep();
    assert(control.faults&GIMBAL_FAULT_CONFIG);
    Setup(); cfg[1].soft_recovery_speed=-1; RecoveryStep();
    assert(control.faults&GIMBAL_FAULT_CONFIG);
    Setup(); cfg[1].soft_recovery_speed=0.1f; fb[0].joint=1.05f; RecoveryStep();
    assert(!control.armed); /* Pitch permission never relaxes Yaw bounds */
    /* Reproduce the logged drop from the safe interval back into the lower margin. */
    Setup(); req.axis_mask=2; cfg[1].soft_recovery_speed=0.1f;
    cfg[1].encoder_to_angle_sign=-1; RecoveryStep();
    req.mode=GIMBAL_REQUEST_MANUAL; RecoveryStep();
    control.axis[1].integral=-400;
    fb[1].joint=-1.05f; fb[1].rate=0.3f; RecoveryStep();
    assert(control.axis[1].rate_target==0 && control.axis[1].integral==-400);
    assert(control.axis[1].output<0); /* inward support is not cleared at the soft boundary */
    req.mode=GIMBAL_REQUEST_OFF; RecoveryStep();
    assert(control.axis[1].output==0 && !control.axis[1].rate_control_started);
}

static void TestRecoveryWithRateFluctuation(void)
{
    for (int sign=-1; sign<=1; sign+=2)
    for (int edge=-1; edge<=1; edge+=2)
    {
        Setup(); req.axis_mask=2;
        cfg[1].encoder_to_angle_sign=(float)sign;
        cfg[1].soft_recovery_speed=0.1f;
        fb[1].joint=1.05f*edge;
        RecoveryStep(); req.mode=GIMBAL_REQUEST_MANUAL; RecoveryStep();
        req.rate[1]=-0.5f*edge*sign;
        control.axis[1].integral=-100.0f*edge*sign;
        /* An inward rate spike asks for outward braking. Clamp that voltage,
         * but keep the inward stick ramp and already accumulated support. */
        fb[1].rate=-0.3f*edge*sign; RecoveryStep();
        assert(control.axis[1].output==0);
        assert(control.axis[1].manual_rate*edge*sign<0);
        assert(control.axis[1].integral==-100.0f*edge*sign);
        for (unsigned i=0;i<300;i++)
        {
            fb[1].rate=(i%40<20 ? -0.3f : 0.2f)*edge*sign;
            RecoveryStep();
            assert(control.axis[1].output*edge*sign<=0);
            assert(control.axis[0].output==0 && control.faults==0);
        }
        assert(fabsf(control.axis[1].rate_target+0.1f*edge*sign)<1e-6f);
        fb[1].rate=0; RecoveryStep();
        assert(control.axis[1].output*edge*sign<0);
        req.rate[1]=0;
        for (unsigned i=0;i<200;i++) RecoveryStep();
        assert(control.axis[1].rate_target==0 && control.axis[1].output*edge*sign<0);
        req.rate[1]=0.5f*edge*sign; RecoveryStep();
        assert(control.axis[1].output==0 && control.axis[1].integral==0);
        req.mode=GIMBAL_REQUEST_OFF; req.rate[1]=0; RecoveryStep();
        assert(control.axis[1].manual_rate==0 && control.axis[1].output==0);
    }
}

static void TestSimulatedTracking(void)
{
    Setup(); Enable(); req.rate[0]=0.15f;
    for (int i=0;i<6000;i++)
    {
        if (i==2000) req.rate[0]=0;
        Step();
        /* Normalized damped bench plant, not a physical GM6020 model. */
        fb[0].rate+=(control.axis[0].output/100.0f-2.0f*fb[0].rate)*0.001f;
        fb[0].angle+=fb[0].rate*0.001f;
        fb[0].joint=fb[0].angle;
        assert(control.state==GIMBAL_STATE_MANUAL);
    }
    assert(fb[0].angle>0.2f);
    assert(fabsf(control.axis[0].target-fb[0].angle)<0.002f);
    assert(fabsf(fb[0].rate)<0.002f);
    req.mode=GIMBAL_REQUEST_HOLD; Step();
    assert(fabsf(control.axis[0].target-fb[0].angle)<1e-6f);
}

static void TestPositionHoldAndFeedforward(void)
{
    Setup(); Enable();
    /* A neutral displacement larger than the motion backlog limit must not
     * overwrite either held target, even after a long sustained disturbance. */
    fb[0].angle=fb[0].joint=0.3f;
    fb[1].angle=fb[1].joint=-0.3f;
    for (unsigned i=0;i<1000;i++) Step();
    assert(control.axis[0].target==0 && control.axis[1].target==0);
    assert(control.axis[0].output<0 && control.axis[1].output>0);
    Setup(); Enable(); req.rate[0]=req.rate[1]=0.15f;
    for (unsigned i=0;i<200;i++)
    {
        Step();
        fb[0].angle=control.axis[0].target; fb[0].joint=fb[0].angle;
        fb[0].rate=0.15f;
    }
    assert(control.axis[0].rate_target>0.149f); /* speed without waiting for angle lag */
    assert(control.axis[1].output>control.axis[0].output); /* extra authority under load */
    req.rate[0]=req.rate[1]=0;
    for (unsigned i=0;i<200;i++) Step();
    float held=control.axis[0].target;
    fb[0].rate=0; fb[0].angle-=0.3f; fb[0].joint=fb[0].angle;
    Step(); assert(control.axis[0].target==held && control.axis[0].output>0);
}

static void TestStageTwoEndpointHold(void)
{
    for (int sign=-1;sign<=1;sign+=2)
    for (int edge=-1;edge<=1;edge+=2)
    {
        Setup(); cfg[1].encoder_to_angle_sign=(float)sign;
        cfg[1].soft_recovery_speed=0.1f; fb[1].joint=1.05f*edge;
        Enable(); assert(control.armed && control.axis[1].output==0);
        req.rate[1]=0.5f*edge*sign; Step(); assert(control.axis[1].output==0);
        req.rate[1]=-0.5f*edge*sign;
        for (unsigned i=0;i<200;i++) Step();
        assert(control.axis[1].rate_target*edge*sign<0);
        assert(fabsf(control.axis[1].rate_target)<=0.10001f);
        control.axis[1].integral=-100.0f*edge*sign; /* existing inward load support */
        req.rate[1]=0;
        for (unsigned i=0;i<200;i++) Step();
        assert(control.axis[1].output*edge*sign<0); /* neutral retains integral support */
        fb[1].rate=-0.3f*edge*sign; Step();
        assert(control.axis[1].output*edge*sign<=0); /* never brake outward past soft bound */
        fb[1].rate=0; fb[1].joint=0.9f*edge; fb[1].angle=-0.15f*edge*sign;
        Step(); assert(control.state==GIMBAL_STATE_MANUAL && control.faults==0);
        req.mode=GIMBAL_REQUEST_OFF; Step(); assert(control.axis[1].output==0);
        fb[1].joint=1.1f*edge; Step(); assert(control.faults&GIMBAL_FAULT_LIMIT);
    }
}

static void TestTrackingWithConstantLoad(void)
{
    Setup();
    for (unsigned axis=0;axis<2;axis++)
    {
        cfg[axis].speed_ki=150;
        cfg[axis].rate_filter_hz=axis==0 ? 50.0f : 30.0f;
    }
    Enable();
    for (unsigned i=0;i<40000;i++)
    {
        req.rate[0]=req.rate[1]=(i>=1000 && i<5000) ? 0.15f : 0.0f;
        Step();
        for (unsigned axis=0;axis<2;axis++)
        {
            /* Normalized plant with persistent load, not calibrated motor torque. */
            float load=axis==0 ? 1.5f : -1.5f;
            fb[axis].rate+=(control.axis[axis].output/100.0f-2.0f*fb[axis].rate-load)*0.001f;
            fb[axis].angle+=fb[axis].rate*0.001f;
            fb[axis].joint=fb[axis].angle;
            assert(control.state==GIMBAL_STATE_MANUAL && control.faults==0);
            if (i==4500) assert(fabsf(fb[axis].rate-0.15f)<0.005f);
        }
    }
    for (unsigned axis=0;axis<2;axis++)
    {
        assert(fabsf(control.axis[axis].target-fb[axis].angle)<0.003f);
        assert(fabsf(fb[axis].rate)<0.003f);
        assert(fabsf(fabsf(control.axis[axis].integral)-150.0f)<3.0f);
    }
}

static void TestReleaseCapturesStopTrajectory(void)
{
    for (int direction=-1;direction<=1;direction+=2)
    {
        Setup(); Enable(); req.rate[1]=0.15f*direction;
        /* Tracking lag may accumulate while moving or being resisted. Releasing
         * the stick means stop here, rather than finish that old trajectory. */
        for (unsigned i=0;i<1000;i++) Step();
        assert(fabsf(control.axis[1].target)>0.1f);
        control.axis[1].integral=300.0f*direction;
        req.rate[1]=0; Step();
        assert(fabsf(control.axis[1].target-fb[1].angle)<1e-6f);
        assert(control.axis[1].manual_rate==0);
        assert(control.axis[1].integral==300.0f*direction); /* retain load support */
        float held=control.axis[1].target;
        fb[1].angle=fb[1].joint=0.3f*direction;
        for (unsigned i=0;i<1000;i++) Step();
        assert(control.axis[1].target==held); /* do not follow a later hand push */
        assert(control.axis[1].output*direction<0);

        Setup(); Enable(); req.rate[1]=0.15f*direction;
        for (unsigned i=0;i<1000;i++) Step();
        fb[1].rate=0.15f*direction; req.rate[1]=0;
        Step();
        assert(fabsf(control.axis[1].manual_rate-0.149f*direction)<1e-6f);
        for (unsigned i=0;i<200;i++) Step();
        float stopping_distance=0.15f*0.15f/(2.0f*cfg[1].acceleration_limit);
        assert(fabsf(control.axis[1].target-stopping_distance*direction)<2e-6f);
        assert(control.axis[1].manual_rate==0);
    }
    /* Returning Pitch to neutral must leave a still-commanded Yaw alone. */
    Setup(); Enable(); req.rate[0]=req.rate[1]=0.15f;
    for (unsigned i=0;i<1000;i++) Step();
    float yaw_target=control.axis[0].target;
    req.rate[1]=0; Step();
    assert(control.axis[0].target>yaw_target && control.axis[0].manual_rate==0.15f);
    assert(control.axis[1].target==fb[1].angle);
}

static void TestTrajectoryBrakesAtSoftLimits(void)
{
    for (int sign=-1;sign<=1;sign+=2)
    for (int edge=-1;edge<=1;edge+=2)
    {
        Setup(); cfg[1].encoder_to_angle_sign=(float)sign;
        fb[1].joint=0.99f*edge; Enable(); req.rate[1]=0.5f*edge*sign;
        for (unsigned i=0;i<1000;i++) Step();
        float braking=sqrtf(2.0f*cfg[1].acceleration_limit*0.01f);
        assert(fabsf(control.axis[1].manual_rate)<=braking+1e-5f);
        assert(fabsf(control.axis[1].rate_target)<=braking+1e-5f);
        /* Inward commands remain available for either encoder polarity. */
        req.rate[1]=-0.5f*edge*sign;
        for (unsigned i=0;i<1000;i++) Step();
        assert(control.axis[1].manual_rate*edge*sign<0);
    }
}

static void TestFilteredVibrationAndReset(void)
{
    double unfiltered_energy=0, filtered_energy=0;
    Setup();
    for (unsigned axis=0;axis<2;axis++)
    {
        cfg[axis].speed_kp=10000; cfg[axis].speed_ki=9000;
        cfg[axis].integral_limit=8000; cfg[axis].output_limit=15000;
    }
    cfg[1].rate_filter_hz=30; Enable();
    for (unsigned i=0;i<2000;i++)
    {
        /* Synthetic 100 Hz, 47 deg/s rate noise on both channels. The actual
         * mount's resonance frequency cannot be recovered from 20 Hz logs. */
        fb[0].rate=fb[1].rate=47.0f*GIMBAL_RAD_PER_DEG*sinf(6.283185307f*100.0f*i*0.001f);
        Step();
        if (i>=200)
        {
            unfiltered_energy+=(double)control.axis[0].output*control.axis[0].output;
            filtered_energy+=(double)control.axis[1].output*control.axis[1].output;
        }
    }
    assert(filtered_energy<0.15*unfiltered_energy);
    /* Real sustained motion must pass the filter rather than disappear. */
    fb[1].rate=0.25f;
    for (unsigned i=0;i<200;i++) Step();
    assert(fabsf(control.axis[1].filtered_rate-0.25f)<1e-5f && control.axis[1].output<0);
    req.mode=GIMBAL_REQUEST_OFF; fb[1].rate=-0.1f; Step();
    assert(control.axis[1].output==0 && control.axis[1].integral==0);
    assert(control.axis[1].filtered_rate==-0.1f);
    req.mode=GIMBAL_REQUEST_MANUAL; Step();
    assert(control.axis[1].output>0); /* no stale positive rate across enable */
    fb[1].rate=NAN; Step();
    assert(control.faults&GIMBAL_FAULT_VALUE);
    assert(control.axis[0].output==0 && control.axis[1].output==0);
    Setup(); cfg[1].rate_filter_hz=NAN; Step(); assert(control.faults&GIMBAL_FAULT_CONFIG);
    Setup(); cfg[1].rate_filter_hz=-1; Step(); assert(control.faults&GIMBAL_FAULT_CONFIG);
}

static float ReplayNeutralPitch(uint8_t filtered)
{
    const float pitch_deg[]={-5.071f,-4.968f,-4.796f,-4.691f,-4.840f,-4.902f,-4.865f,-4.792f};
    const float gx_dps[]={7.133f,22.453f,27.702f,-18.258f,-47.371f,-14.412f,28.251f,23.185f};
    Setup(); req.axis_mask=2;
    cfg[1].angle_kp=filtered ? 4 : 8;
    cfg[1].speed_kp=filtered ? 10000 : 20000;
    cfg[1].speed_ki=filtered ? 9000 : 18000;
    cfg[1].rate_filter_hz=filtered ? 30 : 0;
    cfg[1].integral_limit=8000; cfg[1].output_limit=15000;
    fb[1].angle=pitch_deg[0]*GIMBAL_RAD_PER_DEG; Enable();
    float peak=0;
    for (unsigned sample=0;sample<8;sample++)
    {
        fb[1].angle=pitch_deg[sample]*GIMBAL_RAD_PER_DEG;
        fb[1].rate=gx_dps[sample]*GIMBAL_RAD_PER_DEG;
        /* Zero-order replay of the user's 50 ms samples. This tests command
         * sensitivity only; it is not a reconstruction of physical vibration. */
        for (unsigned i=0;i<50;i++)
        {
            Step(); peak=fmaxf(peak,fabsf(control.axis[1].output));
            assert(control.faults==0 && control.axis[0].output==0);
        }
    }
    return peak;
}

int main(void)
{
    TestArming(); TestFaults(); TestLimits(); TestContinuousRotation(); TestGeometryAndStages(); TestSoftRecovery();
    TestRecoveryWithRateFluctuation(); TestSimulatedTracking();
    TestPositionHoldAndFeedforward(); TestStageTwoEndpointHold();
    TestTrackingWithConstantLoad();
    TestReleaseCapturesStopTrajectory(); TestTrajectoryBrakesAtSoftLimits();
    TestFilteredVibrationAndReset();
    assert(ReplayNeutralPitch(1)<0.7f*ReplayNeutralPitch(0));
    puts("PASS: arming, fault latching/recovery, both limit polarities, wrap, Euler rates, stages, anti-windup, simulated tracking");
    return 0;
}
