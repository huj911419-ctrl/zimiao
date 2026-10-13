/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "user_lib.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include <string.h>
// #include "master_process.h"

static INS_t INS;
static IMU_Param_t IMU_Param;
static BMI088Instance *bmi088;
static BMI088_Data_t bmi088_data;
static float sensor_gyro[3], sensor_accel[3];

/*
 * Published snapshot for the future C-board -> vision link.
 * snapshot_guard implements a tiny seqlock so the comm task never observes
 * half-updated quaternion/gyro/timestamp fields.
 */
static INS_Vision_Snapshot_t vision_snapshot;
static volatile uint32_t snapshot_guard;
static uint32_t vision_sample_seq;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40; // 恒温设定温度

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

static void PublishVisionSnapshot(void)
{
    static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    static uint8_t last_q_valid = 0;
    float q_out[4];
    float dot = 0.0f;

    memcpy(q_out, INS.q, sizeof(q_out));

    /*
     * q and -q describe the same attitude. Force sign continuity so a future
     * upper-computer SLERP does not see an artificial 360-degree jump.
     */
    if (last_q_valid)
    {
        for (uint8_t i = 0; i < 4; ++i)
            dot += q_out[i] * last_q[i];

        if (dot < 0.0f)
        {
            for (uint8_t i = 0; i < 4; ++i)
                q_out[i] = -q_out[i];
        }
    }

    memcpy(last_q, q_out, sizeof(last_q));
    last_q_valid = 1;

    snapshot_guard++;
    __DMB();

    memcpy(vision_snapshot.q, q_out, sizeof(vision_snapshot.q));
    memcpy(vision_snapshot.gyro, INS.Gyro, sizeof(vision_snapshot.gyro));
    memcpy(vision_snapshot.sensor_gyro, sensor_gyro, sizeof(sensor_gyro));
    memcpy(vision_snapshot.sensor_accel, sensor_accel, sizeof(sensor_accel));
    vision_snapshot.yaw = INS.Yaw;
    vision_snapshot.pitch = INS.Pitch;
    vision_snapshot.roll = INS.Roll;
    vision_snapshot.timestamp_us = (uint32_t)DWT_GetTimeline_us();
    vision_snapshot.timestamp_ms = HAL_GetTick();
    vision_snapshot.sample_seq = ++vision_sample_seq;
    vision_snapshot.valid = 1U;
    vision_snapshot.calibrated = bmi088->calibration_valid;

    __DMB();
    snapshot_guard++;
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088Acquire(bmi088, &bmi088_data);
        
        acc_init[X] += bmi088_data.acc[X];
        acc_init[Y] += bmi088_data.acc[Y];
        acc_init[Z] += bmi088_data.acc[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    float zero_gyro[3] = {0};
    IMU_Param_Correction(&IMU_Param, zero_gyro, acc_init);
    Norm3d(acc_init);

    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float gravity_dot = Dot3d(acc_init, gravity_norm);
    if (gravity_dot > 1.0f)
        gravity_dot = 1.0f;
    else if (gravity_dot < -1.0f)
        gravity_dot = -1.0f;

    float angle = acosf(gravity_dot);
    Cross3d(acc_init, gravity_norm, axis_rot);

    /*
     * 板子接近水平放置时叉乘轴长度可能接近0。原实现直接 Norm3d()
     * 会产生 NaN，进而污染整个 EKF。接近平行时直接使用单位四元数。
     */
    float axis_len = NormOf3d(axis_rot);
    if (axis_len < 1e-6f || angle < 1e-6f)
    {
        /* Anti-parallel gravity needs a half turn, not the identity. */
        init_q4[0] = gravity_dot < 0.0f ? 0.0f : 1.0f;
        init_q4[1] = gravity_dot < 0.0f ? 1.0f : 0.0f;
        init_q4[2] = 0.0f;
        init_q4[3] = 0.0f;
        return;
    }

    axis_rot[0] /= axis_len;
    axis_rot[1] /= axis_len;
    axis_rot[2] /= axis_len;

    init_q4[0] = cosf(angle / 2.0f);
    init_q4[1] = axis_rot[0] * sinf(angle / 2.0f);
    init_q4[2] = axis_rot[1] * sinf(angle / 2.0f);
    init_q4[3] = axis_rot[2] * sinf(angle / 2.0f);
}

void INS_SetMounting(float yaw, float pitch, float roll)
{
    if (INS.init) return;
    IMU_Param.Yaw = yaw;
    IMU_Param.Pitch = pitch;
    IMU_Param.Roll = roll;
    IMU_Param.flag = 1U;
}

attitude_t *INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return (attitude_t *)&INS.Gyro;

    BMI088_Init_Config_s imu_config = {
        .spi_acc_config={
            .GPIOx=GPIOC,
            .cs_pin=GPIO_PIN_0,
            .spi_handle=&hspi2,
        },
        .spi_gyro_config={
            .GPIOx=GPIOC,
            .cs_pin=GPIO_PIN_3,
            .spi_handle=&hspi2,
        },
        .acc_int_config={
            .exti_mode=EXTI_TRIGGER_FALLING,
            .GPIO_Pin=GPIO_PIN_10,
            .GPIOx=GPIOE,
        },
        .gyro_int_config={
            .exti_mode=EXTI_TRIGGER_FALLING,
            .GPIO_Pin=GPIO_PIN_12,
            .GPIOx=GPIOE,
        },
        .heat_pid_config={
            .Kp=0.0f,
            .Kd=0.0f,
            .Ki=0.0f,
            .MaxOut=0.0f,
            .DeadBand=0.0f,
        },
        .heat_pwm_config={
            .channel=TIM_CHANNEL_4,
            .htim=&htim3,
        },
        .cali_mode=BMI088_CALIBRATE_ONLINE_MODE,
        .work_mode=BMI088_BLOCK_PERIODIC_MODE,
    };

    bmi088 = BMI088Register(&imu_config);

    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    // no need for imu heat init because it's done in BMI088Init()

    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    return (attitude_t *)&INS.Gyro; // @todo: 这里偷懒了,不要这样做! 修改INT_t结构体可能会导致异常,待修复.
}

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        if (!BMI088Acquire(bmi088, &bmi088_data) || !isfinite(dt) || dt <= 0.0f || dt > 0.02f)
            return; /* do not publish a fresh timestamp for failed sensor reads */

        INS.Accel[X] = bmi088_data.acc[X];
        INS.Accel[Y] = bmi088_data.acc[Y];
        INS.Accel[Z] = bmi088_data.acc[Z];
        INS.Gyro[X] = bmi088_data.gyro[X] - bmi088->gyro_offset[X];
        INS.Gyro[Y] = bmi088_data.gyro[Y] - bmi088->gyro_offset[Y];
        INS.Gyro[Z] = bmi088_data.gyro[Z] - bmi088->gyro_offset[Z];

        /* Keep sensor axes for mount identification without changing INS inputs. */
        memcpy(sensor_gyro, INS.Gyro, sizeof(sensor_gyro));
        memcpy(sensor_accel, INS.Accel, sizeof(sensor_accel));

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        PublishVisionSnapshot();

        // VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        // IMU_Temperature_Ctrl();
    }

    if ((count++ % 1000) == 0)
    {
        // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
    }
}

uint8_t INS_GetVisionSnapshot(INS_Vision_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
        return 0U;
    /* Bounded read: a preempted writer must not make a higher-priority reader spin. */
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt)
    {
        uint32_t guard_before = snapshot_guard;
        if (guard_before & 1U)
            continue;

        __DMB();
        memcpy(snapshot, &vision_snapshot, sizeof(*snapshot));
        __DMB();

        uint32_t guard_after = snapshot_guard;
        if (guard_before == guard_after && !(guard_after & 1U)) return snapshot->valid;
    }
    return 0U;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
