#include "gimbal_control.h"
#include <math.h>
#include <string.h>

static float Clamp(float value, float low, float high)
{
    return fminf(fmaxf(value, low), high);
}

float GimbalAngleDelta(float to, float from)
{
    return remainderf(to - from, 6.283185307179586f);
}

int32_t GimbalEncoderDelta(uint16_t to, uint16_t from)
{
    int32_t delta = (int32_t)to - (int32_t)from;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    return delta;
}

uint8_t GimbalEulerRates(float pitch, float roll, const float gyro[3], float rate[2])
{
    if (!isfinite(pitch) || !isfinite(roll) || fabsf(roll) > 60.0f * GIMBAL_RAD_PER_DEG ||
        !isfinite(gyro[0]) || !isfinite(gyro[1]) || !isfinite(gyro[2])) return 0U;
    float projection = sinf(pitch) * gyro[1] + cosf(pitch) * gyro[2];
    rate[0] = projection / cosf(roll);
    rate[1] = gyro[0] + tanf(roll) * projection;
    return 1U;
}

void GimbalControlInit(Gimbal_Control_s *control)
{
    memset(control, 0, sizeof(*control));
}

static void ResetAxes(Gimbal_Control_s *control, const Gimbal_Axis_Feedback_s feedback[2])
{
    memset(control->axis, 0, sizeof(control->axis));
    for (uint8_t i = 0; i < 2U; ++i)
    {
        control->axis[i].target = isfinite(feedback[i].angle) ? feedback[i].angle : 0.0f;
        /* Seed with current feedback: OFF/fault recovery must not retain a
         * previous movement's filter history or assume the joint is still. */
        control->axis[i].filtered_rate = isfinite(feedback[i].rate) ? feedback[i].rate : 0.0f;
        control->axis[i].rate_filter_initialized = 1U;
    }
}

static uint8_t ConfigValid(const Gimbal_Axis_Config_s *c)
{
    return isfinite(c->angle_kp) && c->angle_kp >= 0.0f &&
           isfinite(c->speed_kp) && c->speed_kp >= 0.0f &&
           isfinite(c->speed_ki) && c->speed_ki >= 0.0f &&
           isfinite(c->integral_limit) && c->integral_limit >= 0.0f &&
           isfinite(c->output_limit) && c->output_limit > 0.0f &&
           isfinite(c->speed_limit) && c->speed_limit > 0.0f &&
           isfinite(c->acceleration_limit) && c->acceleration_limit > 0.0f &&
           isfinite(c->max_follow_error) && c->max_follow_error > 0.0f &&
           fabsf(c->encoder_to_angle_sign) == 1.0f &&
           isfinite(c->soft_recovery_speed) && c->soft_recovery_speed >= 0.0f &&
           isfinite(c->rate_filter_hz) && c->rate_filter_hz >= 0.0f;
}

void GimbalControlStep(Gimbal_Control_s *control, const Gimbal_Axis_Config_s config[2],
                       const Gimbal_Axis_Feedback_s feedback[2], const Gimbal_Request_s *request,
                       float dt, uint32_t faults, uint8_t stage)
{
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.02f) faults |= GIMBAL_FAULT_TIMING;
    if (stage > 2U || request->axis_mask == 0U || request->axis_mask > 3U ||
        request->mode < GIMBAL_REQUEST_OFF || request->mode > GIMBAL_REQUEST_AUTO)
        faults |= GIMBAL_FAULT_CONFIG;
    if (request->mode == GIMBAL_REQUEST_AUTO && stage != 2U) faults |= GIMBAL_FAULT_CONFIG;
    for (uint8_t i = 0; i < 2U; ++i)
    {
        const Gimbal_Axis_Feedback_s *f = &feedback[i];
        if (!ConfigValid(&config[i]) || f->continuous_rotation > 1U) faults |= GIMBAL_FAULT_CONFIG;
        if (!isfinite(f->angle) || !isfinite(f->rate) || !isfinite(f->joint) ||
            !isfinite(f->soft_min) || !isfinite(f->soft_max) ||
            !isfinite(f->hard_min) || !isfinite(f->hard_max) || !isfinite(request->rate[i]) ||
            (request->mode == GIMBAL_REQUEST_AUTO && !isfinite(request->target[i])))
            faults |= GIMBAL_FAULT_VALUE;
        if ((request->axis_mask & (1U << i)) && !f->continuous_rotation)
        {
            if (!(f->hard_min < f->soft_min && f->soft_min < f->soft_max && f->soft_max < f->hard_max))
                faults |= GIMBAL_FAULT_CONFIG;
            if (f->joint <= f->hard_min || f->joint >= f->hard_max) faults |= GIMBAL_FAULT_LIMIT;
        }
    }
    if (faults)
    {
        control->faults |= faults;
        control->state = GIMBAL_STATE_FAULT;
        control->armed = 0U;
        ResetAxes(control, feedback);
        return;
    }
    if (request->mode == GIMBAL_REQUEST_OFF || stage == 0U)
    {
        ResetAxes(control, feedback);
        control->armed = request->sticks_centered && stage != 0U;
        for (uint8_t i = 0; i < 2U; ++i)
            if ((request->axis_mask & (1U << i)) &&
                !feedback[i].continuous_rotation &&
                (feedback[i].joint < feedback[i].soft_min || feedback[i].joint > feedback[i].soft_max) &&
                !(config[i].soft_recovery_speed > 0.0f))
                control->armed = 0U;
        control->axis_mask = request->axis_mask;
        control->last_mode = GIMBAL_REQUEST_OFF;
        if (control->armed || stage == 0U) control->faults = 0U;
        control->state = control->armed ? GIMBAL_STATE_READY : GIMBAL_STATE_OFF;
        return;
    }
    /* No automatic restart after fault, axis change, or enable with a displaced stick. */
    if (!control->armed || request->axis_mask != control->axis_mask ||
        (control->last_mode == GIMBAL_REQUEST_OFF && !request->sticks_centered))
    {
        control->armed = 0U;
        control->state = control->faults ? GIMBAL_STATE_FAULT : GIMBAL_STATE_OFF;
        ResetAxes(control, feedback);
        return;
    }
    if (request->mode != control->last_mode) ResetAxes(control, feedback);
    control->last_mode = request->mode;
    control->state = request->mode == GIMBAL_REQUEST_AUTO ? GIMBAL_STATE_AUTO :
        (request->mode == GIMBAL_REQUEST_HOLD ? GIMBAL_STATE_HOLD : GIMBAL_STATE_MANUAL);
    for (uint8_t i = 0; i < 2U; ++i)
    {
        Gimbal_Axis_State_s *s = &control->axis[i];
        const Gimbal_Axis_Config_s *c = &config[i];
        const Gimbal_Axis_Feedback_s *f = &feedback[i];
        if (!(control->axis_mask & (1U << i)))
        {
            memset(s, 0, sizeof(*s));
            s->target = f->angle;
            continue;
        }
        /* Filter only the PI feedback, after raw-value/freshness/rate fault
         * checks in the adapter. Also seed a manual stop from this rate. */
        if (!s->rate_filter_initialized || c->rate_filter_hz == 0.0f)
            s->filtered_rate = f->rate;
        else
        {
            float tau = 1.0f / (6.283185307179586f * c->rate_filter_hz);
            s->filtered_rate += dt / (tau + dt) * (f->rate - s->filtered_rate);
        }
        s->rate_filter_initialized = 1U;
        float wanted = request->mode == GIMBAL_REQUEST_MANUAL ? Clamp(request->rate[i], -c->speed_limit, c->speed_limit) : 0.0f;
        if (stage == 2U && request->mode == GIMBAL_REQUEST_MANUAL &&
            wanted == 0.0f && s->last_manual_request != 0.0f)
        {
            /* Neutral means stop the measured motion here. Discard accumulated
             * moving-target lag once, then generate a bounded deceleration.
             * Preserve integral load support and never recapture while held. */
            s->target = f->angle;
            s->manual_rate = Clamp(s->filtered_rate, -c->speed_limit, c->speed_limit);
        }
        s->last_manual_request = wanted;
        uint8_t recovering = !f->continuous_rotation && c->soft_recovery_speed > 0.0f &&
            (f->joint < f->soft_min || f->joint > f->soft_max);
        if (recovering)
        {
            float inward = f->joint < f->soft_min ? 1.0f : -1.0f;
            /* At initial rest, require an inward stick. After control has started,
             * neutral retains inward PI support instead of dropping the load. */
            uint8_t keep_neutral = request->mode == GIMBAL_REQUEST_MANUAL &&
                wanted == 0.0f && s->rate_control_started;
            if (stage == 1U && wanted * c->encoder_to_angle_sign * inward <= 0.0f && !keep_neutral)
            {
                memset(s, 0, sizeof(*s));
                s->target = f->angle;
                continue;
            }
            float recovery_limit = fminf(c->speed_limit, c->soft_recovery_speed);
            wanted = Clamp(wanted, -recovery_limit, recovery_limit);
            /* In stage 2, neutral holds the captured rest pose. An outward
             * stick is ignored without discarding the load-support integral. */
            if (stage == 2U && wanted * c->encoder_to_angle_sign * inward < 0.0f)
                wanted = 0.0f;
            /* Drop a previous outward slew immediately when entering the margin. */
            s->manual_rate = c->encoder_to_angle_sign * inward *
                Clamp(s->manual_rate * c->encoder_to_angle_sign * inward, 0.0f, recovery_limit);
        }
        if (c->soft_recovery_speed > 0.0f && (!recovering || wanted != 0.0f || stage == 2U))
            s->rate_control_started = 1U;
        if (request->mode == GIMBAL_REQUEST_AUTO)
        {
            /* Map wrapped yaw to nearest continuous feedback; absolute pitch is direct.
             * No joystick integration or moving-goal follow-error clipping in AUTO. */
            s->target = i == 0U ? f->angle + GimbalAngleDelta(request->target[i], f->angle) : request->target[i];
            s->manual_rate = 0.0f;
        }
        float previous_rate = s->manual_rate;
        s->manual_rate += Clamp(wanted - s->manual_rate, -c->acceleration_limit * dt, c->acceleration_limit * dt);
        /* Apply encoder braking to the trajectory itself, not only the final
         * PI reference. Otherwise the position goal keeps advancing at a bound. */
        if (!f->continuous_rotation)
        {
            float manual_joint_rate = s->manual_rate * c->encoder_to_angle_sign;
            float manual_room = manual_joint_rate >= 0.0f ? f->soft_max - f->joint : f->joint - f->soft_min;
            float manual_braking = sqrtf(2.0f * c->acceleration_limit * fmaxf(manual_room, 0.0f));
            s->manual_rate = Clamp(s->manual_rate, -manual_braking, manual_braking);
        }
        s->target += (stage == 2U ? 0.5f * (previous_rate + s->manual_rate) : s->manual_rate) * dt;
        /* Bound trajectory backlog during motion, but do not move a neutral
         * hold target to follow an external disturbance. */
        if (stage == 1U || s->manual_rate != 0.0f)
            s->target = Clamp(s->target, f->angle - c->max_follow_error, f->angle + c->max_follow_error);
        /* Moving attitude bounds preserve encoder room when the base moves. */
        if (!f->continuous_rotation)
        {
            float a = f->angle + (f->soft_min - f->joint) * c->encoder_to_angle_sign;
            float b = f->angle + (f->soft_max - f->joint) * c->encoder_to_angle_sign;
            if (stage == 2U && recovering)
            {
                /* Permit hold at the powered-off rest position without auto-lift.
                 * The final voltage guard still forbids outward motion here. */
                if (f->joint < f->soft_min) a = f->angle;
                else b = f->angle;
            }
            s->target = Clamp(s->target, fminf(a, b), fmaxf(a, b));
        }
        float speed = stage == 1U ? s->manual_rate :
            s->manual_rate + c->angle_kp * (s->target - f->angle);
        speed = Clamp(speed, -c->speed_limit, c->speed_limit);
        if (recovering) speed = Clamp(speed, -c->soft_recovery_speed, c->soft_recovery_speed);
        s->rate_target = speed;
        if (!f->continuous_rotation)
        {
            float joint_speed = speed * c->encoder_to_angle_sign;
            float room = joint_speed >= 0.0f ? f->soft_max - f->joint : f->joint - f->soft_min;
            float braking = sqrtf(2.0f * c->acceleration_limit * fmaxf(room, 0.0f));
            s->rate_target = Clamp(speed, -braking, braking);
        }
        float error = s->rate_target - s->filtered_rate;
        float candidate = Clamp(s->integral + c->speed_ki * error * dt, -c->integral_limit, c->integral_limit);
        float raw = c->speed_kp * error + candidate;
        if (fabsf(raw) <= c->output_limit || raw * error < 0.0f) s->integral = candidate;
        s->output = Clamp(c->speed_kp * error + s->integral, -c->output_limit, c->output_limit);
        float joint_output = s->output * c->encoder_to_angle_sign;
        if (!f->continuous_rotation &&
            ((f->joint <= f->soft_min && joint_output < 0.0f) || (f->joint >= f->soft_max && joint_output > 0.0f)))
        {
            s->output = 0.0f;
            if (recovering)
            {
                /* Blocking outward braking must not restart the inward stick
                 * ramp on each gyro fluctuation. Keep only inward integral. */
                float inward = f->joint < f->soft_min ? 1.0f : -1.0f;
                if (s->integral * c->encoder_to_angle_sign * inward < 0.0f)
                    s->integral = 0.0f;
            }
            else
            {
                s->integral = 0.0f;
                s->manual_rate = 0.0f;
            }
        }
    }
}
