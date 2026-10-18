#include "auto_aim_coordinates.h"
#include <math.h>

static uint8_t Normalize(const float q[4], float result[4])
{
    float n = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (!isfinite(q[i])) return 0U;
        n += q[i] * q[i];
    }
    if (fabsf(n - 1.0f) > 0.1f) return 0U;
    for (uint8_t i = 0U; i < 4U; ++i) result[i] = q[i] / sqrtf(n);
    return 1U;
}

static void Multiply(const float a[4], const float b[4], float c[4])
{
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

uint8_t AutoAimOrientation(const float ins[4], const float world[4],
                           const float body[4], float hero[4])
{
    float i[4], w[4], b[4], temp[4];
    if (!Normalize(ins, i) || !Normalize(world, w) || !Normalize(body, b)) return 0U;
    Multiply(w, i, temp);
    Multiply(temp, b, hero);
    return 1U;
}

uint8_t AutoAimAngles(float yaw, float pitch, const float world[4],
                     const float body[4], float target[2])
{
    if (!isfinite(yaw) || !isfinite(pitch) || fabsf(pitch) >= 1.5708f) return 0U;
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float hero[4] = {cy*cp, -sy*sp, cy*sp, sy*cp};
    float w[4], b[4], ins[4];
    if (!Normalize(world, w) || !Normalize(body, b)) return 0U;
    for (uint8_t i = 1U; i < 4U; ++i) { w[i] = -w[i]; b[i] = -b[i]; }
    if (!AutoAimOrientation(hero, w, b, ins)) return 0U;
    float roll_sin = 2.0f * (ins[0]*ins[2] - ins[1]*ins[3]);
    if (fabsf(roll_sin) > 0.8660254f) return 0U; /* same repository +/-60 deg guard */
    target[0] = atan2f(2.0f*(ins[0]*ins[3] + ins[1]*ins[2]),
                      2.0f*(ins[0]*ins[0] + ins[1]*ins[1]) - 1.0f);
    target[1] = atan2f(2.0f*(ins[0]*ins[1] + ins[2]*ins[3]),
                      2.0f*(ins[0]*ins[0] + ins[3]*ins[3]) - 1.0f);
    return 1U;
}
