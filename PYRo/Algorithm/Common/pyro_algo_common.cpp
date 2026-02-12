#include "pyro_algo_common.h"

namespace pyro
{
float wrap2pi_f32(float input)
{
    return fmodf(input, 2 * PI);
}

float radps_to_rpm(const float radps)
{
    return radps * 9.5492966f;
}

float calculate_angle_diff(float current, float target)
{
    const float diff = fabs(current - target);
    return fmin(diff, 2 * PI - diff);
}

} // namespace pyro