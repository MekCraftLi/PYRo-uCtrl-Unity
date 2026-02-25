#ifndef __PYRO_SENTRY_GIMBAL_H__
#define __PYRO_SENTRY_GIMBAL_H__

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_ins.h" // 能不能用得上待定

namespace pyro
{

struct gimbal_cmd_t : cmd_base_t
{
    float target_yaw_imu_angle;
    float target_pitch_imu_angle;

    gimbal_cmd_t() : target_yaw_imu_angle(0), target_pitch_imu_angle(0)
    {
    }


};


}

#endif
