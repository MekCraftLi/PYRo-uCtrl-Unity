//
// Created by pason on 2026/2/2.
//

#ifndef PYRO_PYRO_YAW_H
#define PYRO_PYRO_YAW_H

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"

namespace pyro
{
struct yaw_cmd_t : cmd_base_t
{
    float wz;

    yaw_cmd_t() : wz(0)
    {
    }
};

}

#endif // PYRO_PYRO_YAW_H
