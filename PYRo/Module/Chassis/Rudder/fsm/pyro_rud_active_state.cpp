#include "pyro_rud_chassis.h"

namespace pyro
{
void rud_chassis_t::fsm_active_t::on_enter(owner *owner)
{
    // 使能所有电机（有力状态），保留原有使能逻辑
    owner->_ctx.motor.rudder[0]->enable();
    owner->_ctx.motor.rudder[1]->enable();
    owner->_ctx.motor.rudder[2]->enable();
    owner->_ctx.motor.rudder[3]->enable();
    owner->_ctx.motor.wheel[0]->enable();
    owner->_ctx.motor.wheel[1]->enable();
    owner->_ctx.motor.wheel[2]->enable();
    owner->_ctx.motor.wheel[3]->enable();
}

void rud_chassis_t::fsm_active_t::on_execute(owner *owner)
{
    if (rud_chassis_t::drive_mode_t::MOVING == owner->_ctx.drive_mode)
    {
        this->change_state(&_moving_state);
    }
    else if (rud_chassis_t::drive_mode_t::BRAKING == owner->_ctx.drive_mode)
    {
        this->change_state(&_braking_state);
    }
    else if(rud_chassis_t::drive_mode_t::TURNING == owner->_ctx.drive_mode)
    {
        this->change_state(&_turning_state);
    }

    owner->_kinematics_solve();
}

void rud_chassis_t::fsm_active_t::on_exit(owner *owner)
{
}

}