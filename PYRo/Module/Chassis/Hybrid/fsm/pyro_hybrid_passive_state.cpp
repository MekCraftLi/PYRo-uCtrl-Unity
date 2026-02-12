#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.rud_config.motor.mecanum[0]->disable();
    owner->_ctx.rud_config.motor.mecanum[1]->disable();
    owner->_ctx.rud_config.motor.mecanum[2]->disable();
    owner->_ctx.rud_config.motor.mecanum[3]->disable();
    owner->_ctx.rud_config.motor.leg[0]->disable();
    owner->_ctx.rud_config.motor.leg[1]->disable();
    // owner->_ctx.rud_config.motor.track[0]  ->disable();
    // owner->_ctx.rud_config.motor.track[1]  ->disable();
}

void hybrid_chassis_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.rud_config.motor.mecanum[0]->send_torque(0);
    owner->_ctx.rud_config.motor.mecanum[1]->send_torque(0);
    owner->_ctx.rud_config.motor.mecanum[2]->send_torque(0);
    owner->_ctx.rud_config.motor.mecanum[3]->send_torque(0);
    owner->_ctx.rud_config.motor.leg[0]->send_torque(0);
    owner->_ctx.rud_config.motor.leg[1]->send_torque(0);
    // owner->_ctx.rud_config.motor.track[0]  ->send_torque(0);
    // owner->_ctx.rud_config.motor.track[1]  ->send_torque(0);
}

void hybrid_chassis_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro