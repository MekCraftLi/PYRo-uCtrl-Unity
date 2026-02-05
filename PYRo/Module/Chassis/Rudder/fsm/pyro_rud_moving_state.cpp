//
// Created by pason on 2026/2/3.
//

#include "pyro_rud_chassis.h"

namespace pyro
{
void rud_chassis_t::fsm_active_t::state_moving_t::enter(rud_chassis_t *owner)
{

}

void rud_chassis_t::fsm_active_t::state_moving_t::execute(rud_chassis_t *owner)
{
    if (owner->_cmd->vx == 0 && owner->_cmd->vy == 0)
    {
        owner->_ctx.drive_mode = rud_chassis_t::drive_mode_t::BRAKING;
        // 直接设置制动角度，避免先锁last_angle
        owner->_ctx.data.target_states.modules[0].angle = PI / 4;
        owner->_ctx.data.target_states.modules[1].angle = -PI / 4;
        owner->_ctx.data.target_states.modules[2].angle = PI / 4;
        owner->_ctx.data.target_states.modules[3].angle = -PI / 4;
        // 立即发送制动指令
        _chassis_control(&owner->_ctx);
        _send_motor_command(&owner->_ctx);
        return; // 直接返回，不再执行后续moving逻辑
    }

    if (abs(owner->_ctx.data.current_states.modules[0].speed) < 0.005f &&
        abs(owner->_ctx.data.current_states.modules[1].speed) < 0.005f &&
        abs(owner->_ctx.data.current_states.modules[2].speed) < 0.005f &&
        abs(owner->_ctx.data.current_states.modules[3].speed) < 0.005f)
    {
        owner->_ctx.drive_mode = rud_chassis_t::drive_mode_t::BRAKING;
        owner->_ctx.data.target_states.modules[0].angle = PI / 4;
        owner->_ctx.data.target_states.modules[1].angle = -PI / 4;
        owner->_ctx.data.target_states.modules[2].angle = PI / 4;
        owner->_ctx.data.target_states.modules[3].angle = -PI / 4;
    }


    // for (int i = 0; i < 4; i++)
    // {
    //     if (owner->_ctx.data.target_states.modules[i].angle == 0)
    //         owner->_ctx.data.target_states.modules[i].angle = last_angle[i];
    //     last_angle[i] = owner->_ctx.data.target_states.modules[i].angle;
    // }

    _chassis_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);
}

void rud_chassis_t::fsm_active_t::state_moving_t::exit(rud_chassis_t *owner)
{

}


} // namespace pyro