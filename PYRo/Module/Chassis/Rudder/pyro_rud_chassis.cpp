#include "pyro_rud_chassis.h"

namespace pyro
{
// static float _mps_to_rpm(const float mps, const float radius)
// {
//     // v = w * r  -> w = v / r
//     // RPM = w * 60 / 2pi
//     if (radius < 1e-4f)
//         return 0.0f;
//     return (mps / radius) * 9.5492966f;
// }

static float _radps_to_rpm(const float radps)
{
    // RPM = (w * 60) / (2 * pi)
    return radps * 9.5492966f;
}

rud_chassis_t::rud_chassis_t()
    : chassis_base_t("rudder", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data = {};
}

void rud_chassis_t::_init()
{
    _kinematics = new rudder_kin_t(0.35f, 0.41f);

    _ctx.motor.rudder[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                  can_hub_t::can2); // FL Rudder
    _ctx.motor.rudder[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                  can_hub_t::can2); // BL Rudder
    _ctx.motor.rudder[2] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                  can_hub_t::can1); // BR Rudder
    _ctx.motor.rudder[3] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                  can_hub_t::can1); // FR Rudder

    _ctx.motor.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                  can_hub_t::can2); // FL Wheel
    _ctx.motor.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                  can_hub_t::can2); // BL Wheel
    _ctx.motor.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                  can_hub_t::can1); // BR Wheel
    _ctx.motor.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                  can_hub_t::can1); // FR Wheel

    // _ctx.motor.yaw =
    //     new dm_motor_drv_t();

    _ctx.pid.wheel_pid[0] = 
        new pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    _ctx.pid.wheel_pid[1] = 
        new pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    _ctx.pid.wheel_pid[2] = 
        new pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    _ctx.pid.wheel_pid[3] = 
        new pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        
    _ctx.pid.rud_pos_pid[0] = 
        new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    _ctx.pid.rud_pos_pid[1] = 
        new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    _ctx.pid.rud_pos_pid[2] = 
        new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    _ctx.pid.rud_pos_pid[3] =
        new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);

    _ctx.pid.rud_spd_pid[0] = 
        new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    _ctx.pid.rud_spd_pid[1] = 
        new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    _ctx.pid.rud_spd_pid[2] = 
        new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    _ctx.pid.rud_spd_pid[3] = 
        new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);

    _ctx.hardware.power_meter = new powermeter_drv_t(0x212, can_hub_t::can2);
    _ctx.power.data           = new powermeter_data();
    }

void rud_chassis_t::_update_feedback()
{
    _ctx.motor.rudder[0]->update_feedback();
    _ctx.motor.rudder[1]->update_feedback();
    _ctx.motor.rudder[2]->update_feedback();
    _ctx.motor.rudder[3]->update_feedback();
    _ctx.motor.wheel[0]->update_feedback();
    _ctx.motor.wheel[1]->update_feedback();
    _ctx.motor.wheel[2]->update_feedback();
    _ctx.motor.wheel[3]->update_feedback();

    // 1. 四个舵机的角度和角速度
    _ctx.data.current_states.modules[rudder_kin_t::FL].angle =
        _ctx.motor.rudder[0]->get_current_position();
    _ctx.data.current_states.modules[rudder_kin_t::FR].angle =
        _ctx.motor.rudder[1]->get_current_position();
    _ctx.data.current_states.modules[rudder_kin_t::BL].angle =
        _ctx.motor.rudder[2]->get_current_position();
    _ctx.data.current_states.modules[rudder_kin_t::BR].angle =
        _ctx.motor.rudder[3]->get_current_position();

    _ctx.data.current_rud_radps[0] =
        _ctx.motor.rudder[0]->get_current_rotate();
    _ctx.data.current_rud_radps[1] =
        _ctx.motor.rudder[1]->get_current_rotate();
    _ctx.data.current_rud_radps[2] =
        _ctx.motor.rudder[2]->get_current_rotate();
    _ctx.data.current_rud_radps[3] =
        _ctx.motor.rudder[3]->get_current_rotate();

    // 2. 四个轮子的 RPM
    _ctx.data.current_states.modules[rudder_kin_t::FL].speed =
        _radps_to_rpm(_ctx.motor.wheel[0]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _ctx.data.current_states.modules[rudder_kin_t::FR].speed =
        _radps_to_rpm(_ctx.motor.wheel[1]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _ctx.data.current_states.modules[rudder_kin_t::BL].speed =
        _radps_to_rpm(_ctx.motor.wheel[2]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _ctx.data.current_states.modules[rudder_kin_t::BR].speed =
        _radps_to_rpm(_ctx.motor.wheel[3]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
}

void rud_chassis_t::_kinematics_solve()
{
    static rudder_kin_t::rudder_states_t solved_states{};

    solved_states = _kinematics->solve(_ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz,
                       _ctx.data.current_states);
}

void rud_chassis_t::_chassis_control(rud_ctx_t *ctx)
{
    for (int i = 0; i < 4; i++)
    {
        // 舵机位置环

        float rud_pos_output = ctx->pid.rud_pos_pid[i]->calculate(
            ctx->data.target_states.modules[i].angle,
            ctx->data.current_states.modules[i].angle);

        // 舵机速度环
        ctx->data.out_rud_torque[i] = ctx->pid.rud_spd_pid[i]->calculate(
            rud_pos_output,
            ctx->data.current_rud_radps[i]);

        // 轮子速度环
        ctx->data.out_wheel_torque[i] = ctx->pid.wheel_pid[i]->calculate(
            ctx->data.target_states.modules[i].speed,
            ctx->data.current_states.modules[i].speed);
    }
}

void rud_chassis_t::_send_motor_command(rud_ctx_t *ctx)
{
    // 发送舵机扭矩命令
    ctx->motor.rudder[0]->send_torque(ctx->data.out_rud_torque[0]);
    ctx->motor.rudder[1]->send_torque(ctx->data.out_rud_torque[1]);
    ctx->motor.rudder[2]->send_torque(ctx->data.out_rud_torque[2]);
    ctx->motor.rudder[3]->send_torque(ctx->data.out_rud_torque[3]);

    // 发送轮子扭矩命令
    ctx->motor.wheel[0]->send_torque(ctx->data.out_wheel_torque[0]);
    ctx->motor.wheel[1]->send_torque(ctx->data.out_wheel_torque[1]);
    ctx->motor.wheel[2]->send_torque(ctx->data.out_wheel_torque[2]);
    ctx->motor.wheel[3]->send_torque(ctx->data.out_wheel_torque[3]);
}

void rud_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_cmd[_read_index];

    if(cmd_base_t::mode_t::ZERO_FORCE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if(cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}   

}