//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

float cangle = 0.0f;
float tangle = 0.0f;
float imu_cangle = 0.0f;

namespace pyro
{
    
float yaw{}, pitch{}, roll{};
float wrap_pi(float angle)
{
    while (angle > PI)
        angle -= 2 * PI;
    while (angle < -PI)
        angle += 2 * PI;
    return angle;
}

yaw_t::yaw_t() : module_base_t("yaw", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

float yaw_t::get_yaw_error() const
{
    float target_yaw_imu_angle = wrap_pi(_ctx.cmd->target_yaw_imu_angle);
    if (abs(target_yaw_imu_angle - _ctx.data.current_yaw_imu_angle < 0.01f))
        return 0;
    else
        return target_yaw_imu_angle - _ctx.data.current_yaw_imu_angle;
}

void yaw_t::_init()
{
    _ctx.motor.yaw = new dm_motor_drv_t(0x01, 0x02, pyro::can_hub_t::can2);
    _ctx.motor.yaw->set_position_range(-PI, PI);
    _ctx.motor.yaw->set_rotate_range(-20, 20);
    _ctx.motor.yaw->set_torque_range(-10, 10);

    _ctx.pid.yaw_pos_pid =
        new pid_t(20.0f, 0.2f, 0.02f, 0.5f, 10.0f, 15, 150, 4);
    _ctx.pid.yaw_spd_pid =
        new pid_t(0.3f, 0.003f, 0.0003f, 0.1f, 3.0f, 15, 150, 4);

    _ctx.config.yaw_offset = 0;
}

void yaw_t::_update_feedback()
{
    ins_drv_t *ins = ins_drv_t::get_instance();
    _ctx.motor.yaw->update_feedback();

    // yaw轴当前角度（电机角度， -PI ~ PI）
    _ctx.data.current_yaw_angle = wrap_pi(
        _ctx.motor.yaw->get_current_position() - _ctx.config.yaw_offset);

    // 这里需要获取底盘imu数据减去大yaw的机械角度得到yaw轴的imu角度
    ins->get_angles_n(&yaw, &pitch, &roll);
    _ctx.data.current_yaw_imu_angle = wrap_pi(yaw - _ctx.data.current_yaw_angle);

    // yaw电机当前角速度
    _ctx.data.current_yaw_radps     = _ctx.motor.yaw->get_current_rotate();
}

void yaw_t::_yaw_control(yaw_ctx_t *ctx)
{
    ctx->data.target_yaw_imu_angle = wrap_pi(ctx->cmd->target_yaw_imu_angle);

    cangle                         = ctx->data.current_yaw_angle;
    tangle                         = ctx->data.target_yaw_imu_angle;
    imu_cangle                     = ctx->data.current_yaw_imu_angle;

    float yaw_pos_output           = -ctx->pid.yaw_pos_pid->calculate(
        ctx->data.target_yaw_imu_angle, ctx->data.current_yaw_imu_angle);

    ctx->data.out_yaw_torque = ctx->pid.yaw_spd_pid->calculate(
        yaw_pos_output, ctx->data.current_yaw_radps);
}

void yaw_t::_send_motor_command(yaw_ctx_t *ctx)
{
    ctx->motor.yaw->send_torque(ctx->data.out_yaw_torque);
}

void yaw_t::_fsm_execute()
{
    _ctx.cmd = &_cmd[_read_index];

    if (cmd_base_t::mode_t::ZERO_FORCE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}

} // namespace pyro
