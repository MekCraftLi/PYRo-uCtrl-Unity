#include "pyro_rud_chassis.h"

namespace pyro
{
/**********************************************************************/
float cspeed[4]{};
float tspeed[4]{};
float ctorque[4]{};
/**********************************************************************/
// static float _mps_to_rpm(const float mps, const float radius)
// {
//     // v = w * r  -> w = v / r
//     // RPM = w * 60 / 2pi
//     if (radius < 1e-4f)
//         return 0.0f;
//     return (mps / radius) * 9.5492966f;
// }
//
// static float _radps_to_rpm(const float radps)
// {
//     // RPM = (w * 60) / (2 * pi)
//     return radps * 9.5492966f;
// }

rud_chassis_t::rud_chassis_t()
    : module_base_t("rudder", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

void rud_chassis_t::_init()
{
    _kinematics = new rudder_kin_t(_ctx.wheelbase, _ctx.track_width);
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
    // 舵机当前角度（-PI ~ PI）
    _ctx.data.current_states.modules[rudder_kin_t::FL].angle =
        _ctx.motor.rudder[0]->get_current_position() -
        _ctx.rudder_pos_moving_offset[0];
    _ctx.data.current_states.modules[rudder_kin_t::FR].angle =
        _ctx.motor.rudder[1]->get_current_position() -
        _ctx.rudder_pos_moving_offset[1];
    _ctx.data.current_states.modules[rudder_kin_t::BL].angle =
        _ctx.motor.rudder[2]->get_current_position() -
        _ctx.rudder_pos_moving_offset[2];
    _ctx.data.current_states.modules[rudder_kin_t::BR].angle =
        _ctx.motor.rudder[3]->get_current_position() -
        _ctx.rudder_pos_moving_offset[3];
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.data.current_states.modules[i].angle > PI)
            _ctx.data.current_states.modules[i].angle -= 2 * PI;
        else if (_ctx.data.current_states.modules[i].angle < -PI)
            _ctx.data.current_states.modules[i].angle += 2 * PI;
    }
    // 舵机当前角速度
    _ctx.data.current_rud_radps[0] = _ctx.motor.rudder[0]->get_current_rotate();
    _ctx.data.current_rud_radps[1] = _ctx.motor.rudder[1]->get_current_rotate();
    _ctx.data.current_rud_radps[2] = _ctx.motor.rudder[2]->get_current_rotate();
    _ctx.data.current_rud_radps[3] = _ctx.motor.rudder[3]->get_current_rotate();

    // 2. 四个轮子的 RPM
    _ctx.data.current_states.modules[rudder_kin_t::FL].speed =
        _ctx.motor.wheel[0]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;

    _ctx.data.current_states.modules[rudder_kin_t::FR].speed =
        _ctx.motor.wheel[1]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;

    _ctx.data.current_states.modules[rudder_kin_t::BL].speed =
        _ctx.motor.wheel[2]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;

    _ctx.data.current_states.modules[rudder_kin_t::BR].speed =
        _ctx.motor.wheel[3]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;
}

void rud_chassis_t::_kinematics_solve()
{
    if (_ctx.cmd->follow_yaw == true)
    {
        _ctx.cmd->wz =
            _ctx.pid.follow_yaw_pid->calculate(0, _ctx.cmd->yaw_error);
    }
    _ctx.data.target_states = _kinematics->solve(
        _ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz, _ctx.data.current_states);
}

void rud_chassis_t::_chassis_control(rud_ctx_t *ctx)
{
    for (int i = 0; i < 4; i++)
    {
        // 舵机位置环

        const float rud_pos_output = ctx->pid.rud_pos_pid[i]->calculate(
            ctx->data.target_states.modules[i].angle,
            ctx->data.current_states.modules[i].angle);

        // 舵机速度环
        ctx->data.out_rud_torque[i] = ctx->pid.rud_spd_pid[i]->calculate(
            rud_pos_output, ctx->data.current_rud_radps[i]);

        // 轮子速度环
        ctx->data.out_wheel_torque[i] = ctx->pid.wheel_pid[i]->calculate(
            ctx->data.target_states.modules[i].speed,
            ctx->data.current_states.modules[i].speed);
        cspeed[i] = ctx->data.current_states.modules[i].speed;
        tspeed[i] = ctx->data.target_states.modules[i].speed;
    }

#if POWER_CONTROL_USE
    std::vector<power_control_drv_t::motor_data_t> motor_data;

    power_control_drv_t &power_controller = power_control_drv_t::get_instance();
    for (int i = 0; i < POWERCONTROL_NUM; i++)
    {
        motor_data.at(i).gyro       = ctx->data.current_states.modules[i].angle;
        motor_data.at(i).torque_cmd = ctx->data.out_wheel_torque[i];
        motor_data.at(i).power_predict = power_controller.motor_power_predict(
            i, motor_data.at(i).torque_cmd, motor_data.at(i).gyro);
    }
    // 不平均分配
    float custom_ratios[POWERCONTROL_NUM] = {0.1f, 0.1f, 0.1f, 0.1f};
    power_controller.calculate_restricted_torques(
        motor_data.data(), POWERCONTROL_NUM, POWER_LIMIT, custom_ratios);

    // 平均分配
    power_controller.calculate_restricted_torques(
        motor_data.data(), POWERCONTROL_NUM, POWER_LIMIT);
    for (int i = 0; i < POWERCONTROL_NUM; i++)
    {
        ctx->data.out_wheel_torque[i] = motor_data.at(i).restricted_torque;
    }


#endif
}

void rud_chassis_t::_send_motor_command(rud_ctx_t *ctx)
{
    // 发送舵机扭矩命令
    for (int i = 0; i < 4; i++)
    {
        ctx->motor.rudder[i]->send_torque(ctx->data.out_rud_torque[i]);
    }

    // 发送轮子扭矩命令
    for (int i = 0; i < 4; i++)
    {
        ctorque[i] = ctx->data.out_wheel_torque[i];
        ctx->motor.wheel[i]->send_torque(ctx->data.out_wheel_torque[i]);
    }
}

void rud_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}

status_t rud_chassis_t::config(void *cfg_t)
{
    CHECK_POINT_NULL(cfg_t);
    rud_chassis_t::cfg_t *cfg_ptr =
        static_cast<pyro::rud_chassis_t::cfg_t *>(cfg_t);
    _ctx.wheelbase        = cfg_ptr->wheelbase;
    _ctx.track_width      = cfg_ptr->track_width;
    _ctx.wheel_radius     = cfg_ptr->wheel_radius;
    _ctx.gear_ratio       = cfg_ptr->gear_ratio;
    _ctx.powercontrol_num = cfg_ptr->powercontrol_num;
    _ctx.power_limit      = cfg_ptr->power_limit;
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.rudder[i] = new dji_gm_6020_motor_drv_t(
            cfg_ptr->motor_cfg[i].rudder_id, cfg_ptr->motor_cfg[i].rudder_can);
        _ctx.motor.wheel[i] = new dji_m3508_motor_drv_t(
            cfg_ptr->motor_cfg[i].wheel_id, cfg_ptr->motor_cfg[i].wheel_can);
        _ctx.pid.rud_pos_pid[i] = cfg_ptr->pid_cfg.rud_pos_pid[i];
        _ctx.pid.rud_spd_pid[i] = cfg_ptr->pid_cfg.rud_spd_pid[i];
        _ctx.pid.wheel_pid[i]   = cfg_ptr->pid_cfg.wheel_pid[i];
        _ctx.rudder_pos_moving_offset[i] =
            cfg_ptr->rud_offset.rudder_pos_moving_offset[i];
    }
    _ctx.pid.follow_yaw_pid   = cfg_ptr->pid_cfg.follow_yaw_pid;
    _ctx.hardware.power_meter = cfg_ptr->power_meter;

    return PYRO_OK;
}


} // namespace pyro