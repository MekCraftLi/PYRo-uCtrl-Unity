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

rud_chassis_t::rud_chassis_t(int temp)
    : module_base_t("rudder", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

void rud_chassis_t::_init()
{
    _kinematics = new rudder_kin_t(0.36f, 0.36f);

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

    _ctx.pid.wheel_pid[0]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    _ctx.pid.wheel_pid[1]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    _ctx.pid.wheel_pid[2]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    _ctx.pid.wheel_pid[3]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);

    _ctx.pid.rud_pos_pid[0] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    _ctx.pid.rud_pos_pid[1] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    _ctx.pid.rud_pos_pid[2] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    _ctx.pid.rud_pos_pid[3] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);

    _ctx.pid.rud_spd_pid[0] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    _ctx.pid.rud_spd_pid[1] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    _ctx.pid.rud_spd_pid[2] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    _ctx.pid.rud_spd_pid[3] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);

    _ctx.pid.follow_yaw_pid = new pid_t(3.5f, 0.01f, 0.01f, 0.1f, 5.0f);

    _ctx.config.rudder_pos_moving_offset[0] = 1.01472831f;
    _ctx.config.rudder_pos_moving_offset[1] = -0.29145637f;
    _ctx.config.rudder_pos_moving_offset[2] = -1.87299052f;
    _ctx.config.rudder_pos_moving_offset[3] = -1.04003897f;

    power_control_drv_t &power_controller = power_control_drv_t::get_instance();
    power_control_drv_t::motor_coefficient_t coef1;
    coef1.k1 = 0;
    coef1.k2 = 0;
    coef1.k3 = 0;
    coef1.k4 = 0;
    power_controller.set_motor_coefficient(1, coef1);

    power_control_drv_t::motor_coefficient_t coef2;
    coef2.k1 = 0;
    coef2.k2 = 0;
    coef2.k3 = 0;
    coef2.k4 = 0;
    power_controller.set_motor_coefficient(2, coef2);

    power_control_drv_t::motor_coefficient_t coef3;
    coef3.k1 = 0;
    coef3.k2 = 0;
    coef3.k3 = 0;
    coef3.k4 = 0;
    power_controller.set_motor_coefficient(3, coef3);

    power_control_drv_t::motor_coefficient_t coef4;
    coef4.k1 = 0;
    coef4.k2 = 0;
    coef4.k3 = 0;
    coef4.k4 = 0;
    power_controller.set_motor_coefficient(4, coef4);

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
    // 舵机当前角度（-PI ~ PI）
    _ctx.data.current_states.modules[rudder_kin_t::FL].angle =
        _ctx.motor.rudder[0]->get_current_position() -
        _ctx.config.rudder_pos_moving_offset[0];
    _ctx.data.current_states.modules[rudder_kin_t::FR].angle =
        _ctx.motor.rudder[1]->get_current_position() -
        _ctx.config.rudder_pos_moving_offset[1];
    _ctx.data.current_states.modules[rudder_kin_t::BL].angle =
        _ctx.motor.rudder[2]->get_current_position() -
        _ctx.config.rudder_pos_moving_offset[2];
    _ctx.data.current_states.modules[rudder_kin_t::BR].angle =
        _ctx.motor.rudder[3]->get_current_position() -
        _ctx.config.rudder_pos_moving_offset[3];
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
    _ctx.cmd = &_cmd[_read_index];

    if (cmd_base_t::mode_t::ZERO_FORCE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}

} // namespace pyro