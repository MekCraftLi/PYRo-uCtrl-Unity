#include "pyro_hybrid_chassis.h"
#include "pyro_algo_common.h"
#include <arm_math.h> // 引入 CMSIS-DSP 库

namespace pyro
{

// =========================================================
// 构造与初始化
// =========================================================

hybrid_chassis_t::hybrid_chassis_t() : module_base_t("hybrid")
{
    _ctx = {};
}

void hybrid_chassis_t::_init()
{
    _ctx.motor  = _module_deps.motor_deps;
    _ctx.pid    = _module_deps.pid_deps;

    // 使用 config.h 中的参数初始化运动学模型
    _kinematics = new hybrid_kin_t(0.648f, MEC_WHEELBASE, MEC_TRACK_WIDTH);
}

void hybrid_chassis_t::_update_feedback()
{
    // 1. 更新所有电机反馈
    for (const auto &i : _ctx.motor.mecanum)
        i->update_feedback();
    for (const auto &i : _ctx.motor.track)
        i->update_feedback();
    for (const auto &i : _ctx.motor.leg)
        i->update_feedback();
    _ctx.motor.yaw->update_feedback();

    // 2. 读取 IMU 数据作为底盘姿态反馈
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.current_yaw_rad,
                                          &_ctx.data.current_pitch_rad,
                                          &_ctx.data.current_roll_rad);

    // 3. 转换并记录转速与位置
    for (int i = 0; i < 4; i++)
        _ctx.data.current_wheel_rpm[i] =
            radps_to_rpm(_ctx.motor.mecanum[i]->get_current_rotate());

    for (int i = 0; i < 2; i++)
        _ctx.data.current_track_rpm[i] =
            radps_to_rpm(_ctx.motor.track[i]->get_current_rotate());

    // 左右腿对称性修正：对右腿(leg[1])的读取数据取反，抹平机械差异
    _ctx.data.current_leg_rad[0]   = _ctx.motor.leg[0]->get_current_position();
    _ctx.data.current_leg_radps[0] = _ctx.motor.leg[0]->get_current_rotate();
    _ctx.data.current_leg_rad[1]   = -_ctx.motor.leg[1]->get_current_position();
    _ctx.data.current_leg_radps[1] = -_ctx.motor.leg[1]->get_current_rotate();
}

// =========================================================
// 核心解算与控制逻辑
// =========================================================

void hybrid_chassis_t::_kinematics_solve()
{
    const auto wheel_speeds = _kinematics->solve(
        _ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz, _ctx.cmd->track_en);

    // 麦轮转速分配 (右侧反转视底层驱动而定，此处按常规处理)
    _ctx.data.target_wheel_rpm[0] =
        mps_to_rpm(wheel_speeds.mec_fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] =
        -mps_to_rpm(wheel_speeds.mec_fr, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] =
        mps_to_rpm(wheel_speeds.mec_bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] =
        -mps_to_rpm(wheel_speeds.mec_br, WHEEL_RADIUS);

    // 履带分配 (差速模型)
    if (_ctx.cmd->track_en)
    {
        _ctx.data.target_track_rpm[0] = mps_to_rpm(
            _ctx.cmd->vx - _ctx.cmd->wz * TRACK_SPACING / 2.0f, TRACK_RADIUS);
        _ctx.data.target_track_rpm[1] = -mps_to_rpm(
            _ctx.cmd->vx + _ctx.cmd->wz * TRACK_SPACING / 2.0f, TRACK_RADIUS);
    }
    else
    {
        _ctx.data.target_track_rpm[0] = 0.0f;
        _ctx.data.target_track_rpm[1] = 0.0f;
    }
}

void hybrid_chassis_t::_leg_control()
{
    const float pitch     = _ctx.data.current_pitch_rad;
    const float roll      = _ctx.data.current_roll_rad;

    // 预计算 DSP 三角函数
    const float cos_pitch = arm_cos_f32(pitch);
    const float sin_pitch = arm_sin_f32(pitch);

    // 1. 计算姿态维稳所需的宏观虚拟力
    const float f_pitch   = _ctx.pid.pitch_pid->calculate(0.0f, pitch);
    const float f_roll    = _ctx.pid.roll_pid->calculate(0.0f, roll);

    for (int i = 0; i < 2; i++)
    {
        const float theta     = _ctx.data.current_leg_rad[i];
        const float theta_dot = _ctx.data.current_leg_radps[i];

        // 2. 多项式求解雅可比及端点坐标
        const float j_x =
            evaluate_polynomial(theta, JX_POLY_COEF, JX_POLY_DEGREE);
        const float j_y =
            evaluate_polynomial(theta, JY_POLY_COEF, JY_POLY_DEGREE);
        const float x_b =
            evaluate_polynomial(theta, XB_POLY_COEF, XB_POLY_DEGREE);
        const float y_b =
            evaluate_polynomial(theta, YB_POLY_COEF, YB_POLY_DEGREE);

        // 3. 计算重力前馈补偿 (使用预计算的 DSP 三角函数)
        const float y_wheel   = y_b - H_HIP_OFFSET;
        const float numerator = DIST_FRONT * cos_pitch - H_COG * sin_pitch;
        const float denominator =
            (DIST_FRONT + DIST_HIP + x_b) * cos_pitch + y_wheel * sin_pitch;

        float f_gravity_ff = 0.0f;
        if (fabsf(denominator) > 1e-4f) // 使用硬件 FPU 支持的 fabsf
        {
            f_gravity_ff = (MASS * GRAVITY * numerator) / denominator;
        }

        // 4. 提取动态雅可比标量
        const float j_dynamic   = j_y * cos_pitch + j_x * sin_pitch;

        // 5. 将任务空间的力分别映射为关节空间的力矩
        const float tau_gravity = j_dynamic * (0.5f * f_gravity_ff);
        float tau_pid  = j_dynamic * (f_pitch + (i == 0 ? f_roll : -f_roll));

        // 6. 虚拟阻尼墙限位保护
        float tau_wall = 0.0f;
        if (theta > LEG_MAX_POS - LEG_POS_BUFFER_RAD)
        {
            tau_wall =
                -LEG_K_WALL * (theta - (LEG_MAX_POS - LEG_POS_BUFFER_RAD)) -
                LEG_D_WALL * theta_dot;
            tau_wall = fminf(0.0f, tau_wall); // 使用硬件 FPU 的 fminf
        }
        else if (theta < LEG_MIN_POS + LEG_POS_BUFFER_RAD)
        {
            tau_wall =
                LEG_K_WALL * ((LEG_MIN_POS + LEG_POS_BUFFER_RAD) - theta) -
                LEG_D_WALL * theta_dot;
            tau_wall = fmaxf(0.0f, tau_wall); // 使用硬件 FPU 的 fmaxf
        }

        // 7. 力矩饱和安全限制 (基于优先级的削峰逻辑)
        const float tau_priority = tau_gravity + tau_wall;
        float tau_total          = 0.0f;

        if (fabsf(tau_priority + tau_pid) > LEG_MAX_TORQUE)
        {
            if (fabsf(tau_priority) >= LEG_MAX_TORQUE)
            {
                tau_total =
                    (tau_priority > 0.0f) ? LEG_MAX_TORQUE : -LEG_MAX_TORQUE;
            }
            else
            {
                const float tau_avail = LEG_MAX_TORQUE - fabsf(tau_priority);
                tau_pid   = (tau_pid > 0.0f) ? tau_avail : -tau_avail;
                tau_total = tau_priority + tau_pid;
            }
        }
        else
        {
            tau_total = tau_priority + tau_pid;
        }

        // 8. 输出并再次针对右腿作符号映射
        _ctx.data.out_leg_torque[i] = (i == 0 ? 1.0f : -1.0f) * tau_total;
    }
}

void hybrid_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_mecanum_torque[i] = _ctx.pid.mecanum_pid[i]->calculate(
            _ctx.data.target_wheel_rpm[i], _ctx.data.current_wheel_rpm[i]);
    }
}

void hybrid_chassis_t::_track_control()
{
    for (int i = 0; i < 2; i++)
    {
        _ctx.data.out_track_torque[i] = _ctx.pid.track_pid[i]->calculate(
            _ctx.data.target_track_rpm[i], _ctx.data.current_track_rpm[i]);
    }
}

void hybrid_chassis_t::_send_motor_command() const
{
    for (int i = 0; i < 4; i++)
        _ctx.motor.mecanum[i]->send_torque(_ctx.data.out_mecanum_torque[i]);
    for (int i = 0; i < 2; i++)
        _ctx.motor.track[i]->send_torque(_ctx.data.out_track_torque[i]);
    for (int i = 0; i < 2; i++)
        _ctx.motor.leg[i]->send_torque(_ctx.data.out_leg_torque[i]);
}

// =========================================================
// 核心运行时与状态机
// =========================================================

void hybrid_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

// --- 子状态占位 (巡航与爬坡) ---
void hybrid_chassis_t::fsm_active_t::cruising_state_t::enter(owner *owner)
{
}
void hybrid_chassis_t::fsm_active_t::cruising_state_t::execute(owner *owner)
{
}
void hybrid_chassis_t::fsm_active_t::cruising_state_t::exit(owner *owner)
{
}

void hybrid_chassis_t::fsm_active_t::climbing_state_t::enter(owner *owner)
{
}
void hybrid_chassis_t::fsm_active_t::climbing_state_t::execute(owner *owner)
{
}
void hybrid_chassis_t::fsm_active_t::climbing_state_t::exit(owner *owner)
{
}

} // namespace pyro