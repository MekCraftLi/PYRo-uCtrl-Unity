#ifndef __PYRO_RUD_CHASSIS_H__
#define __PYRO_RUD_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_chassis_base.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_kin_rudder.h"
#include "pyro_motor_base.h"
#include "pyro_powermeter.h"

namespace pyro
{

// 定义舵轮特有的命令结构（如果有额外参数）
struct rud_cmd_t : cmd_base_t
{
    rud_cmd_t();
};

// 继承模板基类，传入具体的命令类型
class rud_chassis_t final 
    : public chassis_base_t<rud_chassis_t, rud_cmd_t>
{
    friend class chassis_base_t;
    friend class vofa_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct rud_ctx_t;
    
  public:
    rud_chassis_t(const rud_chassis_t &)            = delete;
    rud_chassis_t &operator=(const rud_chassis_t &) = delete;

    // 添加设置轮子初始量的方法
    void set_wheel_initial_offset(int index, float offset);
    void set_all_wheels_initial_offset(const float offsets[4]);

  private:
    rud_chassis_t();
    ~rud_chassis_t() override = default;

    // --- 基类接口 ---
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    void _kinematics_solve();
    static void _chassis_control(rud_ctx_t *ctx);
    static void _send_motor_command(rud_ctx_t *ctx);

    rudder_kin_t *_kinematics{nullptr};

    // 电机句柄
    struct motor_ctx_t
    {
        motor_base_t *rudder[4]{nullptr};
        motor_base_t *wheel[4]{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *rud_pos_pid[4]{nullptr};
        pid_t *rud_spd_pid[4]{nullptr};
        pid_t *wheel_pid[4]{nullptr};
    };

    struct data_ctx_t
    {
        float current_rud_rad[4]{};
        float current_rud_radps[4]{};
        float current_wheel_rpm[4]{};

        float target_rud_rad[4]{};
        float target_rud_radps[4]{};
        float target_wheel_rpm[4]{};

        float out_rud_torque[4]{};
        float out_wheel_torque[4]{};
    };

    struct hardware_ctx_t
    {
        powermeter_drv_t *power_meter{nullptr};
    };

    struct power_ctx_t
    {
        powermeter_data *data{nullptr};
    };

    struct rud_ctx_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        hardware_ctx_t hardware;
        power_ctx_t power;
        data_ctx_t data;
        cmd_base_t *cmd;
    };

    struct debug_ctx_t
    {
        float debug_rud_torque[4]{};
    };

    rud_ctx_t _ctx;
    debug_ctx_t debug_data;

    using owner = rud_chassis_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct state_active_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    state_passive_t _state_passive;
    state_active_t _state_active;
    fsm_t<owner> _main_fsm;

    static constexpr float RUD_RADIUS   = 0.050f;
};

} // namespace pyro
#endif