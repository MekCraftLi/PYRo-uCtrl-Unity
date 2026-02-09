#ifndef __PYRO_RUD_CHASSIS_H__
#define __PYRO_RUD_CHASSIS_H__

#define POWER_CONTROL_USE 0

#include "pyro_algo_pid.h"
#include "pyro_algo_common.h"
#include "pyro_core_def.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_kin_rudder.h"
#include "pyro_motor_base.h"
#include "pyro_powermeter.h"
#include "pyro_power_control_drv.h"

namespace pyro
{

// 定义舵轮特有的命令结构（如果有额外参数）
struct rud_cmd_t : cmd_base_t
{
    float vx, vy, wz, yaw_error;
    bool follow_yaw;
    rud_cmd_t() : vx(0), vy(0), wz(0), yaw_error(0), follow_yaw(false)
    {
    }
};

// 继承模板基类，传入具体的命令类型
class rud_chassis_t final : public module_base_t<rud_chassis_t, rud_cmd_t>
{
    friend class module_base_t;
    friend class chassis_base_t;
    friend class vofa_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct rud_ctx_t;

  public:
    rud_chassis_t(const rud_chassis_t &)            = delete;
    rud_chassis_t &operator=(const rud_chassis_t &) = delete;

    struct motor_cfg_t
    {
        dji_motor_tx_frame_t::register_id_t rudder_id;
        can_hub_t::which_can rudder_can;
        dji_motor_tx_frame_t::register_id_t wheel_id;
        can_hub_t::which_can wheel_can;
    };

    struct offset_cfg_t
    {
        float rudder_pos_moving_offset[4];
    };

    struct pid_cfg_t
    {
        pid_t *rud_pos_pid[4]{nullptr};
        pid_t *rud_spd_pid[4]{nullptr};
        pid_t *wheel_pid[4]{nullptr};
        pid_t *follow_yaw_pid{nullptr};
    };

    struct power_ctx_t
    {
        powermeter_data *data{nullptr};
    };


    struct cfg_t
    {
        float wheelbase{};   // the distance between the front and back wheels
        float track_width{}; // the distance between the left and right wheels
        float wheel_radius{};
        float gear_ratio{};
        uint8_t powercontrol_num{};
        uint8_t power_limit{};
        motor_cfg_t motor_cfg[4]{}; //
        pid_cfg_t pid_cfg;
        offset_cfg_t rud_offset{}; //
        powermeter_drv_t *power_meter{nullptr};
    };

    status_t config(void *) override;

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
        pid_t *follow_yaw_pid{nullptr};
    };

    struct hardware_ctx_t
    {
        powermeter_drv_t *power_meter{nullptr};
    };

    struct data_ctx_t
    {
        rudder_kin_t::rudder_states_t current_states{};
        rudder_kin_t::rudder_states_t target_states{};

        float current_rud_radps[4]{};

        float out_rud_torque[4]{};
        float out_wheel_torque[4]{};
    };

    enum class drive_mode_t
    {
        MOVING,  // Normal driving mode
        BRAKING, // Braking mode (Stopping)
        TURNING,
    };

    struct rud_ctx_t
    {
        float wheelbase{};
        float track_width{};
        float wheel_radius{};
        float gear_ratio{};
        float rudder_pos_moving_offset[4]{};
        uint8_t powercontrol_num{};
        uint8_t power_limit{};
        motor_ctx_t motor;
        pid_ctx_t pid;
        hardware_ctx_t hardware;
        power_ctx_t power;
        data_ctx_t data;
        rud_cmd_t *cmd{};
        drive_mode_t drive_mode;
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

    struct fsm_active_t : public fsm_t<owner>
    {
        // 子状态
        struct state_moving_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_braking_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_turning_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_moving_t _moving_state;
        state_braking_t _braking_state;
        state_turning_t _turning_state;
    };

    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;

    static constexpr float RUD_RADIUS         = 0.060f;
    static constexpr uint8_t POWERCONTROL_NUM = 4;
    static constexpr uint8_t POWER_LIMIT      = 80;
};

} // namespace pyro
#endif