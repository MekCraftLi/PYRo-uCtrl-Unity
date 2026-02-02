#ifndef __PYRO_MEC_CHASSIS_H__
#define __PYRO_MEC_CHASSIS_H__

#include "pyro_chassis_base.h"
#include "pyro_motor_base.h"

namespace pyro
{
class mec_chassis_t final : public module_base_t
{
  public:
    struct cmd_mec_t final : cmd_base_t
    {
        cmd_mec_t() : cmd_base_t(type_t::MECANUM)
        {
        }
        // Additional mecanum-specific command parameters can be added here
    };
    mec_chassis_t() : module_base_t(type_t::MECANUM)
    {
    }
    ~mec_chassis_t() override;

    void init() override;
    void set_command(const cmd_base_t &cmd) override;
    void update_feedback() override;
    void kinematics_solve() override;
    void chassis_control() override;
    void power_control() override;
    void send_motor_command() override;
    
    // 添加设置轮子初始量的方法
    void set_wheel_initial_offset(int index, float offset);
    void set_all_wheels_initial_offset(const float offsets[4]);

  private:
    motor_base_t *_wheel_motor[4]{}; // FL, FR, BL, BR

    // Add private members and methods as needed
};
} // namespace pyro
#endif