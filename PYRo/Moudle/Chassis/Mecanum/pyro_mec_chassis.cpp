#include "pyro_mec_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_kin_mecanum.h"

namespace pyro
{

mec_chassis_t::~mec_chassis_t()
{
}

void mec_chassis_t::init()
{
    // 初始化麦克纳姆轮底盘的电机
    _wheel_motor[0] = dji_m3508_motor_drv_t::get_instance(0); // FL
    _wheel_motor[1] = dji_m3508_motor_drv_t::get_instance(1); // FR
    _wheel_motor[2] = dji_m3508_motor_drv_t::get_instance(2); // BL
    _wheel_motor[3] = dji_m3508_motor_drv_t::get_instance(3); // BR
}

void mec_chassis_t::set_command(const cmd_base_t &cmd)
{
    // 实现设置命令的逻辑
}

void mec_chassis_t::update_feedback()
{
    // 更新四个轮子的反馈数据
    for (int i = 0; i < 4; i++)
    {
        if (_wheel_motor[i] != nullptr)
        {
            _wheel_motor[i]->update_feedback();
        }
    }
}

void mec_chassis_t::kinematics_solve()
{
    // 实现运动学解算逻辑
}

void mec_chassis_t::chassis_control()
{
    // 实现底盘控制逻辑
}

void mec_chassis_t::power_control()
{
    // 实现功率控制逻辑
}

void mec_chassis_t::send_motor_command()
{
    // 发送电机控制命令
}

void mec_chassis_t::set_wheel_initial_offset(int index, float offset)
{
    if (index >= 0 && index < 4) {
        if (_wheel_motor[index] != nullptr) {
            _wheel_motor[index]->set_offset(offset);
        }
    }
}

void mec_chassis_t::set_all_wheels_initial_offset(const float offsets[4])
{
    for (int i = 0; i < 4; i++) {
        if (_wheel_motor[i] != nullptr) {
            _wheel_motor[i]->set_offset(offsets[i]);
        }
    }
}

} // namespace pyro