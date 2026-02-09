#include "pyro_core_config.h"
#if CONTROL_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_rud_chassis.h"
#include "pyro_rw_lock.h"
#include "stm32h723xx.h"
#include "Yaw/pyro_yaw.h"

#ifdef __cplusplus

using namespace pyro;
extern "C"
{

    void pyro_control_demo(void *arg)
    {
        rud_cmd_t rud_cmd_obj{};
        yaw_cmd_t yaw_cmd_obj{};
        dr16_drv_t::dr16_ctrl_t dr16_data;

        rud_chassis_t::cfg_t rud_chassis_cfg{};
        rud_chassis_cfg.wheelbase    = 0.36f;
        rud_chassis_cfg.track_width  = 0.36f;
        rud_chassis_cfg.wheel_radius = 0.06f;
        rud_chassis_cfg.gear_ratio =
            dji_m3508_motor_drv_t::reciprocal_reduction_ratio;
        rud_chassis_cfg.powercontrol_num        = 4;
        rud_chassis_cfg.power_limit             = 80;
        rud_chassis_cfg.motor_cfg[0].rudder_id  = dji_motor_tx_frame_t::id_1;
        rud_chassis_cfg.motor_cfg[0].rudder_can = can_hub_t::can2;
        rud_chassis_cfg.motor_cfg[0].wheel_id   = dji_motor_tx_frame_t::id_1;
        rud_chassis_cfg.motor_cfg[0].wheel_can  = can_hub_t::can2;

        rud_chassis_cfg.motor_cfg[1].rudder_id  = dji_motor_tx_frame_t::id_2;
        rud_chassis_cfg.motor_cfg[1].rudder_can = can_hub_t::can2;
        rud_chassis_cfg.motor_cfg[1].wheel_id   = dji_motor_tx_frame_t::id_2;
        rud_chassis_cfg.motor_cfg[1].wheel_can  = can_hub_t::can2;

        rud_chassis_cfg.motor_cfg[2].rudder_id  = dji_motor_tx_frame_t::id_3;
        rud_chassis_cfg.motor_cfg[2].rudder_can = can_hub_t::can1;
        rud_chassis_cfg.motor_cfg[2].wheel_id   = dji_motor_tx_frame_t::id_3;
        rud_chassis_cfg.motor_cfg[2].wheel_can  = can_hub_t::can1;

        rud_chassis_cfg.motor_cfg[3].rudder_id  = dji_motor_tx_frame_t::id_4;
        rud_chassis_cfg.motor_cfg[3].rudder_can = can_hub_t::can1;
        rud_chassis_cfg.motor_cfg[3].wheel_id   = dji_motor_tx_frame_t::id_4;
        rud_chassis_cfg.motor_cfg[3].wheel_can  = can_hub_t::can1;

        rud_chassis_cfg.pid_cfg.rud_pos_pid[0] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rud_chassis_cfg.pid_cfg.rud_pos_pid[1] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rud_chassis_cfg.pid_cfg.rud_pos_pid[2] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rud_chassis_cfg.pid_cfg.rud_pos_pid[3] =
            new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);

        rud_chassis_cfg.pid_cfg.rud_spd_pid[0] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rud_chassis_cfg.pid_cfg.rud_spd_pid[1] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rud_chassis_cfg.pid_cfg.rud_spd_pid[2] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rud_chassis_cfg.pid_cfg.rud_spd_pid[3] =
            new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);

        rud_chassis_cfg.pid_cfg.wheel_pid[0] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        rud_chassis_cfg.pid_cfg.wheel_pid[1] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        rud_chassis_cfg.pid_cfg.wheel_pid[2] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        rud_chassis_cfg.pid_cfg.wheel_pid[3] =
            new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);

        rud_chassis_cfg.pid_cfg.follow_yaw_pid =
            new pid_t(3.5f, 0.01f, 0.01f, 0.1f, 5.0f);
        rud_chassis_cfg.rud_offset.rudder_pos_moving_offset[0] = 1.01472831f;
        rud_chassis_cfg.rud_offset.rudder_pos_moving_offset[1] = -0.29145637f;
        rud_chassis_cfg.rud_offset.rudder_pos_moving_offset[2] = -1.87299052f;
        rud_chassis_cfg.rud_offset.rudder_pos_moving_offset[3] = -1.04003897f;

        rud_chassis_cfg.power_meter =
            new powermeter_drv_t(0x212, can_hub_t::can2);

        power_control_drv_t &power_controller =
            power_control_drv_t::get_instance(4);
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

        yaw_t::cfg_t yaw_cfg{};
        yaw_cfg.motor_cfg.can_id           = 0x01;
        yaw_cfg.motor_cfg.master_id        = 0x02;
        yaw_cfg.motor_cfg.yaw_can          = can_hub_t::can2;
        yaw_cfg.motor_cfg.min_pos_range    = -PI;
        yaw_cfg.motor_cfg.max_pos_range    = PI;
        yaw_cfg.motor_cfg.min_rotate_range = -20;
        yaw_cfg.motor_cfg.max_rotate_range = 20;
        yaw_cfg.motor_cfg.min_torque_range = -10;
        yaw_cfg.motor_cfg.max_torque_range = 10;
        yaw_cfg.offset_cfg.yaw_offset      = -2.40028524f;
        yaw_cfg.pid_cfg.yaw_pos_pid =
            new pid_t(15.0f, 0.1f, 0.02f, 0.5f, 8.0f, 15, 150, 4);
        yaw_cfg.pid_cfg.yaw_spd_pid =
            new pid_t(0.2f, 0.002f, 0.0003f, 0.1f, 3.0f, 15, 150, 4);

        rud_chassis_t *chassis_instance =
            rud_chassis_t::instance(&rud_chassis_cfg);
        yaw_t *yaw_instance = yaw_t::instance(&yaw_cfg);

        CHECK_POINT_NULL_BLOCK(chassis_instance);
        CHECK_POINT_NULL_BLOCK(yaw_instance);

        chassis_instance->start();
        yaw_instance->start();

        while (true)
        {
            {
                rc_drv_t *dr16_drv = rc_hub_t::get_instance(rc_hub_t::DR16);
                read_scope_lock rc_read_lock(dr16_drv->get_lock());
                const auto *p_dr16 =
                    static_cast<const dr16_drv_t::dr16_ctrl_t *>(
                        dr16_drv->read());
                if (p_dr16 != nullptr)
                    dr16_data = *p_dr16;
            }
            if (abs(dr16_data.rc.ch_lx) < 0.3f &&
                abs(dr16_data.rc.ch_ly) < 0.3f &&
                abs(dr16_data.rc.ch_rx) < 0.3f)
            {
                dr16_data.rc.ch_lx = 0.0f;
                dr16_data.rc.ch_ly = 0.0f;
                dr16_data.rc.ch_rx = 0.0f;
            }

            // chassis_control
            if (dr16_drv_t::sw_state_t::SW_UP == dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode       = cmd_base_t::mode_t::PASSIVE;
                yaw_cmd_obj.mode       = cmd_base_t::mode_t::PASSIVE;
                rud_cmd_obj.follow_yaw = false;
                rud_cmd_obj.timestamp  = 0;
                rud_cmd_obj.vx         = 0.0f;
                rud_cmd_obj.vy         = 0.0f;
                rud_cmd_obj.wz         = 0.0f;
                rud_cmd_obj.yaw_error  = 0.0f;
            }
            else if (dr16_drv_t::sw_state_t::SW_MID == dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode       = cmd_base_t::mode_t::ACTIVE;
                yaw_cmd_obj.mode       = cmd_base_t::mode_t::ACTIVE;
                rud_cmd_obj.follow_yaw = true;
                rud_cmd_obj.timestamp  = 0;
                rud_cmd_obj.vx         = dr16_data.rc.ch_lx * 2.0f;
                rud_cmd_obj.vy         = dr16_data.rc.ch_ly * 2.0f;
                yaw_cmd_obj.target_yaw_imu_angle -= dr16_data.rc.ch_rx * 0.01f;
                rud_cmd_obj.yaw_error = yaw_instance->get_yaw_error();
            }
            else if (dr16_drv_t::sw_state_t::SW_DOWN == dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode       = cmd_base_t::mode_t::ACTIVE;
                yaw_cmd_obj.mode       = cmd_base_t::mode_t::ACTIVE;
                rud_cmd_obj.follow_yaw = false;
                rud_cmd_obj.timestamp  = 0;
                rud_cmd_obj.vx =
                    dr16_data.rc.ch_lx * cosf(yaw_instance->get_yaw_error()) -
                    dr16_data.rc.ch_ly * sinf(yaw_instance->get_yaw_error());
                rud_cmd_obj.vy =
                    dr16_data.rc.ch_ly * cosf(yaw_instance->get_yaw_error()) +
                    dr16_data.rc.ch_lx * sinf(yaw_instance->get_yaw_error());
                rud_cmd_obj.wz = 2.0f;
                yaw_cmd_obj.target_yaw_imu_angle -= dr16_data.rc.ch_rx * 0.01f;
            }

            chassis_instance->set_command(rud_cmd_obj);
            yaw_instance->set_command(yaw_cmd_obj);

            vTaskDelay(1);
        }
    }
}
#endif
#endif
