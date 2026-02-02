#include "pyro_core_config.h"
#if CONTROL_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_rud_chassis.h"
#include "pyro_rw_lock.h"
#include "stm32h723xx.h"

#ifdef __cplusplus

extern "C"
{
    void pyro_control_demo(void *arg)
    {
        pyro::cmd_base_t rud_cmd_obj{};
        pyro::dr16_drv_t::dr16_ctrl_t dr16_data;
        pyro::rud_chassis_t::instance()->start();

        while (true)
        {
            {
                pyro::rc_drv_t *dr16_drv =
                    pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
                pyro::read_scope_lock rc_read_lock(dr16_drv->get_lock());
                const auto *p_dr16 =
                    static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(
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
            if(pyro::dr16_drv_t::sw_state_t::SW_UP == dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode      = pyro::cmd_base_t::mode_t::ZERO_FORCE;
                rud_cmd_obj.timestamp = 0;
                rud_cmd_obj.vx        = 0.0f;
                rud_cmd_obj.vy        = 0.0f;
                rud_cmd_obj.wz        = 0.0f;
            }
            else if (pyro::dr16_drv_t::sw_state_t::SW_MID == dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode      = pyro::cmd_base_t::mode_t::ACTIVE;
                rud_cmd_obj.timestamp = 0;
                rud_cmd_obj.vx        = 1;
                rud_cmd_obj.vy        = 1;
                rud_cmd_obj.wz        = 1;
            }
            else if (pyro::dr16_drv_t::sw_state_t::SW_DOWN ==
                     dr16_data.rc.s_r.state)
            {
                rud_cmd_obj.mode      = pyro::cmd_base_t::mode_t::ACTIVE;
                rud_cmd_obj.timestamp = 0;
                rud_cmd_obj.vx        = dr16_data.rc.ch_lx;
                rud_cmd_obj.vy        = dr16_data.rc.ch_ly;
                rud_cmd_obj.wz        = 2.0f;
            }

            pyro::rud_chassis_t::instance()->set_command(rud_cmd_obj);
        }
    }
}
#endif
#endif
