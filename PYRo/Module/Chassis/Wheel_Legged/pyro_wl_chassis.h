#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_module_base.h"

namespace pyro
{
//command structure for wheel-legged chassis
struct wl_cmd_t final : public cmd_base_t
{
    /* Linear velocity in x direction(m/s), positive toward the front of chassis
       negative toward the back of chassis */
    float vx;   
    /* Linear velocity in y direction(m/s), positive toward the left of chassis
       negative toward the right of chassis */
    float vy;  
    /* Angular velocity in z direction(rad/s), positive for counter-clockwise
       negative for clockwise */
    float vz;
    /* Leg length of both sides after normalization, value is between 0 and 1*/
    float l_leg;
    float r_leg;

    /* Construct function, set zero values */
    wl_cmd_t() : vx(0), vy(0), vz(0), l_leg(0), r_leg(0)
    {
    }
};

class wl_chassis_t final : public module_base_t<wl_chassis_t, wl_cmd_t>
{
    friend class module_base_t<wl_chassis_t, wl_cmd_t>;

public:
    wl_chassis_t(const wl_chassis_t &)            = delete;
    wl_chassis_t &operator=(const wl_chassis_t &) = delete;

private:
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    /* base interface define */
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    /* private function*/
    
};
}
#endif
