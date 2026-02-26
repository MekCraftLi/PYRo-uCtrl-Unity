/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 15:27:37
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-08 14:52:32
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

 #include "pyro_wl_chassis.h"


 namespace pyro
 {
wl_chassis_t::wl_chassis_t(cfg_t *cfg) : module_base_t("wl_chassis")
{
    /* Initialize kinematic solver with given coefficients. */
    _kinematic_solver.init(&cfg->phi_k, &cfg->polar_k, 
                                                   &cfg->vmc_k);
    /* Save LQR coefficients */
    memcpy(_lqr_cof, cfg->lqr_coef, sizeof(float) * 36);

    /* Save wheel radius and reduction ratio */
    _wheel_radius = cfg->wheel_radius;
    _reduction_ratio = cfg->reduction_ratio;

    /* Initialize joint motor driver */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i] = new dm_motor_drv_t(cfg->joint_motor_cfg[i].tx_id,
                                           cfg->joint_motor_cfg[i].rx_id,
                                           cfg->joint_motor_cfg[i].can);
        _motor_offset[i] = cfg->joint_motor_cfg[i].offset_angle;
        _motor_drv[i]->set_rotate_range(cfg->rotate_min, 
                                                  cfg->rotate_max);
        _motor_drv[i]->set_position_range(cfg->position_min,
                                                  cfg->position_max);
        _motor_drv[i]->set_torque_range(cfg->torque_min,
                                                  cfg->torque_max);
    }
    /* Initialize wheel motor driver */
    for(uint8_t i = 0; i < 2; i++)
    {
        _wheel_drv[i] = 
               new dji_m3508_motor_drv_t(cfg->wheel_motor_cfg[i].tx_id,
                                     cfg->wheel_motor_cfg[i].can);
    }
}
}