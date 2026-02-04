#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_common.h"
#include "arm_math.h"
#include "pyro_rc_hub.h"

using namespace pyro;

#define R_MOTOR1_OFFSET 2.66f
#define R_MOTOR2_OFFSET 3.979092653f
#define L_MOTOR1_OFFSET 0.0f
#define L_MOTOR2_OFFSET 0.0f

//the cofficients for phi calculation
#define PHI_K0 22506.0f
#define PHI_K1 475976946.0f
#define PHI_K2 296955904.0f
#define PHI_K3 179021042.0f
#define PHI_K4 18922.0f

//the cofficients for position in polar coordinates
#define POLAR_K0 236976927.0f
#define POLAR_K1 21059.0f
#define POLAR_K2 946100000.0f
#define POLAR_K3 100000.0f

#define BARYCENTER_K0 0.0332911f
#define BARYCENTER_K1 -0.5908261f
#define BARYCENTER_K2 0.2033493f
const pyro::dr16_drv_t::dr16_ctrl_t *rc_data;

dm_motor_drv_t* r_motor1;
dm_motor_drv_t* r_motor2;
dm_motor_drv_t* l_motor1;
dm_motor_drv_t* l_motor2;
dji_m3508_motor_drv_t* r_wheel;
dji_m3508_motor_drv_t* l_wheel;

float r_theta1, r_theta2, l_theta1, l_theta2;
float r_phi1, r_phi2, l_phi1, l_phi2;
float r_alpha, l_alpha;
float r_l, l_l;
float r_T[2],r_F[2],l_T[2],l_F[2],T_val[4];
float r_barycenter, l_barycenter;
arm_matrix_instance_f32 T;
rc_drv_t *dr16_drv;

float calc_barycenter(float leg_length);

status_t enable(void);
status_t disable(void);
status_t update_feedback(void);
status_t kinomatic_solve(float theta1, float theta2,
                         float* phi1, float* phi2,
                         float* alpha, float* l);
                         
void update_transform_matrix(float phi1, float phi2,
                             float theta1, float theta2,
                             float alpha, float l, float* T);


extern "C" void pyro_chassis(void* argument)
{
    //Init

    //Init RC
    dr16_drv = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);

    //Init matrix
    arm_mat_init_f32(&T, 2, 2, T_val);
    
    //Init motors
    r_motor1 = new dm_motor_drv_t(0x01, 0x11,
                                                     can_hub_t::can1);
    r_motor1->set_rotate_range(-54.0f, 54.0f);
    r_motor1->set_position_range(-12.5f, 12.5f);
    r_motor1->set_rotate_range(-45.0f, 45.0f);
    r_motor2 = new dm_motor_drv_t(0x02, 0x12,
                                                     can_hub_t::can1);
    r_motor2->set_rotate_range(-54.0f, 54.0f);
    r_motor2->set_position_range(-12.5f, 12.5f);
    r_motor2->set_rotate_range(-45.0f, 45.0f);
    l_motor1 = new dm_motor_drv_t(0x03, 0x13,
                                                     can_hub_t::can2);
    l_motor1->set_rotate_range(-54.0f, 54.0f);
    l_motor1->set_position_range(-12.5f, 12.5f);
    l_motor1->set_rotate_range(-45.0f, 45.0f);
    l_motor2 = new dm_motor_drv_t(0x04, 0x14,
                                                     can_hub_t::can2);
    l_motor2->set_rotate_range(-54.0f, 54.0f);
    l_motor2->set_position_range(-12.5f, 12.5f);
    l_motor2->set_rotate_range(-45.0f, 45.0f);

    r_wheel = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, 
                                                        can_hub_t::can1);
    l_wheel = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, 
                                                        can_hub_t::can2);
    while(1)
    {   

        float temp1, temp2;
        // update the angle of motor
           // caculate other angle
        // VMC transform
        r_F[0] = 1.0f;
        r_F[1] = 1.0f;
        
        r_barycenter = calc_barycenter(r_l); 
        update_transform_matrix(r_phi1, r_phi2, 
                              r_theta1, r_theta2,
                               r_alpha, r_l, T.pData);
        arm_mat_vec_mult_f32(&T, r_F, r_T);
            arm_sqrt_f32(  (PHI_K1 - PHI_K2*arm_cos_f32(r_theta1 - r_theta2) - PHI_K3*arm_cos_f32(2*r_theta1 - 2*r_theta2)), &temp1);
        arm_atan2_f32((PHI_K0*arm_sin_f32(r_theta1) - PHI_K0*arm_sin_f32(r_theta2)+temp1) /(PHI_K0*arm_cos_f32(r_theta1) - PHI_K0*arm_cos_f32(r_theta2) + PHI_K4*arm_cos_f32(r_theta1 - r_theta2) - PHI_K4),1.0f,&temp2);
        r_phi1 = (2 * temp2);
        arm_atan2_f32((PHI_K0*arm_sin_f32(r_theta1) - PHI_K0*arm_sin_f32(r_theta2)+temp1) /(PHI_K0*arm_cos_f32(r_theta1) - PHI_K0*arm_cos_f32(r_theta2) - PHI_K4*arm_cos_f32(r_theta1 - r_theta2) + PHI_K4),1.0f,&temp2);
        r_phi2 = (2 * temp2);  
        arm_atan2_f32((POLAR_K0*arm_sin_f32(r_phi1))/POLAR_K2 + (POLAR_K1*arm_sin_f32(r_theta1))/POLAR_K3, (POLAR_K0*arm_cos_f32(r_phi1))/POLAR_K2 + (POLAR_K1*arm_cos_f32(r_theta1))/POLAR_K3, &r_alpha);
        
        // caculate the position in polar coordinates
        temp1 = (POLAR_K0*arm_cos_f32(r_phi1))/POLAR_K2 + (POLAR_K1*arm_cos_f32(r_theta1))/POLAR_K3;
        temp1 = temp1 * temp1;
        temp2 = (POLAR_K0*arm_sin_f32(r_phi1))/POLAR_K2 + (POLAR_K1*arm_sin_f32(r_theta1))/POLAR_K3;
        temp2 = temp2 * temp2;
        arm_sqrt_f32(temp1 + temp2, &r_l);


        vTaskDelay(1);

    }
}

status_t enable(void)
{
    status_t ret;

    ret = r_wheel->enable();
    CHECK_PYRO_RET(ret);
    ret = l_wheel->enable();
    CHECK_PYRO_RET(ret);
    
    if(dm_motor_drv_t::ok != r_motor1->get_error_code())
    {
        ret = r_motor1->clear_error();
        CHECK_PYRO_RET(ret);
    }
    ret = r_motor1->enable();
    CHECK_PYRO_RET(ret);
    if(dm_motor_drv_t::ok != r_motor2->get_error_code())
    {
        ret = r_motor2->clear_error();
        CHECK_PYRO_RET(ret);
    }
    if(dm_motor_drv_t::ok != l_motor1->get_error_code())
    {
        ret = l_motor1->clear_error();
        CHECK_PYRO_RET(ret);
    }
    ret = l_motor1->enable();
    CHECK_PYRO_RET(ret);
    if(dm_motor_drv_t::ok != l_motor2->get_error_code())
    {
        ret = l_motor2->clear_error();
        CHECK_PYRO_RET(ret);
    }
    ret = l_motor2->enable();
    CHECK_PYRO_RET(ret);

    return PYRO_OK;
}

status_t disable(void)
{
    status_t ret;

    ret = r_wheel->disable();
    CHECK_PYRO_RET(ret)
    ret = l_wheel->disable();
    CHECK_PYRO_RET(ret);
    ret = r_motor1->disable();
    CHECK_PYRO_RET(ret);
    ret = r_motor2->disable();
    CHECK_PYRO_RET(ret);
    ret = l_motor1->disable();
    CHECK_PYRO_RET(ret);
    ret = l_motor2->disable();
    CHECK_PYRO_RET(ret);
    
    return PYRO_OK;
}


status_t update_feedback(void)
{
    status_t ret;
    //update RC data
    read_scope_lock rc_read_lock(dr16_drv->get_lock());
    rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(
                                                            dr16_drv->read());
    //update motor feedback
    ret = r_motor1->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = r_motor2->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = l_motor1->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = l_motor2->update_feedback();
    CHECK_PYRO_RET(ret);

    ret = r_wheel->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = l_wheel->update_feedback();
    CHECK_PYRO_RET(ret);

    //update theta of both legs, due to the motor installation direction,
    //right leg theta is opposite to motot feedback.
    r_theta1 = -(r_motor1->get_current_position()) + R_MOTOR1_OFFSET;
    r_theta2 = -(r_motor2->get_current_position()) + R_MOTOR2_OFFSET;
    l_theta1 =  (l_motor1->get_current_position()) + L_MOTOR1_OFFSET;
    l_theta2 =  (l_motor2->get_current_position()) + L_MOTOR2_OFFSET;

}
void update_transform_matrix(float phi1, float phi2,
                             float theta1, float theta2,
                             float alpha, float l, float* T)
{
    T[0] = (21059*arm_cos_f32(phi2)*arm_sin_f32(alpha)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_cos_f32(alpha)*arm_sin_f32(phi2)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2));
    T[1] = -((21059*arm_cos_f32(alpha)*arm_cos_f32(phi2)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_sin_f32(alpha)*arm_sin_f32(phi2)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2)))/l;
    T[2] = (21059*arm_cos_f32(alpha)*arm_sin_f32(phi1)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_cos_f32(phi1)*arm_sin_f32(alpha)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2));
    T[3] = ((21059*arm_cos_f32(alpha)*arm_cos_f32(phi1)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_sin_f32(alpha)*arm_sin_f32(phi1)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2)))/l; 

}



float calc_barycenter(float leg_length)
{
    return BARYCENTER_K0 + BARYCENTER_K1 * leg_length + BARYCENTER_K2 * leg_length * leg_length;
}