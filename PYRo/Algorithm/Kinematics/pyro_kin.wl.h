/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 19:32:10
 * @LastEditors: Vod vod0575@outlook
 * @LastEditTime: 2026-02-06 22:09:11
 * @Description: 
 * The kinematic solve algorithm for wheel legged robot. If you want to use,
 * define a variable which type is wheel_legged_kin_t, than call its init 
 * function. After init, you can call solve function and vmc function to 
 * solve physical angles and do force mapping 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */


#ifndef __PYRO_KIN_WL_H__
#define __PYRO_KIN_WL_H__

#include "pyro_core_def.h"

#include "arm_math.h"// IWYU pragma: keep
#include <cstdint>
namespace pyro
{

class wheel_legged_kin_t
{
public:
/*The position of joints, the middle point between j1 and j2 is the origin.
 
       j2-ivl-j1      j6     <-------- derction of forward
       /       \     /  \
      br        \  elr   \
     /           \ /      \
     j3          j5       j7   j5-j6-j7-j8 are a parallelogram
      \         /  \      /
       lr      /   ebr   /
        \     /      \  / 
         \   /        j8 
          \ /         /
          j4         /
                    /          origin, j4 and j9 are in the same line
                   /
                  /
                 /
                /
               j9
*/
   //the cofficients for phi solve
    struct phi_k_t
    {
       float k0;
       float k1;
       float k2;
       float k3;
       float k4;
    };
    
    //the cofficients for polar coordinates solve
    struct polar_k_t
    {
       float k0;
       float k1;
       float k2;
       float k3;
    };
    
    //the cofficients for VMC transform matrix
    struct vmc_k_t
    {
       float k0;
       float k1;
    };
    
    /**
     * @description:  
       Solve polar coordinations of the end point and angles between litte rod 
       and direction of movement through the angles between big rod and 
       direction of movement.
     * @param {float} theta1
       Angle between j2-j3 and direction of movement, counter clockwise is 
       positive(rad).
     * @param {float} theta2 
       Angle between j1-j5 and direction of movement, counter clockwise if 
       positive(rad).
     * @param {float} *phi1  
       Return solved angle between j3-j4 and direction of movment, counter 
       clockwise is positive(rad).
     * @param {float} *phi2
       Return solved angle bwtween j4-j5 and directio of movement, counter 
       clockwise is positive(rad).
     * @param {float} *length 
       Return solved polar radius of j9(m).
     * @param {float} *alpha
       Return solved polar angle of j9, counter clockwise is positive(rad).
     * @return {*}
       PYRO_OK: OK.
       PYRO_NOT_FOUND: Call this function before init.
       PYRO_PARAM_ERROR: The point of parameter is null.
     */
    status_t solve(float theta1, float theta2, 
                   float *phi1,  float *phi2, 
                   float *length, float *alpha);
    
    /**
     * @description: 
       Return VMC transform matrix value.
     * @param {float} theta1
     * Angle between j2-j3 and direction of movement, counter clockwise is 
       positive(rad). 
     * @param {float} theta2  
       Angle between j1-j5 and direction of movement, counter clockwise if 
       positive(rad).
     * @param {float} phi1
       Angle between j3-j4 and direction of movement, counter clockwise if 
       positive(rad).
     * @param {float} phi2
       Angle between j4-j5 and direction of movement, counter clockwise if 
       positive(rad).  
     * @param {float} length
       Polar radius of j9(m).
     * @param {float} alpha
       Polar angle of j9(rad).
     * @return {*}
       PYRO_OK: OK
       PYRO_NOT_FOUND: Call this function before init.
       PYRO_PARAM_ERROR: The point of parameter is null.
     */
    status_t get_VMC_value(float theta1, float theta2,
                           float phi1, float phi2, 
                           float length, float alpha);
   
private: 

   uint8_t _init_flag;
   

}
}
#endif
