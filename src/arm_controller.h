/**
 * @file arm_controller.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include "al5d_lib.h"

class ArmController : public Al5d
{

    public:

        ArmController(void);
        void SetHomePosition(void);
        uint8_t SetArm(float x, float y, float z, float grip_angle_d);
        uint8_t Update(uint16_t time_since_last_update_ms);
        bool hasReachedDesiredPosition();

    private:

        //Private vars
        float current_x;
        float current_y;
        float current_z;
        float current_grip_angle;
        float desired_x;
        float desired_y;
        float desired_z;
        float desired_grip_angle;
        float MAX_MM_PER_SECOND_UPDATE = 25.0;
        float MAX_ANGLE_PER_SECOND_GRIPPER_UPDATE = 20.0;

        //Private helper functions
        float CalculateShoulderAngle(float x, float y, float z, float grip_angle_d);
        float CalculateHeightErrorFromShoulderAngleAndHeight(float shoulder_angle, float height);
        float Calc70mmCorrection(float shoulder_angle);
        float Calc15mmCorrection(float shoulder_angle);
        float MapFloat(float x, float in_min, float in_max,
                float out_min, float out_max);

        //Error correction polynomial constants
        const float EC_70mm_POW4_COEFF = -0.000001;
        const float EC_70mm_POW3_COEFF = 0.0001;
        const float EC_70mm_POW2_COEFF = -0.0043;
        const float EC_70mm_POW1_COEFF = 0.0177;
        const float EC_70mm_POW0_COEFF = 5.9731;
        const float EC_15mm_POW4_COEFF = -0.000004;
        const float EC_15mm_POW3_COEFF = 0.00005;
        const float EC_15mm_POW2_COEFF = -0.002;
        const float EC_15mm_POW1_COEFF = -0.0025;
        const float EC_15mm_POW0_COEFF = 11.223;

};


#endif
