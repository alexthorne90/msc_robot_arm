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
        uint8_t SetArm(float x, float y, float z, float grip_angle_d);

    private:

        float CalculateShoulderAngle(float x, float y, float z, float grip_angle_d);
        float CalculateHeightErrorFromShoulderAngleAndHeight(float shoulder_angle, float height);

};


#endif
