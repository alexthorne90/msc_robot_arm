/**
 * @file arm_controller.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "arm_controller.h"

//#define ARM_CONTROLLER_DEBUG

ArmController::ArmController(void) : Al5d()
{
}

uint8_t ArmController::SetArm(float x, float y, float z, float grip_angle_d)
{
    float original_shoulder_angle =
        CalculateShoulderAngle(x, y, z, grip_angle_d);
    float updated_z = z -
        CalculateHeightErrorFromShoulderAngleAndHeight(
                original_shoulder_angle, z);
#ifdef ARM_CONTROLLER_DEBUG
    Serial.print("Updated height to ");
    Serial.print(updated_z);
    Serial.print(" based on original shoulder angle of ");
    Serial.println(original_shoulder_angle);
#endif

    return Al5d::SetArm(x, y, updated_z, grip_angle_d);
}

float ArmController::CalculateShoulderAngle(float x, float y, float z, float grip_angle_d)
{
    //pre-calculations
    float hum_sq = HUMERUS*HUMERUS;
    float uln_sq = ULNA*ULNA;
    //grip angle in radians for use in calculations
    float grip_angle_r = radians( grip_angle_d );
    //Radial distance from x,y coordinates
    float rdist = sqrt(( x * x ) + ( y * y ));
    //Grip offsets calculated based on grip angle
    float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
    float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
    //Wrist position */
    float wrist_z = ( z - grip_off_z ) - BASE_HGT;
    float wrist_y = rdist - grip_off_y;

#ifdef ARM_CONTROLLER_DEBUG
    Serial.println("Arm controller original calculations:");
    Serial.print("wrist height = ");
    Serial.print(wrist_z);
    Serial.print("   and wrist depth = ");
    Serial.println(wrist_y);
#endif

    //Shoulder to wrist distance ( AKA sw )
    float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
    float s_w_sqrt = sqrt( s_w );
    //s_w angle to ground
    float a1 = atan2( wrist_z, wrist_y );
    //s_w angle to humerus
    float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));

    //shoulder angle
    float shl_angle_r = a1 + a2;
    float shl_angle_d = degrees( shl_angle_r );

#ifdef ARM_CONTROLLER_DEBUG
    Serial.print("Calculated shoulder angle = ");
    Serial.println(shl_angle_d - 90);
#endif

    return (shl_angle_d - 90);
}

float ArmController::CalculateHeightErrorFromShoulderAngleAndHeight(float shoulder_angle, float height)
{
    float correction_70mm = Calc70mmCorrection(shoulder_angle);
    float correction_15mm = Calc15mmCorrection(shoulder_angle);
    float desired_correction = MapFloat(height, 15.0, 70.0,
            correction_15mm, correction_70mm);

#ifdef ARM_CONTROLLER_DEBUG
    Serial.print("Correction at 70mm height:  ");
    Serial.println(correction_70mm);
    Serial.print("Correction at 15mm height:  ");
    Serial.println(correction_15mm);
    Serial.print("Maps to correction of ");
    Serial.print(desired_correction);
    Serial.print(" for height ");
    Serial.println(height);
#endif

    return desired_correction;
}

float ArmController::Calc70mmCorrection(float shoulder_angle)
{
    return EC_70mm_POW4_COEFF * pow(shoulder_angle, 4) +
        EC_70mm_POW3_COEFF * pow(shoulder_angle, 3) +
        EC_70mm_POW2_COEFF * pow(shoulder_angle, 2) +
        EC_70mm_POW1_COEFF * shoulder_angle +
        EC_70mm_POW0_COEFF;
}

float ArmController::Calc15mmCorrection(float shoulder_angle)
{
    return EC_15mm_POW4_COEFF * pow(shoulder_angle, 4) +
        EC_15mm_POW3_COEFF * pow(shoulder_angle, 3) +
        EC_15mm_POW2_COEFF * pow(shoulder_angle, 2) +
        EC_15mm_POW1_COEFF * shoulder_angle +
        EC_15mm_POW0_COEFF;
}

float ArmController::MapFloat(float x, float in_min, float in_max,
        float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
