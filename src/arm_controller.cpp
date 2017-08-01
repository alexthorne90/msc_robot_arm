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

void ArmController::SetHomePosition(void)
{
    Al5d::SetHomePosition();
    current_x = GetCurrentX();
    current_y = GetCurrentY();
    current_z = GetCurrentZ();
    desired_x = current_x;
    desired_y = current_y;
    desired_z = current_z;
}

uint8_t ArmController::SetArm(float x, float y, float z, float grip_angle_d)
{
    desired_x = x;
    desired_y = y;
    desired_z = z;
    desired_grip_angle = grip_angle_d;
    return 0;
}

uint8_t ArmController::Update(uint16_t time_since_last_update_ms)
{
    float next_x = desired_x;
    float next_y = desired_y;
    float next_z = desired_z;
    float delta_x = abs(next_x - current_x);
    float delta_y = abs(next_y - current_y);
    float delta_z = abs(next_z - current_z);
    float largest_delta = 0;
    float delta_factor = 0;
    float max_move_this_update = (MAX_MM_PER_SECOND_UPDATE /
            (float)(1000.0 / time_since_last_update_ms));

    if (delta_x > max_move_this_update || delta_y > max_move_this_update ||
            delta_z > max_move_this_update)
    {
        largest_delta = max(delta_x, delta_y);
        largest_delta = max(largest_delta, delta_z);
        delta_factor = largest_delta / max_move_this_update;
        next_x = (next_x > current_x) ? current_x + (delta_x / delta_factor) :
            current_x - (delta_x / delta_factor);
        next_y = (next_y > current_y) ? current_y + (delta_y / delta_factor) :
            current_y - (delta_y / delta_factor);
        next_z = (next_z > current_z) ? current_z + (delta_z / delta_factor) :
            current_z - (delta_z / delta_factor);
    }

    float original_shoulder_angle =
        CalculateShoulderAngle(next_x, next_y, next_z, desired_grip_angle);
    float error_corrected_z = next_z -
        CalculateHeightErrorFromShoulderAngleAndHeight(
                original_shoulder_angle, next_z);

#ifdef ARM_CONTROLLER_DEBUG
    Serial.print("Set arm to x = ");
    Serial.print(next_x);
    Serial.print(" y = ");
    Serial.print(next_y);
    Serial.print(" z = ");
    Serial.println(next_z);
    Serial.print("Updated height to ");
    Serial.print(error_corrected_z);
    Serial.print(" based on original shoulder angle of ");
    Serial.println(original_shoulder_angle);
#endif


    current_x = next_x;
    current_y = next_y;
    current_z = next_z;
    return Al5d::SetArm(next_x, next_y, error_corrected_z, desired_grip_angle);
}

bool ArmController::hasReachedDesiredPosition()
{
    return (current_x == desired_x) &&
        (current_y == desired_y) &&
        (current_z == desired_z);
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
