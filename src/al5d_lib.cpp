/**
 * @file al5d_lib.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "al5d_lib.h"

#define DEBUG

Al5d::Al5d()
{
	//initialize variables
    current_x = 0;
    current_y = 0;
    current_z = 0;
    current_base_angle = 0;
    current_shoulder_angle = 0;
    current_elbow_angle = 0;
    current_wrist_angle = 0;
}

void Al5d::AttachMotors()
{
    //attach motors
    base_servo.attach(BASE_PIN, 554, 2425);
    shoulder_servo.attach(SHOULDER_PIN, 556, 2420);
    elbow_servo.attach(ELBOW_PIN, 556, 2410);
    wrist_servo.attach(WRIST_PIN, 553, 2520);
    gripper_servo.attach(GRIPPER_PIN);
}

void Al5d::SetHomePosition()
{
    base_servo.writeMicroseconds(BASE_SERVO_HOME_US);
    delay(15);
    shoulder_servo.writeMicroseconds(SHOULDER_SERVO_HOME_US);
    delay(15);
    elbow_servo.writeMicroseconds(ELBOW_SERVO_HOME_US);
    delay(15);
    wrist_servo.writeMicroseconds(WRIST_SERVO_HOME_US);
    delay(15);

    current_x = 0;
    current_y = ULNA + GRIPPER;
    current_z = BASE_HGT + HUMERUS;
    current_base_angle = 0;
    current_shoulder_angle = 90;
    current_elbow_angle = 90;
    current_wrist_angle = 90;
}

uint8_t Al5d::SetArm(float x, float y, float z, float grip_angle_d)
{
    //pre-calculations
    float hum_sq = HUMERUS*HUMERUS;
    float uln_sq = ULNA*ULNA;
    //grip angle in radians for use in calculations
    float grip_angle_r = radians( grip_angle_d );
    //Base angle and radial distance from x,y coordinates
    float bas_angle_r = atan2( x, y );
    float rdist = sqrt(( x * x ) + ( y * y ));
    //Grip offsets calculated based on grip angle
    float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
    float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
    //Wrist position */
    float wrist_z = ( z - grip_off_z ) - BASE_HGT;
    float wrist_y = rdist - grip_off_y;

#ifdef DEBUG
    Serial.print("Calculated wrist height = ");
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

#ifdef DEBUG
    Serial.print("Calculated A1 = ");
    Serial.print(degrees(a1));
    Serial.print("    A2 = ");
    Serial.println(degrees(a2));
#endif

    //shoulder angle
    float shl_angle_r = a1 + a2;
    float shl_angle_d = degrees( shl_angle_r );
    //elbow angle
    float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
    float elb_angle_d = degrees( elb_angle_r );
    float elb_angle_dn = -( 180.0 - elb_angle_d );
    //wrist angle
    float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;

    //Servo pulses
    float bas_servopulse = BASE_SERVO_HOME_US
        - (( degrees( bas_angle_r )) * (1.0 / BASE_SERVO_DEGREE_PER_US) );
    float shl_servopulse = SHOULDER_SERVO_HOME_US
        + (( shl_angle_d - 90.0 ) * (1.0 / SHOULDER_SERVO_DEGREE_PER_US) );
    float elb_servopulse = ELBOW_SERVO_HOME_US
        +  (( elb_angle_d - 90.0 ) * (1.0 / ELBOW_SERVO_DEGREE_PER_US) );
    float wri_servopulse = WRIST_SERVO_HOME_US
        - ( wri_angle_d  * (1.0 / WRIST_SERVO_DEGREE_PER_US) );

#ifdef DEBUG
    Serial.print("Wrote to base:  ");
    Serial.println(bas_servopulse);
    Serial.print("Wrote to shoulder:  ");
    Serial.println(shl_servopulse);
    Serial.print("Wrote to elbow:  ");
    Serial.println(elb_servopulse);
    Serial.print("Wrote to wrist:  ");
    Serial.println(wri_servopulse);
#endif

    base_servo.writeMicroseconds(bas_servopulse);
    delay(15);
    shoulder_servo.writeMicroseconds(shl_servopulse);
    delay(15);
    elbow_servo.writeMicroseconds(elb_servopulse);
    delay(15);
    wrist_servo.writeMicroseconds(wri_servopulse);
    delay(15);

    current_x = x;
    current_y = y;
    current_z = z;
    current_base_angle = degrees(bas_angle_r);
    current_shoulder_angle = shl_angle_d - 90;
    current_elbow_angle = elb_angle_r - 90;
    current_wrist_angle = wri_angle_d;

    return 0;
}

float Al5d::GetCurrentX()
{
    return current_x;
}

float Al5d::GetCurrentY()
{
    return current_y;
}

float Al5d::GetCurrentZ()
{
    return current_z;
}

float Al5d::GetCurrentBaseAngle()
{
    return current_base_angle;
}

float Al5d::GetCurrentShoulderAngle()
{
    return current_shoulder_angle;
}

float Al5d::GetCurrentElbowAngle()
{
    return current_elbow_angle;
}

float Al5d::GetCurrentWristAngle()
{
    return current_wrist_angle;
}
