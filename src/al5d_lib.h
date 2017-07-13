/**
 * @file al5d_lib.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef AL5D_LIB_H
#define AL5D_LIB_H

#include <Arduino.h>
#include <Servo.h>

class Al5d {

    public:

        Al5d();

        void AttachMotors();
        void SetHomePosition();
        uint8_t SetArm(float x, float y, float z, float grip_angle_d);
        float GetCurrentX();
        float GetCurrentY();
        float GetCurrentZ();
        float GetCurrentBaseAngle();
        float GetCurrentShoulderAngle();
        float GetCurrentElbowAngle();
        float GetCurrentWristAngle();

    private:

        float current_x;
        float current_y;
        float current_z;
        float current_base_angle;
        float current_shoulder_angle;
        float current_elbow_angle;
        float current_wrist_angle;
        Servo base_servo;
        Servo elbow_servo;
        Servo shoulder_servo;
        Servo wrist_servo;
        Servo gripper_servo;

        //Arm Servo pins
        const uint8_t BASE_PIN      = 4;
        const uint8_t SHOULDER_PIN  = 3;
        const uint8_t ELBOW_PIN     = 10;
        const uint8_t WRIST_PIN     = 13;
        const uint8_t GRIPPER_PIN   = 12;

        //Arm dimensions (mm)
        const float BASE_HGT    = 70.0;     //base hight 2.736"
        const float HUMERUS     = 146.0;    //shoulder-to-elbow "bone" 5.748"
        const float ULNA        = 185.0;    //elbow-to-wrist "bone" 7.283"
        const float GRIPPER     = 86.5;     //gripper length to tip 3.406"

        //Servo motor "home" positions
        const float BASE_SERVO_HOME_US      = 1510.0;
        const float SHOULDER_SERVO_HOME_US  = 1485.0;
        const float ELBOW_SERVO_HOME_US     = 1685.0;
        const float WRIST_SERVO_HOME_US     = 1520.0;

        //Servo angle scale factors
        const float BASE_SERVO_DEGREE_PER_US        = 0.102;
        const float SHOULDER_SERVO_DEGREE_PER_US    = 0.105;
        const float ELBOW_SERVO_DEGREE_PER_US       = 0.109;
        const float WRIST_SERVO_DEGREE_PER_US       = 0.100;
};

#endif
