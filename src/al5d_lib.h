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

    protected:

        //Arm dimensions (mm)
        const float BASE_HGT    = 70.0;     //base hight 2.736"
        const float HUMERUS     = 146.0;    //shoulder-to-elbow "bone" 5.748"
        const float ULNA        = 185.0;    //elbow-to-wrist "bone" 7.283"
        const float GRIPPER     = 86.5;     //gripper length to tip 3.406"

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

        //Servo motor "home" positions
        const float BASE_SERVO_HOME_US      = 1510.0;
        const float SHOULDER_SERVO_HOME_US  = 1485.0;
        const float ELBOW_SERVO_HOME_US     = 1685.0;
        const float WRIST_SERVO_HOME_US     = 1520.0;

        //Arm positions at servo home positions
        const float ARM_HOME_X                  = 0.0;
        const float ARM_HOME_Y                  = ULNA + GRIPPER;
        const float ARM_HOME_Z                  = BASE_HGT + HUMERUS;
        const float ARM_HOME_BASE_ANGLE         = 0.0;
        const float ARM_HOME_SHOULDER_ANGLE     = 90.0;
        const float ARM_HOME_ELBOW_ANGLE        = 90.0;
        const float ARM_HOME_WRIST_ANGLE        = 0.0;

        //Servo angle scale factors
        const float BASE_SERVO_DEGREE_PER_US        = 0.102;
        const float SHOULDER_SERVO_DEGREE_PER_US    = 0.105;
        const float ELBOW_SERVO_DEGREE_PER_US       = 0.109;
        const float WRIST_SERVO_DEGREE_PER_US       = 0.100;

        //Servo min and max microsecond positions
        const uint16_t BASE_SERVO_MIN_POS       = 554;
        const uint16_t BASE_SERVO_MAX_POS       = 2425;
        const uint16_t SHOULDER_SERVO_MIN_POS   = 556;
        const uint16_t SHOULDER_SERVO_MAX_POS   = 2420;
        const uint16_t ELBOW_SERVO_MIN_POS      = 556;
        const uint16_t ELBOW_SERVO_MAX_POS      = 2410;
        const uint16_t WRIST_SERVO_MIN_POS      = 553;
        const uint16_t WRIST_SERVO_MAX_POS      = 2520;
        const uint16_t GRIPPER_SERVO_MIN_POS    = 1000;
        const uint16_t GRIPPER_SERVO_MAX_POS    = 2000;

        //Servo update delay
        const uint8_t SERVO_UPDATE_DELAY_MS     = 15;
};

#endif
