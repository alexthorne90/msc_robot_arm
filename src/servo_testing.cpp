#include <Arduino.h>
#include <Servo.h>

//Undefine to remove debug prints
#define DEBUG


//Arm Servo pins
#define Base_pin 4
#define Shoulder_pin 3
#define Elbow_pin 10
#define Wrist_pin 13
#define Gripper_pin 12


//Arm dimensions (mm)
#define BASE_HGT 69.5       //base hight 2.736"
#define HUMERUS 146.0       //shoulder-to-elbow "bone" 5.748"
#define ULNA 185.0          //elbow-to-wrist "bone" 7.283"
#define GRIPPER 86.5        //gripper length to tip 3.406"


//pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;


//Servo Objects
Servo Elb;
Servo Shldr;
Servo Wrist;
Servo Base;
Servo Gripper;

//Servo motor "home" positions
#define BASE_SERVO_HOME_US      1510.0
#define SHOULDER_SERVO_HOME_US  1485.0
#define ELBOW_SERVO_HOME_US     1685.0
#define WRIST_SERVO_HOME_US     1520.0

//Servo angle scale factors
#define BASE_SERVO_DEGREE_PER_US        0.1011
#define SHOULDER_SERVO_DEGREE_PER_US    0.105
#define ELBOW_SERVO_DEGREE_PER_US       0.0987
#define WRIST_SERVO_DEGREE_PER_US       0.0947


void setup()
{
    Serial.begin(115200);

    Base.attach(Base_pin, 554, 2425);
    Shldr.attach(Shoulder_pin, 556, 2420);
    Elb.attach(Elbow_pin, 556, 2410);
    Wrist.attach(Wrist_pin, 553, 2520);
    Gripper.attach(Gripper_pin);

    Base.writeMicroseconds(BASE_SERVO_HOME_US);
    Shldr.writeMicroseconds(SHOULDER_SERVO_HOME_US);
    Elb.writeMicroseconds(ELBOW_SERVO_HOME_US);
    Wrist.writeMicroseconds(WRIST_SERVO_HOME_US);

}


/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
void set_arm( float x, float y, float z, float grip_angle_d )
{
    float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
    /* Base angle and radial distance from x,y coordinates */
    float bas_angle_r = atan2( x, y );
    float rdist = sqrt(( x * x ) + ( y * y ));
    /* rdist is y coordinate for the arm */
    y = rdist;
    /* Grip offsets calculated based on grip angle */
    float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
    float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
    /* Wrist position */
    float wrist_z = ( z - grip_off_z ) - BASE_HGT;
    float wrist_y = y - grip_off_y;

#ifdef DEBUG
    Serial.print("Calculated wrist height = ");
    Serial.print(wrist_z);
    Serial.print("   and wrist depth = ");
    Serial.println(wrist_y);
#endif

    /* Shoulder to wrist distance ( AKA sw ) */
    float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
    float s_w_sqrt = sqrt( s_w );
    /* s_w angle to ground */
    float a1 = atan2( wrist_z, wrist_y );
    /* s_w angle to humerus */
    float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));

#ifdef DEBUG
    Serial.print("Calculated A1 = ");
    Serial.print(degrees(a1));
    Serial.print("    A2 = ");
    Serial.println(degrees(a2));
#endif

    /* shoulder angle */
    float shl_angle_r = a1 + a2;
    float shl_angle_d = degrees( shl_angle_r );
    /* elbow angle */
    float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
    float elb_angle_d = degrees( elb_angle_r );
    float elb_angle_dn = -( 180.0 - elb_angle_d );
    /* wrist angle */
    float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;

    /* Servo pulses */
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

    Base.writeMicroseconds(bas_servopulse);
    Shldr.writeMicroseconds(shl_servopulse);
    Elb.writeMicroseconds(elb_servopulse);
    Wrist.writeMicroseconds(wri_servopulse);

}



void loop()
{
    int x_coordinate;
    int y_coordinate;
    int z_coordinate = 110;
    int sign_factor = 1;
    int gripper_pos = 1500;

    //for (y_coordinate = 120; y_coordinate <= 170; y_coordinate += 5)
    //{
    //    for (x_coordinate = (-60 * sign_factor); x_coordinate != (60 * sign_factor); x_coordinate += (5 * sign_factor))
    //    {
    //        set_arm(x_coordinate, y_coordinate, z_coordinate, -90);
    //        delay(100);
    //    }
    //    sign_factor *= -1;
    //}

    //while(1);


    String x_input;
    String y_input;
    String z_input;
    int command;
    delay(3000);
    Serial.println("");
    Serial.println("");
    Serial.print("Input X coordinate (side to side):  ");
    while (!(Serial.available() > 0))
    {
    }
    x_input = Serial.readString();
    x_coordinate = x_input.toInt();
    Serial.println(x_coordinate);

    Serial.print("Input Y coordinate (dist from base center):  ");
    while (!(Serial.available() > 0))
    {
    }
    y_input = Serial.readString();
    y_coordinate = y_input.toInt();
    Serial.println(y_coordinate);

    Serial.print("Input Z coordinate (height):  ");
    while (!(Serial.available() > 0))
    {
    }
    z_input = Serial.readString();
    z_coordinate = z_input.toInt();
    Serial.println(z_coordinate);
    Serial.println("Type 'g' to go or 'q' to cancel or 'c' 'o' for gipper control");

    while (1)
    {
        command = Serial.read();
        if (command == 'g')
        {
            Serial.println("IK go!");
            set_arm(x_coordinate, y_coordinate, z_coordinate, -90);
            break;
        }
        else if (command == 'q')
        {
            Serial.println("Input cancelled, try again");
            break;
        }
        else if (command == 'c')
        {
            gripper_pos += 25;
            Gripper.writeMicroseconds(gripper_pos);
            command = 0;
        }
        else if (command == 'o')
        {
            gripper_pos -= 25;
            Gripper.writeMicroseconds(gripper_pos);
            command = 0;
        }
    }
    delay(2000);

}
