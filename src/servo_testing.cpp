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


void setup()
{
    Serial.begin(115200);

    Base.attach(Base_pin);
    Shldr.attach(Shoulder_pin);
    Elb.attach(Elbow_pin);
    Wrist.attach(Wrist_pin);
    Gripper.attach(Gripper_pin);

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
    float bas_servopulse = 1500.0 - (( degrees( bas_angle_r )) * 9.804 );
    float shl_servopulse = 1485.0 + (( shl_angle_d - 90.0 ) * 9.524 );
    float elb_servopulse = 1660.0 +  (( elb_angle_d - 90.0 ) * 9.174 );
    float wri_servopulse = 1500.0 - ( wri_angle_d  * 10.0 );

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
    int z_coordinate;
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
    Serial.println("Type 'g' to go or 'q' to cancel");

    while (1)
    {
        command = Serial.read();
        if (command == 'g')
        {
            Serial.println("IK go!");
            set_arm(x_coordinate, y_coordinate, z_coordinate, 0);
            break;
        }
        else if (command == 'q')
        {
            Serial.println("Input cancelled, try again");
            break;
        }
    }
    delay(2000);

}
