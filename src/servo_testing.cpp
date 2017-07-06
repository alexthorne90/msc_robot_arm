#include <Arduino.h>
#include <Servo.h>

#define DEBUG

//Select which arm by uncommenting the corresponding line
#define AL5D


const float A = 5.75;
const float B = 7.375;


//Arm Servo pins
#define Base_pin 4
#define Shoulder_pin 3
#define Elbow_pin 10
#define Wrist_pin 13
#define Gripper_pin 12


//Radians to Degrees constant
const float rtod = 57.295779;

//Arm Speed Variables
float Speed = 1.0;
int sps = 3;

//Servo Objects
Servo Elb;
Servo Shldr;
Servo Wrist;
Servo Base;
Servo Gripper;

//Arm Current Pos
float X = 4;
float Y = 4;
int Z = 90;
int G = 90;
float WA = 0;

//Arm temp pos
float tmpx = 4;
float tmpy = 4;
int tmpz = 90;
int tmpg = 90;
int tmpwr = 90;
float tmpwa = 0;


void setup()
{
    Serial.begin(115200);

    Base.attach(Base_pin, 553, 2425);
    Shldr.attach(Shoulder_pin, 556, 2420);
    Elb.attach(Elbow_pin, 556, 2410);
    Wrist.attach(Wrist_pin, 553, 2520);
    Gripper.attach(Gripper_pin);

    delay(1000);
    Base.writeMicroseconds(1510);
    Shldr.writeMicroseconds(1485);
    Elb.writeMicroseconds(1685);
    Wrist.writeMicroseconds(1520);

}



/*
int Arm(float x, float y, float z, int g, float wa) //Here's all the Inverse Kinematics to control the arm
{
    float M = sqrt((y*y)+(x*x));
    if(M <= 0)
    {
#ifdef DEBUG
        Serial.println("Negative or 0 M, return error");
#endif
        return 1;
    }
    float A1 = atan(y/x);
    if(x <= 0)
    {
#ifdef DEBUG
        Serial.println("Negative or 0 x, return error");
#endif
        return 1;
    }
    float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
    float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
    float Shoulder = A1 + A2;
    Elbow = Elbow * rtod;
    Shoulder = Shoulder * rtod;
    if((int)Elbow <= 0 || (int)Shoulder <= 0)
    {
#ifdef DEBUG
        Serial.println("Negative or 0 Elbow or Shoulder, return error");
#endif
        return 1;
    }
    float Wris = abs(wa - Elbow - Shoulder) - 90;
    Elb.write(180 - Elbow);
    Shldr.write(Shoulder);
    Wrist.write(180 - Wris);
    Base.write(z);
#ifdef DEBUG
    Serial.print("Wrote to Elb ");
    Serial.println(180 - Elbow);
    Serial.print("Wrote to Shldr ");
    Serial.println(Shoulder);
    Serial.print("Wrote to Wrist ");
    Serial.println(180 - Wris);
    Serial.print("Wrote to Base ");
    Serial.println(z);
#endif
    Gripper.write(g);
    Y = tmpy;
    X = tmpx;
    Z = tmpz;
    WA = tmpwa;
    G = tmpg;
#ifdef DEBUG
    Serial.print("Y = ");
    Serial.println(Y);
    Serial.print("X = ");
    Serial.println(X);
    Serial.print("Z = ");
    Serial.println(Z);
    Serial.print("WA = ");
    Serial.println(WA);
#endif
    return 0;
}
*/


void loop()
{
    int incomingByte;
    int servoSetting;
    String servoStr;

    if (Serial.available() > 0) {
        Serial.println("");
        Serial.println("");
        servoStr = Serial.readString();

        servoSetting = servoStr.toInt();
        Serial.print("Servo input = ");
        Serial.println(servoSetting);

        Serial.println("Type 'b' 's' 'e' or 'w' to go for that servo or 'q' to cancel");
        while (1)
        {
            incomingByte = Serial.read();
            if (incomingByte == 'b')
            {
                if (servoSetting > 556 && servoSetting < 2520)
                {
                    Serial.print("Wrote ");
                    Serial.print(servoSetting);
                    Serial.println(" to the servo base");
                    //Update the servo desired here
                    Base.writeMicroseconds(servoSetting);
                }
                else
                {
                    Serial.println("Invalid servo setting");
                }
                break;
            }
            else if (incomingByte == 's')
            {
                if (servoSetting > 556 && servoSetting < 2520)
                {
                    Serial.print("Wrote ");
                    Serial.print(servoSetting);
                    Serial.println(" to the servo shldr");
                    //Update the servo desired here
                    Shldr.writeMicroseconds(servoSetting);
                }
                else
                {
                    Serial.println("Invalid servo setting");
                }
                break;
            }
            else if (incomingByte == 'e')
            {
                if (servoSetting > 556 && servoSetting < 2520)
                {
                    Serial.print("Wrote ");
                    Serial.print(servoSetting);
                    Serial.println(" to the servo elbow");
                    //Update the servo desired here
                    Elb.writeMicroseconds(servoSetting);
                }
                else
                {
                    Serial.println("Invalid servo setting");
                }
                break;
            }
            else if (incomingByte == 'w')
            {
                if (servoSetting > 556 && servoSetting < 2520)
                {
                    Serial.print("Wrote ");
                    Serial.print(servoSetting);
                    Serial.println(" to the servo wrist");
                    //Update the servo desired here
                    Wrist.writeMicroseconds(servoSetting);
                }
                else
                {
                    Serial.println("Invalid servo setting");
                }
                break;
            }
            else if (incomingByte == 'q')
            {
                Serial.println("Input cancelled, try again");
                break;
            }
        }
        delay(2000);
    }
}
