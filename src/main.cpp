#include <Arduino.h>
#include "metal_detector_lib.h"
#include "arm_controller.h"

static MetalDetector metal_detector;
static ArmController arm;

static const int MD_UPDATE_PERIOD_MS = 150;
static unsigned long last_md_update_millis = 0;

static const int ARM_UPDATE_PERIOD_MS = 25;
static unsigned long last_arm_update_millis = 0;

static const int TEST_UPDATE_PERIOD_MS = 2000;
static unsigned long last_test_update_millis;
static const float test_x = 0;
static const float test_y = 150;
static const float test_gripper_angle = -90;
static float test_height_mm = 85.0;
static const float TEST_HEIGHT_INCREMENT_MM = 0.5;
static const float TEST_MIN_HEIGHT_MM = 41.5;

bool isTimeForArmUpdate(void);
bool isTimeForMDUpdate(void);
bool isTimeForTestUpdate(void);
void ArmUpdate(void);
void MDUpdate(void);
void TestUpdate(void);

void setup()
{
	Serial.begin(115200);
	delay(1000);
    metal_detector.ConnectToSensor();
    arm.AttachMotors();
    Serial.println("Sensor connected and motors attached");
	delay(1000);
    metal_detector.ConfigureSensor();
    arm.SetHomePosition();
    arm.SetMMPerSecondArmSpeed(25);
    Serial.println("Sensor configured and arm at home position");
	delay(1000);
    arm.SetArm(test_x, test_y, test_height_mm, test_gripper_angle);
    arm.Update(ARM_UPDATE_PERIOD_MS);
    while (!arm.hasReachedDesiredPosition())
    {
        delay(ARM_UPDATE_PERIOD_MS);
        arm.Update(ARM_UPDATE_PERIOD_MS);
        metal_detector.Update();
    }
    Serial.println("Initial arm test position set");
    last_md_update_millis = millis();
    last_arm_update_millis = millis();
	Serial.println("All initialized, begin test.");
	Serial.print("Initial inductance reading = ");
	Serial.println(metal_detector.GetCh0InductanceuH());
}

void loop()
{
    if (isTimeForArmUpdate())
    {
        ArmUpdate();
    }
    if (isTimeForMDUpdate())
    {
        MDUpdate();
    }
    if (isTimeForTestUpdate())
    {
        TestUpdate();
    }
}

bool isTimeForArmUpdate(void)
{
    unsigned long current_millis = millis();
    bool is_time = (current_millis >= (last_arm_update_millis + ARM_UPDATE_PERIOD_MS));
    if (is_time)
    {
        last_arm_update_millis = current_millis;
    }
    return is_time;
}

void ArmUpdate(void)
{
    arm.Update(ARM_UPDATE_PERIOD_MS);
}

bool isTimeForMDUpdate(void)
{
    unsigned long current_millis = millis();
    bool is_time = (current_millis >= (last_md_update_millis + MD_UPDATE_PERIOD_MS));
    if (is_time)
    {
        last_md_update_millis = current_millis;
    }
    return is_time;
}

void MDUpdate(void)
{
    metal_detector.Update();
}

bool isTimeForTestUpdate(void)
{
    unsigned long current_millis = millis();
    bool is_time = (current_millis >= (last_test_update_millis + TEST_UPDATE_PERIOD_MS));
    if (is_time)
    {
        last_test_update_millis = current_millis;
    }
    return is_time;
}

void TestUpdate(void)
{
    Serial.print("LDC inductance ");
    Serial.print(metal_detector.GetCh0InductanceuH(), 6);
    Serial.print("  at height ");
    Serial.println(test_height_mm);

    if (test_height_mm > TEST_MIN_HEIGHT_MM)
    {
        test_height_mm -= TEST_HEIGHT_INCREMENT_MM;
        arm.SetArm(test_x, test_y, test_height_mm, test_gripper_angle);
    }
}
