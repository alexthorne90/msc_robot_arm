#include <Arduino.h>
#include "metal_detector_lib.h"
#include "arm_controller.h"

static MetalDetector metal_detector;
static ArmController arm;

static const int MD_UPDATE_PERIOD_MS = 50;
static unsigned long last_md_update_millis = 0;

static const int ARM_UPDATE_PERIOD_MS = 25;
static unsigned long last_arm_update_millis = 0;

static uint16_t test_counter = 0;
static const uint16_t NUM_TESTS_DESIRED = 1000;
static const int TEST_UPDATE_PERIOD_MS = 200;
static unsigned long last_test_update_millis;
static const float test_y = 150;
static const float test_gripper_angle = -90;
static float test_z = 46.5;
static const float test_initial_reading_z = test_z + 40;
static const float TEST_X_INCREMENT_MM = 0.5;
static const float TEST_X_MAX_DELTA = 50.0;
static const float TEST_Z_CORRECTION_INCREMENT_MM = 0.25;
static float test_x = 0;
static int8_t current_dir = -1;

static float initial_inductance;
static const float MIN_DESIRED_INDUCTANCE_DELTA = 0.2343;
static const float MAX_DESIRED_INDUCTANCE_DELTA = 0.4785;

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
    arm.SetArm(test_x, test_y, test_initial_reading_z, test_gripper_angle);
    arm.Update(ARM_UPDATE_PERIOD_MS);
    while (!arm.hasReachedDesiredPosition())
    {
        delay(ARM_UPDATE_PERIOD_MS);
        arm.Update(ARM_UPDATE_PERIOD_MS);
        metal_detector.Update();
    }
    Serial.println("Initial arm test position set");
	Serial.print("Initial inductance reading = ");
    initial_inductance = metal_detector.GetCh0InductanceuH();
	Serial.println(initial_inductance, 6);
    delay(1000);
    arm.SetArm(test_x, test_y, test_z, test_gripper_angle);
    arm.Update(ARM_UPDATE_PERIOD_MS);
    while (!arm.hasReachedDesiredPosition())
    {
        delay(ARM_UPDATE_PERIOD_MS);
        arm.Update(ARM_UPDATE_PERIOD_MS);
        metal_detector.Update();
    }
    last_md_update_millis = millis();
    last_arm_update_millis = millis();
    last_test_update_millis = millis();
    Serial.print("Arm lowered to initial test position, height = ");
    Serial.println(test_z);
	Serial.println("All initialized, begin test.");
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
    float current_inductance = metal_detector.GetCh0InductanceuH();
    Serial.print("Inductance ");
    Serial.print(current_inductance, 6);
    Serial.print("  x ");
    Serial.print(test_x);
    Serial.print("   z ");
    Serial.println(test_z);
    test_counter ++;
    if (test_counter >= NUM_TESTS_DESIRED)
    {
        Serial.println("\nTest complete");
        while (1);
    }

    if ((initial_inductance - current_inductance) > MAX_DESIRED_INDUCTANCE_DELTA)
    {
        test_z += TEST_Z_CORRECTION_INCREMENT_MM;
    }
    if ((initial_inductance - current_inductance) < MIN_DESIRED_INDUCTANCE_DELTA)
    {
        test_z -= TEST_Z_CORRECTION_INCREMENT_MM;
    }

    if (current_dir == -1)
    {
        test_x -= TEST_X_INCREMENT_MM;
    }
    if (current_dir == 1)
    {
        test_x += TEST_X_INCREMENT_MM;
    }
    if ((test_x * current_dir) >= TEST_X_MAX_DELTA)
    {
        current_dir *= -1;
    }

    arm.SetArm(test_x, test_y, test_z, test_gripper_angle);
}
