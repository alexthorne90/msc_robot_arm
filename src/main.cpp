#include <Arduino.h>
#include "metal_detector_lib.h"

static MetalDetector metal_detector;

void setup()
{
    Serial.begin(115200);
    metal_detector.ConnectToSensor();
    delay(500);
    metal_detector.ConfigureSensor();
    delay(500);
}

void loop()
{
    metal_detector.Update();
    Serial.print("Frequency value ");
    Serial.println(metal_detector.GetCh0FreqHz());
    Serial.print("Inductance value ");
    Serial.println(metal_detector.GetCh0InductanceuH(), 6);
    delay(500);
}
