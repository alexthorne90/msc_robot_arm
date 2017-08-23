#include <Arduino.h>
#include "metal_scanner.h"

static MetalScanner scanner;

static const int UPDATE_PERIOD_MS = 50;
static unsigned long last_update_millis = 0;

bool isTimeForUpdate(void);
void Update(void);

void setup()
{
	Serial.begin(115200);
	delay(1000);
    scanner.HardwareSetup();
    delay(1000);
    scanner.SetHomePosition();
    delay(1000);
    scanner.SetReferenceInductance();
    if (scanner.SetScanOrigin(-50, 150, 46.5))
        Serial.println("Invalid origin");
    if (scanner.SetDesiredHorizontalTravelMM(100))
        Serial.println("Invalid horiz travel");
    if (scanner.SetDesiredDepthTravelMM(0))
        Serial.println("Invalid depth travel");
    if (scanner.SetDepthScanIncrementMM(0))
        Serial.println("Invalid depth scan inc");
    if (scanner.SetTravelIncrementMM(0.5))
        Serial.println("Invalid travel inc");
    if (scanner.SetHeightCorrectionMM(1.0))
        Serial.println("Invalid height correct");
    if (scanner.SetDesiredInductanceDelta(1.0))
        Serial.println("Invalid inductance");
    if (scanner.SetInductanceDeltaTolerance(0.4))
        Serial.println("Invalid inductance tolerance");
    scanner.SetMMPerSecondArmSpeed(15.0);
    last_update_millis = millis();
	Serial.println("Starting test");
}

void loop()
{
    if (isTimeForUpdate())
    {
        Update();
    }
}

bool isTimeForUpdate(void)
{
    unsigned long current_millis = millis();
    bool is_time = (current_millis >= (last_update_millis + UPDATE_PERIOD_MS));
    if (is_time)
    {
        last_update_millis = current_millis;
    }
    return is_time;
}

void Update(void)
{
    scanner.Update(UPDATE_PERIOD_MS);
    if (scanner.isScanComplete())
    {
        Serial.print("Out of bounds ");
        Serial.print(scanner.GetOutOfBoundsPercentage());
        Serial.println("%");
        while(1);
    }
}
