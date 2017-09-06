#include <Arduino.h>
#include "scheduler.h"
#include "shape_mapping_scanner.h"

static Scheduler scheduler;
static const int UPDATE_PERIOD_MS = 50;
static void AppUpdate(void);

static ShapeMappingScanner scanner;

void setup()
{
	Serial.begin(115200);
	delay(1000);
    scanner.HardwareSetup();
    delay(1000);
    scanner.SetReferenceInductance();
    scanner.SetHomePosition();
    scanner.SetScanOrigin(-50, 150, 40);
    scanner.SetDesiredInductanceDelta(0.25);
    scanner.SetInductanceDeltaTolerance(0.1);
	Serial.println("Hello world");
    scheduler.ScheduleFunction(AppUpdate, UPDATE_PERIOD_MS);
}

void loop()
{
    scheduler.Update();
}

static void AppUpdate(void)
{
    scanner.Update(UPDATE_PERIOD_MS);
    if (scanner.isScanComplete())
    {
        Serial.println("Test complete - restarting.");
        delay(2000);
        scanner.ResetScan();
        scanner.SetReferenceInductance();
    }
}
