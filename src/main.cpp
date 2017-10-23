#include <Arduino.h>
#include "scheduler.h"
#include "shape_mapping_scanner.h"
#include "defined_area_scanner.h"

static Scheduler scheduler;
static const int UPDATE_PERIOD_MS = 100;
static void AppUpdate(void);

// Scanner objects
// Switch the commented out portions for other type of scanning
static ShapeMappingScanner scanner;
//static DefinedAreaScanner scanner;

void setup()
{
	Serial.begin(115200);
    scheduler.ScheduleFunction(AppUpdate, UPDATE_PERIOD_MS);
    scanner.HardwareSetup();
    delay(1000);
    scanner.SetReferenceInductance();
    scanner.SetHomePosition();
    scanner.SetScanOrigin(0, 150, 40);

    // Uncomment these for use of the DefinedAreaScanner
    // and update parameters if necessary
    //scanner.SetDesiredHorizontalTravelMM(100);
    //scanner.SetDesiredDepthTravelMM(40);
    //scanner.SetDepthScanIncrementMM(5);

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
        Serial.println("Scan complete!");
        while(1);
    }
}
