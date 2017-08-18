#include <Arduino.h>
#include "scheduler.h"

static Scheduler scheduler;
static const int UPDATE_PERIOD_MS = 50;

void setup()
{
	Serial.begin(115200);
	delay(1000);
	Serial.println("Hello world");
    //scheduler.ScheduleFunction(TASK, UPDATE_PERIOD_MS);
}

void loop()
{
    scheduler.Update();
}
