#include <Arduino.h>
#include "scheduler.h"

Scheduler scheduler;

void UpdateFunction(void);

void setup()
{
    Serial.begin(115200);
    Serial.println("Begin");
    scheduler.ScheduleFunction(UpdateFunction, 2000);
}

void loop()
{
    scheduler.Update();
}

void UpdateFunction(void)
{
    Serial.println("I got called!");
}
