#include <Arduino.h>

static const int UPDATE_PERIOD_MS = 200;
static unsigned long last_update_millis = 0;

bool isTimeForUpdate(void);
void Update(void);

void setup()
{
	Serial.begin(115200);
	delay(1000);
    last_update_millis = millis();
	Serial.println("Hello world");
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
}
