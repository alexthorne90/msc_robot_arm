#include <Arduino.h>

static const long NUM_TESTS = 1000;
static const long TIME_BETWEEN_TESTS_MS = 10;

static long start_time = 0;
static long end_time = 0;
static long this_run_time = 0;
static long worst_time = 0;
static long total_time = 0;

void setup()
{
	Serial.begin(115200);
	delay(1000);
}

void loop()
{
	Serial.println("Hello world");
    for (int i = 0; i < NUM_TESTS; i ++)
    {
        start_time = millis();

        //function_to_test();

        end_time = millis();
        this_run_time = end_time - start_time;
        if (this_run_time > worst_time)
        {
            worst_time = this_run_time;
            Serial.print("New worst time ");
            Serial.println(worst_time);
        }
        total_time += this_run_time;
        delay(TIME_BETWEEN_TESTS_MS);
    }

    Serial.println();
    Serial.println("--- Test results ---");
    Serial.print("Number of iterations: ");
    Serial.println(NUM_TESTS);
    Serial.print("Worst case runtime (ms): ");
    Serial.println(worst_time);
    Serial.print("Average runtime (ms): ");
    Serial.println(total_time / NUM_TESTS);

    while (1);
}
