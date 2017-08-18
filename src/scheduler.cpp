/**
 * @file scheduler.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "scheduler.h"

Scheduler::Scheduler(void)
{
    last_update = millis();
    update_period = 0;
    function = NULL;
}

void Scheduler::ScheduleFunction(void (*f)(void), uint16_t update_period_ms)
{
    update_period = update_period_ms;
    function = f;
}

void Scheduler::Update(void)
{
    unsigned long current_millis;
    if (function == NULL)
    {
        return;
    }
    if (update_period <= 0)
    {
        return;
    }

    current_millis = millis();
    if (current_millis >= (last_update + update_period))
    {
        last_update = current_millis;
        function();
    }
}
