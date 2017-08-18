/**
 * @file scheduler.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>

class Scheduler {

    public:

        Scheduler(void);
        void ScheduleFunction(void (*f)(void), uint16_t update_period_ms);
        void Update(void);

    private:

        //private vars
        uint16_t update_period;
        unsigned long last_update;
        void (*function)(void);
};

#endif
