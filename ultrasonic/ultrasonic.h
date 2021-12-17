#ifndef ultrasonic_h
#define ultrasonic_h
#include <mbed.h>

DigitalOut USSTrigger(p11);
Timer ActiveTime;
Ticker TriggerTiming;
InterruptIn USSEcho(p12);


class ultrasonic {
    private:
        unsigned long ActiveWidth;
        void Trigger(void);
        void RiseEcho(void);
        void FallEcho(void);

    public:
        unsigned short USSDistance;
        void init(void);
        void Output_Monitor(unsigned short Value);
        
}