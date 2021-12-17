#include "ultrasonic.h"

DigitalOut USSTrigger(p11);
Timer ActiveTime;
Ticker TriggerTiming;
InterruptIn USSEcho(p12);

ultrasonic::ultrasonic(){
    init();
}

void ultrasonic::Trigger(void) {
    USSTrigger = 1;
    wait_us(10);
    USSTrigger = 0;
}

void ultrasonic::RiseEcho(void) {
    ActiveTime.start();
}

void ultrasonic::FallEcho(void) {
    ActiveTime.stop();
    ActiveWidth = ActiveTime.read_us();
    USSDistance = ActiveWidth * 0.0170;
    ActiveTime.reset();
}

void init(void) {
    TriggerTiming.attach( Trigger, 0.060 );
    USSEcho.rise(RiseEcho);
    USSEcho.fall(FallEcho);
}

void Output_Monitor(unsigned short Value) {
    printf("%d[cm]\r\n", Value);
}
