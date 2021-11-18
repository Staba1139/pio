#include "ESCout.h"
#include "mbed.h"

ESCout1::ESCout1() : PwmOut(p26) {
    throttle = 1276.0f;
    throttle_low = 0.001f;
    pwmval = 0.004f;
    PID_value = 0.0f;
    PwmOut::period(pwmval);
    PwmOut::pulsewidth(throttle_low);
}

ESCout2::ESCout2() : PwmOut(p25) {
    throttle = 1300.0f;
    throttle_low = 0.001f;
    pwmval = 0.004f;
    PID_value = 0.0f;
    PwmOut::period(pwmval);
    PwmOut::pulsewidth(throttle_low);
}

ESCout1::~ESCout1() {

}

ESCout2::~ESCout2() {

}



void ESCout1::ESC1_output(float PID_value) {
    PwmOut::pulsewidth_us((int)(throttle + PID_value));
}

void ESCout2::ESC2_output(float PID_value) {
    PwmOut::pulsewidth_us((int)(throttle - PID_value));
}