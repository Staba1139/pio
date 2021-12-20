#include "ESCout.h"
#include "mbed.h"

ESCout1::ESCout1() : PwmOut(p26) {
    throttle = 1000.0f;
    throttle_low = 0.001f;
    pwmval = 0.02f;
    PID_value_0 = 0.0f;
    PID_value_1 = 0.0f;
    PwmOut::period(pwmval);
    PwmOut::pulsewidth(throttle_low);
}

ESCout2::ESCout2() : PwmOut(p25) {
    throttle = 1000.0f;
    throttle_low = 0.001f;
    pwmval = 0.02f;
    PID_value_0 = 0.0f;
    PID_value_1 = 0.0f;
    PwmOut::period(pwmval);
    PwmOut::pulsewidth(throttle_low);
}

ESCout3::ESCout3() : PwmOut(p24) {
    throttle = 1000.0f;
    throttle_low = 0.001f;
    pwmval = 0.02f;
    PID_value_0 = 0.0f;
    PID_value_1 = 0.0f;
    PwmOut::period(pwmval);
    PwmOut::pulsewidth(throttle_low);
}

ESCout4::ESCout4() : PwmOut(p23) {
    throttle = 1000.0f;
    throttle_low = 0.001f;
    pwmval = 0.02f;
    PID_value_0 = 0.0f;
    PID_value_1 = 0.0f;
    PwmOut::period(pwmval);
    PwmOut::pulsewidth(throttle_low);
}

ESCout1::~ESCout1() {

}

ESCout2::~ESCout2() {

}

ESCout3::~ESCout3() {

}

ESCout4::~ESCout4() {

}

    //angle[0] ....> 1+2 (+) vs 3+4 (-)
    //angle[1] ....> 1+4 (+) vs 2+3 (-)

void ESCout1::ESC1_output(float PID_value_0, float PID_value_1, float thrust_o) {
    PwmOut::pulsewidth_us((int)(996.0f + thrust_o + (PID_value_0 + PID_value_1)));
}

void ESCout2::ESC2_output(float PID_value_0, float PID_value_1, float thrust_o) {
    PwmOut::pulsewidth_us((int)(996.0f + thrust_o + (PID_value_0 - PID_value_1)));
}

void ESCout3::ESC3_output(float PID_value_0, float PID_value_1, float thrust_o) {
    PwmOut::pulsewidth_us((int)(1000.0f + thrust_o - (PID_value_0 + PID_value_1)));
}

void ESCout4::ESC4_output(float PID_value_0, float PID_value_1, float thrust_o) {
    PwmOut::pulsewidth_us((int)(1000.0f + thrust_o - (PID_value_0 - PID_value_1)));
}