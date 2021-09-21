#ifndef ESCout_h
#define ESCout_h
#include <mbed.h>

class ESCout1 : public PwmOut {
    private:
        float throttle_low;
        float pwmval;
    public:
        float PID_value;
        float throttle;
        ESCout1(void);
        ~ESCout1();
        void ESC1_output(float PID_value);
};

class ESCout2 : public PwmOut {
    private:
        float throttle_low;
        float pwmval;
    public:
        float PID_value;
        float throttle;
        ESCout2(void);
        ~ESCout2();
        void ESC2_output(float PID_value);
};

#endif