#ifndef ESCout_h
#define ESCout_h
#include <mbed.h>

class ESCout1 : public PwmOut {
    private:
        float throttle_low;
        float pwmval;
    public:
        float PID_value_0;
        float PID_value_1;
        float throttle;
        ESCout1(void);
        ~ESCout1();
        void ESC1_output(float PID_value_0, float PID_value_1);
};

class ESCout2 : public PwmOut {
    private:
        float throttle_low;
        float pwmval;
    public:
        float PID_value_0;
        float PID_value_1;
        float throttle;
        ESCout2(void);
        ~ESCout2();
        void ESC2_output(float PID_value_0, float PID_value_1);
};

class ESCout3 : public PwmOut {
    private:
        float throttle_low;
        float pwmval;
    public:
        float PID_value_0;
        float PID_value_1;
        float throttle;
        ESCout3(void);
        ~ESCout3();
        void ESC3_output(float PID_value_0, float PID_value_1);
};

class ESCout4 : public PwmOut {
    private:
        float throttle_low;
        float pwmval;
    public:
        float PID_value_0;
        float PID_value_1;
        float throttle;
        ESCout4(void);
        ~ESCout4();
        void ESC4_output(float PID_value_0, float PID_value_1);
};

#endif