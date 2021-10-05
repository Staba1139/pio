#ifndef PIDIO_h
#define PIDIO_h
#include <PIDcontroller.h>

class PIDIO: public PID {
    private:

    public:
        PIDIO(void);
        void PID_setParameter(float Kp, float Ki, float Kd, float inputMin, float inputMax, float outputMin, float outputMax, float setpoint);
        float PID_velocity_process(float x_angle_velo);
        float PID_process(float x_angle);
};

#endif
