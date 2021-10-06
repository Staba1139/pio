#include "PIDIO.h"
#include "mbed.h"
#include "PIDcontroller.h"



PIDIO::PIDIO() : PID(1.0f, 0.1f, 0.1f, 0.03f) {
}

/*Set Parameter (float Kp, float Ki, float Kd, float tSample, float inputMin, float inputMax, float outputMin, float outputMax, float setpoint)*/
void PIDIO::PID_setParameter(float Kp, float Ki, float Kd, float inputMin, float inputMax, float outputMin, float outputMax, float setpoint) {
    PID::setInputLimits(inputMin, inputMax);
    PID::setOutputLimits(outputMin, outputMax);
    PID::setGain(Kp, Ki, Kd);
    PID::setSetPoint(setpoint);
}

/* Angular Velocity PID Process (float angular_velocity_input) */
float PIDIO::PID_velocity_process(float x_angle_velo) {
    PID::setProcessValue(x_angle_velo);
    return PID::compute();
}
/* Angle PID Process (float angle_input) */
float PIDIO::PID_process(float x_angle) {
    PID::setProcessValue(x_angle);
    return PID::compute();
}