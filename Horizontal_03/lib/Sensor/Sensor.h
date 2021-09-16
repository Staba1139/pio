#ifndef Sensor_h
#define Sensor_h
#include <LSM6DS33.h>
#include <MadgwickAHRS.h>

#define initCycle 5
#define aveNum 5

class Sensor: public LSM6DS33, public Madgwick {
    private:
        float pwmval;
        float throttle_low;
        float d_accel[3] = {};
        int i;
        int cnt;
        float t_roll[aveNum], t_pitch[aveNum], t_yaw[aveNum];
        float o_angle[3] = {};

    public:
        float i_accel[3] = {};
        float i_gyro[3] = {};
        float accel[3] = {};
        float gyro[3] = {};
        float angle[3] = {};
        Sensor(void);
        void Preprocess(void);
        void calcAngle(void);

};
#endif