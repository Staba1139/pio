#include "Sensor.h"
#include "mbed.h"
#include "LSM6DS33.h"
#include "MadgwickAHRS.h"



Sensor::Sensor() : LSM6DS33(p9, p10, LSM6DS33_AG_I2C_ADDR(1)) , Madgwick(){
    LSM6DS33::begin(LSM6DS33::G_SCALE_245DPS, LSM6DS33::A_SCALE_2G, LSM6DS33::G_ODR_104, LSM6DS33::A_ODR_104);
}

void Sensor::Preprocess(void) {
    
    wait_us(8000000);

    for (i = 0; i < initCycle; i++) {
        LSM6DS33::readAll();
        i_accel[0] += LSM6DS33::ax;
        i_accel[1] += LSM6DS33::ay;
        i_accel[2] += LSM6DS33::az;
        i_gyro[0] += LSM6DS33::gx;
        i_gyro[1] += LSM6DS33::gy;
        i_gyro[2] += LSM6DS33::gz;
        wait_us(100000);
    }
    //除算を避けるため，5で割る処理を0.2を乗算する処理とする
    i_accel[0] *= 0.2f;
    i_accel[1] *= 0.2f;
    i_accel[2] *= 0.2f;
    i_gyro[0] *= 0.2f;
    i_gyro[1] *= 0.2f;
    i_gyro[2] *= 0.2f;

    invNorm = Madgwick::invSqrt(i_accel[0] * i_accel[0] + i_accel[1] * i_accel[1] + i_accel[2] * i_accel[2]);
    for(i=0;i<3;++i) i_accel[i] *= invNorm;

    
    Madgwick::updateIMU(i_gyro[0], i_gyro[1], i_gyro[2], i_accel[0], i_accel[1], i_accel[2]);
    i_angle[0] = Madgwick::getRoll();
    i_angle[1] = Madgwick::getPitch();
    i_angle[2] = Madgwick::getYaw();

}

void Sensor::calcAngle() {

    //--------------------------- Values of Accelerometer and Gyroscope
    for(i = 0; i < 3; ++i) o_angle[i] = 0.0f;
    for(i = 0; i < 3; ++i) angle[i] = 0.0f;
    LSM6DS33::readAll();
    for(i = 0; i < 3; ++i) d_accel[i] = 0.0f;

/*
    for(i=0; i<3; i++) {
      LSM6DS33::readAccel();
      d_accel[0] += LSM6DS33::ax - i_accel[0];
      d_accel[1] += LSM6DS33::ay - i_accel[1];
      d_accel[2] += LSM6DS33::az;
    }
    accel[0] = d_accel[0] / 3;
    accel[1] = d_accel[1] / 3;
    accel[2] = d_accel[2] / 3;
*/
    accel[0] = LSM6DS33::ax;
    accel[1] = LSM6DS33::ay;
    accel[2] = LSM6DS33::az;
    gyro[0] = LSM6DS33::gx - i_gyro[0];
    gyro[1] = LSM6DS33::gy - i_gyro[1];
    gyro[2] = LSM6DS33::gz - i_gyro[2];


//    invNorm = Madgwick::invSqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
//    for(i=0;i<3;++i) accel[i] *= invNorm;


    if(cnt == aveNum) {
      cnt = 0;
    }
    //-------------------------- Angle Orientation with Madgwick Filter --------------------------------------------------------------
    Madgwick::updateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
/*    
    angle[0] = Madgwick::getPitch() - i_angle[0];
    angle[1] = Madgwick::getRoll() - i_angle[1];
    angle[2] = Madgwick::getYaw() - i_angle[2];
*/
    t_roll[cnt] = Madgwick::getRoll();
    t_pitch[cnt] = Madgwick::getPitch();
    t_yaw[cnt] = Madgwick::getYaw();
    cnt++;

    for(i=0;i<aveNum; i++) {
      o_angle[0] += t_roll[i];
      o_angle[1] += t_pitch[i];
      o_angle[2] += t_yaw[i];
    }
        
    angle[0] = (o_angle[0] / aveNum);
    angle[1] = (o_angle[1] / aveNum);
    angle[2] = (o_angle[2] / aveNum);

    if(angle[2] >180.0f) {
      angle[2] *=-1.0f;
    }
}