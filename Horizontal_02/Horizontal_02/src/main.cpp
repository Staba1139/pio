#include <mbed.h>
#include <LSM6DS33.h>
#include <MadgwickAHRS.h>
#include <PIDcontroller.h>

#define aveNum      5
#define initCycle   5
#define aveCycle    50

LSM6DS33 sensor(p9, p10, LSM6DS33_AG_I2C_ADDR(1));
Madgwick comAng;


asm(".global _printf_float");

PwmOut motor1(p26);
PwmOut motor2(p25);
const float throttle_low = 0.001f;


float accel[3] = {};        // 0: ax, 1: ay, 2: az                value of accelerometer
float gyro[3] = {};         // 0: gx, 1: gy, 2: gz                value of gyroscope
float i_accel[3] = {};     // 0: i_ax, 1: i_ay, 2: i_az           initial value of accelerometer
float i_gyro[3] = {};      // 0: i_gx, 1: i_gy, 2: i_gz           initial value of gyroscope
float d_accel[3] = {};

float angle[3] = {};        // 0: roll, 1: pitch, 2: yaw          value of angle
float o_angle[3] = {};      // 0: o_roll, 1: o_pitch, 2: o_yaw    value of old angle
float reference_angle[3] = {};      // 0: r_roll, 1: r_pitch, 2: r_yaw    reference value of angle
float calc_angle[3] = {};      // 0: r_roll, 1: r_pitch, 2: r_yaw    calc value of angle

                              //Invalid Sensor Value
float roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
float temp_1;

float t_roll[aveNum], t_pitch[aveNum], t_yaw[aveNum];

int i;
int cnt;
int whole_count;
float pulsewidth_calc[2] = {1200.0f, 1200.0f};

int main() {
  
  float pwmval = 0.004f;
  sensor.begin(sensor.G_SCALE_245DPS, sensor.A_SCALE_2G, sensor.G_ODR_104, sensor.A_ODR_104);

    motor1.period(pwmval);
    motor2.period(pwmval);
    
    motor1.pulsewidth(throttle_low);
    motor2.pulsewidth(throttle_low);

  wait_us(8000000);

  //-----------------------------------------Sensor Pre-Process-----------------------------------------------------------------------
  

  //オフセット除去
  for(i=0; i<initCycle; i++) {
    sensor.readAll();
    i_accel[0] += sensor.ax;
    i_accel[1] += sensor.ay;
    i_accel[2] += sensor.az;
    i_gyro[0] += sensor.gx;
    i_gyro[1] += sensor.gy;
    i_gyro[2] += sensor.gz;

    wait_us(100000);

  }

  //除算を避けるため，5で割る処理を0.2を乗算する処理とする
  i_accel[0] *= 0.2f;
  i_accel[1] *= 0.2f;
  i_accel[2] *= 0.2f;
  i_gyro[0] *= 0.2f;
  i_gyro[1] *= 0.2f;
  i_gyro[2] *= 0.2f;


 //-----------------------------------------End of Sensor Pre-Process-----------------------------------------------------------------------

  while(1) {
    for(i = 0; i < 3; ++i) o_angle[i] = 0.0f;
    for(i = 0; i < 3; ++i) angle[i] = 0.0f;


    sensor.readAll();


    //Smoosing Accelerometer Value
    for(i=0; i<3; i++) {
      d_accel[i] = 0.0f;
    }

    for(i=0; i<3; i++) {
      sensor.readAccel();
      d_accel[0] += sensor.ax - i_accel[0];
      d_accel[1] += sensor.ay - i_accel[1];
      d_accel[2] += sensor.az;
    }
    accel[0] = d_accel[0] / 3;
    accel[1] = d_accel[1] / 3;
    accel[2] = d_accel[2] / 3;

    //accel[0] = sensor.ax - i_accel[0];
    //accel[1] = sensor.ay - i_accel[1];
    //accel[2] = sensor.az;
    gyro[0] = sensor.gx - i_gyro[0];
    gyro[1] = sensor.gy - i_gyro[1];
    gyro[2] = sensor.gz - i_gyro[2];

    /*---------- Compute Angles Using MADGWICK FILTER ----------*/



    if(cnt == aveNum) {
      cnt = 0;
    }
    comAng.updateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
    
    t_roll[cnt] = comAng.getRoll();
    t_pitch[cnt] = comAng.getPitch();
    t_yaw[cnt] = comAng.getYaw();
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

    //comAng.updateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
    //angle[0] = comAng.getRoll();
    //angle[1] = comAng.getPitch();
    //angle[2] = comAng.getYaw();

//    for(i = 0; i < 3; ++i) {
//      if(std::abs(angle[i]) < 0.5f) angle[i] = 0.0f;
//    }

    /*---------- END Compute Angles Using MADGWICK FILTER ----------*/  


/*--------------- PID Control---------------------------------------*/
    calc_angle[1] = angle[1] - reference_angle[1];
    pulsewidth_calc[0] = 1.5f * calc_angle[1] + 1300.0f;
    pulsewidth_calc[1] = -1.5f * calc_angle[1] + 1304.0f;
    for(i = 0; i < 2; ++i) {
      if(pulsewidth_calc[i] < 1000.0f) pulsewidth_calc[i] = 1000.0f;
      if(pulsewidth_calc[i] > 1800.0f) pulsewidth_calc[i] = 1800.0f;

    }
    motor1.pulsewidth_us((int)pulsewidth_calc[0]);
    motor2.pulsewidth_us((int)pulsewidth_calc[1]);

/*--------------- End of PID Control---------------------------------------*/

    sensor.readTemp();
    temp_1 = sensor.temperature_c;

    if(whole_count >= 2) {
      //printf(",%.2f,%.2f, %.2f\n", angle[1], pulsewidth_calc[0], pulsewidth_calc[1]);
      printf(",%.2f,%.2f\n", angle[0], angle[1]);
      //printf(",%.4f,%.4f,%.4f\n", accel[0], accel[1], accel[2]);
      //printf(", %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", d_gyro[0], d_gyro[1], d_gyro[2], d_accel[0], d_accel[1], d_accel[2]);
      //printf(", %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", sensor.ax, sensor.ay, sensor.az, sensor.gx, sensor.gy, sensor.gz);
      //printf("%.2f\n", temp_1);
      whole_count = 0;
    }

    //printf(",,%.2f,%.2f,%.2f\n", angle[0], angle[1], angle[2]);
    //printf("Roll: %.2f [deg],Pitch: %.2f [deg]\n", roll, pitch);
    //printf("2: %.2f,\t%.2f,\t%.2f\n", roll_init, pitch_init, yaw_init);
    //printf("3: %.2f,\t%.2f,\t%.2f\n", gx_init, gy_init, gz_init);
    //printf(", %4f, %4f, %4f, %4f\n", comAng.q0, comAng.q1, comAng.q2, comAng.q3);
    //wait_us(1000);
    //printf(",%.3f\n", gx);
    //printf(", %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", d_gyro[0], d_gyro[1], d_gyro[2], accel[0], accel[1], accel[2]);
    //printf(", %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", gyro[0]*0.01745329f, gyro[1]*0.01745329f, gyro[2]*0.01745329f, accel[0], accel[1], accel[2]);
    whole_count++;
    //wait_us(1000);
  }
}
