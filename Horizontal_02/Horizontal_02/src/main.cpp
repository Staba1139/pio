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



float accel[3] = {};        // 0: ax, 1: ay, 2: az                value of accelerometer
float gyro[3] = {};         // 0: gx, 1: gy, 2: gz                value of gyroscope
float o1_accel[3] = {};     // 0: o1_ax, 1: o1_ay, 2: o1_az       value of old accelerometer
float o1_gyro[3] = {};      // 0: o1_gx, 1: o1_gy, 2: o1_gz       value of old gyroscope
float o2_accel[3] = {};     // 0: o2_ax, 1: o2_ay, 2: o2_az       value of 2nd old accelerometer
float o2_gyro[3] = {};      // 0: o2_gx, 1: o2_gy, 2: o2_gz       value of 2nd old gyroscope
float i_accel[3] = {};     // 0: i_ax, 1: i_ay, 2: i_az           initial value of accelerometer
float i_gyro[3] = {};      // 0: i_gx, 1: i_gy, 2: i_gz           initial value of gyroscope
float t_gyro[3] = {};       // 0: t_gx, 1: t_gy, 2: t_gz          temp value of current gyroscope
float d_gyro[3] = {};       // 0: d_gx, 1: d_gy, 2: d_gz          value of current gyroscope to display
float mag[3] = {};          // 0: mx, 1: my, 2: mz                value of magnetoscope

float angle[3] = {};        // 0: roll, 1: pitch, 2: yaw          value of angle
float o_angle[3] = {};      // 0: o_roll, 1: o_pitch, 2: o_yaw    value of old angle
float i_angle[3] = {};      // 0: i_roll, 1: i_pitch, 2: i_yaw    initial value of angle

                              //Invalid Sensor Value
float roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;

float t_roll[aveNum], t_pitch[aveNum], t_yaw[aveNum];

int i;
int cnt;
int whole_count;

int main() {
  
  sensor.begin(sensor.G_SCALE_245DPS, sensor.A_SCALE_2G, sensor.G_ODR_104, sensor.A_ODR_208);


  wait_us(5000000);

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

  while(1) {
    for(i = 0; i < 3; ++i) o_angle[i] = 0.0f;
    for(i = 0; i < 3; ++i) angle[i] = 0.0f;


    sensor.readAll();
    accel[0] = sensor.ax - i_accel[0];
    accel[1] = sensor.ay - i_accel[1];
    accel[2] = sensor.az - i_accel[2] + 1.0f;
    d_gyro[0] = sensor.gx - i_gyro[0];
    d_gyro[1] = sensor.gy - i_gyro[1];
    d_gyro[2] = sensor.gz - i_gyro[2];

    for(i = 0; i < 3; ++i) {
      if(std::abs(d_gyro[i] < 0.11f)) {
        d_gyro[i] = 0.0f;
      } 
    }

    /*---------- Compute Angles Using MADGWICK FILTER ----------*/

    if(cnt == aveNum) {
      cnt = 0;
    }
    comAng.updateIMU(d_gyro[0], d_gyro[1], d_gyro[2], accel[0], accel[1], accel[2]);
    
    t_roll[cnt] = comAng.getRoll();
    t_pitch[cnt] = comAng.getPitch();
    t_yaw[cnt] = comAng.getYaw();
    cnt++;

    for(i=0;i<aveNum; i++) {
      o_angle[0] += t_roll[i];
      o_angle[1] += t_pitch[i];
      o_angle[2] += t_yaw[i];
    }
        
    angle[0] = (o_angle[0] * 0.2f);
    angle[1] = (o_angle[1] * 0.2f);
    angle[2] = (o_angle[2] * 0.2f);

    if(angle[2] >180.0f) {
      angle[2] *=-1.0f;
    }

    for(i = 0; i < 3; ++i) {
      if(std::abs(angle[i]) < 0.3f) angle[i] = 0.0f;
    }

    //angle[0] = comAng.getRoll();
    //angle[1] = comAng.getPitch();
    //angle[2] = comAng.getYaw();

    /*---------- END Compute Angles Using MADGWICK FILTER ----------*/  

    if(whole_count >= 10) {
      printf(",%.2f,%.2f\n", angle[0], angle[1]);
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
    wait_us(10000);
  }
}
