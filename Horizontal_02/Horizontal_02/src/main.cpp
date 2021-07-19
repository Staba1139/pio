#include <mbed.h>
#include <LSM6DS33.h>
#include <MadgwickAHRS.h>
#include <PIDcontroller.h>

#define aveNum      5
#define initCycle   5
#define aveCycle    50

LSM6DS33 sensor(p9, p10, LSM6DS33_AG_I2C_ADDR(1));
Madgwick comAng;
Serial serial(USBTX, USBRX, "debug", 9600);

asm(".global _printf_float");

float ax = 0.0, ay = 0.0, az = 0.0, gx = 0.0, gy = 0.0, gz = 0.0; //Sensor Value
float mx = 0.0, my = 0.0, mz = 0.0;                               //Invalid Sensor Value
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
float roll_init = 0.0, pitch_init = 0.0, yaw_init = 0.0;

float t_roll[aveNum], t_pitch[aveNum], t_yaw[aveNum];

float ax_init = 0.0, ay_init = 0.0, az_init = 0.0, gx_init = 0.0, gy_init = 0.0, gz_init = 0.0;

int i;
int cnt;

int main() {
  
  sensor.begin(sensor.G_SCALE_1000DPS, sensor.A_SCALE_2G, sensor.G_ODR_104, sensor.A_ODR_208);

  wait_us(5000000);

  for(i=0; i<initCycle; i++) {
    sensor.readAll();
    ax_init += sensor.ax;
    ay_init += sensor.ay;
    gx_init += sensor.gx;
    gy_init += sensor.gy;
    gz_init += sensor.gz;

    wait_us(100000);

  }

  //除算を避けるため，5で割る処理を0.2を乗算する処理とする
  ax_init *= 0.2f;
  ay_init *= 0.2f;
  gx_init *= 0.2f;
  gy_init *= 0.2f;
  gz_init *= 0.2f;

  sensor.readAll();
  ax = sensor.ax - ax_init;
  ay = sensor.ay - ay_init;
  az = sensor.az ;
  gx = sensor.gx - gx_init;
  gy = sensor.gy - gy_init;
  gz = sensor.gz - gz_init;
        
  comAng.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  for(i=0;i<aveNum; i++){
    roll_ += comAng.getRoll();
    pitch_ += comAng.getPitch();
    yaw_ += comAng.getYaw();
    //wait_us(aveCycle);
  }
        
  roll_init = roll_ * 0.2f;
  pitch_init = pitch_ * 0.2f;
  yaw_init = yaw_ * 0.2f;


  while(1) {
    sensor.readAll();
    ax = sensor.ax - ax_init;
    ay = sensor.ay - ay_init;
    az = sensor.az ;
    gx = sensor.gx - gx_init;
    gy = sensor.gy - gy_init;
    gz = sensor.gz - gz_init;

    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;

    if(yaw >180.0) {
      yaw *=-1.0f;
    }
    
    /*---Compute Angles Using MADGWICK FILTER---*/

    if(cnt == aveNum) {
      cnt = 0;
    }
    comAng.updateIMU(gx, gy, gz, ax, ay, az);

    t_roll[cnt] = comAng.getRoll();
    t_pitch[cnt] = comAng.getPitch();
    t_yaw[cnt] = comAng.getYaw();
    cnt++;

    for(i=0;i<aveNum; i++) {
      roll_ += t_roll[i];
      pitch_ += t_pitch[i];
      yaw_ += t_yaw[i];
    }
        
    roll = (roll_ * 0.2f) - roll_init;
    pitch = (pitch_ * 0.2f) - pitch_init;
    yaw = (yaw_ * 0.2f) - yaw_init;



    /*---END Compute Angles Using MADGWICK FILTER---*/  

    printf(",%.2f,%.2f\n", roll, pitch);
    //printf(",,%.2f,%.2f,%.2f\n", roll, pitch, yaw);
    //printf("Roll: %.2f [deg],Pitch: %.2f [deg]\n", roll, pitch);
    //printf("2: %.2f,\t%.2f,\t%.2f\n", roll_init, pitch_init, yaw_init);
    //printf("3: %.2f,\t%.2f,\t%.2f\n", gx_init, gy_init, gz_init);

    //wait_us(1000);
    //printf(",%.3f\n", gx);
    //printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", gx, gy, gz, ax, ay, az);
    wait_us(10000);
  }
}