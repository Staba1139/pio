#include <mbed.h>
#include <Sensor.h>
#include <PIDIO.h>
#include <ESCout.h>

Sensor angle;
PIDIO PID_c;
Timer angle_timer;
ESCout1 ESC1_output;
ESCout2 ESC2_output;
ESCout3 ESC3_output;
ESCout4 ESC4_output;
DigitalOut light01(LED1);
DigitalOut light02(LED2);
DigitalOut light03(LED3);
DigitalOut light04(LED4);
Serial device(p13, p14);


asm(".global _printf_float");

int whole_count;
unsigned int angleTimer;
float throttle = 1200.0f;
float angle_velo_ref;
float PID_value_0;
float PID_value_1;
float throttle_low;
float pwmval;
float angleAbs_0;
float angleAbs_1;


int main() {
  angle.Preprocess();
/*
    PwmOut m1(p26);
    PwmOut m2(p25);
    throttle_low = 0.001f;
    pwmval = 0.004f;
    m1.period(pwmval);
    m2.period(pwmval);
    m1.pulsewidth(throttle_low);
    m2.pulsewidth(throttle_low);
*/
  angle_timer.start();

  while(1) {
    angle.calcAngle();

    //angle[0] ....> 1+2 vs 3+4
    //angle[1] ....> 1+4 vs 2+3

    if(abs(angle.angle[1]) < 0.2f) {
      PID_value_1 = 0.0f;
    }
    if(abs(angle.angle[0]) < 0.2f) {
      PID_value_0 = 0.0f;
    }
    else {
      angleAbs_0 = abs(angle.angle[0]);
      angleAbs_1 = abs(angle.angle[1]);
      PID_c.PID_setParameter(1.0f, 0.5f, 1.0f, -1.0f*angleAbs_0, angleAbs_0, -(-0.0004*pow(angleAbs_0, 3)+2.1f*angleAbs_0), -0.0004*pow(angleAbs_0, 3)+2.1f*angleAbs_0, 0.0f);
      PID_value_0 = PID_c.PID_velocity_process(angle.angle[0]);
      PID_c.PID_setParameter(1.0f, 0.5f, 1.0f, -1.0f*angleAbs_1, angleAbs_1, -(-0.0004*pow(angleAbs_1, 3)+2.1f*angleAbs_1), -0.0004*pow(angleAbs_1, 3)+2.1f*angleAbs_1, 0.0f);
      PID_value_1 = PID_c.PID_velocity_process(angle.angle[1]);
      
      //PID_c.PID_setParameter(4.0f, 25.0f, 70.0f, -1.0f*angleAbs_1, angleAbs_1, -(0.0005*pow(angleAbs_1, 3)+0.74f*angleAbs_1), 0.0005*pow(angleAbs_1, 3)+0.74f*angleAbs_1, 0.0f);
      //PID_c.PID_setParameter(4.0f, 25.0f, 70.0f, -1.0f*angleAbs_1, angleAbs_1, -2.1f*angleAbs_1, 2.1f*angleAbs_1, 0.0f);
      

    }

    angleTimer = angle_timer.read_us();
    //printf("%d,%.3f\n", angleTimer, PID_value_1);
    ESC1_output.ESC1_output(PID_value_0, PID_value_1);
    ESC2_output.ESC2_output(PID_value_0, PID_value_1);
    ESC3_output.ESC3_output(PID_value_0, PID_value_1);
    ESC4_output.ESC4_output(PID_value_0, PID_value_1);

//    if(whole_count >= 1) {
      device.printf("%d,%.2f,%.2f\n", angleTimer, angle.angle[0], angle.angle[1]);
      //printf(",%.2f,%.2f\n", angle.gyro[0], angle.gyro[1]);
      //printf("%d,%.2f,%.2f,%.2f\n", angleTimer, angle.accel[0], angle.accel[1], angle.accel[2]);
      //printf("%d,%.5f,%.5f\n", angleTimer, angle.gyro[0], angle.gyro[1]);
      whole_count = 0;
//    }

    if(angleTimer >=4294967294) {
      angle_timer.reset();
      angleTimer = 0;
    }
    whole_count++;
    //wait_us(100);
  }
}
