#include <mbed.h>
#include <Sensor.h>
#include <PIDIO.h>

Sensor angle;
PIDIO PID_c;
Timer angle_timer;

asm(".global _printf_float");

int whole_count;
unsigned int angleTimer;
float throttle = 1300.0f;
float PID_value;
float throttle_low;
float pwmval;

int main() {
  angle.Preprocess();

    PwmOut m1(p26);
    PwmOut m2(p25);
    throttle_low = 0.001f;
    pwmval = 0.004f;
    m1.period(pwmval);
    m2.period(pwmval);
    m1.pulsewidth(throttle_low);
    m2.pulsewidth(throttle_low);

  angle_timer.start();

  while(1) {
    // Calcurate Angle
    angleTimer = angle_timer.read_us();
    printf("%d\n", angleTimer);
    
    angle.calcAngle();
    angleTimer = angle_timer.read_us();
    printf("%d\n", angleTimer);


    // PID Process (float Kp, float Ki, float Kd, float tSample, float inputMin, float inputMax, float outputMin, float outputMax, float setpoint)
    PID_c.PID_setParameter(1.0f, 0.0f, 0.0f, 0.01f, -45.00f, 45.00f, -40.0f, 40.0f, 0.01f);
    PID_value = PID_c.PID_velocity_process(angle.angle[1]);
    m1.pulsewidth_us((int)(throttle + PID_value));
    m2.pulsewidth_us((int)(throttle - PID_value));


    if(whole_count >= 2) {
      printf(",%d,%.2f,%.2f\n", angleTimer, angle.angle[0], angle.angle[1]);
      whole_count = 0;
    }

    if(angleTimer >=4294967294) angleTimer = 0;
    whole_count++;
    wait_us(1000);
  }
}
