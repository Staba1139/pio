#include <mbed.h>
#include <Sensor.h>
#include <PIDIO.h>
#include <ESCout.h>

Sensor angle;
PIDIO PID_c;
Timer angle_timer;
ESCout1 ESC1_output;
ESCout2 ESC2_output;
DigitalOut light01(LED1);
DigitalOut light02(LED2);
DigitalOut light03(LED3);
DigitalOut light04(LED4);


asm(".global _printf_float");

int whole_count;
unsigned int angleTimer;
float throttle = 1200.0f;
float angle_velo_ref;
float PID_value;
float throttle_low;
float pwmval;
float kp1, ki1, kd1;
float kp2, ki2, kd2;
float outmin1, outmax1;
float inmin1, inmax1;
float outmin2, outmax2;
float inmin2, inmax2;


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
    // Calcurate Angle
    //angleTimer = angle_timer.read_us();
    //printf("%d\n", angleTimer);
    
    angle.calcAngle();
    //angleTimer = angle_timer.read_us();
    //printf("%d\n", angleTimer);

/*
    // PID Process (float Kp, float Ki, float Kd, float inputMin, float inputMax, float outputMin, float outputMax, float setpoint)
    // --- Angle P Control ---
    PID_c.PID_setParameter(10.0f, 0.0f, 0.0f, -40.00f, 40.00f, -150.0f, 150.0f, 0.0f);
    PID_value = PID_c.PID_velocity_process(angle.angle[1]);
    if((int)PID_value%4 >1) PID_value -= (int)PID_value%4;
    //printf("%.3f\n", PID_value);
    ESC1_output.ESC1_output(PID_value);
    ESC2_output.ESC2_output(PID_value);
*/


  // --- Angle P Control + Angle Velocity PID Control ---

/*
    if(abs(angle.angle[1]) > 0.8f) {
      if(abs(angle.angle[1]) > 20.0f) {
        kp1 = 1.0f;
        ki1 = kd1 = 0.0f;
        inmax1 = 40.0f;
        inmin1 = -1*inmax1;
        outmax1 = 150.0f;
        outmin1 = -1*outmax1;
        kp2 = 2.0f;
        ki2 = 0.0f;
        kd2 = 0.0f;
        inmax2 = 150.0f;
        inmin2 = -1*inmax2;
        outmax2 = 50.0f;
        outmin2 = -1*outmax2;      
      }
      else {
        kp1 = 2.0f;
        ki1 = kd1 = 0.0f;
        inmax1 = 40.0f;
        inmin1 = -inmax1;
        outmax1 = 50.0f;
        outmin1 = -outmax1;
        kp2 = 5.0f;
        ki2 = 25.0f;
        kd2 = 50.0f;
        inmax2 = 50.0f;
        inmin2 = -inmax2;
        outmax2 = 28.0f;
        outmin2 = -outmax2;      
      }

      PID_c.PID_setParameter(kp1, ki1, kd1, inmin1, inmax1, outmin1, outmax1, 0.0f);
      angle_velo_ref = PID_c.PID_velocity_process(angle.angle[1]);
      PID_c.PID_setParameter(kp2, ki2, kd2, inmin2, inmax2, outmin2, outmax2, angle_velo_ref);
      PID_value = PID_c.PID_velocity_process(angle.gyro[1]);
      if((int)PID_value%4 >1) PID_value -= (int)PID_value%4;

    }
    else {
      PID_value = 0.0f;
    }

    
*/

///*
    if(abs(angle.angle[1]) < 0.8f) {
      PID_value = 0.0f;
    }
//    else if(angle.angle[1] < -20.0f || angle.angle[1] > 20.0f) {
 //     PID_c.PID_setParameter(2.0f, 0.0f, 0.0f, -40.00f, 40.00f, -70.0f, 70.0f, 0.0f);
  //    PID_value = PID_c.PID_velocity_process(angle.angle[1]);
  //  }
    else {
      PID_c.PID_setParameter(2.0f, 0.0f, 0.0f, -20.00f, 20.00f, -100.0f, 100.0f, 0.0f);
      angle_velo_ref = PID_c.PID_velocity_process(angle.angle[1]);

      PID_c.PID_setParameter(5.0f, 25.0f, 50.0f, -100.0f, 100.0f, -24.0f, 24.0f, angle_velo_ref);
      PID_value = PID_c.PID_velocity_process(angle.gyro[1]);
      if((int)PID_value%4 >1) PID_value -= (int)PID_value%4;
    }
//*/


    angleTimer = angle_timer.read_us();
    //printf("%d\n", angleTimer);

    //printf("%d,%.3f\n", angleTimer, PID_value);
    ESC1_output.ESC1_output(PID_value);
    ESC2_output.ESC2_output(PID_value);
/*
    m1.pulsewidth_us((int)(throttle + PID_value));
    m2.pulsewidth_us((int)(throttle - PID_value));
*/
//    if(whole_count >= 1) {
      //printf(",%d,%.2f,%.2f\n", angleTimer, angle.angle[0], angle.angle[1]);
      //printf(",%.2f,%.2f\n", angle.gyro[0], angle.gyro[1]);
      //printf("%d,%.2f,%.2f,%.2f\n", angleTimer, angle.accel[0], angle.accel[1], angle.accel[2]);
      printf("%d,%.5f,%.5f\n", angleTimer, angle.gyro[0], angle.gyro[1]);
      whole_count = 0;
//    }

    if(angleTimer >=4294967294) angleTimer = 0;
    whole_count++;
    //wait_us(100);
  }
}
