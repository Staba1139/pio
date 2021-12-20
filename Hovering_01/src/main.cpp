#include <mbed.h>
#include <Sensor.h>
#include <PIDIO.h>
#include <ESCout.h>
#include <HCSR04.h>

Sensor angle;
PIDIO PID_c0;
PIDIO PID_c1;
PIDIO PID_h;
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
Serial PC(USBTX, USBRX);
HCSR04 SR04(p12, p11);

asm(".global _printf_float");

int whole_count;
int sensor_count;
unsigned int angleTimer;
float throttle = 1200.0f;
float angle_velo_ref;
float PID_value_0;
float PID_value_1;
float throttle_low;
float pwmval;
float angleAbs_0;
float angleAbs_1;
float current_height;
float height_i;
float thrust_o;


int main() {
  SR04.startMeasurement();
  SR04.setRanges(2.0f, 200.0f);
  angle.Preprocess();
  angle_timer.start();
  height_i = SR04.getDistance_cm();
  
    

  while(1) {
    angle.calcAngle();
    SR04.startMeasurement();
    current_height = SR04.getDistance_cm() - height_i;
    if(current_height <=0.0f) current_height = 0.0f;

    if(sensor_count >= 20){
    PID_h.PID_setParameter(0.1f, 1.0f, 0.05f, 0.0f, 200.0f, 100.0f, 400.0f, 5.0f);
    thrust_o = PID_h.PID_process(current_height);
    if(thrust_o > 400.0f) thrust_o = 400.0f;
    if(thrust_o < 0.0f) thrust_o = 0.0f;
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
        PID_c0.PID_setParameter(4.0f, 1.0f, 40.0f, -1.0f*angleAbs_0, angleAbs_0, -(-0.0004*pow(angleAbs_0, 3) + 0.3f*angleAbs_0), -0.0004*pow(angleAbs_0, 3) + 0.3f*angleAbs_0, 0.0f);
        PID_value_0 = PID_c0.PID_process(angle.angle[0]);
        PID_c1.PID_setParameter(4.0f, 1.0f, 40.0f, -1.0f*angleAbs_1, angleAbs_1, -(-0.0004*pow(angleAbs_1, 3) + 0.3f*angleAbs_1), -0.0004*pow(angleAbs_1, 3) + 0.3f*angleAbs_1, 0.0f);
        PID_value_1 = PID_c1.PID_process(angle.angle[1]);
      
      }
      angleTimer = angle_timer.read_us();
      device.printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", angleTimer, angle.angle[0], angle.angle[1],PID_value_0, PID_value_1, current_height);
      sensor_count = 0;  
    
    ESC1_output.ESC1_output(PID_value_0, PID_value_1, thrust_o);
    ESC4_output.ESC4_output(PID_value_0, PID_value_1, thrust_o);
    ESC2_output.ESC2_output(PID_value_0, PID_value_1, thrust_o);
    ESC3_output.ESC3_output(PID_value_0, PID_value_1, thrust_o);
   

    if(whole_count >= 10) {
      
      //printf(",%.2f,%.2f\n", angle.gyro[0], angle.gyro[1]);
      //printf("%d,%.2f,%.2f,%.2f\n", angleTimer, angle.accel[0], angle.accel[1], angle.accel[2]);
      //printf("%d,%.5f,%.5f\n", angleTimer, angle.gyro[0], angle.gyro[1]);
      whole_count = 0;
      printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", angleTimer, angle.angle[0], angle.angle[1],PID_value_0, PID_value_1, current_height, thrust_o);
      
    }
    
    if(angleTimer >=4294967294) {
      angle_timer.reset();
      angleTimer = 0;
    }
    sensor_count++;
    whole_count++;
    //wait_us(300000);
    }
  }
}
