#include <mbed.h>
#include <Sensor.h>

Sensor angle;

asm(".global _printf_float");

int whole_count;

int main() {
  angle.Preprocess();

  while(1) {
    angle.calcAngle();
    if(whole_count >= 2) {
      printf(",%.2f,%.2f\n", angle.angle[0], angle.angle[1]);
      whole_count = 0;
    }

    whole_count++;
    wait_us(1000);
  }
}
