#include <mbed.h>
#include <LPS25H.h>

LPS25H Pressure_c(p9, p10, LPS25H_V_CHIP_ADDR);
Timer P_timer;
float pressure_raw;
unsigned int time_1;

int main() {
  P_timer.start();
  while(1) {
    Pressure_c.get();
    pressure_raw = Pressure_c.pressure()*4096;
    time_1 = P_timer.read_us();
    printf("%d,%.2f\n",time_1, pressure_raw);

    if(time_1 >=4294967294) {
      P_timer.reset();
      time_1 = 0;
    }
  }
}
